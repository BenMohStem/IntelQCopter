#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <chrono>
#include <cstdint>
#include <vector>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

class UartBridge:public rclcpp::Node {
 public:
  UartBridge():Node("esp32_uart_bridge") {
    const auto port=declare_parameter<std::string>("port","");
    if(!port.empty())open(port);
    pub_=create_publisher<std_msgs::msg::UInt8MultiArray>("fc/status",10);
    sub_=create_subscription<std_msgs::msg::UInt8MultiArray>("fc/companion_tx",10,[this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr m){
      if(m->data.size()!=28||m->data[0]!='Q'||m->data[1]!='N')return;
      if(!write(m->data.data(),m->data.size()))RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"UART unavailable or partial write");
    });
    timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{receive();});
    RCLCPP_INFO(get_logger(),"UART port='%s' baud=115200; setpoints only, no physical arming command",port.c_str());
  }
  ~UartBridge(){
#ifdef _WIN32
    if(handle_!=INVALID_HANDLE_VALUE)CloseHandle(handle_);
#else
    if(fd_>=0)::close(fd_);
#endif
  }
 private:
  void open(const std::string&port){
#ifdef _WIN32
    handle_=CreateFileA(port.c_str(),GENERIC_READ|GENERIC_WRITE,0,nullptr,OPEN_EXISTING,0,nullptr);
    if(handle_==INVALID_HANDLE_VALUE)throw std::runtime_error("Cannot open UART");
    DCB cfg{};cfg.DCBlength=sizeof(cfg);GetCommState(handle_,&cfg);cfg.BaudRate=CBR_115200;cfg.ByteSize=8;cfg.Parity=NOPARITY;cfg.StopBits=ONESTOPBIT;
    if(!SetCommState(handle_,&cfg))throw std::runtime_error("UART configuration failed");
    COMMTIMEOUTS timeout{};timeout.ReadIntervalTimeout=MAXDWORD;SetCommTimeouts(handle_,&timeout);
#else
    fd_=::open(port.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd_<0)throw std::runtime_error("Cannot open UART");
    termios cfg{};if(tcgetattr(fd_,&cfg))throw std::runtime_error("UART attributes unavailable");cfmakeraw(&cfg);cfsetispeed(&cfg,B115200);cfsetospeed(&cfg,B115200);cfg.c_cflag|=CLOCAL|CREAD;if(tcsetattr(fd_,TCSANOW,&cfg))throw std::runtime_error("UART configuration failed");
#endif
  }
  bool write(const uint8_t*p,size_t n){
#ifdef _WIN32
    DWORD sent=0;return handle_!=INVALID_HANDLE_VALUE&&WriteFile(handle_,p,DWORD(n),&sent,nullptr)&&sent==n;
#else
    return fd_>=0&&::write(fd_,p,n)==static_cast<ssize_t>(n);
#endif
  }
  void receive(){
    uint8_t data[128];int n=0;
#ifdef _WIN32
    DWORD count=0;if(handle_!=INVALID_HANDLE_VALUE&&ReadFile(handle_,data,sizeof(data),&count,nullptr))n=int(count);
#else
    if(fd_>=0)n=int(::read(fd_,data,sizeof(data)));
#endif
    for(int i=0;i<n;i++){
      uint8_t byte=data[i];if(used_==0&&byte!='Q')continue;
      if(used_==1&&byte!='S'){used_=byte=='Q'?1:0;continue;}
      frame_[used_++]=byte;
      if(used_==16){used_=0;uint32_t crc=~0u;for(unsigned j=0;j<12;j++){crc^=frame_[j];for(int k=0;k<8;k++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}crc=~crc;
        uint32_t expected=0;for(unsigned j=0;j<4;j++)expected|=uint32_t(frame_[12+j])<<(8*j);
        if(crc==expected&&frame_[2]==1){std_msgs::msg::UInt8MultiArray msg;msg.data.assign(frame_,frame_+16);pub_->publish(msg);}
      }
    }
  }
#ifdef _WIN32
  HANDLE handle_=INVALID_HANDLE_VALUE;
#else
  int fd_=-1;
#endif
  uint8_t frame_[16]{};unsigned used_=0;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<UartBridge>());rclcpp::shutdown();}
