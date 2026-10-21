#include <rclcpp/rclcpp.hpp>
#include "profile.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <chrono>
#include <cstdint>
#include <vector>
#include <array>
#include <cerrno>
#include <cstring>
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
  UartBridge():Node("esp32_uart_bridge"),profile_(*this) {
    port_=declare_parameter<std::string>("port","");
    pub_=create_publisher<std_msgs::msg::UInt8MultiArray>("fc/status",10);
    sub_=create_subscription<std_msgs::msg::UInt8MultiArray>("fc/companion_tx",10,[this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr m){
      const auto &p=m->data;
      if(p.size()!=28||p[0]!='Q'||p[1]!='N'||p[2]!=1||(p[3]!=1&&p[3]!=3)||crc(p.data(),24)!=get32(p.data()+24)){profile_.reject();return;}
      for(unsigned i=0;i<4;i++){float v;std::memcpy(&v,p.data()+8+4*i,4);const float limit=i<2?35.f:i==2?180.f:3.f;
        if(!std::isfinite(v)||std::abs(v)>limit){profile_.reject();return;}}
      std::copy(p.begin(),p.end(),latest_.begin());latest_time_=Clock::now();have_latest_=true;
    });
    timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{receive();});
    RCLCPP_INFO(get_logger(),"UART port='%s' baud=115200; setpoints only, no physical arming command",port_.c_str());
  }
  ~UartBridge(){close_port();}
 private:
  void close_port(){
#ifdef _WIN32
    if(handle_!=INVALID_HANDLE_VALUE)CloseHandle(handle_);handle_=INVALID_HANDLE_VALUE;
#else
    if(fd_>=0)::close(fd_);fd_=-1;
#endif
  }
 private:
  CallbackProfile profile_;
  using Clock=std::chrono::steady_clock;
  std::string port_;Clock::time_point latest_time_{},tx_time_{},last_open_{};
  std::array<uint8_t,28> latest_{},tx_{};bool have_latest_=false;size_t tx_offset_=28;
  static uint32_t get32(const uint8_t*p){uint32_t v=0;for(unsigned i=0;i<4;i++)v|=uint32_t(p[i])<<(8*i);return v;}
  static uint32_t crc(const uint8_t*p,unsigned n){uint32_t c=~0u;for(unsigned i=0;i<n;i++){c^=p[i];for(unsigned k=0;k<8;k++)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
  bool connected()const{
#ifdef _WIN32
    return handle_!=INVALID_HANDLE_VALUE;
#else
    return fd_>=0;
#endif
  }
  void open(const std::string&port){
#ifdef _WIN32
    handle_=CreateFileA(port.c_str(),GENERIC_READ|GENERIC_WRITE,0,nullptr,OPEN_EXISTING,0,nullptr);
    if(handle_==INVALID_HANDLE_VALUE)throw std::runtime_error("Cannot open UART");
    DCB cfg{};cfg.DCBlength=sizeof(cfg);GetCommState(handle_,&cfg);cfg.BaudRate=CBR_115200;cfg.ByteSize=8;cfg.Parity=NOPARITY;cfg.StopBits=ONESTOPBIT;
    if(!SetCommState(handle_,&cfg))throw std::runtime_error("UART configuration failed");
    COMMTIMEOUTS timeout{};timeout.ReadIntervalTimeout=MAXDWORD;timeout.WriteTotalTimeoutConstant=5;if(!SetCommTimeouts(handle_,&timeout))throw std::runtime_error("UART timeouts failed");
#else
    fd_=::open(port.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd_<0)throw std::runtime_error("Cannot open UART");
    termios cfg{};if(tcgetattr(fd_,&cfg))throw std::runtime_error("UART attributes unavailable");cfmakeraw(&cfg);cfsetispeed(&cfg,B115200);cfsetospeed(&cfg,B115200);cfg.c_cflag|=CLOCAL|CREAD;if(tcsetattr(fd_,TCSANOW,&cfg))throw std::runtime_error("UART configuration failed");
#endif
  }
  int write_some(const uint8_t*p,size_t n){
#ifdef _WIN32
    DWORD sent=0;if(!WriteFile(handle_,p,DWORD(n),&sent,nullptr))return -1;return int(sent);
#else
    auto sent=::write(fd_,p,n);if(sent<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR))return 0;return int(sent);
#endif
  }
  void pump_tx(){
    const auto now=Clock::now();
    if(!connected()&&!port_.empty()&&std::chrono::duration<double>(now-last_open_).count()>2){
      last_open_=now;try{open(port_);}catch(const std::exception&e){close_port();RCLCPP_WARN(get_logger(),"UART retry: %s",e.what());}}
    if(tx_offset_<28&&std::chrono::duration<double>(now-tx_time_).count()>.1){tx_offset_=28;profile_.reject();}
    if(have_latest_&&std::chrono::duration<double>(now-latest_time_).count()>.1){have_latest_=false;profile_.reject();}
    if(!connected())return;
    if(tx_offset_==28&&have_latest_){tx_=latest_;tx_offset_=0;tx_time_=latest_time_;have_latest_=false;}
    if(tx_offset_<28){int sent=write_some(tx_.data()+tx_offset_,28-tx_offset_);
      if(sent<0){close_port();tx_offset_=28;profile_.reject();}else tx_offset_+=size_t(sent);}
  }
  void receive(){
    CallbackProfile::Scope scope(profile_);
    pump_tx();
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
        else {profile_.reject();for(unsigned j=1;j<16;j++)if(frame_[j]=='Q'&&(j==15||frame_[j+1]=='S')){used_=16-j;std::memmove(frame_,frame_+j,used_);break;}}
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
