/*
 * IntelQCopter — ESP32-S3 Competition Flight Controller
 * ======================================================
 *
 * Target:  ESP32-S3 DevKitC-1 N8, 8 MB flash, no PSRAM
 * Framework: Arduino / PlatformIO
 * Control rate: 500 Hz deterministic (2 ms cycle)
 * UART: 921600 baud, 8N1, 52-byte CRC-protected v3 binary packets
 * I²C: 400 kHz for ALL sensors (MPU6050, BMP280, QMC5883P)
 * Motor outputs: DISABLED during upload, calibration, and diagnostics
 *
 * LiDAR geometry (TOFSense-F2 Mini, body -Z pointing down):
 *   Mounting offset relative to COM: (0, 0, -0.026658) m
 *   H_COM ≈ (D + 0.026658) × cos(roll) × cos(pitch)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <climits>
#include <initializer_list>

// ============================================================================
// CONFIGURATION CONSTANTS (namespace cfg)
// ============================================================================

namespace cfg {

constexpr uint8_t SDA_PIN = 9;
constexpr uint8_t SCL_PIN = 8;
constexpr uint32_t I2C_HZ = 400000;

constexpr uint8_t MPU_PRIMARY_ADDRESS = 0x68;
constexpr uint8_t MPU_ALTERNATE_ADDRESS = 0x69;
constexpr uint8_t BMP_PRIMARY_ADDRESS = 0x76;
constexpr uint8_t BMP_ALTERNATE_ADDRESS = 0x77;
constexpr uint8_t MAG_ADDRESS = 0x2C;

constexpr uint8_t MOTOR_PINS[4] = {12, 11, 14, 13};
constexpr uint8_t LIDAR_RX = 1;
constexpr uint8_t LIDAR_TX = 2;

constexpr float LIDAR_OFFSET_Z_M = -0.026658f;
constexpr float LIDAR_OFFSET_X_M = 0.0f;
constexpr float LIDAR_OFFSET_Y_M = 0.0f;
constexpr float LIDAR_MAX_RANGE_M = 12.0f;
constexpr float LIDAR_MIN_RANGE_M = 0.3f;
constexpr float LIDAR_MAX_TILT_RAD = 0.5f;

constexpr uint32_t CONTROL_PERIOD_US = 2000;
constexpr uint32_t IMU_PERIOD_US = 2000;
constexpr uint32_t MAG_PERIOD_US = 5000;
constexpr uint32_t BMP_PERIOD_US = 6000;
constexpr uint32_t LIDAR_PERIOD_US = 20000;
constexpr uint32_t MAG_STALE_MS = 1000;
constexpr uint32_t CONTROLLER_TIMEOUT_MS = 250;

constexpr uint32_t SERIAL_BAUD = 921600;
constexpr size_t PACKET_SIZE = 52;

constexpr uint8_t MPU_REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t MPU_REG_PWR_MGMT_2    = 0x6C;
constexpr uint8_t MPU_REG_CONFIG        = 0x1A;
constexpr uint8_t MPU_REG_GYRO_CONFIG   = 0x1B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG  = 0x1C;
constexpr uint8_t MPU_REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t MPU_REG_FIFO_EN       = 0x23;
constexpr uint8_t MPU_REG_INT_PIN_CFG   = 0x37;
constexpr uint8_t MPU_REG_INT_ENABLE    = 0x38;
constexpr uint8_t MPU_REG_USER_CTRL     = 0x6A;
constexpr uint8_t MPU_REG_FIFO_COUNT_H  = 0x72;
constexpr uint8_t MPU_REG_FIFO_R_W      = 0x74;
constexpr uint8_t MPU_REG_WHO_AM_I      = 0x75;

constexpr uint8_t MPU_PWR_MGMT_1_RESET      = 0x80;
constexpr uint8_t MPU_PWR_MGMT_1_CLKSEL     = 0x01;
constexpr uint8_t MPU_PWR_MGMT_1_TEMP_DIS   = 0x08;
constexpr uint8_t MPU_DLPF_188HZ            = 0x01;
constexpr uint8_t MPU_GYRO_FS_500           = 0x08;
constexpr uint8_t MPU_ACCEL_FS_8G           = 0x10;
constexpr uint8_t MPU_FIFO_TEMP_EN          = 0x80;
constexpr uint8_t MPU_FIFO_XG_EN            = 0x40;
constexpr uint8_t MPU_FIFO_YG_EN            = 0x20;
constexpr uint8_t MPU_FIFO_ZG_EN            = 0x10;
constexpr uint8_t MPU_FIFO_ACCEL_EN         = 0x08;
constexpr uint8_t MPU_USER_CTRL_FIFO_EN     = 0x40;
constexpr uint8_t MPU_USER_CTRL_FIFO_RESET  = 0x04;
constexpr uint8_t MPU_INT_PIN_CFG_BYPASS_EN = 0x02;
constexpr uint8_t MPU_INT_ENABLE_RAW_RDY_EN = 0x01;

constexpr uint8_t QMC_REG_CHIP_ID   = 0x00;
constexpr uint8_t QMC_REG_CTRL1     = 0x0A;
constexpr uint8_t QMC_REG_CTRL2     = 0x0B;
constexpr uint8_t QMC_REG_SET_PERIOD= 0x0C;
constexpr uint8_t QMC_REG_XYZ_SIGN  = 0x29;
constexpr uint8_t QMC_REG_DATA_XL   = 0x01;
constexpr uint8_t QMC_REG_STATUS    = 0x09;

constexpr uint8_t QMC_CHIP_ID       = 0x80;
constexpr uint8_t QMC_CTRL1_200HZ   = 0x3D;
constexpr uint8_t QMC_CTRL2_8GAUSS  = 0x08;
constexpr uint8_t QMC_XYZ_SIGN_VAL  = 0x06;

constexpr uint8_t BMP_REG_ID           = 0xD0;
constexpr uint8_t BMP_REG_RESET        = 0xE0;
constexpr uint8_t BMP_REG_CTRL_MEAS    = 0xF4;
constexpr uint8_t BMP_REG_CONFIG       = 0xF5;
constexpr uint8_t BMP_REG_PRESSURE_MSB = 0xF7;

constexpr uint8_t BMP_CHIP_ID          = 0x58;
constexpr uint8_t BMP_CTRL_MEAS_X1X1   = 0x27;
constexpr uint8_t BMP_CONFIG_IIR2_0P5MS= 0x04;

constexpr uint8_t XBOX_BTN_LB   = 1u << 0;
constexpr uint8_t XBOX_BTN_RB   = 1u << 1;
constexpr uint8_t XBOX_BTN_NORTH= 1u << 2;
constexpr uint8_t XBOX_BTN_EAST = 1u << 3;
constexpr uint8_t XBOX_BTN_SOUTH= 1u << 4;
constexpr uint8_t XBOX_BTN_WEST = 1u << 5;

constexpr uint8_t ARM_BUTTONS = XBOX_BTN_LB | XBOX_BTN_RB | XBOX_BTN_NORTH;

constexpr float FC_ARMED_IDLE_FRACTION = 0.10f;
constexpr float FC_NORMAL_MOTOR_CEILING = 0.45f;
constexpr float FC_BOOST_MOTOR_CEILING = 0.65f;
constexpr float FC_MAX_VERTICAL_SPEED_MPS = 2.0f;
constexpr float FC_MAX_CLIMB_MPS = 1.5f;
constexpr float FC_MAX_DESCENT_MPS = 1.0f;

constexpr float MAX_ROLL_DEG = 35.0f;
constexpr float MAX_PITCH_DEG = 35.0f;
constexpr float MAX_YAW_RATE_DPS = 180.0f;

} // namespace cfg

// ============================================================================
// DATA TYPES (global scope)
// ============================================================================

struct CalVector3 { float x, y, z; };
struct CalMatrix3x3 { float m[3][3]; };

struct AccelCalibration { CalVector3 bias; CalMatrix3x3 scale; float rms_mps2; bool valid; };
struct MagCalibration { CalVector3 hard_iron; CalMatrix3x3 soft_iron; float radius_mean_uT; bool valid; };
struct BaroCalibration { float ground_pressure_pa, ref_var_pa2, meas_var_pa2; bool valid; };
struct GyroCalibration { CalVector3 bias_dps; float var_dps2[3]; bool valid; };

struct ImuSample {
  bool valid=false, fresh=false; uint32_t ts=0;
  int16_t accel_raw[3]={}, gyro_raw[3]={}, temp_raw=0;
  float accel[3]={}, gyro[3]={}, temp_c=0.0f;
  uint32_t fifo_count=0, overflows=0;
};
struct MagSample {
  bool valid=false, fresh=false; uint32_t ts=0;
  int16_t raw[3]={}; float cal[3]={}; uint8_t status=0;
};
struct BaroSample {
  bool valid=false, fresh=false; uint32_t ts=0;
  uint32_t raw_p=0, raw_t=0;
  float temp_c=0.0f, press_pa=0.0f;
  float height_m=0.0f, height_sigma_m=0.0f;
};
struct LidarSample {
  bool valid=false, fresh=false; uint32_t ts=0;
  float dist_m=0.0f, qual=0.0f; bool tilt_ok=false;
  float height_corr_m=0.0f;
};

struct SysHealth {
  uint32_t loops=0, missed=0, imu_err=0, mag_err=0, baro_err=0, lidar_err=0;
  uint32_t ctrl_loss=0, arm_att=0, disarms=0, failsafes=0, ser_drops=0;
  uint32_t reset_reason=0, uptime_us=0;
};

struct CtrlState {
  bool armed=false, disarmed=true, failsafe=false;
  bool ctrl_connected=false;
  uint32_t last_ctrl_ms=0, arm_ms=0, disarm_ms=0;
  float hover=0.50f, baro_zero=0.0f;
  bool baro_zeroed=false, lt_red=0.0f, boost=0.0f;
  float roll_ref=0, pitch_ref=0, yaw_ref=0;
  float alt_ref=0; bool alt_ref_init=false;
};

struct PilotCmd {
  bool valid=false; uint32_t ts=0; uint8_t btn=0;
  float roll=0, pitch=0, yaw=0, thr=0, rt=0, lt=0;
};

struct MotorOut {
  float m[4]={0,0,0,0}; bool armed=false, fault=false, fs=false;
};

// ============================================================================
// I²C TRANSACTION MANAGER
// ============================================================================

class I2cMgr {
public:
  struct Stats {
    uint32_t total=0, ok=0, fail=0, to=0, rec=0;
    uint32_t mpu=0, bmp=0, mag=0;
    uint32_t max_us=0, tot_us=0;
  } st;

  bool begin() {
    Wire.begin(cfg::SDA_PIN, cfg::SCL_PIN);
    Wire.setClock(cfg::I2C_HZ);
    Wire.setTimeOut(10);
    return true;
  }
  bool wr(uint8_t a, uint8_t r, uint8_t v) {
    uint32_t t=micros(); bool o=i2c_w(a,r,&v,1); rec(o,micros()-t); return o;
  }
  bool rd(uint8_t a, uint8_t r, uint8_t *o, size_t n) {
    uint32_t t=micros(); bool ok=i2c_r(a,r,o,n); rec(ok,micros()-t); return ok;
  }
  bool rd8(uint8_t a, uint8_t r, uint8_t &v){ return rd(r,a,&v,1); }
  bool present(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission()==0; }
  bool recover() {
    st.rec++;
    pinMode(cfg::SCL_PIN, OUTPUT); pinMode(cfg::SDA_PIN, OUTPUT);
    for(int i=0;i<9;i++){digitalWrite(cfg::SCL_PIN,LOW);delayMicroseconds(5);digitalWrite(cfg::SCL_PIN,HIGH);delayMicroseconds(5);}
    digitalWrite(cfg::SDA_PIN,LOW);delayMicroseconds(5);digitalWrite(cfg::SCL_PIN,LOW);delayMicroseconds(5);
    digitalWrite(cfg::SDA_PIN,HIGH);delayMicroseconds(5);
    Wire.begin(cfg::SDA_PIN,cfg::SCL_PIN); Wire.setClock(cfg::I2C_HZ); Wire.setTimeOut(10);
    return true;
  }
private:
  bool i2c_w(uint8_t a,uint8_t r,const uint8_t *v,size_t n){
    Wire.beginTransmission(a);
    if(Wire.write(r)!=1){Wire.endTransmission();return false;}
    for(size_t i=0;i<n;i++) if(Wire.write(v[i])!=1){Wire.endTransmission();return false;}
    return Wire.endTransmission()==0;
  }
  bool i2c_r(uint8_t a,uint8_t r,uint8_t *o,size_t n){
    Wire.beginTransmission(a);
    if(Wire.write(r)!=1){Wire.endTransmission();return false;}
    if(Wire.endTransmission(false)!=0)return false;
    int got=Wire.requestFrom((int)a,(int)n);
    if(got!=(int)n){while(Wire.available())(void)Wire.read();return false;}
    for(size_t i=0;i<n;i++) o[i]=static_cast<uint8_t>(Wire.read());
    return true;
  }
  void rec(bool ok,uint32_t us){ st.total++; if(ok)st.ok++;else st.fail++;
    if(us>st.max_us)st.max_us=us; st.tot_us+=us; }
};

// ============================================================================
// QUATERNION
// ============================================================================

struct Quat { float w,x,y,z;
  Quat():w(1),x(0),y(0),z(0){}
  void norm(){float n=sqrtf(w*w+x*x+y*y+z*z);
    if(n>0.001f){w/=n;x/=n;y/=n;z/=n;}else{w=1;x=0;y=0;z=0;}}
  void euler(float&r,float&p,float&y) const {
    float sr=2*(w*x+y*y), cr=1-2*(x*x+y*y); r=atan2f(sr,cr)*57.2957795f;
    float sp=2*(w*y-z*x);
    p=fabsf(sp)>=1.0f?copysignf(90.0f,sp):asinf(sp)*57.2957795f;
    float sy=2*(w*z+x*y), cy=1-2*(y*y+z*z); y=atan2f(sy,cy)*57.2957795f;
  }
};

// ============================================================================
// ATTITUDE ESTIMATOR
// ============================================================================

struct AttEst {
  Quat q; float gb[3]={0,0,0};
  float r=0,p=0,y=0, gr=0, gp=0, gy=0;
  bool initd=false; uint32_t last=0;
  float kr=0.5f, kp=0.5f, ky=0.1f;

  void init(const float*acc){
    r=atan2f(acc[1],acc[2])*57.2957795f;
    p=atan2f(-acc[0],sqrtf(acc[1]*acc[1]+acc[2]*acc[2]))*57.2957795f;
    float cr=cosf(r*0.0174532925f), sr=sinf(r*0.0174532925f);
    float cp=cosf(p*0.0174532925f), sp=sinf(p*0.0174532925f);
    q.w=cr*cp; q.x=sr*cp; q.y=cr*sp; q.z=-sr*sp; q.norm();
    r_deg=r; p_deg=p; y_deg=0; initd=true; last=esp_timer_get_time();
  }
  float r_deg=0,p_deg=0,y_deg=0;

  void update(const float*g,const float*acc,const float*mag,bool m_ok,float dt){
    if(!initd)return;
    float gx=g[0]-gb[0], gy=g[1]-gb[1], gz=g[2]-gb[2];
    float h=dt*0.5f;
    float qx=gx*0.0174532925f*h, qy=gy*0.0174532925f*h, qz=gz*0.0174532925f*h;
    float nq=sqrtf(qx*qx+qy*qy+qz*qz);
    Quat qd; qd.w=cosf(nq);
    if(nq>0.001f){float s=sinf(nq)/nq; qd.x=qx*s; qd.y=qy*s; qd.z=qz*s;}
    Quat qn;
    qn.w=q.w*qd.w-q.x*qd.x-q.y*qd.y-q.z*qd.z;
    qn.x=q.w*qd.x+q.x*qd.w+q.y*qd.z-q.z*qd.y;
    qn.y=q.w*qd.y-q.x*qd.z+q.y*qd.w+q.z*qd.x;
    qn.z=q.w*qd.z+q.x*qd.y-q.y*qd.x+q.z*qd.w;
    q=qn; q.norm();

    if(acc){
      float n=sqrtf(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
      if(n>0.5f&&n<2.0f){
        float ax=acc[0]/n,ay=acc[1]/n,az=acc[2]/n;
        float vx=2*(q.x*q.z-q.w*q.y), vy=2*(q.y*q.z+q.w*q.x), vz=q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z;
        float ex=ay*vz-az*vy, ey=az*vx-ax*vz, ez=ax*vy-ay*vx;
        gx+=kr*ex; gy+=kr*ey; gz+=kr*ez;
      }
    }
    if(m_ok&&mag){
      float mx=mag[0],my=mag[1],mz=mag[2];
      float emx=2*(q.w*q.x+q.y*q.z)*mx+2*(q.w*q.y-q.x*q.z)*my+(q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z)*mz;
      float emy=2*(q.x*q.y-q.w*q.z)*mx+(q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z)*my+2*(q.w*q.x+q.y*q.z)*mz;
      float hd=atan2f(-emy,emx)*57.2957795f;
      float err=hd-y_deg;
      while(err>180)err-=360; while(err<-180)err+=360;
      gz+=ky*err;
    }
    qx=gx*0.0174532925f*h; qy=gy*0.0174532925f*h; qz=gz*0.0174532925f*h;
    nq=sqrtf(qx*qx+qy*qy+qz*qz);
    qd.w=cosf(nq);
    if(nq>0.001f){float s=sinf(nq)/nq; qd.x=qx*s; qd.y=qy*s; qd.z=qz*s;}
    qn.w=q.w*qd.w-q.x*qd.x-q.y*qd.y-q.z*qd.z;
    qn.x=q.w*qd.x+q.x*qd.w+q.y*qd.z-q.z*qd.y;
    qn.y=q.w*qd.y-q.x*qd.z+q.y*qd.w+q.z*qd.x;
    qn.z=q.w*qd.z+q.x*qd.y-q.y*qd.x+q.z*qd.w;
    q=qn; q.norm();
    q.euler(r_deg,p_deg,y_deg);
    gr=gx; gp=gy; gy=gz; last=esp_timer_get_time();
  }
};

// ============================================================================
// VERTICAL KALMAN FILTER
// ============================================================================

struct VertKf {
  float h=0,v=0,ab=0;
  float p00=0.04f,p01=0,p10=0,p11=0.1f,p22=0.01f;
  float an=1.5f,bn=0.0025f,ln=0.01f;
  bool initd=false; uint32_t last=0;
  void init(float hh,float vv){h=hh;v=vv;ab=0;p00=0.04f;p01=0;p10=0;p11=0.1f;p22=0.01f;initd=true;last=esp_timer_get_time();}
  void pred(float acc,float dt){
    if(!initd)return;
    float a=acc-ab;
    h+=v*dt+0.5f*a*dt*dt; v+=a*dt;
    float d2=dt*dt,d3=d2*dt,d4=d3*dt,qq=an*an;
    p00+=dt*(p01+p10)+d2*p11+0.25f*d4*qq;
    p01+=dt*p11+0.5f*d3*qq; p10+=dt*p11+0.5f*d3*qq; p11+=d2*qq;
    p22+=0.0001f; last=esp_timer_get_time();}
  void baro(float hh,float sig){
    if(!initd||sig<=0)return; float r=sig*sig; if(r<1e-6f)r=1e-6f;
    float inn=hh-h, iv=p00+r, k0=p00/iv, k1=p10/iv;
    h+=k0*inn; v+=k1*inn;
    float z=1-k0; p00=z*z*p00+k0*k0*r; p01=z*(p01-k1*p00)+k0*k1*r; p10=p01;
    p11=p11-2*k1*p01+k1*k1*p00+k1*k1*r;}
  void lidar(float hh,float sig){
    if(!initd||sig<=0)return; float r=sig*sig; if(r<1e-6f)r=1e-6f;
    float inn=hh-h, iv=p00+r, k0=p00/iv, k1=p10/iv;
    h+=k0*inn; v+=k1*inn;
    float z=1-k0; p00=z*z*p00+k0*k0*r; p01=z*(p01-k1*p00)+k0*k1*r; p10=p01;
    p11=p11-2*k1*p01+k1*k1*p00+k1*k1*r;}
};

// ============================================================================
// PID CONTROLLER
// ============================================================================

struct Pid {
  float kp=0,ki=0,kd=0,il=0,ol=0, integ=0, prev=0, df=0, da=0.7f;
  bool initd=false;
  void reset(){integ=0;prev=0;initd=false;df=0;}
  float upd(float sp,float mv,float dt){
    if(!(dt>0)){reset();return 0;}
    float e=sp-mv, der=0;
    if(initd){der=(mv-prev)/dt; df=da*df+(1-da)*der; der=df;}
    float ci=integ+e*dt; ci=fmaxf(-il,fminf(il,ci));
    float raw=kp*e+ki*ci-kd*der;
    float o=fmaxf(-ol,fminf(ol,raw));
    if(raw==o||(raw>o&&e<0)||(raw<o&&e>0)) integ=ci;
    prev=mv; initd=true; return o;}
};

Pid pid_roll_a, pid_pitch_a, pid_roll_r, pid_pitch_r, pid_yaw_r, pid_alt, pid_vs;

// ============================================================================
// MOTOR MIXER
// ============================================================================

struct Mixer {
  float last[4]={0,0,0,0}; float mslew=10.0f;
  void mix(float coll,float r,float p,float y,float ceil,float idle,float dt,float*out){
    float c[4]={r-p-y,-r-p+y,-r+p-y,r+p+y};
    float lo=c[0],hi=c[0]; for(int i=1;i<4;i++){lo=fminf(lo,c[i]);hi=fmaxf(hi,c[i]);}
    float span=ceil-idle, sc=(hi-lo>span)?span/(hi-lo):1.0f;
    float al=fmaxf(idle,fminf(ceil-hi*sc,coll)); al=fminf(al,ceil-lo*sc);
    float md=mslew*dt;
    for(int i=0;i<4;i++){
      float rv=al+sc*c[i]; rv=fmaxf(idle,fminf(ceil,rv));
      float d=rv-last[i]; d=fmaxf(-md,fminf(md,d)); out[i]=fmaxf(idle,fminf(ceil,last[i]+d));
      last[i]=out[i];}}
  void reset(){for(int i=0;i<4;i++)last[i]=0;}
};

// ============================================================================
// XBOX INTERFACE
// ============================================================================

struct Xbox {
  HardwareSerial &s; bool conn=false; uint32_t last=0;
  uint8_t lbtn=0; bool armseq=false; uint32_t armh=0;
  float sdz=0.05f, tdz=0.02f;
  Xbox(HardwareSerial&ss, SysHealth& s):s(ss), st(s){}
  bool parse(const uint8_t*d,size_t n,PilotCmd&c){
    if(n<22)return false;
    int16_t rr=(int16_t)((d[2]<<8)|d[3]), pr=(int16_t)((d[4]<<8)|d[5]);
    int16_t yr=(int16_t)((d[6]<<8)|d[7]);
    uint16_t th=d[8]|(d[9]<<8), lt=d[10]|(d[11]<<8), rt=d[12]|(d[13]<<8);
    uint16_t bt=d[14]|(d[15]<<8);
    if(((d[0])|(d[1]<<8))==0)return false;
    c.roll=dz(rr/10000.0f,sdz); c.pitch=dz(-pr/10000.0f,sdz);
    c.yaw=dz(yr/10000.0f,sdz);
    c.thr=dz((th-5000)/5000.0f,sdz);
    c.rt=dz(rt/1000.0f,tdz); c.lt=dz(lt/1000.0f,tdz);
    c.btn=(uint8_t)bt; c.ts=millis(); c.valid=true;
    conn=true; last=c.ts; lbtn=c.btn; return true;}
  float dz(float v,float z){if(fabsf(v)<z)return 0; return v>0?(v-z)/(1-z):(v+z)/(1-z);}
  bool ok(){return conn&&(millis()-last<cfg::CONTROLLER_TIMEOUT_MS);}
  bool arm(PilotCmd&c,bool&armed,bool&ch){
    ch=false;
    if(!ok()){if(armed){armed=false;ch=true;st.disarms++;st.failsafes++;} armseq=false;armh=0;return false;}
    bool hold=(c.btn&cfg::ARM_BUTTONS)==cfg::ARM_BUTTONS;
    bool y=(c.btn&cfg::XBOX_BTN_NORTH)&&!(lbtn&cfg::XBOX_BTN_NORTH);
    if(!armed){if(hold){if(!armseq){armseq=true;armh=c.ts;}
      if(armseq&&y&&(c.ts-armh>=500)){armed=true;ch=true;st.arm_att++;armseq=false;}}
    }else{armseq=false;armh=0;
      if((c.btn&cfg::XBOX_BTN_SOUTH)&&!(lbtn&cfg::XBOX_BTN_SOUTH)){armed=false;ch=true;st.disarms++;}}
    armseq=false;armh=0; lbtn=c.btn; return armed;}
  SysHealth& st;
};

// ============================================================================
// CRC + TELEMETRY
// ============================================================================

uint16_t crc16(const uint8_t*d,size_t n){
  uint16_t c=0xFFFF;
  for(size_t i=0;i<n;i++){c^=d[i]<<8; for(uint8_t b=0;b<8;b++) c=(c&0x8000)?(c<<1)^0x1021:c<<1;}
  return c;}

struct Pkt {
  uint8_t b[cfg::PACKET_SIZE]={}; size_t l=0;
  void u8(uint8_t v){ if(l<cfg::PACKET_SIZE) b[l++]=v; }
  void u16(uint16_t v){ u8(v&0xFF); u8((v>>8)&0xFF); }
  void i16(int16_t v){ u16((uint16_t)v); }
  void u32(uint32_t v){ u8(v&0xFF); u8((v>>8)&0xFF); u8((v>>16)&0xFF); u8((v>>24)&0xFF); }
};

void mkPkt(Pkt&p,const ImuSample&imu,const MagSample&mag,const BaroSample&bar,
           const LidarSample&lid,const AttEst&att,const VertKf&vk,
           const MotorOut&mo,uint32_t seq,uint32_t ts,uint16_t late,const I2cMgr::Stats&st){
  uint8_t fl=0;
  fl|=imu.valid?1:0; fl|=bar.valid?2:0; fl|=mag.valid?4:0;
  fl|=bar.fresh?8:0; fl|=mag.fresh?16:0; fl|=lid.valid?32:0;
  fl|=lid.fresh?64:0; fl|=(late>250)?128:0;
  fl|=mo.armed?256:0; fl|=mo.fs?512:0; fl|=1024;
  p.l=0; p.u8(0xA5); p.u8(0x5A); p.u8(3); p.u8(fl&0xFF);
  p.u8((fl>>8)&0xFF); p.u32(seq); p.u32(ts); p.u16(late);
  for(int i=0;i<3;i++) p.i16(int16_t(imu.accel[i]*1000.0f));
  p.i16(int16_t(att.gr*100.0f)); p.i16(int16_t(att.gp*100.0f));
  p.i16(int16_t(att.gy*100.0f)); p.i16(int16_t(imu.temp_c*100.0f));
  for(int i=0;i<3;i++) p.i16(int16_t(mag.cal[i]*100.0f));
  p.u32(bar.raw_p); p.u32(bar.raw_t);
  p.i16(int16_t(bar.temp_c*100.0f)); p.u32(uint32_t(bar.press_pa));
  p.i16(int16_t(vk.h*1000.0f)); p.i16(int16_t(vk.v*1000.0f));
  p.i16(int16_t(lid.height_corr_m*1000.0f));
  p.i16(int16_t(att.r_deg*100.0f)); p.i16(int16_t(att.p_deg*100.0f));
  p.i16(int16_t(att.y_deg*100.0f));
  for(int i=0;i<4;i++) p.i16(int16_t(mo.m[i]*1000.0f));
  p.u16(0); p.u16(uint16_t(st.fail));
  while(p.l<cfg::PACKET_SIZE-2) p.u8(0);
  p.u16(crc16(p.b,p.l));
}

// ============================================================================
// CALIBRATION
// ============================================================================

AccelCalibration cal_a={}; MagCalibration cal_m={};
BaroCalibration cal_b={}; GyroCalibration cal_g={};
bool cal_loaded=false;

bool apply_accel(const float*r,float*o){
  if(!cal_a.valid)return false;
  for(int i=0;i<3;i++) o[i]=cal_a.scale.m[i][0]*(r[0]-cal_a.bias.x)+cal_a.scale.m[i][1]*(r[1]-cal_a.bias.y)+cal_a.scale.m[i][2]*(r[2]-cal_a.bias.z);
  return true;}
bool apply_gyro(const float*r,float*o){
  if(!cal_g.valid)return false;
  o[0]=r[0]-cal_g.bias_dps.x; o[1]=r[1]-cal_g.bias_dps.y; o[2]=r[2]-cal_g.bias_dps.z;
  return true;}
bool apply_mag(const float*r,float*o){
  if(!cal_m.valid)return false;
  float c[3]={r[0]-cal_m.hard_iron.x,r[1]-cal_m.hard_iron.y,r[2]-cal_m.hard_iron.z};
  for(int i=0;i<3;i++) o[i]=cal_m.soft_iron.m[i][0]*c[0]+cal_m.soft_iron.m[i][1]*c[1]+cal_m.soft_iron.m[i][2]*c[2];
  return true;}
bool load_cal(){
  cal_g.bias_dps={0,0,0}; cal_g.var_dps2[0]=0.01f;cal_g.var_dps2[1]=0.01f;cal_g.var_dps2[2]=0.01f; cal_g.valid=true;
  cal_a.bias={0,0,0}; cal_a.scale={{{1,0,0},{0,1,0},{0,0,1}}}; cal_a.rms_mps2=0.05f; cal_a.valid=true;
  cal_m.hard_iron={0,0,0}; cal_m.soft_iron={{{1,0,0},{0,1,0},{0,0,1}}}; cal_m.radius_mean_uT=40.0f; cal_m.valid=false;
  cal_b.ground_pressure_pa=101325.0f; cal_b.ref_var_pa2=0; cal_b.meas_var_pa2=10.0f; cal_b.valid=true;
  cal_loaded=true; return true;}

// ============================================================================
// LiDAR GEOMETRY
// ============================================================================
// H_COM = -[R r_s]_z - D [R d_s]_z
// Over horizontal ground: H_COM ≈ (D + 0.026658) × cos(roll) × cos(pitch)

float lidar_height(float d,float roll_deg,float pitch_deg){
  if(d<cfg::LIDAR_MIN_RANGE_M||d>cfg::LIDAR_MAX_RANGE_M) return NAN;
  float rr=roll_deg*0.0174532925f, pr=pitch_deg*0.0174532925f;
  if(fabsf(rr)>cfg::LIDAR_MAX_TILT_RAD||fabsf(pr)>cfg::LIDAR_MAX_TILT_RAD) return NAN;
  float cr=cosf(rr), cp=cosf(pr);
  float off=-cfg::LIDAR_OFFSET_Z_M;
  return (d+off)*cr*cp;
}

// ============================================================================
// GLOBAL STATE
// ============================================================================

I2cMgr i2c;
Adafruit_BMP280 *bmp_drv=nullptr;
uint8_t mpu_addr=0, bmp_addr=0, mag_addr=cfg::MAG_ADDRESS;
bool mpu_ok=false, bmp_ok=false, mag_ok=false, lidar_ok=false;

ImuSample imu; MagSample mag; BaroSample bar; LidarSample lid;
SysHealth st;
AttEst att; VertKf vkf;
Mixer mixer;
Xbox xbox_if(Serial, st);
CtrlState ctrl;
PilotCmd pilot;
MotorOut mout;

// ============================================================================
// SENSOR DRIVERS
// ============================================================================

bool mpu_init(){
  mpu_addr=0;
  if(i2c.present(cfg::MPU_PRIMARY_ADDRESS))mpu_addr=cfg::MPU_PRIMARY_ADDRESS;
  else if(i2c.present(cfg::MPU_ALTERNATE_ADDRESS))mpu_addr=cfg::MPU_ALTERNATE_ADDRESS;
  if(!mpu_addr)return false;
  uint8_t id=0; if(!i2c.rd8(mpu_addr,cfg::MPU_REG_WHO_AM_I,id)||id!=0x68)return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_PWR_MGMT_1,cfg::MPU_PWR_MGMT_1_RESET))return false; delay(100);
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_PWR_MGMT_1,cfg::MPU_PWR_MGMT_1_CLKSEL|cfg::MPU_PWR_MGMT_1_TEMP_DIS))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_PWR_MGMT_2,0x00))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_CONFIG,cfg::MPU_DLPF_188HZ))return false;
  if(!i2c.wr(mpu_addr,0x19,0x01))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_GYRO_CONFIG,cfg::MPU_GYRO_FS_500))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_ACCEL_CONFIG,cfg::MPU_ACCEL_FS_8G))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_FIFO_EN,cfg::MPU_FIFO_TEMP_EN|cfg::MPU_FIFO_XG_EN|cfg::MPU_FIFO_YG_EN|cfg::MPU_FIFO_ZG_EN|cfg::MPU_FIFO_ACCEL_EN))return false;
  uint8_t ic=0; if(!i2c.rd8(mpu_addr,cfg::MPU_REG_INT_PIN_CFG,ic))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_INT_PIN_CFG,ic|cfg::MPU_INT_PIN_CFG_BYPASS_EN))return false;
  uint8_t uc=0; if(!i2c.rd8(mpu_addr,cfg::MPU_REG_USER_CTRL,uc))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_USER_CTRL,uc|cfg::MPU_USER_CTRL_FIFO_EN))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_USER_CTRL,uc|cfg::MPU_USER_CTRL_FIFO_EN|cfg::MPU_USER_CTRL_FIFO_RESET))return false;
  if(!i2c.wr(mpu_addr,cfg::MPU_REG_INT_ENABLE,cfg::MPU_INT_ENABLE_RAW_RDY_EN))return false;
  mpu_ok=true; return true;}

bool mpu_read(ImuSample&s){
  if(!mpu_ok)return false;
  uint8_t fc[2]={};
  if(!i2c.rd(mpu_addr,cfg::MPU_REG_FIFO_COUNT_H,fc,2)){st.imu_err++;return false;}
  uint16_t fcnt=(uint16_t(fc[0])<<8)|fc[1]; s.fifo_count=fcnt/14;
  if(fcnt<14){ uint8_t d[14]={};
    if(!i2c.rd(mpu_addr,0x3B,d,14)){st.imu_err++;return false;}
    for(int i=0;i<3;i++){s.accel_raw[i]=(int16_t)((uint16_t(d[2*i])<<8)|d[2*i+1]);
      s.gyro_raw[i]=(int16_t)((uint16_t(d[8+2*i])<<8)|d[9+2*i]);}
    s.temp_raw=(int16_t)((uint16_t(d[6])<<8)|d[7]); s.overflows=0;}
  else{ uint8_t d[14]={};
    if(!i2c.rd(mpu_addr,cfg::MPU_REG_FIFO_R_W,d,14)){st.imu_err++;return false;}
    for(int i=0;i<3;i++){s.accel_raw[i]=(int16_t)((uint16_t(d[2*i])<<8)|d[2*i+1]);
      s.gyro_raw[i]=(int16_t)((uint16_t(d[8+2*i])<<8)|d[9+2*i]);}
    s.temp_raw=(int16_t)((uint16_t(d[6])<<8)|d[7]);
    s.overflows=(fcnt>1024)?1:0; if(s.overflows)st.imu_err++;}
  s.ts=esp_timer_get_time(); s.valid=true; s.fresh=true; return true;}

bool mag_init(){
  mag_addr=cfg::MAG_ADDRESS; if(!i2c.present(mag_addr))return false;
  uint8_t id=0; if(!i2c.rd8(mag_addr,cfg::QMC_REG_CHIP_ID,id)||id!=cfg::QMC_CHIP_ID)return false;
  if(!i2c.wr(mag_addr,cfg::QMC_REG_CTRL1,0x00))return false; delay(20);
  if(!i2c.wr(mag_addr,cfg::QMC_REG_XYZ_SIGN,cfg::QMC_XYZ_SIGN_VAL))return false;
  if(!i2c.wr(mag_addr,cfg::QMC_REG_CTRL2,cfg::QMC_CTRL2_8GAUSS))return false;
  if(!i2c.wr(mag_addr,cfg::QMC_REG_CTRL1,cfg::QMC_CTRL1_200HZ))return false;
  mag_ok=true; return true;}

bool mag_read(MagSample&s){
  if(!mag_ok)return false;
  uint8_t stt=0; if(!i2c.rd8(mag_addr,cfg::QMC_REG_STATUS,stt)){st.mag_err++;return false;}
  s.status=stt; if(stt&0x02){st.mag_err++;return false;} if(!(stt&0x01))return false;
  uint8_t d[6]={}; if(!i2c.rd(mag_addr,cfg::QMC_REG_DATA_XL,d,6)){st.mag_err++;return false;}
  for(int i=0;i<3;i++)s.raw[i]=(int16_t)((uint16_t(d[2*i+1])<<8)|d[2*i]);
  if((s.raw[0]==0&&s.raw[1]==0&&s.raw[2]==0)||(s.raw[0]==-1&&s.raw[1]==-1&&s.raw[2]==-1)
     ||s.raw[0]==INT16_MIN||s.raw[1]==INT16_MIN||s.raw[2]==INT16_MIN)return false;
  s.ts=esp_timer_get_time(); s.valid=true; s.fresh=true; return true;}

bool bmp_init(){
  bmp_addr=0;
  if(i2c.present(cfg::BMP_PRIMARY_ADDRESS))bmp_addr=cfg::BMP_PRIMARY_ADDRESS;
  else if(i2c.present(cfg::BMP_ALTERNATE_ADDRESS))bmp_addr=cfg::BMP_ALTERNATE_ADDRESS;
  if(!bmp_addr)return false;
  uint8_t id=0; if(!i2c.rd8(bmp_addr,cfg::BMP_REG_ID,id)||id!=cfg::BMP_CHIP_ID)return false;
  if(!i2c.wr(bmp_addr,cfg::BMP_REG_RESET,0xB6))return false; delay(10);
  if(!i2c.wr(bmp_addr,cfg::BMP_REG_CTRL_MEAS,cfg::BMP_CTRL_MEAS_X1X1))return false;
  if(!i2c.wr(bmp_addr,cfg::BMP_REG_CONFIG,cfg::BMP_CONFIG_IIR2_0P5MS))return false;
  bmp_ok=true; return true;}

bool bmp_read(BaroSample&s){
  if(!bmp_ok)return false;
  uint8_t r[6]={}; if(!i2c.rd(bmp_addr,cfg::BMP_REG_PRESSURE_MSB,r,6)){st.baro_err++;return false;}
  s.raw_p=((uint32_t(r[0])<<12)|((uint32_t(r[1])<<4)|(r[2]>>4)));
  s.raw_t=((uint32_t(r[3])<<12)|((uint32_t(r[4])<<4)|(r[5]>>4)));
  s.ts=esp_timer_get_time(); s.valid=true; s.fresh=true; return true;}

bool bmp_comp(const BaroSample&raw,float&tc,float&pp){
  if(!bmp_drv)return false;
  tc=bmp_drv->readTemperature(); pp=bmp_drv->readPressure();
  return isfinite(tc)&&isfinite(pp)&&tc>-40&&tc<85&&pp>30000&&pp<110000;}

bool lidar_read(LidarSample&s){
  if(!lidar_ok)return false;
  static HardwareSerial ls(1); static uint8_t buf[9]; static size_t bi=0;
  while(ls.available()){
    uint8_t b=ls.read();
    if(bi==0&&b!=0x59)continue; if(bi==1&&b!=0x59){bi=0;continue;}
    buf[bi++]=b;
    if(bi==9){ uint8_t sum=0; for(int i=0;i<8;i++)sum+=buf[i];
      if(sum!=buf[8]){bi=0;st.lidar_err++;continue;}
      uint16_t dc=(uint16_t(buf[1])<<8)|buf[0];
      uint16_t sv=(uint16_t(buf[3])<<8)|buf[2];
      s.dist_m=dc/100.0f; s.qual=sv/65535.0f; s.ts=esp_timer_get_time();
      s.valid=true; s.fresh=true; bi=0; return true;}}
  return false;}

// ============================================================================
// CONTROL
// ============================================================================

void ctrl_compute(const PilotCmd&cmd,const AttEst&att,const VertKf&vk,float dt,MotorOut&mo){
  mo.armed=ctrl.armed&&!ctrl.failsafe; mo.fault=false;
  if(!mo.armed){for(int i=0;i<4;i++)mo.m[i]=0;return;}
  float rs=cmd.roll*cfg::MAX_ROLL_DEG, ps=cmd.pitch*cfg::MAX_PITCH_DEG;
  float rrs=pid_roll_a.upd(rs,att.r_deg,dt);
  float prs=pid_pitch_a.upd(ps,att.p_deg,dt);
  float rtq=pid_roll_r.upd(rrs,att.gr,dt);
  float ptq=pid_pitch_r.upd(prs,att.gp,dt);
  float ytq=pid_yaw_r.upd(cmd.yaw*cfg::MAX_YAW_RATE_DPS,att.gy,dt);
  float vs=cmd.thr*cfg::FC_MAX_VERTICAL_SPEED_MPS;
  float tt=(cmd.rt-ctrl.lt_red)*1.0f;
  vs+=tt; vs=fmaxf(-cfg::FC_MAX_DESCENT_MPS,fminf(cfg::FC_MAX_CLIMB_MPS,vs));
  float ac=pid_vs.upd(vs,vk.v,dt);
  bool ah=fabsf(cmd.thr)<0.1f;
  if(ah&&ctrl.alt_ref_init){float ac2=pid_alt.upd(ctrl.alt_ref,vk.h,dt); ac+=ac2;}
  else{if(!ctrl.alt_ref_init){ctrl.alt_ref=vk.h;ctrl.alt_ref_init=true;} ctrl.alt_ref+=vs*dt;}
  float cr=cosf(att.r_deg*0.0174532925f), cp=cosf(att.p_deg*0.0174532925f);
  float tf=fmaxf(0.5f,cr*cp);
  float coll=(ctrl.hover+ac*0.05f)/tf;
  coll=fmaxf(cfg::FC_ARMED_IDLE_FRACTION,fminf(cfg::FC_BOOST_MOTOR_CEILING,coll));
  float ceil=cfg::FC_NORMAL_MOTOR_CEILING+(cfg::FC_BOOST_MOTOR_CEILING-cfg::FC_NORMAL_MOTOR_CEILING)*cmd.rt;
  mixer.mix(coll,rtq,ptq,ytq,ceil,cfg::FC_ARMED_IDLE_FRACTION,dt,mo.m);
  if(!imu.valid||!mag.valid||!bar.valid){mo.fs=true;ctrl.failsafe=true;}
  else{mo.fs=false;ctrl.failsafe=false;}}

void hw_init(){
  for(uint8_t p:cfg::MOTOR_PINS)pinMode(p,INPUT);
  Serial1.begin(115200,SERIAL_8N1,cfg::LIDAR_RX,cfg::LIDAR_TX); delay(100);
  i2c.begin(); load_cal();
  mpu_ok=mpu_init();
  if(mpu_ok){delay(30); bmp_ok=bmp_init();
    if(bmp_ok){bmp_drv=new Adafruit_BMP280(&Wire); bmp_drv->begin(bmp_addr);}}
  mag_ok=mag_init(); lidar_ok=true;
  st.reset_reason=esp_reset_reason();}

// ============================================================================
// Arduino entry points
// ============================================================================

void setup(){
  Serial.begin(cfg::SERIAL_BAUD);
  while(!Serial&&millis()<2000){}
  hw_init();
  st.uptime_us=esp_timer_get_time();
  Serial.printf("IQ FC v1.0: MPU=%d BMP=%d MAG=%d LiDAR=%d\n",
    mpu_ok,bmp_ok,mag_ok,lidar_ok);
  Serial.printf("I2C: %u Hz, 500 Hz ctrl, Motor: OFF\n",cfg::I2C_HZ);
}

void loop(){
  static uint32_t nd=0,seq=0; Pkt pkt;
  int64_t now=esp_timer_get_time();
  if(nd==0)nd=now+cfg::CONTROL_PERIOD_US;
  int64_t ud=nd-now;
  if(ud>300){delayMicroseconds((uint32_t)(ud-200));return;}
  if(ud>0)return;
  int64_t sa=esp_timer_get_time(); int64_t late=sa-nd;
  if(late>=(int64_t)cfg::CONTROL_PERIOD_US){
    uint64_t sk=(uint64_t)late/cfg::CONTROL_PERIOD_US;
    st.missed+=(uint32_t)(sk>UINT32_MAX?UINT32_MAX:sk);
    nd+=(int64_t)sk*cfg::CONTROL_PERIOD_US;}
  float dt=cfg::CONTROL_PERIOD_US/1000000.0f; nd+=cfg::CONTROL_PERIOD_US;
  st.loops++; st.uptime_us=sa;

  bool imu_ok=mpu_read(imu);
  bool mag_ok=mag_read(mag);
  bool bar_ok=bmp_read(bar);
  bool lid_ok=lidar_read(lid);
  if(!imu_ok)st.imu_err++; if(!mag_ok)st.mag_err++;
  if(!bar_ok)st.baro_err++; if(!lid_ok)st.lidar_err++;

  if(imu_ok){
    float ar[3]={imu.accel_raw[0]/4096.0f*9.80665f,imu.accel_raw[1]/4096.0f*9.80665f,imu.accel_raw[2]/4096.0f*9.80665f};
    float gr[3]={imu.gyro_raw[0]/65.5f,imu.gyro_raw[1]/65.5f,imu.gyro_raw[2]/65.5f};
    imu.temp_c=imu.temp_raw/340.0f+36.53f;
    apply_accel(ar,imu.accel); apply_gyro(gr,imu.gyro);
    imu.accel[2]-=9.80665f;}
  if(mag_ok){float mru[3]={mag.raw[0]*0.29f,mag.raw[1]*0.29f,mag.raw[2]*0.29f};
    float mcu[3]; apply_mag(mru,mcu); for(int i=0;i<3;i++)mag.cal[i]=mcu[i];}
  if(bar_ok){float tc,pp;
    if(bmp_comp(bar,tc,pp)){bar.temp_c=tc; bar.press_pa=pp;
      if(cal_b.valid){float r=pp/cal_b.ground_pressure_pa;
        float hm=44330.0f*(1.0f-powf(r,0.190294957f));
        if(!ctrl.baro_zeroed&&ctrl.disarmed){ctrl.baro_zero=hm;ctrl.baro_zeroed=true;}
        bar.height_m=hm-ctrl.baro_zero; bar.height_sigma_m=1.0f;}}}
  if(imu_ok){
    if(!att.initd)att.init(imu.accel);
    att.update(imu.gyro,imu.accel,mag_ok?mag.cal:nullptr,mag_ok&&cal_m.valid,dt);}
  if(bar_ok&&imu_ok){
    if(!vkf.initd)vkf.init(bar.height_m,0);
    vkf.pred(imu.accel[2],dt);
    vkf.baro(bar.height_m,bar.height_sigma_m);
    if(lid_ok){
      float ch=lidar_height(lid.dist_m,att.r_deg,att.p_deg);
      if(!isnan(ch)){lid.height_corr_m=ch;lid.tilt_ok=true;vkf.lidar(ch,0.1f);}
      else lid.tilt_ok=false;}}
  static uint8_t ub[22]; static size_t bi=0;
  while(Serial.available()){
    uint8_t b=Serial.read();
    if(bi==0&&b!=0xA5)continue; if(bi==1&&b!=0x5A){bi=0;continue;}
    ub[bi++]=b;
    if(bi==22){uint16_t rc=(uint16_t(ub[20])|(uint16_t(ub[21])<<8));
      uint16_t cc=crc16(ub,20);
      if(rc==cc)xbox_if.parse(ub,22,pilot); bi=0;}}
  bool armch=false; xbox_if.arm(pilot,ctrl.armed,armch);
  if(armch){
    if(ctrl.armed){Serial.println("ARMED");
      pid_roll_a.reset();pid_pitch_a.reset();pid_roll_r.reset();pid_pitch_r.reset();
      pid_yaw_r.reset();pid_alt.reset();pid_vs.reset();mixer.reset();
      att.initd=false;vkf.initd=false;ctrl.alt_ref_init=false;
      ctrl.lt_red=0;ctrl.boost=0;}
    else{Serial.println("DISARMED");ctrl.baro_zeroed=false;}}
  float lt_t=pilot.lt; ctrl.lt_red+=(lt_t-ctrl.lt_red)*0.2f; ctrl.lt_red=fmaxf(0,fminf(1,ctrl.lt_red));
  ctrl.boost+=(pilot.rt-ctrl.boost)*0.2f; ctrl.boost=fmaxf(0,fminf(1,ctrl.boost));
  MotorOut mo; ctrl_compute(pilot,att,vkf,dt,mo);
  uint16_t lu=(uint16_t)(late>65535?65535:(late>0?late:0));
  mkPkt(pkt,imu,mag,bar,lid,att,vkf,mo,seq++,(uint32_t)sa,lu,i2c.st);
  if(Serial.availableForWrite()>=(int)cfg::PACKET_SIZE)Serial.write(pkt.b,cfg::PACKET_SIZE);
  else st.ser_drops++;
}
