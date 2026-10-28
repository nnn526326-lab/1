/* ============================================================================
 * NEXEND FC — RASPBERRY PI PICO (RP2040) — SINGLE FILE — rev 6.5
 * rev 6.5: Auto-PID save-if-improved + SD audit trail
 * [NXR] rev 6.6: integrasi NEXEND Remote RP2040 (lihat daftar patch [NXR] di dokumen)
 * [NXL] rev 6.7: jalur radio diganti ke NEXEND_LINK_V1 (ACK payload) agar cocok dgn Remote RP2040 v1.0.0
 * [1R]  rev 6.8: 2x nRF24 -> 1x nRF24 (Radio B dihapus; CE=GP2 CSN=GP3)
 * [BTNLED] User button GP24 + WS2812 GP23 status colors and multi-click actions
 * ========================================================================== */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <LittleFS.h>
#include <SD.h>
#include <math.h>
#include <RF24.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <QMC5883LCompass.h>
#include <hardware/watchdog.h>

class Preferences {
public:
  bool begin(const char* ns, bool /*readOnly*/ = false){
    ns_ = ns;
    if (!fs_ready_){ fs_ready_ = LittleFS.begin(); if (!fs_ready_){ LittleFS.format(); fs_ready_ = LittleFS.begin(); } }
    return fs_ready_;
  }
  void end(){}
  size_t getBytesLength(const char* key){
    File f = LittleFS.open(path(key), "r");
    if (!f) return 0;
    size_t sz = f.size(); f.close(); return sz;
  }
  size_t getBytes(const char* key, void* buf, size_t maxLen){
    File f = LittleFS.open(path(key), "r");
    if (!f) return 0;
    size_t n = f.read((uint8_t*)buf, maxLen);
    f.close(); return n;
  }
  bool putBytes(const char* key, const void* buf, size_t len){
    File f = LittleFS.open(path(key), "w");
    if (!f) return false;
    size_t n = f.write((const uint8_t*)buf, len);
    f.close(); return n == len;
  }
  bool remove(const char* key){ return LittleFS.remove(path(key)); }
private:
  String ns_;
  bool fs_ready_ = false;
  String path(const char* key){ return "/" + ns_ + "_" + String(key) + ".bin"; }
};
using namespace fs;

struct PidGains; struct PidLimits; struct PidState; struct PidProfile;
struct ChemCurve; struct GeoPoint; struct ArmChecks;
struct ArbiterIn; struct ArbiterOut; struct FsmIn; struct RcInput;
struct EkfState; struct AltEst; struct PosEst;
enum BattChem : uint8_t;

#ifndef NEXEND_BENCH_MODE
  #define NEXEND_BENCH_MODE 0
#endif
#define NEXEND_HAVE_TOF        1
#define NEXEND_HAVE_ESC_TELEM  0
#define NEXEND_HAVE_RTC        0
#define NEXEND_HAVE_BUZZER     1
#define NEXEND_HAVE_SD         1
#define NEXEND_WDT_TIMEOUT_S   3

#if NEXEND_HAVE_TOF
  #include <VL53L0X.h>
  static VL53L0X g_tof;
  static bool g_tof_ready = false;   // [BOOTFIX] read_tof() hanya jalan kalau init() sukses
  static constexpr uint8_t  TOF_I2C_ADDR   = 0x29;
  static constexpr uint16_t TOF_MM_MIN     = 30;
  static constexpr uint16_t TOF_MM_MAX     = 2000;
  static constexpr uint32_t TOF_TIMING_US  = 33000;
  static constexpr uint32_t TOF_PERIOD_MS  = 33;
  static constexpr uint16_t TOF_TIMEOUT_MS = 60;
  static constexpr uint32_t TOF_READ_MIN_MS = 30;
#endif

// [1S] Jumlah sel baterai. Mengabaikan nilai 'cells' yang tersimpan di LittleFS (config lama berisi 2).
#ifndef NEXEND_BATT_CELLS
  #define NEXEND_BATT_CELLS 1
#endif
#define NEXEND_FW_MAJOR 1
#define NEXEND_FW_MINOR 5
#define NEXEND_FW_PATCH 5   // [1R] 1.5.2: 1 radio nRF24; [1S] 1.5.3: baterai 1S; [RA] 1.5.4: RTH maks via link; [USB] 1.5.5: telemetri USB
#define NEXEND_FW_BUILD 0
#define NEXEND_FW_PACKED ((uint32_t)((NEXEND_FW_MAJOR<<24)|(NEXEND_FW_MINOR<<16)|(NEXEND_FW_PATCH<<8)|NEXEND_FW_BUILD))

static constexpr int PIN_MOTOR[4]   = {10, 11, 12, 13};
static constexpr int PIN_NRF_CE     = 2,  PIN_NRF_CSN   = 3;   // [1R] 1 modul nRF24 (= bekas Radio A). Radio B (GP6/GP7) DIHAPUS -> GP6/GP7 bebas
static constexpr int PIN_GPS_RX     = 9,  PIN_GPS_TX    = 8;
static constexpr int PIN_I2C_SDA    = 20, PIN_I2C_SCL   = 21;
static constexpr int PIN_LED_STATUS = LED_BUILTIN;
static constexpr int PIN_BUZZER     = 15;
static constexpr int PIN_ESC_LED = 14;
static constexpr int PIN_USER_BUTTON = 24; // tombol aktif-LOW ke GND
static constexpr int PIN_STATUS_RGB = 23;  // WS2812 / NeoPixel
static Adafruit_NeoPixel g_status_rgb(1, PIN_STATUS_RGB, NEO_GRB + NEO_KHZ800);
static constexpr int ESC_LED_ON  = HIGH;
static constexpr int ESC_LED_OFF = LOW;
static constexpr uint32_t LED_FPS_DIV = 100;
static uint32_t g_boot_ms = 0;
static constexpr int PIN_SD_CS      = 22;
static constexpr uint8_t ADDR_MPU=0x68, ADDR_BMP=0x76, ADDR_QMC=0x0D, ADDR_INA=0x40;

#define NEX_PROTO_VER  0x04
#define NEX_SYNC0      0xA5
#define NEX_SYNC1      0x5A
#define NEX_CFG_MAGIC  0x4E584346u
#define NEX_STAT_MAGIC 0x4E585354u
#define NEX_CFG_VER    7

namespace lim {
  constexpr uint16_t RC_MIN=1000,RC_MID=1500,RC_MAX=2000,RC_DZ=30;
  constexpr uint16_t RC_SANITY_MIN=800, RC_SANITY_MAX=2200;
  constexpr uint32_t RC_AGE_MS=150, TELEM_AGE_MS=400;
  constexpr float    MAX_TILT=45.f*DEG_TO_RAD;
  constexpr float    INVERT_DEG=110.f;
  constexpr uint32_t INVERT_MS=400;
  constexpr float    CRASH_G=4.0f;
  constexpr uint32_t CRASH_MS=80;
  constexpr uint8_t  CELLS_DEFAULT=1;   // [1S] baterai 1S (sebelumnya 2)
  constexpr uint32_t LINK_PROMOTE_N=8;
  constexpr uint16_t LOITER_DZ_ENTER=40;
  constexpr uint16_t LOITER_DZ_EXIT =80;
  constexpr float    LOITER_MAX_CLIMB=2.5f;
  constexpr float    LOITER_MAX_DESC =2.0f;
  constexpr float    LOITER_VMAX     =3.0f;   // [LP] kecepatan horizontal maks dari stick roll/pitch saat loiter (m/s)
  constexpr float    LOITER_POS_KP   =0.5f;   // [LP] error posisi (m) -> kecepatan target (m/s)
  constexpr float    LOITER_VPOS_MAX =1.5f;   // [LP] batas kecepatan koreksi posisi (m/s)
  constexpr float    LOITER_LATCH_V  =0.5f;   // [LP] kunci titik hanya bila kecepatan GPS < nilai ini (m/s)
  constexpr float    LOITER_LOST_M   =15.f;   // [LP] error posisi lebih besar dari ini = kunci dianggap tidak valid, lepas
  constexpr uint32_t LAND_DISARM_MS  =3000;
  constexpr float    GYRO_BIAS_MAX_DELTA_DPS=500.0f;  // [AHRS] dulu 4.0: kalibrasi ulang ditolak bila bias lama sampah
  constexpr float    ACCEL_BIAS_MAX_DELTA_G =2.0f;    // [AHRS] dulu 0.10: kalibrasi ulang ditolak bila bias lama sampah
  constexpr float    MAG_SCALE_MIN=0.5f, MAG_SCALE_MAX=2.0f;
  constexpr float    HDOP_MAX=3.0f;
  constexpr uint8_t  SATS_MIN=6;
  constexpr uint32_t GPS_AGE_MS=1000;
  constexpr uint32_t BB_LOG_PERIOD_MS = 10;
  constexpr uint8_t  AUTO_STAB_STRENGTH_DEFAULT=0;
  constexpr uint8_t  AUTO_PID_MAX_ADJUST_PCT=20;
  constexpr uint16_t AUTO_PID_UPDATE_MS=5000;
  constexpr uint16_t AUTO_PID_SMOOTHING_MS=2000;
  constexpr float    YAW_ANGLE_KP=2.5f;
  constexpr float    YAW_RATE_LIMIT=3.0f;
  constexpr float    RTH_CLIMB_EXTRA=5.f;
  constexpr float    RTH_CLIMB_DEFAULT=20.f;
  constexpr uint16_t RTH_MAX_ALT_MIN=10;       // [RA] batas valid ketinggian RTH maksimal (m); harus sama dgn remote & APK
  constexpr uint16_t RTH_MAX_ALT_MAX=120;
  constexpr uint16_t RTH_MAX_ALT_DEFAULT=60;   // dipakai bila config belum pernah diatur (nilai 0 / di luar rentang)
  constexpr float    RTH_SPEED_DEFAULT=4.f;
  constexpr uint16_t RTH_GATE_DISPLAY_MS=2000;
  constexpr float    IDLE_SPIN=0.06f;
}

enum Health : uint8_t { H_OK=0,H_WARN,H_INVALID,H_FAILED };
enum SubsysId : uint8_t {
  S_IMU=0,S_GYRO,S_ACCEL,S_BARO,S_MAG,S_GPS,S_BATT,S_TOF,
  S_RADIO_A,S_RSV_B,S_ESC,S_MOTOR,S_BLACKBOX,S_CFG,S_FW,S_SD,S_COUNT
};
enum FlightState : uint8_t {
  FS_BOOT=0,FS_SYS_CHECK,FS_GPS_SEARCH,FS_READY,FS_ARMED_IDLE,FS_TAKEOFF,
  FS_FLIGHT,FS_LANDING,FS_LANDED,FS_POST_LANDING_LOCK,FS_DISARMED,
  FS_FAILSAFE=100,FS_RTH,FS_INVERTED,FS_MOTOR_FAULT,FS_CRITICAL_SENSOR,FS_CRASH,
  FS_FAILSAFE_DESCEND=120
};
enum ArmState : uint8_t {
  AS_DISARMED=0, AS_PREARM, AS_GESTURE, AS_ARMED_IDLE, AS_IDLE_SPIN,
  AS_TAKEOFF_READY, AS_FLIGHT
};
enum SysStatus : uint8_t { SS_NORMAL=0,SS_WARNING,SS_FAULT,SS_CRITICAL,SS_LOCKED };
enum WarnLevel : uint8_t { WL_INFO=0,WL_WARNING,WL_CRITICAL };
enum SafetyCause : uint8_t {
  SC_NONE=0,SC_DISARM,SC_CRIT_SENSOR,SC_MOTOR_FAULT,SC_INVERTED,SC_CRASH,
  SC_FAILSAFE,SC_LOW_BATT,SC_LANDING_PROT,SC_RTH,SC_NORMAL
};
enum LinkState : uint8_t { LS_INIT=0,LS_VALIDATING,LS_ACTIVE,LS_LOST };
enum FlightMode : uint8_t {
  FM_STABILIZE=0,FM_ALTHOLD,FM_LOITER,FM_RTH,FM_FAILSAFE_DESCEND
};
enum BattChem : uint8_t { BC_LIPO=0,BC_LIION,BC_LIFEPO4,BC_NIMH,BC_CUSTOM };
enum PktType : uint8_t {
  PKT_RC=0x10,PKT_TELEM_A=0x20,PKT_TELEM_B=0x21,PKT_TELEM_C=0x22,PKT_TELEM_D=0x23,
  PKT_TELEM_E=0x24,
  PKT_EVENT=0x30,PKT_CMD=0x40,PKT_ACK=0x41,PKT_BB_REQ=0x50,PKT_BB_DAT=0x51,
};
enum RcFlags : uint8_t {
  RC_F_ARM_REQ=1<<0,RC_F_DISARM=1<<1,RC_F_RTH=1<<2,RC_F_CALIB=1<<3,
  RC_F_BTN_UP=1<<4,RC_F_BTN_OK=1<<5,RC_F_BTN_DOWN=1<<6,RC_F_EMERG=1<<7,
};
enum NexEvent : uint8_t {
  EV_BOOT=1,EV_SENSOR_FAIL,EV_GPS_FIX,EV_HOME_SET,EV_RADIO_UP,EV_RADIO_LOST,
  EV_FAILSAFE,EV_RTH_START,EV_RTH_DONE,EV_BATT_WARN,EV_BATT_CRIT,EV_INVERTED,
  EV_MOTOR_FAULT,EV_MOTOR_CUT,EV_LANDING,EV_CRASH,EV_TAKEOFF_ABORT,
  EV_POST_LAND_LOCK,EV_DISARM_MANUAL,EV_ARM,EV_CFG_SAVE,EV_CALIB,EV_TOF,
  EV_ESC_FAULT,EV_AUTOTUNE_START,EV_AUTOTUNE_DONE,EV_AUTOTUNE_ABORT,
  EV_ADAPT_PID,EV_RTH_BLOCK,EV_SELFTEST,EV_BATT_SAG,EV_RTH_AUTOLAND,
  EV_RTH_AUTODISARM,EV_LOITER_ENTER,EV_LOITER_EXIT,EV_I2C_RECOVER,
  EV_FW_OK,EV_FW_BAD,EV_FAILSAFE_DESCEND,EV_LOWBATT_RTH,EV_RADIO_LOST_SAFE,
  EV_HOME_RESET,EV_SENSOR_NAN,EV_MOTOR_CUT_NAN,EV_SD_MOUNT,EV_SD_FAIL,
  EV_ADAPT_SAVED
};
enum NexCmd : uint8_t {
  CMD_NONE=0,CMD_CALIB_GYRO,CMD_CALIB_ACCEL,CMD_CALIB_MAG,CMD_SET_PID,
  CMD_SET_PROFILE,CMD_AUTOTUNE_START,CMD_AUTOTUNE_ABORT,CMD_SAVE_CFG,
  CMD_LOAD_CFG,CMD_ERASE_BLACKBOX,CMD_ADAPTIVE_TOGGLE,CMD_FACTORY_RESET,
  CMD_CLEAR_LOCKS,CMD_BB_REQ,CMD_CALIB_INA,CMD_SET_BATT_CAP,
  CMD_CALIB_TOF,CMD_CALIB_ABORT,CMD_RESET_HOME,CMD_SET_CHEM,
  CMD_SET_STABILIZER,CMD_SET_AUTO_STAB,CMD_SET_AUTO_STRENGTH,
  CMD_PID_RESET,CMD_PID_SAVE_PROFILE
};
enum TuneAxes : uint8_t { TA_ROLL=0,TA_PITCH=1,TA_YAW=2,TA_ALL=3 };
enum CalibState : uint8_t { CS_IDLE=0,CS_REQ,CS_IN_PROGRESS,CS_SUCCESS,CS_FAILED,CS_ABORTED };
enum WarnId : uint16_t {
  W_NONE=0,W_IMU_FAIL,W_ACCEL_FAIL,W_GYRO_FAIL,W_BARO_FAIL,W_MAG_FAIL,
  W_GPS_FAIL,W_GPS_LOW_SAT,W_BATT_LOW,W_BATT_CRIT,W_BATT_SAG,W_TOF_FAIL,
  W_RADIO_A_LOST,W_RSV_RADIO_B,W_LINK_LOST,W_INVERTED,W_CRASH,
  W_MOTOR_FAULT,W_TAKEOFF_ABORT,W_POST_LAND_LOCK,W_CRIT_SENSOR,
  W_RTH_BLOCKED_BATT,W_RTH_BLOCKED_GPS,W_RTH_BLOCKED_HOME,
  W_AUTOTUNE_RUNNING,W_AUTOTUNE_FAIL,W_ESC_TEMP_HIGH,W_ESC_ERR,
  W_CFG_INVALID,W_CAL_REQUIRED,W_INA_UNCAL,W_FW_INTEGRITY,W_I2C_RECOVER,
  W_CAL_BAD_DELTA,W_LOWBATT_RTH,W_SENSOR_NAN,
  W_RTH_BLOCKED_MANUAL,W_STAB_OFF,W_AUTO_STAB_DISABLED,W_SD_MISSING
};
enum ArmReject : uint8_t {
  AR_OK=0,AR_IMU,AR_ACCEL,AR_GYRO,AR_BARO,AR_MAG,AR_GPS,AR_TOF,AR_BATT,
  AR_RADIO_A,AR_RSV_RADIO_B,AR_ESC,AR_MOTOR,AR_RC,AR_INVERTED,AR_CRASH_LOCK,
  AR_MOTOR_LOCK,AR_POST_LAND_LOCK,AR_NO_SYNC,AR_CFG,AR_THROTTLE,AR_FW,
  AR_GESTURE_TIMEOUT, // [NXR] 22: PREARM/GESTURE tidak selesai dalam 3 s
};
enum CalibResult : uint8_t {
  CAL_OK=0,CAL_REJECT_RANGE,CAL_REJECT_NOISE,CAL_REJECT_TIMEOUT,
  CAL_REJECT_NOT_LEVEL,CAL_REJECT_MAG_BAD,CAL_REJECT_ABORT,
  CAL_REJECT_DELTA_TOO_BIG,CAL_REJECT_BUSY,CAL_REJECT_ARMED,
};

#pragma pack(push,1)
struct PktHeader { uint8_t s0,s1,ver,type,seq,flags; };
struct PktRc { PktHeader h; uint16_t throttle,roll,pitch,yaw; uint8_t buttons,arm_req; uint16_t remote_mv,crc; };
struct PktTelemA { PktHeader h; uint8_t fsm,arm,sys,warn; uint16_t batt_mv; int16_t batt_ca; int16_t alt_dm; uint16_t home_dist_m; uint16_t gs_cms; uint8_t sats,fix; uint16_t crc; };
struct PktTelemB { PktHeader h; int16_t heading_cdeg; uint8_t rA_q,rB_q,rA_st,rB_st; uint8_t motor_mask,tof_st; uint16_t tof_mm; uint8_t mode,rth_gate; uint16_t crc; };
struct PktTelemC { PktHeader h; uint8_t health[16]; uint8_t warn_top_lvl; uint16_t warn_top; uint8_t calib_ok,calib_state,calib_progress,fw_flags; uint16_t crc; };
struct PktTelemD { PktHeader h; int32_t lat_e7,lon_e7; uint32_t fw_packed; uint16_t mah_used,wh_centi; uint8_t pct,batt_flags; uint8_t motor_pct[4]; int16_t cell_mv_min; uint16_t crc; };
struct PktTelemE {
  PktHeader h;
  uint8_t stab_on, auto_stab_on, auto_strength, active_profile;
  int16_t roll_kp_x100,  roll_ki_x100,  roll_kd_x100;
  int16_t pitch_kp_x100, pitch_ki_x100, pitch_kd_x100;
  int16_t yaw_kp_x100,   yaw_ki_x100,   yaw_kd_x100;
  int16_t adaptive_adjust_pct;
  uint16_t crc;
};
struct PktEvent { PktHeader h; uint32_t t_ms; uint8_t sev,ev; uint16_t arg,crc; };
struct PktCmd   { PktHeader h; uint8_t cmd,idx; int32_t arg; uint16_t crc; };
struct PktAck   { PktHeader h; uint8_t acked,result; uint16_t crc; };
struct PktBbReq { PktHeader h; uint32_t off; uint16_t len,crc; };
struct PktBbDat { PktHeader h; uint32_t off; uint16_t len; uint8_t data[20]; uint16_t crc; };
#pragma pack(pop)

static constexpr uint8_t NEX_PKT_SIZE = 32;

static inline uint16_t nex_crc16(const uint8_t* d,size_t n){
  uint16_t c=0xFFFF;
  for (size_t i=0;i<n;i++){ c^=(uint16_t)d[i]<<8;
    for (int b=0;b<8;b++) c=(c&0x8000)?(c<<1)^0x1021:(c<<1); }
  return c;
}
template<typename T> static inline void nex_seal(T& p){ p.crc = nex_crc16((const uint8_t*)&p, sizeof(T)-sizeof(p.crc)); }
template<typename T> static inline bool nex_verify(const T& p){
  if (p.h.s0!=NEX_SYNC0 || p.h.s1!=NEX_SYNC1) return false;
  if (p.h.ver!=NEX_PROTO_VER) return false;
  return nex_crc16((const uint8_t*)&p, sizeof(T)-sizeof(p.crc))==p.crc;
}
static inline bool nex_check_header(const uint8_t* buf, uint8_t type) {
  if (buf[0]!=NEX_SYNC0 || buf[1]!=NEX_SYNC1) return false;
  if (buf[2]!=NEX_PROTO_VER) return false;
  if (buf[3]!=type) return false;
  return true;
}
static inline float wrap_pi(float a){ while(a>(float)M_PI)a-=2.f*(float)M_PI; while(a<-(float)M_PI)a+=2.f*(float)M_PI; return a; }
static inline float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline bool is_finite_arr(const float* v, int n){
  for (int i=0;i<n;i++) if (!isfinite(v[i])) return false;
  return true;
}

// ============ STORAGE ============
static bool g_sd_ok  = false;
static bool g_lfs_ok = false;

static bool storage_begin_all(){
  if (!LittleFS.begin()){ LittleFS.format(); g_lfs_ok = LittleFS.begin(); }
  else g_lfs_ok = true;

#if NEXEND_HAVE_SD
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  delay(20);
  g_sd_ok = false;
  if (SD.begin(PIN_SD_CS, 8000000, SPI)) {
    File tf = SD.open("/.sdtest", "w");
    bool good = false;
    if (tf){
      tf.print("ok"); tf.close();
      File rf = SD.open("/.sdtest", "r");
      if (rf){
        char b[4]={0};
        int n = rf.read((uint8_t*)b, 2);
        rf.close();
        good = (n==2 && b[0]=='o' && b[1]=='k');
      }
      SD.remove("/.sdtest");
    }
    g_sd_ok = good;
  }
#endif
  return g_lfs_ok || g_sd_ok;
}

// ============ PID ============
struct PidGains { float kp=0,ki=0,kd=0,kff=0; };
struct PidLimits { float i_max=0.3f,out_max=1.0f,d_lpf_hz=80.0f,slew_per_s=30.0f; };
struct PidState  { float i=0,d_filt=0,last_out=0,last_err=0; };
struct PidProfile {
  PidGains rate_roll, rate_pitch, rate_yaw;
  PidGains ang_roll,  ang_pitch;
  PidGains alt_pos,   alt_vel;
  PidGains pos_n,     pos_e;
  PidGains vel_n,     vel_e;
  uint8_t  stabilizer_enabled;
  uint8_t  auto_stab_enabled;
  uint8_t  auto_stab_strength;
  uint8_t  _reserved;
};
static float pid_step(const PidGains& g, PidState& s, float sp,float meas,float rate_meas,float dt,const PidLimits& lim){
  if (!isfinite(sp)||!isfinite(meas)||!isfinite(rate_meas)||!isfinite(dt)){ s.i=0;s.d_filt=0;s.last_out=0;s.last_err=0; return 0.f; }
  if (dt<=0.f || dt>0.05f){ s.last_out=0.f; return 0.f; }
  float err = sp - meas;
  float p = g.kp * err;
  float i_new = s.i + g.ki * err * dt;
  if (i_new >  lim.i_max) i_new =  lim.i_max;
  if (i_new < -lim.i_max) i_new = -lim.i_max;
  float d_raw = -g.kd * rate_meas;
  float rc = 1.f/(2.f*(float)M_PI*lim.d_lpf_hz);
  float alpha = dt/(rc+dt);
  s.d_filt += alpha*(d_raw - s.d_filt);
  float ff = g.kff * sp;
  float out = p + i_new + s.d_filt + ff;
  if (!isfinite(out)){ s.i=0;s.d_filt=0;s.last_out=0; return 0.f; }
  if (out >  lim.out_max) out =  lim.out_max;
  if (out < -lim.out_max) out = -lim.out_max;
  bool sat = (out>=lim.out_max && err>0.f)||(out<=-lim.out_max && err<0.f);
  if (!sat) s.i = i_new;
  float max_step = lim.slew_per_s * dt;
  float delta = out - s.last_out;
  if (delta >  max_step) delta =  max_step;
  if (delta < -max_step) delta = -max_step;
  out = s.last_out + delta;
  if (!isfinite(out)){ s.last_out=0.f; return 0.f; }
  s.last_out = out; s.last_err = err;
  return out;
}

// ============ CONFIG ============
#pragma pack(push,1)
struct NexConfig {
  uint32_t magic; uint16_t version; uint16_t rth_max_alt_m; /* [RA] eks 'reserved'; 0 = belum diatur */
  PidProfile pid; PidProfile pid_profiles[3];
  uint8_t  active_profile;
  float    gyro_bias[3]; float accel_bias[3];
  float    mag_offset[3]; float mag_scale[3];
  uint16_t rc_min, rc_max; uint16_t rc_center[4];
  uint8_t  cells;
  float    low_cell_v, crit_cell_v;
  float    rth_climb_alt, rth_speed;
  uint8_t  failsafe_action;
  uint8_t  adaptive_enabled;
  uint16_t batt_capacity_mah;
  uint16_t ina_cal_value;
  uint8_t  ina_cal_ok;
  uint8_t  batt_chem;
  float    declination_rad;
  uint16_t thr_center_us;
  uint8_t  loiter_enabled;
  float    hdop_max;
  float    failsafe_desc_rate;
  uint8_t  stabilizer_enabled;
  uint8_t  auto_stab_enabled;
  uint8_t  auto_stab_strength;
  uint8_t  auto_pid_max_adjust;
  uint16_t auto_pid_update_ms;
  uint16_t auto_pid_smoothing_ms;
  uint16_t crc;
};
#pragma pack(pop)

struct FlightStats {
  uint32_t total_flight_s; uint16_t flight_count;
  float    max_alt_m, max_dist_m, max_speed_mps, min_batt_v, max_current_a;
  uint16_t rth_count, failsafe_count, motor_fault_count, crash_count;
  uint16_t landing_count, takeoff_abort_count;
  uint32_t last_flight_s; uint32_t magic; uint16_t crc;
};

static Preferences g_nvs_cfg;
static Preferences g_nvs_stats;

class ConfigStore {
public:
  bool begin(){
    if (!g_nvs_cfg.begin("nexend", false)) return false;
    size_t sz = g_nvs_cfg.getBytesLength("cfg");
    if (sz != sizeof(NexConfig)) return false;
    g_nvs_cfg.getBytes("cfg", &c_, sizeof(c_));
    if (c_.magic != NEX_CFG_MAGIC) return false;
    if (c_.version != NEX_CFG_VER) return false;
    if (c_.crc != crc(c_)) return false;
    valid_ = true; return true;
  }
  bool save(const NexConfig& in){
    NexConfig c = in;
    c.magic = NEX_CFG_MAGIC; c.version = NEX_CFG_VER; c.crc = crc(c);
    if (!g_nvs_cfg.putBytes("cfg", &c, sizeof(c))) return false;
    c_ = c; valid_ = true; dirty_ = false; return true;
  }
  void mark_dirty(){ dirty_ = true; }
  bool dirty() const { return dirty_; }
  bool flush_if_dirty(){ if (!dirty_||!valid_) return true; return save(c_); }
  void make_defaults(){
    memset(&c_, 0, sizeof(c_));
    c_.cells = lim::CELLS_DEFAULT;
    c_.batt_chem = BC_LIPO;
    c_.low_cell_v = 3.4f; c_.crit_cell_v = 3.2f;
    c_.batt_capacity_mah = 3000;
    c_.failsafe_action = 0;
    c_.adaptive_enabled = 0;
    c_.thr_center_us = 1500; c_.loiter_enabled = 1; c_.declination_rad = 0.f;
    c_.hdop_max = lim::HDOP_MAX; c_.failsafe_desc_rate = 1.0f;
    c_.rth_climb_alt = lim::RTH_CLIMB_DEFAULT;
    c_.rth_max_alt_m = lim::RTH_MAX_ALT_DEFAULT;
    c_.rth_speed     = lim::RTH_SPEED_DEFAULT;
    c_.pid.rate_roll  = {2.4f,0.06f,0.020f,0.f};
    c_.pid.rate_pitch = {2.4f,0.06f,0.020f,0.f};
    c_.pid.rate_yaw   = {3.2f,0.03f,0.000f,0.f};
    c_.pid.ang_roll   = {7.0f,0.00f,0.000f,0.f};
    c_.pid.ang_pitch  = {7.0f,0.00f,0.000f,0.f};
    c_.pid.alt_pos    = {1.4f,0.06f,0.000f,0.f};
    c_.pid.alt_vel    = {2.4f,0.12f,0.050f,0.f};
    c_.pid.pos_n      = {0.9f,0.02f,0.000f,0.f};
    c_.pid.pos_e      = {0.9f,0.02f,0.000f,0.f};
    c_.pid.stabilizer_enabled  = 1;
    c_.pid.auto_stab_enabled   = 0;
    c_.pid.auto_stab_strength  = 0;
    for (int i=0;i<3;i++) c_.pid_profiles[i] = c_.pid;
    c_.active_profile = 1; c_.rc_min = 1000; c_.rc_max = 2000;
    c_.stabilizer_enabled     = 1;
    c_.auto_stab_enabled      = 0;
    c_.auto_stab_strength     = lim::AUTO_STAB_STRENGTH_DEFAULT;
    c_.auto_pid_max_adjust    = lim::AUTO_PID_MAX_ADJUST_PCT;
    c_.auto_pid_update_ms     = lim::AUTO_PID_UPDATE_MS;
    c_.auto_pid_smoothing_ms  = lim::AUTO_PID_SMOOTHING_MS;
  }
  bool factory_reset(){ g_nvs_cfg.remove("cfg"); valid_=false; dirty_=true; return true; }
  bool valid() const { return valid_; }
  NexConfig& c(){ return c_; }
  const NexConfig& c() const { return c_; }
private:
  NexConfig c_{}; bool valid_=false; bool dirty_=false;
  static uint16_t crc(const NexConfig& x){ return nex_crc16((const uint8_t*)&x, sizeof(NexConfig)-sizeof(x.crc)); }
};
static ConfigStore g_cfg;
// [RA] Ketinggian RTH maksimal yang BENAR-BENAR dipakai (m). Nilai tak valid/0 -> default.
static uint16_t rth_max_alt_effective(){
  if (g_cfg.valid()){
    uint16_t v=g_cfg.c().rth_max_alt_m;
    if (v>=lim::RTH_MAX_ALT_MIN && v<=lim::RTH_MAX_ALT_MAX) return v;
  }
  return lim::RTH_MAX_ALT_DEFAULT;
}

class StatsStore {
public:
  bool begin(){
    if (!g_nvs_stats.begin("nexstats", false)) return false;
    size_t sz = g_nvs_stats.getBytesLength("s");
    if (sz == sizeof(FlightStats)){
      g_nvs_stats.getBytes("s", &s_, sizeof(s_));
      if (s_.magic != NEX_STAT_MAGIC || s_.crc != crc(s_)){ memset(&s_,0,sizeof(s_)); s_.magic=NEX_STAT_MAGIC; }
    } else { memset(&s_,0,sizeof(s_)); s_.magic=NEX_STAT_MAGIC; }
    if (s_.min_batt_v == 0) s_.min_batt_v = 99.f;
    return true;
  }
  void save(){ s_.crc = crc(s_); g_nvs_stats.putBytes("s",&s_,sizeof(s_)); }
  FlightStats& s(){ return s_; }
  void reset(){ memset(&s_,0,sizeof(s_)); s_.magic=NEX_STAT_MAGIC; s_.min_batt_v=99.f; save(); }
  void on_flight_start(){ in_flight_=true; s_.flight_count++; save(); }
  void on_flight_end(uint32_t dur_ms){
    if (!in_flight_) return;
    s_.last_flight_s = dur_ms/1000; s_.total_flight_s += s_.last_flight_s;
    in_flight_ = false; save();
  }
  void on_rth(){ s_.rth_count++; save(); }
  void on_failsafe(){ s_.failsafe_count++; save(); }
  void on_motor_fault(){ s_.motor_fault_count++; save(); }
  void on_crash(){ s_.crash_count++; save(); }
  void on_landing(){ s_.landing_count++; save(); }
  void on_takeoff_abort(){ s_.takeoff_abort_count++; save(); }
  void live(float alt,float dist,float spd,float v,float a){
    if (alt>s_.max_alt_m) s_.max_alt_m=alt;
    if (dist>s_.max_dist_m) s_.max_dist_m=dist;
    if (spd>s_.max_speed_mps) s_.max_speed_mps=spd;
    if (v>0 && v<s_.min_batt_v) s_.min_batt_v=v;
    if (a>s_.max_current_a) s_.max_current_a=a;
  }
private:
  FlightStats s_{}; bool in_flight_=false;
  static uint16_t crc(const FlightStats& x){ return nex_crc16((const uint8_t*)&x, sizeof(FlightStats)-sizeof(x.crc)); }
};
static StatsStore g_stats;

// ============ HEALTH ============
struct SensorSlot { Health st=H_INVALID; uint32_t last_ok=0, timeout=500; uint8_t conf=0; uint16_t faults=0; };
class HealthReg {
public:
  void begin(){
    uint32_t t = (uint32_t)millis();
    for (int i=0;i<S_COUNT;i++){ g_[i].st=H_INVALID; g_[i].last_ok=t; g_[i].conf=0; }
    g_[S_IMU].timeout=100; g_[S_GYRO].timeout=100; g_[S_ACCEL].timeout=100;
    g_[S_BARO].timeout=300; g_[S_MAG].timeout=400;
    g_[S_GPS].timeout=2500; g_[S_BATT].timeout=600;
    g_[S_TOF].timeout=200; g_[S_RADIO_A].timeout=lim::RC_AGE_MS;
    g_[S_ESC].timeout=600;
    g_[S_SD].timeout=60000;
    g_[S_RSV_B].st=H_OK; g_[S_RSV_B].conf=100; // [1R] slot cadangan (eks Radio B): tidak dipakai, tidak boleh memicu fault
  }
  void ok(SubsysId id,uint32_t now,uint8_t c=100){
    auto& s=g_[id]; s.last_ok=now; s.conf=c;
    if (s.st==H_FAILED) return;
    s.st = (c>=80)?H_OK:(c>=40?H_WARN:H_INVALID);
  }
  void bad(SubsysId id,uint32_t now,bool hard){
    auto& s=g_[id]; s.faults++;
    if (hard){ s.st=H_FAILED; s.conf=0; }
    else { s.conf=(s.conf>30)?s.conf-30:0; s.st=(s.conf==0)?H_INVALID:H_WARN; }
    (void)now;
  }
  void tick(uint32_t now){
    for (int i=0;i<S_COUNT;i++){
      if (g_[i].st==H_FAILED) continue;
      if (i==S_SD || i==S_RSV_B) continue;
      if (now-g_[i].last_ok > g_[i].timeout){ g_[i].st=H_FAILED; g_[i].conf=0; }
    }
  }
  Health st(SubsysId id) const { return g_[id].st; }
  bool usable(SubsysId id) const { return g_[id].st<=H_WARN; }
  bool failed(SubsysId id) const { return g_[id].st==H_FAILED; }
  uint8_t conf(SubsysId id) const { return g_[id].conf; }
  SysStatus aggregate() const {
    static const SubsysId crit[] = { S_IMU,S_GYRO,S_ACCEL,S_BATT,S_RADIO_A,S_CFG,S_FW };
    for (SubsysId id : crit) if (g_[id].st==H_FAILED) return SS_CRITICAL;
    for (int i=0;i<S_COUNT;i++) if (g_[i].st==H_FAILED)  return SS_FAULT;
    for (int i=0;i<S_COUNT;i++) if (g_[i].st==H_INVALID) return SS_WARNING;
    for (int i=0;i<S_COUNT;i++) if (g_[i].st==H_WARN)    return SS_WARNING;
    return SS_NORMAL;
  }
  void pack(uint8_t out[16]) const {
    memset(out,0,16);
    for (int i=0;i<S_COUNT && i<32;i++) out[i/4] |= (g_[i].st & 3) << ((i%4)*2);
  }
private:
  SensorSlot g_[S_COUNT];
};
static HealthReg g_health;

// ============ BATTERY ============
struct LiPoPt { float v; uint8_t pct; };
static const LiPoPt LIPO_TBL[] = {
  {4.20f,100},{4.15f,95},{4.11f,90},{4.08f,85},{4.02f,80},{3.98f,75},
  {3.95f,70},{3.91f,65},{3.87f,60},{3.85f,55},{3.84f,50},{3.82f,45},
  {3.80f,40},{3.79f,35},{3.77f,30},{3.75f,25},{3.73f,20},{3.71f,15},
  {3.69f,10},{3.61f,5},{3.27f,0}
};
static const LiPoPt LIION_TBL[] = {
  {4.20f,100},{4.10f,90},{4.00f,80},{3.92f,70},{3.85f,60},{3.78f,50},
  {3.70f,40},{3.62f,30},{3.52f,20},{3.40f,10},{3.00f,0}
};
static const LiPoPt LIFEPO4_TBL[] = {
  {3.65f,100},{3.45f,95},{3.40f,90},{3.35f,80},{3.33f,70},{3.32f,60},
  {3.30f,50},{3.28f,40},{3.26f,30},{3.24f,20},{3.20f,10},{2.50f,0}
};
static const LiPoPt NIMH_TBL[] = {
  {1.45f,100},{1.40f,95},{1.35f,90},{1.30f,80},{1.25f,60},{1.20f,40},
  {1.15f,20},{1.05f,5},{1.00f,0}
};
struct ChemCurve { const LiPoPt* pts; int n; float full_v; float warn_v; float crit_v; float rth_v; };

static const ChemCurve& chem_curve(BattChem c) {
  static const ChemCurve LIPO_CC    = { LIPO_TBL,   (int)(sizeof(LIPO_TBL)/sizeof(LIPO_TBL[0])),     4.20f,3.40f,3.20f,3.50f };
  static const ChemCurve LIION_CC   = { LIION_TBL,  (int)(sizeof(LIION_TBL)/sizeof(LIION_TBL[0])),   4.20f,3.30f,3.00f,3.40f };
  static const ChemCurve LIFEPO4_CC = { LIFEPO4_TBL,(int)(sizeof(LIFEPO4_TBL)/sizeof(LIFEPO4_TBL[0])),3.65f,3.20f,2.80f,3.30f };
  static const ChemCurve NIMH_CC    = { NIMH_TBL,   (int)(sizeof(NIMH_TBL)/sizeof(NIMH_TBL[0])),     1.45f,1.15f,1.00f,1.20f };
  switch (c) {
    case BC_LIPO:   return LIPO_CC;
    case BC_LIION:  return LIION_CC;
    case BC_LIFEPO4:return LIFEPO4_CC;
    case BC_NIMH:   return NIMH_CC;
    case BC_CUSTOM: return LIPO_CC;
  }
  return LIPO_CC;
}
static uint8_t cell_pct_from_v(float v, BattChem c) {
  const ChemCurve& cc = chem_curve(c);
  if (v >= cc.pts[0].v) return 100;
  if (v <= cc.pts[cc.n-1].v) return 0;
  for (int i=0;i<cc.n-1;i++){
    if (v <= cc.pts[i].v && v >= cc.pts[i+1].v){
      float v0=cc.pts[i].v, v1=cc.pts[i+1].v;
      float p0=cc.pts[i].pct, p1=cc.pts[i+1].pct;
      float t=(v-v1)/(v0-v1);
      return (uint8_t)(p1 + t*(p0-p1));
    }
  }
  return 0;
}
struct BattHist { uint8_t pct; uint16_t mv; int16_t ca; };
static constexpr int BATT_HIST_N = 120;
class BatteryMgr {
public:
  void begin(uint8_t cells, uint16_t cap_mah, uint16_t ina_cal, bool cal_ok, BattChem chem){
    cells_ = cells?cells:lim::CELLS_DEFAULT;
    cap_mah_ = cap_mah?cap_mah:3000;
    ina_cal_ok_ = cal_ok; chem_ = chem;
    const ChemCurve& cc = chem_curve(chem_);
    warn_v_=cc.warn_v; crit_v_=cc.crit_v; rth_v_=cc.rth_v;
    (void)ina_cal;
    v_slow_=v_fast_=0; mah_used_=0; wh_used_=0;
    hist_head_=0; hist_n_=0; last_us_=0; last_hist_ms_=0;
    sag_until_ms_=0; pct_=100; cell_rest_=cc.full_v; last_a_=0;
  }
  void set_chemistry(BattChem c){
    chem_=c; const ChemCurve& cc=chem_curve(c);
    warn_v_=cc.warn_v; crit_v_=cc.crit_v; rth_v_=cc.rth_v;
  }
  void set_cal(bool ok){ ina_cal_ok_=ok; }
  void set_current(float a){ last_a_=a; }
  void update(float v_pack, float a_pack, uint32_t now_ms, bool sensors_ok){
    if (!isfinite(v_pack)||!isfinite(a_pack)) return;
    if (v_pack<2.f || v_pack>60.f) return;
    if (v_slow_==0.f){ v_slow_=v_fast_=v_pack; last_us_=now_ms; }
    v_fast_ += 0.5f*(v_pack-v_fast_);
    v_slow_ += 0.02f*(v_pack-v_slow_);
    uint32_t dt_ms = now_ms-last_us_; last_us_=now_ms;
    if (dt_ms>0 && dt_ms<1000){
      float dt_h = dt_ms / 3600000.0f;
      float mah = a_pack * 1000.0f * dt_h;
      if (mah>0.f) mah_used_ += (uint16_t)(mah+0.5f);
      wh_used_ += v_pack * a_pack * dt_h * 100.0f;
    }
    float r_int_pack = 0.010f*cells_;
    float sag = a_pack*r_int_pack;
    float v_rest = v_pack + sag;
    const ChemCurve& cc = chem_curve(chem_);
    float v_rest_max = cells_*cc.full_v;
    if (v_rest > v_rest_max) v_rest = v_rest_max;
    cell_rest_ = v_rest/cells_;
    pct_ = cell_pct_from_v(cell_rest_, chem_);
    if (a_pack>3.0f && (v_slow_-v_fast_)>0.6f){
      if (sag_until_ms_<now_ms) sag_until_ms_=now_ms+2000;
    }
    sag_active_ = (now_ms < sag_until_ms_);
    if (now_ms-last_hist_ms_ >= 1000){
      last_hist_ms_ = now_ms;
      hist_[hist_head_].pct=pct_;
      hist_[hist_head_].mv =(uint16_t)(v_pack*1000.f);
      hist_[hist_head_].ca =(int16_t)(a_pack*100.f);
      hist_head_=(hist_head_+1)%BATT_HIST_N;
      if (hist_n_<BATT_HIST_N) hist_n_++;
    }
    (void)sensors_ok;
  }
  float pack_v() const { return v_fast_; }
  float pack_v_rest() const { return v_slow_; }
  uint8_t pct() const { return pct_; }
  uint16_t mah_used() const { return mah_used_; }
  uint16_t wh_centi() const { return (uint16_t)clampf(wh_used_,0.f,65000.f); }
  float cell_rest_v() const { return cell_rest_; }
  float cell_load_v() const { return v_fast_/cells_; }
  bool sag_active() const { return sag_active_; }
  bool cal_ok() const { return ina_cal_ok_; }
  uint8_t cells() const { return cells_; }
  uint16_t capacity_mah() const { return cap_mah_; }
  float warn_v() const { return warn_v_; }
  float crit_v() const { return crit_v_; }
  float rth_v() const { return rth_v_; }
  int hist_count() const { return hist_n_; }
  const BattHist& hist(int i) const {
    int start=(hist_head_-hist_n_+BATT_HIST_N)%BATT_HIST_N;
    return hist_[(start+i)%BATT_HIST_N];
  }
  void reset_counters(){ mah_used_=0; wh_used_=0; }
private:
  uint8_t cells_=2; uint16_t cap_mah_=3000; bool ina_cal_ok_=false;
  BattChem chem_=BC_LIPO;
  float warn_v_=3.4f, crit_v_=3.2f, rth_v_=3.5f;
  float v_slow_=0,v_fast_=0,last_a_=0;
  uint16_t mah_used_=0; float wh_used_=0;
  uint8_t pct_=100; float cell_rest_=4.2f;
  bool sag_active_=false; uint32_t sag_until_ms_=0;
  uint32_t last_us_=0,last_hist_ms_=0;
  BattHist hist_[BATT_HIST_N]{}; int hist_head_=0,hist_n_=0;
};
static BatteryMgr g_batt;

// [FIX] Urutan evaluasi operand '|' di C++ tidak dijamin: dua Wire.read() dalam satu ekspresi bisa
// tertukar (byte rendah terbaca dulu -> data IMU kacau). Baca byte tinggi lalu rendah secara eksplisit.
static inline int16_t i2c_rd16be(){ uint8_t h=(uint8_t)Wire.read(); uint8_t l=(uint8_t)Wire.read(); return (int16_t)(((uint16_t)h<<8)|l); }

// ============ MPU6050 ============
class MPU6050D {
public:
  bool begin(){
    Wire.beginTransmission(ADDR_MPU); Wire.write(0x6B); Wire.write(0x00);
    if (Wire.endTransmission()!=0) return false;
    Wire.beginTransmission(ADDR_MPU); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
    Wire.beginTransmission(ADDR_MPU); Wire.write(0x1B); Wire.write(0x18); Wire.endTransmission();
    Wire.beginTransmission(ADDR_MPU); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
    Wire.beginTransmission(ADDR_MPU); Wire.write(0x19); Wire.write(0x04); Wire.endTransmission();
    delay(50); return true;
  }
  bool read(float g[3], float a[3], float& t_c){
    Wire.beginTransmission(ADDR_MPU); Wire.write(0x3B);
    if (Wire.endTransmission(false)!=0) return false;
    if (Wire.requestFrom((int)ADDR_MPU,14)!=14) return false;
    int16_t ax=i2c_rd16be();
    int16_t ay=i2c_rd16be();
    int16_t az=i2c_rd16be();
    int16_t t =i2c_rd16be();
    int16_t gx=i2c_rd16be();
    int16_t gy=i2c_rd16be();
    int16_t gz=i2c_rd16be();
    constexpr float AS=4096.0f, GS=16.4f;
    a[0]=ax/AS; a[1]=ay/AS; a[2]=az/AS;
    g[0]=(gx/GS)*DEG_TO_RAD; g[1]=(gy/GS)*DEG_TO_RAD; g[2]=(gz/GS)*DEG_TO_RAD;
    t_c = t/340.f + 36.53f; return true;
  }
};
static MPU6050D g_imu;

// ============ EKF ============
struct EkfState {
  float q[4]={1,0,0,0}; float bg[3]={0,0,0};
  float P[7][7]={}; float roll=0,pitch=0,yaw=0;
  float gx=0,gy=0,gz=0; float ax=0,ay=0,az=1;
  float conf=0; bool mag_ok=false;
};
// [AHRS] Filter attitude Mahony (gyro + koreksi akselerometer + kompas). Menggantikan "EKF" lama yang tidak stabil
// secara numerik (kovarians jadi negatif -> bias gyro melonjak -> sudut berputar sendiri) dan salah indeks pada koreksi.
// Nama kelas/struct dipertahankan agar bagian lain firmware tidak berubah. BUKAN EKF.
class Ekf {
public:
  void begin(){ st_=EkfState{}; st_.q[0]=1; dt_=0.001f; settle_=0; }
  void set_bias(const float b[3]){ for (int i=0;i<3;i++) st_.bg[i]=clampf(b[i],-0.2f,0.2f); }
  void get_bias(float b[3]) const { b[0]=st_.bg[0]; b[1]=st_.bg[1]; b[2]=st_.bg[2]; }
  const EkfState& state() const { return st_; }
  void predict(const float gyro[3],const float accel[3],float dt){
    if (!isfinite(dt) || dt<=0.f || dt>0.05f) return;
    if (!is_finite_arr(gyro,3) || !is_finite_arr(accel,3)) return;
    dt_=dt;
    st_.ax=accel[0]; st_.ay=accel[1]; st_.az=accel[2];
    float w[3]={ gyro[0]-st_.bg[0], gyro[1]-st_.bg[1], gyro[2]-st_.bg[2] };
    st_.gx=w[0]; st_.gy=w[1]; st_.gz=w[2];
    apply_rate(w,dt);
  }
  void update_accel(const float accel[3]){
    if (!is_finite_arr(accel,3)) return;
    float m = sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    if (m<0.5f || m>1.5f) return;
    float v[3]; gravity_body(v);
    float nn[3]={accel[0]/m,accel[1]/m,accel[2]/m};
    float e[3]; cross(nn,v,e);
    float kp = (settle_<3000) ? 8.0f : 2.0f;            // 1/s; lebih agresif 3 detik pertama agar cepat konvergen
    float trust = (m>0.8f && m<1.2f) ? 1.0f : 0.2f;     // saat akselerasi kuat, percayai gyro
    if (settle_<100000) settle_++;
    float w[3]={ kp*trust*e[0], kp*trust*e[1], kp*trust*e[2] };
    apply_rate(w,dt_);
    const float ki=0.02f*trust;                         // koreksi bias gyro (rad/s per rad/s*s)
    for (int i=0;i<3;i++) st_.bg[i]=clampf(st_.bg[i]-ki*e[i]*dt_,-0.2f,0.2f);
    q2e(st_.q, st_.roll, st_.pitch, st_.yaw);
    float tilt = fmaxf(fabsf(st_.roll),fabsf(st_.pitch));
    st_.conf = (m>0.85f && m<1.15f) ? (1.f - tilt/(float)M_PI) : 0.5f;
  }
  void update_mag(const float mag[3],float decl){
    if (!is_finite_arr(mag,3)) { st_.mag_ok=false; return; }
    float m = sqrtf(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
    if (m<10.f || m>200.f){ st_.mag_ok=false; return; }
    st_.mag_ok=true;
    float nw[4]={0,cosf(decl),sinf(decl),0};
    float qc[4]={st_.q[0],-st_.q[1],-st_.q[2],-st_.q[3]};
    float t[4],r[4]; qmul(qc,nw,t); qmul(t,st_.q,r);
    float pb[3]={r[1],r[2],r[3]};
    float nn[3]={mag[0]/m,mag[1]/m,mag[2]/m};
    float v[3]; gravity_body(v);
    float c[3]; cross(nn,pb,c);
    float ez = c[0]*v[0]+c[1]*v[1]+c[2]*v[2];           // hanya komponen yaw (sumbu gravitasi), roll/pitch tidak terganggu kompas
    const float kpm=0.5f;
    float w[3]={ kpm*ez*v[0], kpm*ez*v[1], kpm*ez*v[2] };
    apply_rate(w,dt_);
    q2e(st_.q, st_.roll, st_.pitch, st_.yaw);
  }
private:
  EkfState st_; float dt_=0.001f; uint32_t settle_=0;
  void apply_rate(const float w[3],float dt){
    float qw[4]={0,w[0],w[1],w[2]}; float qd[4]; qmul(st_.q,qw,qd);
    for (int i=0;i<4;i++) st_.q[i]+=0.5f*qd[i]*dt;
    qnorm(st_.q);
    q2e(st_.q, st_.roll, st_.pitch, st_.yaw);
  }
  void gravity_body(float v[3]) const {
    float q0=st_.q[0],q1=st_.q[1],q2=st_.q[2],q3=st_.q[3];
    v[0]=2.f*(q1*q3-q0*q2); v[1]=2.f*(q0*q1+q2*q3); v[2]=q0*q0-q1*q1-q2*q2+q3*q3;
  }
  static void cross(const float a[3],const float b[3],float o[3]){
    o[0]=a[1]*b[2]-a[2]*b[1]; o[1]=a[2]*b[0]-a[0]*b[2]; o[2]=a[0]*b[1]-a[1]*b[0];
  }
  static void qnorm(float q[4]){
    float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (!isfinite(n)||n<1e-9f){q[0]=1;q[1]=q[2]=q[3]=0;return;}
    q[0]/=n;q[1]/=n;q[2]/=n;q[3]/=n;
  }
  static void qmul(const float a[4],const float b[4],float o[4]){
    o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
  }
  static void q2e(const float q[4],float& r,float& p,float& y){
    float w=q[0],x=q[1],yy=q[2],z=q[3];
    r=atan2f(2*(w*x+yy*z),1-2*(x*x+yy*yy));
    p=asinf(clampf(2*(w*yy-z*x),-1.f,1.f));
    y=atan2f(2*(w*z+x*yy),1-2*(yy*yy+z*z));
  }
};
static Ekf g_ekf;

// ============ ALTITUDE ============
struct AltEst { float alt_m=0,vs_mps=0,baro_bias=0,conf=0; bool baro_ok=false,tof_ok=false; float tof_m=NAN; };
class AltEstimator {
public:
  void begin(){ kf_x_[0]=kf_x_[1]=0; kf_p_[0][0]=kf_p_[1][1]=1; first_=true; }
  void update_baro(float z_raw,float dt){
    if (!isfinite(z_raw)) return;
    if (!isfinite(dt) || dt<=0.f || dt>1.f) dt = 0.02f;
    if (first_){ kf_x_[0]=z_raw; first_=false; st_.alt_m=z_raw; return; }
    float residual = z_raw - st_.alt_m - st_.vs_mps*dt;
    st_.baro_bias += 0.002f*residual*dt*10.f;
    float z = z_raw - st_.baro_bias;
    kf_x_[0] += kf_x_[1]*dt;
    kf_p_[0][0] += dt*(kf_p_[1][0]+kf_p_[0][1]) + dt*dt*kf_p_[1][1] + 0.002f;
    kf_p_[0][1] += dt*kf_p_[1][1]; kf_p_[1][0] += dt*kf_p_[1][1];
    kf_p_[1][1] += 0.02f;
    const float R=0.5f;
    float y = z - kf_x_[0]; float S = kf_p_[0][0] + R;
    float K0 = kf_p_[0][0]/S, K1 = kf_p_[1][0]/S;
    kf_x_[0]+=K0*y; kf_x_[1]+=K1*y;
    float p00=kf_p_[0][0],p01=kf_p_[0][1];
    kf_p_[0][0]-=K0*p00; kf_p_[0][1]-=K0*p01;
    kf_p_[1][0]-=K1*p00; kf_p_[1][1]-=K1*p01;
    st_.alt_m=kf_x_[0]; st_.vs_mps=kf_x_[1];
    st_.baro_ok=true; st_.conf = st_.tof_ok ? 0.95f : 0.7f;
  }
  void update_tof(float m){
    if (!isfinite(m) || m<=0.f || m>4.f){ st_.tof_ok=false; return; }
    st_.tof_ok=true; st_.tof_m=m;
    if (m<1.5f){ kf_x_[0]=0.75f*m+0.25f*kf_x_[0]; st_.alt_m=kf_x_[0]; st_.conf=0.95f; }
  }
  const AltEst& st() const { return st_; }
  AltEst& st() { return st_; }
private:
  AltEst st_{};
  float kf_x_[2]={0,0}, kf_p_[2][2]={{1,0},{0,1}};
  bool first_=true;
};
static AltEstimator g_alt;

// ============ POSITION ============
struct GeoPoint { double lat=0,lon=0; };
static float geo_dist_m(const GeoPoint& a,const GeoPoint& b){
  const double R=6371000.0;
  double dLat=(b.lat-a.lat)*DEG_TO_RAD, dLon=(b.lon-a.lon)*DEG_TO_RAD;
  double m = sin(dLat/2)*sin(dLat/2) + cos(a.lat*DEG_TO_RAD)*cos(b.lat*DEG_TO_RAD)*sin(dLon/2)*sin(dLon/2);
  return (float)(R*2*atan2(sqrt(m),sqrt(1-m)));
}
static float geo_bearing_rad(const GeoPoint& f,const GeoPoint& t){
  double p1=f.lat*DEG_TO_RAD,p2=t.lat*DEG_TO_RAD, dl=(t.lon-f.lon)*DEG_TO_RAD;
  double y=sin(dl)*cos(p2);
  double x=cos(p1)*sin(p2)-sin(p1)*cos(p2)*cos(dl);
  return (float)atan2(y,x);
}
struct PosEst { double lat=0,lon=0; float n=0,e=0,vn=0,ve=0,conf=0; uint32_t last_ms=0; };
class PosEstimator {
public:
  void begin(const GeoPoint& home){ home_=home; st_=PosEst{}; have_=false; }
  void set_home(const GeoPoint& home){ home_=home; st_=PosEst{}; have_=false; }
  void reset(){ st_=PosEst{}; have_=false; }
  void update(double lat,double lon,uint8_t sats,uint8_t fix,uint32_t now){
    if (fix<2 || sats<lim::SATS_MIN){ st_.conf=0; return; }
    float dn=(float)((lat-home_.lat)*111320.0);
    float de=(float)((lon-home_.lon)*111320.0*cos(home_.lat*DEG_TO_RAD));
    uint32_t dt=now-st_.last_ms;
    if (st_.last_ms && dt>0){
      float vn=(dn-st_.n)/(dt*0.001f), ve=(de-st_.e)/(dt*0.001f);
      st_.vn = 0.7f*st_.vn + 0.3f*vn;
      st_.ve = 0.7f*st_.ve + 0.3f*ve;
    }
    st_.lat=lat; st_.lon=lon; st_.n=dn; st_.e=de;
    st_.last_ms=now; st_.conf=clampf(sats/12.0f,0.2f,1.0f);
    have_=true;
  }
  const PosEst& st() const { return st_; }
  const GeoPoint& home() const { return home_; }
  bool have() const { return have_ && st_.conf>0.3f; }
private:
  GeoPoint home_{}; PosEst st_{}; bool have_=false;
};
static PosEstimator g_pos;

// ============ FW INTEGRITY ============
static bool g_fw_ok = false;
static char g_fw_version[32] = {0};
static bool fw_check_integrity(){
  snprintf(g_fw_version, sizeof(g_fw_version), "%d.%d.%d.%d",
           NEXEND_FW_MAJOR, NEXEND_FW_MINOR, NEXEND_FW_PATCH, NEXEND_FW_BUILD);
  char prev[32] = {0};
  File rf = LittleFS.open("/fwver.txt", "r");
  if (rf){ size_t n = rf.read((uint8_t*)prev, sizeof(prev)-1); prev[n]=0; rf.close(); }
  File wf = LittleFS.open("/fwver.txt", "w");
  if (!wf) return false;
  size_t written = wf.print(g_fw_version);
  wf.close();
  if (written == 0) return false;
  File vf = LittleFS.open("/fwver.txt", "r");
  if (!vf) return false;
  char check[32] = {0};
  size_t n = vf.read((uint8_t*)check, sizeof(check)-1); check[n]=0;
  vf.close();
  return strncmp(check, g_fw_version, sizeof(g_fw_version)) == 0;
}

// ============ ARBITER ============
struct ArbiterIn {
  bool disarm_request=false, arm_ok=false, inverted=false, crash=false,
       motor_fault=false, takeoff_abort=false, landing_prot=false,
       failsafe=false, rth_active=false, low_batt=false, crit_batt=false,
       post_landing_lock=false, failsafe_descend=false,
       airborne=false;
  float mix[4]={0,0,0,0};
  SysStatus sensor_st=SS_NORMAL;
};
struct ArbiterOut {
  bool motors_enabled=false; float out[4]={0,0,0,0};
  SafetyCause cause=SC_NONE; SysStatus status=SS_NORMAL; bool latch_lock=false;
};
class SafetyArbiter {
public:
  ArbiterOut evaluate(const ArbiterIn& in){
    ArbiterOut o{}; o.status=in.sensor_st;
    if (in.inverted) L_inv_=true;
    if (in.crash) L_crash_=true;
    if (in.motor_fault) L_mot_=true;
    if (in.sensor_st==SS_CRITICAL && !in.airborne) L_crit_=true;

    if (in.disarm_request) { o.cause=SC_DISARM; goto cut; }
    if (L_crit_ || (in.sensor_st==SS_CRITICAL && !in.airborne)) { o.cause=SC_CRIT_SENSOR; goto cut; }
    if (L_mot_) { o.cause=SC_MOTOR_FAULT; goto cut; }
    if (L_inv_) { o.cause=SC_INVERTED; goto cut; }
    if (L_crash_) { o.cause=SC_CRASH; goto cut; }
    if (in.failsafe && !in.rth_active && !in.failsafe_descend){
      o.cause=SC_FAILSAFE; o.status=SS_FAULT; goto cut;
    }
    if (in.crit_batt){
      if (in.airborne){
        o.cause = SC_LOW_BATT;
        o.status = SS_CRITICAL;
      } else {
        o.cause=SC_LOW_BATT; o.status=SS_CRITICAL; goto cut;
      }
    }
    if (in.low_batt && o.cause==SC_NONE){ o.cause=SC_LOW_BATT; if (o.status<SS_WARNING) o.status=SS_WARNING; }
    if (in.post_landing_lock){ o.cause=SC_LANDING_PROT; goto cut; }
    if (in.landing_prot && in.motor_fault){ o.cause=SC_LANDING_PROT; goto cut; }
    if (in.takeoff_abort){ o.cause=SC_MOTOR_FAULT; o.latch_lock=true; goto cut; }
    if (!in.arm_ok) goto cut;
    if (!is_finite_arr(in.mix,4)) goto cut;
    o.motors_enabled=true;
    for (int i=0;i<4;i++) o.out[i]=clampf(in.mix[i],0.f,1.f);
    return o;
  cut:
    o.motors_enabled=false;
    for (int i=0;i<4;i++) o.out[i]=0.f;
    switch (o.cause){
      case SC_INVERTED: case SC_CRASH: case SC_MOTOR_FAULT: case SC_CRIT_SENSOR:
        o.latch_lock=true; break;
      default: break;
    }
    return o;
  }
  bool has_latch() const { return L_inv_||L_crash_||L_mot_||L_crit_; }
  void clear_latches(){ L_inv_=L_crash_=L_mot_=L_crit_=false; }
private:
  bool L_inv_=false,L_crash_=false,L_mot_=false,L_crit_=false;
};
static SafetyArbiter g_arb;

// ============ LINK RADIO (1x nRF24L01) ============
// [1R] Sebelumnya 2 modul (A ch76 + B ch82). Radio B tidak pernah menerima data nyata
// (status B hanya salinan A) sehingga dihapus: pin, objek RF24, alamat, dan logika failover.
// Konsekuensi: TIDAK ADA redundansi link. Putus link = langsung lost -> failsafe.
struct RadioStats { LinkState st=LS_INIT; uint8_t q=0,last_seq=0,streak=0; uint32_t last_ms=0,last_good_ms=0; bool have=false; PktRc pkt{}; };
static void nxl_poll(RF24& r, RadioStats& s, uint32_t now); // [NXL]
static RF24 g_nrf(PIN_NRF_CE, PIN_NRF_CSN);
static const uint8_t NRF_ADDR[6]="NXFC1"; // [NXL] alamat default remote

static volatile uint32_t g_remote_heard_ms = 0; // [NXR] terakhir kali ada paket NEXEND valid dari remote
static void radio_send(const void* p, uint8_t len){
  // [NXR] writeFast+startListening langsung membatalkan TX (tanpa txStandBy). Pakai write() bertimeout RF24,
  // dan jangan memancar bila remote tidak terdengar (hindari blocking loop kontrol).
  return; // [NXL] telemetri/event/blackbox legacy dimatikan; remote memakai NEXEND_LINK_V1 via ACK payload
  uint32_t nowm=(uint32_t)millis();
  if (g_remote_heard_ms==0 || (uint32_t)(nowm-g_remote_heard_ms)>400) return;
  if (len > NEX_PKT_SIZE) len = NEX_PKT_SIZE;
  uint8_t buf[NEX_PKT_SIZE]; memset(buf, 0, NEX_PKT_SIZE);
  memcpy(buf, p, len);
  g_nrf.stopListening(); g_nrf.write(buf, NEX_PKT_SIZE); g_nrf.startListening();
}

class LinkRadio {
public:
  bool begin(){
    if (!g_nrf.begin()) return false;
    g_nrf.setChannel(76); g_nrf.setDataRate(RF24_1MBPS); g_nrf.setPALevel(RF24_PA_HIGH);
    g_nrf.setAutoAck(true);
    g_nrf.setRetries(2,6);
    g_nrf.setAddressWidth(5); g_nrf.setCRCLength(RF24_CRC_16); // [NXL] harus sama dgn remote
    g_nrf.enableDynamicPayloads(); g_nrf.enableAckPayload();    // [NXL]
    g_nrf.openReadingPipe(1,NRF_ADDR);                          // [NXL] ACK payload dikirim lewat pipe 1
    g_nrf.startListening();
    s_.last_good_ms = (uint32_t)millis();
    s_.st = LS_VALIDATING;
    return true;
  }
  void tick(uint32_t now){
    if (s_.st==LS_INIT && now>50) s_.st=LS_VALIDATING;
    nxl_poll(g_nrf, s_, now);  // [NXL]
    if (s_.st != LS_ACTIVE && s_.streak >= lim::LINK_PROMOTE_N) s_.st = LS_ACTIVE;
    if (s_.st == LS_ACTIVE && (now - s_.last_good_ms) > lim::RC_AGE_MS) { s_.st = LS_LOST; s_.streak = 0; }
  }
  bool get_rc(PktRc& out,uint32_t now){
    bool ok = s_.st==LS_ACTIVE && s_.have && (now-s_.last_ms)<=lim::RC_AGE_MS;
    if (ok){ out=s_.pkt; return true; }
    return false;
  }
  void send(const void* p,uint8_t n){ radio_send(p, n); }
  LinkState st() const { return s_.st; }
  uint8_t   q()  const { return s_.q; }
  bool lost() const { return s_.st==LS_LOST; }
  // [NXR] paket non-RC (CMD/BB_REQ) disimpan di antrean; sebelumnya loop() dan poll_one() berebut FIFO radio
  bool pop_other(uint8_t* out){ if (oq_h_==oq_t_) return false; memcpy(out,oq_[oq_t_],NEX_PKT_SIZE); oq_t_=(oq_t_+1)&3; return true; }
private:
  RadioStats s_;
  uint8_t oq_[4][NEX_PKT_SIZE]; uint8_t oq_h_=0, oq_t_=0;
  void poll_one(RF24& r, RadioStats& s, uint32_t now){
    if (r.available()){
      uint8_t buf[NEX_PKT_SIZE];
      r.read(buf, NEX_PKT_SIZE);
      if (buf[0]==NEX_SYNC0 && buf[1]==NEX_SYNC1) g_remote_heard_ms=(uint32_t)millis(); // [NXR]
      if (nex_check_header(buf, PKT_CMD) || nex_check_header(buf, PKT_BB_REQ)){ // [NXR]
        uint8_t nh=(oq_h_+1)&3;
        if (nh!=oq_t_){ memcpy(oq_[oq_h_],buf,NEX_PKT_SIZE); oq_h_=nh; }
        return;
      }
      if (nex_check_header(buf, PKT_RC)){
        PktRc p; memcpy(&p, buf, sizeof(p));
        if (accept(p, s.last_seq, s.last_ms, now)){
          s.pkt=p; s.have=true; s.last_good_ms=now;
          if (s.streak<255) s.streak++;
          s.q = (s.q<95)?s.q+5:100;
          return;
        }
      }
      s.q = (s.q>3)?s.q-3:0;
      if (s.streak) s.streak--;
    } else {
      s.q = (s.q>1)?s.q-1:0;
    }
  }
  bool accept(const PktRc& p,uint8_t& last_seq,uint32_t& last_ms,uint32_t now){
    if (!nex_verify(p)) return false;
    uint16_t lo = lim::RC_SANITY_MIN, hi = lim::RC_SANITY_MAX;
    if (p.throttle<lo || p.throttle>hi) return false;
    if (p.roll<lo || p.roll>hi) return false;
    if (p.pitch<lo || p.pitch>hi) return false;
    if (p.yaw<lo || p.yaw>hi) return false;
    if (p.h.seq == last_seq) return false;
    last_seq = p.h.seq; last_ms = now;
    return true;
  }
};
static LinkRadio g_radio;

// ============ WARNINGS ============
struct WarnEntry { uint16_t id; WarnLevel lvl; uint32_t first_ms,last_ms; uint16_t cnt; };
class WarnMgr {
public:
  void begin(){ n_=0; memset(e_,0,sizeof(e_)); }
  void raise(WarnId id,WarnLevel lvl){
    uint32_t now=(uint32_t)millis();
    for (int i=0;i<n_;i++) if (e_[i].id==id){
      if (lvl>e_[i].lvl) e_[i].lvl=lvl;
      e_[i].last_ms=now; if (e_[i].cnt<0xFFFF) e_[i].cnt++;
      return;
    }
    if (n_>=32) return;
    e_[n_]={ (uint16_t)id, lvl, now, now, 1 }; n_++;
  }
  void clear(WarnId id){
    for (int i=0;i<n_;i++) if (e_[i].id==id){
      for (int j=i;j<n_-1;j++) e_[j]=e_[j+1]; n_--; return;
    }
  }
  void tick(uint32_t now){
    for (int i=0;i<n_;){
      if (e_[i].lvl==WL_INFO && (now-e_[i].last_ms)>5000){
        for (int j=i;j<n_-1;j++) e_[j]=e_[j+1]; n_--;
      } else i++;
    }
  }
  WarnLevel top_level() const { WarnLevel m=WL_INFO; for (int i=0;i<n_;i++) if (e_[i].lvl>m) m=e_[i].lvl; return m; }
  uint16_t top_id() const { WarnLevel m=WL_INFO; uint16_t id=0; for (int i=0;i<n_;i++) if (e_[i].lvl>=m){ m=e_[i].lvl; id=e_[i].id; } return id; }
  uint8_t count() const { return n_; }
private:
  WarnEntry e_[32]{}; uint8_t n_=0;
};
static WarnMgr g_warn;

// ============ I2C RECOVERY ============
static uint16_t g_i2c_err_streak = 0;
static uint32_t g_i2c_last_recover_ms = 0;
static void i2c_recover(uint32_t now){
  if (now - g_i2c_last_recover_ms < 2000) return;
  g_i2c_last_recover_ms = now;
  Wire.end(); delay(2);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT);
  digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  for (int i=0;i<9;i++){
    digitalWrite(PIN_I2C_SCL, LOW); delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW); delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(5);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  Wire.setSDA(PIN_I2C_SDA); Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);
  g_i2c_err_streak = 0;
  g_warn.raise(W_I2C_RECOVER, WL_WARNING);
}
static void i2c_note_error(uint32_t now){
  g_i2c_err_streak++;
  if (g_i2c_err_streak >= 20) i2c_recover(now);
}

// ============ BLACKBOX ============
#pragma pack(push,1)
struct BbFrame {
  uint32_t t_ms; uint16_t throttle; int16_t rc_r,rc_p,rc_y;
  float gyro[3]; float accel[3]; float roll,pitch,yaw;
  float out[4]; int16_t alt_dm,tof_mm;
  uint16_t batt_mv; int16_t batt_ca; uint8_t batt_pct;
  uint8_t fsm,arm,cause,sys,rA_q,rB_q,flags;
  uint8_t stab_on, auto_stab_on, auto_strength, active_profile;
  int16_t adaptive_adjust_pct;
  float roll_sp, pitch_sp, yaw_rate_sp;
};
struct BbEvent { uint32_t t_ms; uint8_t sev,ev; uint16_t arg; };
#pragma pack(pop)

class Blackbox {
public:
  static constexpr const char* BB_PATH = "/bb.bin";
  static constexpr const char* EV_PATH = "/ev.bin";

  bool begin(){
    File f = open_append(BB_PATH);
    ok_ = (bool)f;
    if (f) f.close();
    File e = open_append(EV_PATH);
    if (e) e.close();
    return ok_;
  }
  void frame(const BbFrame& fr){
    if (!ok_) return;
    File f = open_append(BB_PATH);
    if (!f) { ok_ = false; return; }
    f.write((const uint8_t*)&fr, sizeof(fr));
    f.close();
  }
  void flush(){}
  void event(uint8_t sev, uint8_t id, uint16_t arg){
    BbEvent e{ (uint32_t)millis(), sev, id, arg };
    File ef = open_append(EV_PATH);
    if (ef){ ef.write((const uint8_t*)&e, sizeof(e)); ef.close(); }
  }
  bool healthy() const { return ok_; }
  bool on_sd() const { return g_sd_ok; }

  static File open_append(const char* p){
    if (g_sd_ok)  return SD.open(p, "a");
    if (g_lfs_ok) return LittleFS.open(p, "a");
    return File();
  }
private:
  bool ok_ = false;
};
static Blackbox g_bb;

class BlackboxRead {
public:
  bool begin(){
    size_ = 0;
    File f = open_read();
    if (f) { size_ = (uint32_t)f.size(); f.close(); }
    return true;
  }
  uint16_t read_chunk(uint32_t off, uint8_t* out, uint16_t n){
    if (!f_ || (uint32_t)f_.position() != off){
      if (f_) f_.close();
      f_ = open_read();
      if (!f_) return 0;
      if (!f_.seek(off)) return 0;
    }
    int r = f_.read(out, n);
    return (r > 0) ? (uint16_t)r : 0;
  }
  uint32_t size() const { return size_; }
  bool erase(){
    if (f_) f_.close();
    bool ok1, ok2;
    if (g_sd_ok){
      ok1 = SD.remove(Blackbox::BB_PATH);
      ok2 = SD.remove(Blackbox::EV_PATH);
    } else {
      ok1 = LittleFS.remove(Blackbox::BB_PATH);
      ok2 = LittleFS.remove(Blackbox::EV_PATH);
    }
    size_ = 0;
    return ok1 && ok2;
  }
private:
  File f_;
  uint32_t size_ = 0;
  static File open_read(){
    if (g_sd_ok)  return SD.open(Blackbox::BB_PATH, "r");
    if (g_lfs_ok) return LittleFS.open(Blackbox::BB_PATH, "r");
    return File();
  }
};
static BlackboxRead g_bbread;

// ---- Auto-PID audit log ke SD (rev 6.5) ----
// adapt_log_to_sd didefinisikan setelah class AdaptivePid (tipe nested
// AdaptivePid::Snapshot tidak bisa di-forward-declare).

// ============ DETECTORS ============
class CrashDetect {
public:
  void tick(const float accel[3],const float gyro[3],float vs,float alt,bool motors,uint32_t now){
    if (latched_) return;
    float amag=sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    float rot =sqrtf(gyro[0]*gyro[0]+gyro[1]*gyro[1]+gyro[2]*gyro[2]);
    bool impact = amag>lim::CRASH_G, tumbling = rot>12.f, drop = vs<-3.f && alt<3.f;
    int votes=(impact?1:0)+(tumbling?1:0)+(drop?1:0);
    bool trig = motors && votes>=2 && impact;
    if (trig){
      if (streak_==0) first_=now;
      if (streak_<255) streak_++;
      if (now-first_>=lim::CRASH_MS) latched_=true;
    } else { if (streak_) streak_--; if (!streak_) first_=now; }
  }
  bool triggered() const { return latched_; }
  void clear(){ latched_=false; streak_=0; first_=0; }
private: bool latched_=false; uint8_t streak_=0; uint32_t first_=0;
};
static CrashDetect g_crash;

class MotorDiag {
public:
  void tick(const float mix[4],const float gyro[3],const float accel[3],
            float roll,float pitch,bool landing_ctx,bool thr_low,uint32_t now){
    if (latched_) return;
    float rot=sqrtf(gyro[0]*gyro[0]+gyro[1]*gyro[1]+gyro[2]*gyro[2]);
    float amag=sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    bool violent=rot>8.f, impact=amag>3.f;
    float tilt=fmaxf(fabsf(roll),fabsf(pitch));
    bool divergent = tilt > 60.f*DEG_TO_RAD;
    int votes=(violent?1:0)+(impact?1:0)+(divergent?1:0);
    bool trig = landing_ctx && (votes>=2 || (violent&&divergent));
    if (!landing_ctx){
      float cmd_avg=(mix[0]+mix[1]+mix[2]+mix[3])*0.25f;
      if (cmd_avg>0.4f && rot<0.05f && amag>0.9f && amag<1.1f) trig=true;
    }
    (void)thr_low;
    if (trig){
      if (streak_==0) first_=now;
      if (streak_<255) streak_++;
      uint32_t need = landing_ctx ? 60 : 250;
      if (now-first_>=need) latched_=true;
    } else { if (streak_) streak_--; if (!streak_) first_=now; }
  }
  bool fault() const { return latched_; }
  void clear(){ latched_=false; streak_=0; first_=0; }
private: bool latched_=false; uint8_t streak_=0; uint32_t first_=0;
};
static MotorDiag g_motordiag;

class LandingDetect {
public:
  void tick(float tof_m,bool tof_valid,float alt,float vs,float roll,float pitch,float gm,uint16_t thr,uint32_t now){
    near_ = tof_valid && tof_m<0.35f;
    bool low_alt = (tof_valid && tof_m<0.20f) || alt<0.5f;
    bool low_vs  = fabsf(vs)<0.4f;
    bool stable  = fmaxf(fabsf(roll),fabsf(pitch))<0.3f;
    bool low_g   = gm<0.8f;
    bool low_t   = thr<lim::RC_MIN+40;
    int votes = (low_alt?1:0)+(low_vs?1:0)+(stable?1:0)+(low_g?1:0)+(low_t?1:0);
    bool trig = votes>=4;
    if (trig){
      if (streak_==0) first_=now;
      if (streak_<255) streak_++;
      if (now-first_>=400) landed_=true;
    } else {
      if (streak_) streak_--; if (!streak_) first_=now;
      if (landed_ && votes<2) landed_=false;
    }
  }
  bool landed() const { return landed_; }
  bool near_ground() const { return near_; }
  void clear(){ landed_=false; streak_=0; near_=false; }
private: bool landed_=false,near_=false; uint8_t streak_=0; uint32_t first_=0;
};
static LandingDetect g_landing;

class InvertedDet {
public:
  void tick(float roll,float pitch,uint32_t now){
    float td=fmaxf(fabsf(roll),fabsf(pitch))*RAD_TO_DEG;
    if (td>lim::INVERT_DEG){
      if (first_==0) first_=now;
      if (now-first_>=lim::INVERT_MS) latched_=true;
    } else first_=0;
  }
  bool triggered() const { return latched_; }
  void clear(){ latched_=false; first_=0; }
private: bool latched_=false; uint32_t first_=0;
};
static InvertedDet g_inv;

class TakeoffGuard {
public:
  void arm(uint32_t now,float base_alt){ armed_=true; t0_=now; base_alt_=base_alt; abort_=false; }
  void tick(const EkfState& ekf,const AltEst& alt,const float mix[4],bool motors_on,uint32_t now){
    if (!armed_||abort_) return;
    if (now-t0_>2500){ armed_=false; return; }
    float tilt=fmaxf(fabsf(ekf.roll),fabsf(ekf.pitch));
    float rot=sqrtf(ekf.gx*ekf.gx+ekf.gy*ekf.gy+ekf.gz*ekf.gz);
    bool tilted=tilt>35.f*DEG_TO_RAD, spin=rot>10.f;
    bool now_up = alt.alt_m>base_alt_+0.2f;
    float thr_cmd=(mix[0]+mix[1]+mix[2]+mix[3])*0.25f;
    bool commanded = motors_on && thr_cmd>0.35f;
    if (commanded && (tilted||spin) && !now_up && (now-t0_>800)) abort_=true;
  }
  bool aborted() const { return abort_; }
  void clear(){ armed_=false; abort_=false; }
private: bool armed_=false,abort_=false; uint32_t t0_=0; float base_alt_=0;
};
static TakeoffGuard g_takeoff;

// ============ ARMING ============
struct ArmChecks {
  bool imu_ok=true,accel_ok=true,gyro_ok=true,baro_ok=true,mag_ok=true,
       gps_ok=true,tof_ok=true,batt_ok=true,rA_ok=true,
       esc_ok=true,motor_ok=true,rc_valid=true,no_inverted=true,
       no_crash=true,no_motor_lock=true,no_post_lock=true,synced=true,
       cfg_ok=true,fw_ok=true;
};
static ArmReject precheck(const ArmChecks& c){
  if (!c.fw_ok) return AR_FW;
  if (!c.cfg_ok) return AR_CFG;
  if (!c.imu_ok) return AR_IMU;
  if (!c.accel_ok) return AR_ACCEL;
  if (!c.gyro_ok) return AR_GYRO;
  if (!c.baro_ok) return AR_BARO;
  if (!c.mag_ok) return AR_MAG;
  if (!c.gps_ok) return AR_GPS;
  if (!c.batt_ok) return AR_BATT;
  if (!c.rA_ok) return AR_RADIO_A;
  if (!c.esc_ok) return AR_ESC;
  if (!c.motor_ok) return AR_MOTOR;
  if (!c.rc_valid) return AR_RC;
  if (!c.no_inverted) return AR_INVERTED;
  if (!c.no_crash) return AR_CRASH_LOCK;
  if (!c.no_motor_lock) return AR_MOTOR_LOCK;
  if (!c.no_post_lock) return AR_POST_LAND_LOCK;
  if (!c.synced) return AR_NO_SYNC;
  return AR_OK;
}
class Arming {
public:
  void tick(uint32_t now,bool arm_edge,bool disarm_edge,
            uint16_t thr_us,uint16_t roll_us,uint16_t pitch_us,uint16_t yaw_us,
            const ArmChecks& c){
    if (disarm_edge){ st_=AS_DISARMED; reject_=AR_OK; return; }
    bool t_low = (thr_us<=lim::RC_MIN+20);
    switch (st_){
      case AS_DISARMED:
        if (arm_edge && !last_edge_){
          ArmReject r=precheck(c);
          if (r!=AR_OK) reject_=r;
          else { st_=AS_PREARM; t_enter_=now; t_req_=now; reject_=AR_OK; }
        }
        break;
      case AS_PREARM:
        if (now-t_req_>3000){ st_=AS_DISARMED; reject_=AR_GESTURE_TIMEOUT; break; } // [NXR]
        if (!t_low){ reject_=AR_THROTTLE; break; }
        if (now-t_enter_>100){ st_=AS_GESTURE; t_enter_=now; }
        break;
      case AS_GESTURE:
        if (now-t_req_>3000){ st_=AS_DISARMED; reject_=AR_GESTURE_TIMEOUT; break; } // [NXR]
        if (!t_low){ reject_=AR_THROTTLE; st_=AS_PREARM; t_enter_=now; break; }
        if (gesture_match(thr_us,roll_us,pitch_us,yaw_us)){
          st_=AS_ARMED_IDLE; t_enter_=now;
        }
        break;
      case AS_ARMED_IDLE:
        if (thr_us>lim::RC_MIN+60){ st_=AS_IDLE_SPIN; t_enter_=now; }
        break;
      case AS_IDLE_SPIN:
        if (thr_us<lim::RC_MIN+30){ st_=AS_ARMED_IDLE; t_enter_=now; break; }
        if (thr_us>lim::RC_MIN+150){ st_=AS_TAKEOFF_READY; t_enter_=now; }
        break;
      default: break;
    }
    last_edge_=arm_edge;
  }
  void set_flight(){ if (st_>=AS_ARMED_IDLE) st_=AS_FLIGHT; }
  void force_disarm(){ st_=AS_DISARMED; }
  ArmState st() const { return st_; }
  bool armed() const { return st_>=AS_ARMED_IDLE; }
  bool ok() const { return st_>=AS_ARMED_IDLE; }
  bool takeoff_ready() const { return st_==AS_TAKEOFF_READY; }
  bool idle_stage() const { return st_==AS_IDLE_SPIN; }
  ArmReject reject() const { return reject_; }
private:
  ArmState st_=AS_DISARMED; ArmReject reject_=AR_OK; uint32_t t_enter_=0, t_req_=0; bool last_edge_=false;

  static bool gesture_match(uint16_t thr,uint16_t roll,uint16_t pitch,uint16_t yaw){
    uint16_t lo=lim::RC_MIN, hi=lim::RC_MAX;
    if (g_cfg.valid()){
      uint16_t c_lo=g_cfg.c().rc_min, c_hi=g_cfg.c().rc_max;
      if (c_lo>=lim::RC_SANITY_MIN && c_hi<=lim::RC_SANITY_MAX && c_hi>c_lo+200){ lo=c_lo; hi=c_hi; }
    }
    constexpr uint16_t BAND=200;
    bool thr_low    = thr   <= (uint16_t)(lo+100);
    bool yaw_right  = yaw   >= (uint16_t)(hi-BAND);   // stick kiri: yaw KANAN penuh
    bool roll_left  = roll  <= (uint16_t)(lo+BAND);   // stick kanan: roll KIRI penuh
    bool pitch_down = pitch <= (uint16_t)(lo+BAND);   // stick kanan: pitch bawah penuh
    return thr_low && yaw_right && roll_left && pitch_down;
  }
};
static Arming g_arming;

// ============ FSM ============
struct FsmIn {
  uint32_t now; const ArbiterOut* arb;
  bool link_lost,arm_ready,arm_ok,disarm_edge,rth_btn;
  bool rth_ok_batt,rth_ok_gps,rth_ok_home;
  bool landing_detected,gps_fix_ok,home_valid,lock_cleared;
  uint8_t failsafe_action;
  bool lowbatt_rth_trigger;
};
class FlightFsm {
public:
  void begin(uint32_t now){ enter(FS_BOOT,now); }
  void tick(const FsmIn& in){
    const ArbiterOut& a=*in.arb;
    rth_gate_ = (!in.rth_ok_batt?1:0)|(!in.rth_ok_gps?2:0)|(!in.rth_ok_home?4:0);
    if (a.cause==SC_INVERTED) { enter(FS_INVERTED,in.now); return; }
    if (a.cause==SC_CRASH) { enter(FS_CRASH,in.now); return; }
    if (a.cause==SC_MOTOR_FAULT) { enter(FS_MOTOR_FAULT,in.now); return; }
    if (a.cause==SC_CRIT_SENSOR) { enter(FS_CRITICAL_SENSOR,in.now); return; }
    if (in.disarm_edge) { enter(FS_DISARMED,in.now); return; }
    switch (st_){
      case FS_BOOT: enter(FS_SYS_CHECK,in.now); break;
      case FS_SYS_CHECK: enter(FS_GPS_SEARCH,in.now); break;
      case FS_GPS_SEARCH:
        if (in.gps_fix_ok && in.home_valid) enter(FS_READY,in.now);
        break;
      case FS_READY:
        if (in.link_lost){ enter(FS_FAILSAFE,in.now); break; }
        if (in.arm_ok) enter(FS_ARMED_IDLE,in.now);
        break;
      case FS_ARMED_IDLE:
        if (in.disarm_edge){ enter(FS_DISARMED,in.now); break; }
        if (in.link_lost){ enter(FS_FAILSAFE,in.now); break; }
        if (in.arm_ready) enter(FS_TAKEOFF,in.now);
        break;
      case FS_TAKEOFF:
        if (in.now-t_enter_>800) enter(FS_FLIGHT,in.now);
        break;
      case FS_FLIGHT:
        if (in.rth_btn || in.lowbatt_rth_trigger) enter(FS_RTH,in.now);
        else if (in.link_lost) enter(FS_FAILSAFE,in.now);
        else if (in.landing_detected) enter(FS_LANDING,in.now);
        break;
      case FS_LANDING:
        if (in.landing_detected) enter(FS_LANDED,in.now);
        break;
      case FS_LANDED: enter(FS_POST_LANDING_LOCK,in.now); break;
      case FS_POST_LANDING_LOCK:
        if (in.lock_cleared) enter(FS_DISARMED,in.now);
        break;
      case FS_DISARMED:
        if (in.arm_ok) enter(FS_ARMED_IDLE,in.now);
        break;
      case FS_FAILSAFE:
        switch (in.failsafe_action){
          case 0:
            if (in.rth_ok_batt && in.rth_ok_gps && in.rth_ok_home) enter(FS_RTH,in.now);
            else enter(FS_FAILSAFE_DESCEND,in.now);
            break;
          case 1: enter(FS_FAILSAFE_DESCEND,in.now); break;
          case 2:
          default: break;
        }
        break;
      case FS_FAILSAFE_DESCEND:
        if (in.landing_detected) enter(FS_LANDED,in.now);
        break;
      case FS_RTH:
        if (in.landing_detected) enter(FS_LANDED,in.now);
        break;
      case FS_INVERTED: case FS_CRASH: case FS_MOTOR_FAULT: case FS_CRITICAL_SENSOR:
        if (in.lock_cleared) enter(FS_DISARMED,in.now);
        break;
      default: break;
    }
  }
  FlightState st() const { return st_; }
  uint8_t rth_gate() const { return rth_gate_; }
private:
  void enter(FlightState s,uint32_t now){ st_=s; t_enter_=now; }
  FlightState st_=FS_BOOT; uint32_t t_enter_=0; uint8_t rth_gate_=0;
};
static FlightFsm g_fsm;

// ============ RTH ============
class RthV2 {
public:
  enum Phase : uint8_t { R_IDLE=0,R_CLIMB,R_CRUISE,R_DESCEND,R_FINAL,R_DONE };
  void begin(){ active_=false; phase_=R_IDLE; }
  void start(const GeoPoint& home,const PosEst& pos,float alt){
    home_=home; active_=true; phase_=R_CLIMB;
    float climb_alt = lim::RTH_CLIMB_DEFAULT;
    if (g_cfg.valid()) climb_alt = g_cfg.c().rth_climb_alt;
    target_alt_ = fmaxf(alt + lim::RTH_CLIMB_EXTRA, climb_alt);
    target_alt_ = fminf(target_alt_, (float)rth_max_alt_effective()); // [RA] tidak pernah melebihi ketinggian RTH maksimal (bila alt > batas, turun ke batas)
    (void)pos;
  }
  void stop(){ active_=false; phase_=R_IDLE; }
  bool active() const { return active_; }
  uint8_t phase() const { return phase_; }
  void tick(const PosEst& pos,float alt,float yaw_now,float dt,
            float& yaw_sp,float& vn,float& ve,float& alt_sp){
    vn=0; ve=0; alt_sp=alt; yaw_sp=yaw_now;
    if (!active_) return;
    float dist=sqrtf(pos.n*pos.n + pos.e*pos.e);
    float bearing=atan2f(-pos.e, -pos.n);
    yaw_sp=wrap_pi(bearing);
    float cruise_sp = lim::RTH_SPEED_DEFAULT;
    if (g_cfg.valid()) cruise_sp = g_cfg.c().rth_speed;
    switch (phase_){
      case R_CLIMB:
        alt_sp=target_alt_;
        if (fabsf(alt-target_alt_)<1.f) phase_=R_CRUISE;
        break;
      case R_CRUISE:{
        float sp=cruise_sp;
        if (dist<15.f) sp=cruise_sp*0.5f;
        if (dist< 5.f) sp=cruise_sp*0.25f;
        float un = (dist>0.01f)? (-pos.n/dist) : 0.f;
        float ue = (dist>0.01f)? (-pos.e/dist) : 0.f;
        vn=sp*un; ve=sp*ue;
        alt_sp=target_alt_;
        if (dist<2.f){ phase_=R_DESCEND; target_alt_=0.5f; }
        break;
      }
      case R_DESCEND:
        alt_sp=0.5f;
        if (alt<1.f) phase_=R_FINAL;
        break;
      case R_FINAL:
        alt_sp=0.f; phase_=R_DONE;
        break;
      default: break;
    }
    (void)dt;
  }
private: GeoPoint home_{}; bool active_=false; uint8_t phase_=R_IDLE; float target_alt_=lim::RTH_CLIMB_DEFAULT;
};
static RthV2 g_rth;

// ============ FLIGHT CONTROLLER ============
struct RcInput { float thr=0,roll=0,pitch=0,yaw=0; bool valid=false; };
class FlightCtrlV2 {
public:
  void begin(const PidProfile& p){ prof_=p; hover_=0.5f; }
  void set_profile(const PidProfile& p){ prof_=p; }
  const PidProfile& profile() const { return prof_; }
  float hover() const { return hover_; }
  const float* last_motor() const { return last_motor_; }
  float last_roll_sp() const { return last_roll_sp_; }
  float last_pitch_sp() const { return last_pitch_sp_; }
  float last_yaw_rate_sp() const { return last_yaw_rate_sp_; }
  void reset_integrators() {
    ps_ang_r_.i = 0; ps_ang_p_.i = 0;
    ps_rate_r_.i = 0; ps_rate_p_.i = 0; ps_rate_y_.i = 0;
    ps_alt_pos_.i = 0; ps_alt_vel_.i = 0;
    ps_pos_n_.i = 0; ps_pos_e_.i = 0;
    ps_rate_r_.d_filt = 0; ps_rate_p_.d_filt = 0; ps_rate_y_.d_filt = 0;
    ps_ang_r_.d_filt = 0; ps_ang_p_.d_filt = 0;
  }
  void tick(const EkfState& ekf,const AltEst& alt,const PosEst& pos,
            const RcInput& rc,float dt,FlightMode mode,bool armed,
            float alt_sp,bool alt_hold,bool pos_hold,float vn_sp,float ve_sp,
            bool use_rth_yaw, float rth_yaw_sp, bool stabilizer_on){
    for (int i=0;i<4;i++) last_motor_[i]=0.f;
    if (!armed) return;
    if (!isfinite(dt)||dt<=0.f||dt>0.05f) return;
    if (!isfinite(ekf.roll)||!isfinite(ekf.pitch)||!isfinite(ekf.yaw)) return;

    PidLimits lrate{}; lrate.i_max=0.35f; lrate.out_max=1.f; lrate.d_lpf_hz=100.f; lrate.slew_per_s=50.f;
    float ur=0, up=0, uy=0;
    float thr = rc.thr;

    float yaw_rate_sp = 0.f;
    if (use_rth_yaw){
      float yaw_err = wrap_pi(rth_yaw_sp - ekf.yaw);
      yaw_rate_sp = clampf(lim::YAW_ANGLE_KP * yaw_err,
                          -lim::YAW_RATE_LIMIT, lim::YAW_RATE_LIMIT);
    } else {
      yaw_rate_sp = rc.yaw * 3.f;
    }
    last_yaw_rate_sp_ = yaw_rate_sp;

    float roll_sp  = 0.f;
    float pitch_sp = 0.f;

    if (stabilizer_on){
      roll_sp  = rc.roll  * lim::MAX_TILT;
      pitch_sp = rc.pitch * lim::MAX_TILT;

      if (pos_hold && pos.conf > 0.3f){
        PidLimits lp{}; lp.i_max=0.4f; lp.out_max=3.f; lp.d_lpf_hz=2.f; lp.slew_per_s=5.f;
        float vn_des=pid_step(prof_.pos_n,ps_pos_n_,vn_sp,pos.vn,0,dt,lp);
        float ve_des=pid_step(prof_.pos_e,ps_pos_e_,ve_sp,pos.ve,0,dt,lp);
        vn_des=clampf(vn_des,-4.f,4.f); ve_des=clampf(ve_des,-4.f,4.f);
        float cy=cosf(ekf.yaw), sy=sinf(ekf.yaw);
        float vn_err=vn_des-pos.vn, ve_err=ve_des-pos.ve;
        float vf_err= cy*vn_err + sy*ve_err;
        float vr_err=-sy*vn_err + cy*ve_err;
        float tilt_f = atan2f(vf_err,4.f);
        float tilt_r = atan2f(vr_err,4.f);
        pitch_sp = clampf(-tilt_f,-lim::MAX_TILT,lim::MAX_TILT);
        roll_sp  = clampf( tilt_r,-lim::MAX_TILT,lim::MAX_TILT);
      }
      last_roll_sp_  = roll_sp;
      last_pitch_sp_ = pitch_sp;

      PidLimits lang{}; lang.i_max=0.5f; lang.out_max=4.f; lang.d_lpf_hz=60.f; lang.slew_per_s=40.f;
      float rr_sp=pid_step(prof_.ang_roll, ps_ang_r_,roll_sp, ekf.roll, 0,dt,lang);
      float pr_sp=pid_step(prof_.ang_pitch,ps_ang_p_,pitch_sp,ekf.pitch,0,dt,lang);
      ur=pid_step(prof_.rate_roll, ps_rate_r_,rr_sp, ekf.gx,  ekf.gx, dt,lrate);
      up=pid_step(prof_.rate_pitch, ps_rate_p_,pr_sp, ekf.gy,  ekf.gy, dt,lrate);
    } else {
      float rr_sp = rc.roll  * lim::YAW_RATE_LIMIT;
      float pr_sp = rc.pitch * lim::YAW_RATE_LIMIT;
      last_roll_sp_  = 0.f;
      last_pitch_sp_ = 0.f;
      ur=pid_step(prof_.rate_roll, ps_rate_r_,rr_sp, ekf.gx, ekf.gx, dt,lrate);
      up=pid_step(prof_.rate_pitch,ps_rate_p_,pr_sp, ekf.gy, ekf.gy, dt,lrate);
    }

    uy=pid_step(prof_.rate_yaw, ps_rate_y_,yaw_rate_sp, ekf.gz, ekf.gz, dt,lrate);

    if (alt_hold && isfinite(alt_sp) && isfinite(alt.alt_m) && isfinite(alt.vs_mps)){
      PidLimits la{}; la.i_max=0.3f; la.out_max=2.f; la.d_lpf_hz=3.f; la.slew_per_s=6.f;
      float vs_sp=pid_step(prof_.alt_pos,ps_alt_pos_,alt_sp,alt.alt_m,0,dt,la);
      vs_sp=clampf(vs_sp,-2.5f,2.5f);
      PidLimits lv{}; lv.i_max=0.3f; lv.out_max=0.4f; lv.d_lpf_hz=8.f; lv.slew_per_s=20.f;
      float alt_out=pid_step(prof_.alt_vel,ps_alt_vel_,vs_sp,alt.vs_mps,0,dt,lv);
      thr=clampf(hover_+alt_out,0.05f,0.95f);
    }

    float m0=thr-ur+up-uy, m1=thr+ur+up+uy, m2=thr+ur-up-uy, m3=thr-ur-up+uy;
    float mn=fminf(fminf(m0,m1),fminf(m2,m3));
    float mx=fmaxf(fmaxf(m0,m1),fmaxf(m2,m3));
    if (mn < 0.f){ m0-=mn; m1-=mn; m2-=mn; m3-=mn; mx-=mn; }
    if (mx > 1.f){ float k=1.f/mx; m0*=k; m1*=k; m2*=k; m3*=k; }
    last_motor_[0]=clampf(m0,0.f,1.f);
    last_motor_[1]=clampf(m1,0.f,1.f);
    last_motor_[2]=clampf(m2,0.f,1.f);
    last_motor_[3]=clampf(m3,0.f,1.f);
    (void)mode;
  }
  void learn_hover(float thr_cmd,float vs,float dt){
    if (!isfinite(vs)||!isfinite(dt)) return;
    if (fabsf(vs)>0.5f) return;
    float err=thr_cmd-hover_;
    if (fabsf(err)>0.02f) hover_+=0.01f*err*dt;
    hover_=clampf(hover_,0.25f,0.75f);
  }
  PidState& ps_rate_r(){ return ps_rate_r_; }
  PidState& ps_rate_p(){ return ps_rate_p_; }
  PidState& ps_rate_y(){ return ps_rate_y_; }
private:
  PidProfile prof_{};
  PidState ps_rate_r_,ps_rate_p_,ps_rate_y_;
  PidState ps_ang_r_,ps_ang_p_;
  PidState ps_alt_pos_,ps_alt_vel_;
  PidState ps_pos_n_,ps_pos_e_;
  float hover_=0.5f;
  float last_motor_[4]={0,0,0,0};
  float last_roll_sp_=0.f, last_pitch_sp_=0.f, last_yaw_rate_sp_=0.f;
};
static FlightCtrlV2 g_ctrl;

// ============ CALIBRATION ============
class Calibration {
public:
  CalibResult gyro(float out_b[3],const float prev_b[3],bool have_prev){
    const int N=500; float sx=0,sy=0,sz=0,sxx=0,syy=0,szz=0;
    for (int i=0;i<N;i++){
      float g[3],a[3],t;
      if (!g_imu.read(g,a,t)) return CAL_REJECT_TIMEOUT;
      float gx=g[0]*RAD_TO_DEG,gy=g[1]*RAD_TO_DEG,gz=g[2]*RAD_TO_DEG;
      if (fabsf(gx)>50||fabsf(gy)>50||fabsf(gz)>50) return CAL_REJECT_RANGE;
      sx+=gx; sy+=gy; sz+=gz;
      sxx+=gx*gx; syy+=gy*gy; szz+=gz*gz;
      delay(2);
    }
    float mx=sx/N,my=sy/N,mz=sz/N;
    float vx=sxx/N-mx*mx, vy=syy/N-my*my, vz=szz/N-mz*mz;
    if (vx>25||vy>25||vz>25) return CAL_REJECT_NOISE;
    if (fabsf(mx)>10||fabsf(my)>10||fabsf(mz)>10) return CAL_REJECT_RANGE;
    if (have_prev){
      float d0=fabsf(mx-prev_b[0]*RAD_TO_DEG), d1=fabsf(my-prev_b[1]*RAD_TO_DEG), d2=fabsf(mz-prev_b[2]*RAD_TO_DEG);
      if (d0>lim::GYRO_BIAS_MAX_DELTA_DPS||d1>lim::GYRO_BIAS_MAX_DELTA_DPS||d2>lim::GYRO_BIAS_MAX_DELTA_DPS)
        return CAL_REJECT_DELTA_TOO_BIG;
    }
    out_b[0]=mx*DEG_TO_RAD; out_b[1]=my*DEG_TO_RAD; out_b[2]=mz*DEG_TO_RAD;
    return CAL_OK;
  }
  CalibResult accel(float out_b[3],const float prev_b[3],bool have_prev){
    const int N=200; float sx=0,sy=0,sz=0;
    for (int i=0;i<N;i++){
      float g[3],a[3],t;
      if (!g_imu.read(g,a,t)) return CAL_REJECT_TIMEOUT;
      sx+=a[0]; sy+=a[1]; sz+=a[2];
      delay(2);
    }
    sx/=N; sy/=N; sz/=N;
    float m=sqrtf(sx*sx+sy*sy+sz*sz);
    if (fabsf(m-1.f)>0.15f) return CAL_REJECT_NOT_LEVEL;
    float bx=sx, by=sy, bz=sz-1.f;
    if (have_prev){
      if (fabsf(bx-prev_b[0])>lim::ACCEL_BIAS_MAX_DELTA_G||fabsf(by-prev_b[1])>lim::ACCEL_BIAS_MAX_DELTA_G||fabsf(bz-prev_b[2])>lim::ACCEL_BIAS_MAX_DELTA_G)
        return CAL_REJECT_DELTA_TOO_BIG;
    }
    out_b[0]=bx; out_b[1]=by; out_b[2]=bz;
    return CAL_OK;
  }
  CalibResult mag(float off[3],float sc[3],QMC5883LCompass& q,const float prev_off[3],const float prev_sc[3],bool have_prev){
    const int N=800;
    float mn[3]={1e6f,1e6f,1e6f}, mx[3]={-1e6f,-1e6f,-1e6f};
    for (int i=0;i<N;i++){
      q.read();
      float v[3]={(float)q.getX(),(float)q.getY(),(float)q.getZ()};
      for (int k=0;k<3;k++){ if (v[k]<mn[k]) mn[k]=v[k]; if (v[k]>mx[k]) mx[k]=v[k]; }
      delay(5);
    }
    float span[3];
    for (int k=0;k<3;k++){ span[k]=mx[k]-mn[k]; if (span[k]<20.f) return CAL_REJECT_MAG_BAD; }
    float avg=(span[0]+span[1]+span[2])/3.f;
    for (int k=0;k<3;k++){
      off[k]=(mx[k]+mn[k])*0.5f; sc[k]=avg/span[k];
      if (sc[k]<lim::MAG_SCALE_MIN||sc[k]>lim::MAG_SCALE_MAX) return CAL_REJECT_MAG_BAD;
    }
    if (have_prev){
      float dmax=0.f;
      for (int k=0;k<3;k++) dmax=fmaxf(dmax, fabsf(sc[k]-prev_sc[k]));
      if (dmax > 0.4f) return CAL_REJECT_DELTA_TOO_BIG;
      (void)prev_off;
    }
    return CAL_OK;
  }
  CalibResult ina_cal(float measured_a,float reported_a,uint16_t& out_reg){
    if (reported_a < 0.1f) return CAL_REJECT_RANGE;
    float ratio = measured_a / reported_a;
    if (ratio < 0.3f || ratio > 3.0f) return CAL_REJECT_RANGE;
    uint16_t base = 4096;
    float scaled = base * ratio;
    if (scaled < 1000 || scaled > 65535) return CAL_REJECT_RANGE;
    out_reg = (uint16_t)scaled;
    return CAL_OK;
  }
};
static Calibration g_calib;

// ============ CALIB WIZARD ============
struct CalibWizard {
  CalibState state = CS_IDLE;
  uint8_t progress = 0, which = 0;
  uint32_t t_start = 0, t_result = 0;
  CalibResult result = CAL_OK;
  void reset(){ state=CS_IDLE; progress=0; which=0; result=CAL_OK; t_result=0; }
  bool busy() const { return state==CS_REQ || state==CS_IN_PROGRESS; }
};
static CalibWizard g_calwiz;

// ============ AUTOTUNE ============
enum TunePhase : uint8_t { AT_IDLE=0,AT_INIT,AT_EXCITE,AT_MEASURE,AT_EVALUATE,AT_ADJUST,AT_DONE,AT_ABORTED };
class AutoTune {
public:
  void begin(const PidProfile& base){ base_=base; tuned_=base; trial_=base; phase_=AT_IDLE; }
  void start(uint8_t ax,uint32_t now){
    ax_=ax; trial_=base_; tuned_=base_; iter_=0; gi_=0;
    best_=1e9f; resp_n_=0; phase_=AT_INIT; t0_=now;
  }
  void abort(){ phase_=AT_ABORTED; tuned_=base_; }
  bool running() const { return phase_!=AT_IDLE && phase_!=AT_DONE && phase_!=AT_ABORTED; }
  TunePhase phase() const { return phase_; }
  const PidProfile& result() const { return tuned_; }
  void tick(const EkfState& e,float dt,uint32_t now,bool safe_bench){
    if (!safe_bench){ if (running()) abort(); return; }
    switch (phase_){
      case AT_INIT: resp_n_=0; phase_=AT_EXCITE; t0_=now; break;
      case AT_EXCITE: if (now-t0_>200){ phase_=AT_MEASURE; t0_=now; } break;
      case AT_MEASURE:
        if (resp_n_<400){
          float y=(ax_==TA_ROLL)?e.roll:(ax_==TA_PITCH)?e.pitch:e.yaw;
          if (isfinite(y)) resp_[resp_n_++]=y;
        }
        if (now-t0_>1500) phase_=AT_EVALUATE;
        break;
      case AT_EVALUATE: {
        float c=cost();
        if (iter_==0){ best_=c; tuned_=trial_; }
        else if (c<best_*0.98f){ best_=c; tuned_=trial_; }
        else trial_=tuned_;
        phase_=AT_ADJUST; break;
      }
      case AT_ADJUST:
        iter_++;
        if (iter_>=6){ phase_=AT_DONE; break; }
        gi_=(iter_-1)%3;
        apply_gain(1.15f);
        resp_n_=0; phase_=AT_EXCITE; t0_=now;
        break;
      default: break;
    }
    (void)dt;
  }
private:
  PidProfile base_{},trial_{},tuned_{};
  uint8_t ax_=TA_ROLL; TunePhase phase_=AT_IDLE;
  uint8_t iter_=0,gi_=0; uint32_t t0_=0;
  float resp_[400]{}; uint16_t resp_n_=0; float best_=1e9f;
  void apply_gain(float f){
    PidGains* g = (ax_==TA_ROLL)?&trial_.rate_roll : (ax_==TA_PITCH)?&trial_.rate_pitch : &trial_.rate_yaw;
    switch (gi_){
      case 0: g->kp=clampf(g->kp*f,0.4f,6.f); break;
      case 1: g->kd=clampf(g->kd*f,0.f,0.20f); break;
      case 2: g->ki=clampf(g->ki*f,0.f,0.40f); break;
    }
  }
  float cost() const {
    if (resp_n_<20) return 1e9f;
    float dt=0.005f;
    float iae=0,max_y=0;
    float sgn=(resp_[10]>0)?1.f:-1.f;
    int peak=0;
    for (uint16_t i=0;i<resp_n_;i++){
      float y=resp_[i]*sgn;
      if (y>max_y){ max_y=y; peak=i; }
      iae+=fabsf(y)*dt;
    }
    float step=1.f;
    float over=(max_y>step)?(max_y-step)/step:0.f;
    float osc=0;
    for (uint16_t i=peak+20;i<resp_n_;i++){ float y=resp_[i]*sgn; osc+=y*y*dt; }
    uint16_t settle=resp_n_;
    for (uint16_t i=0;i+40<resp_n_;i++){
      bool ok=true;
      for (uint16_t j=i;j<i+40;j++){ float y=resp_[j]*sgn; if (fabsf(y-step)>0.05f){ ok=false; break; } }
      if (ok){ settle=i; break; }
    }
    float ss=0; uint16_t nn=0;
    for (uint16_t i=resp_n_>40?resp_n_-40:0; i<resp_n_; i++){ ss+=resp_[i]*sgn; nn++; }
    float sse = nn ? fabsf(1.f - ss/nn) : 0.f;
    return iae + over*3.f + osc*2.f + settle*dt*0.5f + sse*1.5f;
  }
};
class AutoTuneSeq {
public:
  AutoTune tune;
  bool active=false, all_mode=false;
  uint8_t current_axis=0, last_axis=0;
  void start(uint8_t axis_arg,const PidProfile& base_prof,uint32_t now){
    tune.begin(base_prof);
    if (axis_arg==TA_ALL){ all_mode=true; current_axis=0; last_axis=2; }
    else { all_mode=false; current_axis=axis_arg; last_axis=axis_arg; }
    active=true; tune.start(current_axis, now);
  }
  void abort(){ tune.abort(); active=false; }
  bool running() const { return active && tune.running(); }
  TunePhase phase() const { return tune.phase(); }
  const PidProfile& result() const { return tune.result(); }
  bool tick(const EkfState& e,float dt,uint32_t now,bool safe_bench){
    if (!active) return false;
    tune.tick(e,dt,now,safe_bench);
    if (tune.phase()==AT_DONE){
      if (all_mode && current_axis < last_axis){
        current_axis++;
        tune.begin(tune.result());
        tune.start(current_axis, now);
        return false;
      } else { active=false; return true; }
    }
    if (tune.phase()==AT_ABORTED){ active=false; return true; }
    return false;
  }
};
static AutoTuneSeq g_autotune;

// ============ ADAPTIVE PID (rev 6.5: save-if-improved) ============
class AdaptivePid {
public:
  void begin(const PidProfile& b){
    base_=b; cur_=b; last_=(uint32_t)millis(); reset_acc();
    baseline_quality_=0.f; best_quality_=1e9f; good_windows_=0; pending_save_=false;
  }
  void set_baseline(const PidProfile& p){
    base_=p; cur_=p; reset_acc();
    baseline_quality_=0.f; best_quality_=1e9f; good_windows_=0;
  }
  const PidProfile& profile() const { return cur_; }
  const PidProfile& baseline() const { return base_; }
  int16_t last_adjust_pct() const { return last_adjust_pct_; }

  bool improvement_ready() const { return pending_save_; }
  void clear_improvement(){ pending_save_=false; good_windows_=0; }

  struct Snapshot {
    uint32_t t_ms;
    float roll_kp, roll_ki, roll_kd;
    float pitch_kp, pitch_ki, pitch_kd;
    float yaw_kp, yaw_ki, yaw_kd;
    float quality, baseline_quality;
    int16_t adjust_pct;
    uint8_t strength;
    uint8_t _pad;
  };
  Snapshot snapshot() const {
    Snapshot s{};
    s.t_ms = (uint32_t)millis();
    s.roll_kp  = cur_.rate_roll.kp;  s.roll_ki  = cur_.rate_roll.ki;  s.roll_kd  = cur_.rate_roll.kd;
    s.pitch_kp = cur_.rate_pitch.kp; s.pitch_ki = cur_.rate_pitch.ki; s.pitch_kd = cur_.rate_pitch.kd;
    s.yaw_kp   = cur_.rate_yaw.kp;   s.yaw_ki   = cur_.rate_yaw.ki;   s.yaw_kd   = cur_.rate_yaw.kd;
    s.quality = best_quality_;
    s.baseline_quality = baseline_quality_;
    s.adjust_pct = last_adjust_pct_;
    s.strength = strength;
    return s;
  }

  bool enabled = false;
  uint8_t strength = 0;
  uint8_t max_adjust_pct = lim::AUTO_PID_MAX_ADJUST_PCT;
  uint16_t update_ms = lim::AUTO_PID_UPDATE_MS;
  uint16_t smoothing_ms = lim::AUTO_PID_SMOOTHING_MS;

  void tick(float re,float pe,float ye,uint32_t now,bool safe){
    if (!enabled || !safe || strength==0){
      cur_ = base_;
      reset_acc();
      healthy_ = true;
      last_adjust_pct_ = 0;
      return;
    }
    if (!isfinite(re)||!isfinite(pe)||!isfinite(ye)){ reset_acc(); return; }
    float e[3]={re,pe,ye};
    for (int i=0;i<3;i++){
      if (!isfinite(e[i])){ reset_acc(); return; }
      esq_[i]+=e[i]*e[i];
      if ((e[i]>0)!=(prev_[i]>0) && fabsf(e[i]-prev_[i])>1e-4f) sc_[i]++;
      prev_[i]=e[i];
    }
    n_++;
    if (now-last_ < update_ms) return;
    last_ = now;
    run();
  }
private:
  PidProfile base_{},cur_{};
  float esq_[3]={},prev_[3]={};
  uint16_t sc_[3]={};
  uint32_t n_=0,last_=0;
  bool healthy_=true;
  int16_t last_adjust_pct_=0;

  float baseline_quality_=0.f;
  float best_quality_=1e9f;
  uint8_t good_windows_=0;
  bool pending_save_=false;

  void reset_acc(){ esq_[0]=esq_[1]=esq_[2]=0; sc_[0]=sc_[1]=sc_[2]=0; n_=0; }

  static float bounded_pct(float delta_pct, float max_pct){
    if (delta_pct >  max_pct) return  max_pct;
    if (delta_pct < -max_pct) return -max_pct;
    return delta_pct;
  }

  void run(){
    if (n_ < 200){ reset_acc(); return; }
    float rms[3];
    for (int i=0;i<3;i++) rms[i]=sqrtf(esq_[i]/n_);
    if (!isfinite(rms[0])||!isfinite(rms[1])||!isfinite(rms[2])){ reset_acc(); cur_=base_; return; }

    float quality = rms[0] + rms[1] + rms[2];
    quality += 0.001f * (float)(sc_[0] + sc_[1] + sc_[2]);

    if (baseline_quality_ == 0.f){
      baseline_quality_ = quality;
      best_quality_ = quality;
    }

    PidGains* cur[3]={ &cur_.rate_roll,&cur_.rate_pitch,&cur_.rate_yaw };
    const PidGains* base[3]={ &base_.rate_roll,&base_.rate_pitch,&base_.rate_yaw };

    float S = clampf((float)strength / 100.f, 0.f, 1.f);
    float kp_step_frac = 0.03f * S;
    float kd_step_frac = 0.015f * S;
    float kp_scale_back = 0.03f * S;

    int16_t worst_pct = 0;
    for (int i=0;i<3;i++){
      bool osc = sc_[i] > 40;
      float delta_pct = 0.f;
      if (osc){
        float old_kp = cur[i]->kp;
        cur[i]->kp = clampf(cur[i]->kp*(1.f - kp_scale_back),
                            base[i]->kp*(1.f - (float)max_adjust_pct/100.f),
                            base[i]->kp*(1.f + (float)max_adjust_pct/100.f));
        cur[i]->kd = clampf(cur[i]->kd*(1.f + kd_step_frac),
                            base[i]->kd*(1.f - (float)max_adjust_pct/100.f),
                            base[i]->kd*(1.f + (float)max_adjust_pct/100.f));
        if (old_kp > 1e-6f) delta_pct = ((cur[i]->kp - old_kp)/old_kp)*100.f;
      } else if (rms[i] > 0.15f){
        float old_kp = cur[i]->kp;
        cur[i]->kp = clampf(cur[i]->kp*(1.f + kp_step_frac),
                            base[i]->kp*(1.f - (float)max_adjust_pct/100.f),
                            base[i]->kp*(1.f + (float)max_adjust_pct/100.f));
        if (old_kp > 1e-6f) delta_pct = ((cur[i]->kp - old_kp)/old_kp)*100.f;
      } else if (rms[i] < 0.05f){
        float old_kp = cur[i]->kp;
        cur[i]->kp += (base[i]->kp - cur[i]->kp) * 0.10f;
        cur[i]->kd += (base[i]->kd - cur[i]->kd) * 0.10f;
        if (old_kp > 1e-6f) delta_pct = ((cur[i]->kp - old_kp)/old_kp)*100.f;
      }
      delta_pct = bounded_pct(delta_pct, (float)max_adjust_pct);
      if (fabsf(delta_pct) > fabsf((float)worst_pct)) worst_pct = (int16_t)delta_pct;
    }
    last_adjust_pct_ = worst_pct;

    float hi = 1.f + (float)max_adjust_pct/100.f;
    float lo = 1.f - (float)max_adjust_pct/100.f;
    for (int i=0;i<3;i++){
      cur[i]->kp = clampf(cur[i]->kp, base[i]->kp*lo, base[i]->kp*hi);
      cur[i]->ki = clampf(cur[i]->ki, base[i]->ki*lo, base[i]->ki*hi);
      cur[i]->kd = clampf(cur[i]->kd, base[i]->kd*lo, base[i]->kd*hi);
    }

    // --- Deteksi improvement (rev 6.5) ---
    if (quality < baseline_quality_ * 0.95f && quality < best_quality_ * 0.98f){
      if (good_windows_ < 255) good_windows_++;
      if (good_windows_ >= 3){
        best_quality_ = quality;
        pending_save_ = true;
        good_windows_ = 0;
      }
    } else {
      good_windows_ = 0;
    }

    reset_acc();
  }
};
static AdaptivePid g_adapt;

// ---- Definisikan adapt_log_to_sd sekarang (setelah AdaptivePid) ----
static void adapt_log_to_sd(const AdaptivePid::Snapshot& s){
  if (!g_sd_ok) return;
  File f = SD.open("/adapt.bin", "a");
  if (!f) return;
  f.write((const uint8_t*)&s, sizeof(s));
  f.close();
}

// ============ SELF TEST ============
struct SelfTest {
  bool mcu_ok=false,i2c_ok=false,imu_ok=false,baro_ok=false,mag_ok=false,
       gps_ok=false,batt_ok=false,tof_ok=false,rA_ok=false,
       esc_ok=false,storage_ok=false,cfg_ok=false,calib_ok=false,fw_ok=false,sd_ok=false;
  uint8_t result=2;
  void evaluate(const HealthReg& h,const LinkRadio& r,bool cfg_valid,bool fw){
    mcu_ok=true;
    i2c_ok=h.usable(S_IMU)||h.usable(S_BARO);
    imu_ok=h.usable(S_IMU)&&h.usable(S_ACCEL)&&h.usable(S_GYRO);
    baro_ok=h.usable(S_BARO); mag_ok=h.usable(S_MAG);
    gps_ok=h.usable(S_GPS); batt_ok=h.usable(S_BATT);
    tof_ok=h.usable(S_TOF);
    rA_ok=r.st()!=LS_INIT; esc_ok=true;
    storage_ok=g_bb.healthy(); cfg_ok=cfg_valid; calib_ok=cfg_valid; fw_ok=fw;
    sd_ok = g_sd_ok;
    int fails=0,warns=0;
    bool crit_arr[]={!mcu_ok,!i2c_ok,!imu_ok,!baro_ok,!batt_ok,!storage_ok,!cfg_ok,!fw_ok};
    for (bool b : crit_arr) if (b) fails++;
    bool warn_arr[]={!mag_ok,!gps_ok,!rA_ok,!esc_ok,!calib_ok,!tof_ok,!sd_ok};
    for (bool b : warn_arr) if (b) warns++;
    result = fails ? 2 : (warns ? 1 : 0);
  }
};
static SelfTest g_selftest;

// ============ BUZZER ============
class Buzzer {
public:
  void begin(){ pinMode(PIN_BUZZER,OUTPUT); digitalWrite(PIN_BUZZER,LOW); }
  void boot(){ beep(2000,80); delay(120); beep(2500,80); }
  void arm(){ beep(1500,100); }
  void disarm(){ beep(1000,200); }
  void warn(){ beep(1800,80); delay(100); beep(1800,80); }
  void crit(){ for (int i=0;i<5;i++){ beep(2800,60); delay(60); } }
  void loiter(){ beep(2200,60); }
  void sd_ok(){ beep(2600,50); delay(60); beep(2600,50); }
  void sd_fail(){ beep(400,150); delay(80); beep(400,150); }
  void tick(uint32_t now){ if (!active_) return; if (now>=end_ms_){ digitalWrite(PIN_BUZZER,LOW); active_=false; } }
private:
  void beep(uint16_t hz,uint16_t ms){ tone(PIN_BUZZER,hz,ms); active_=true; end_ms_ = (uint32_t)(millis() + ms + 10); }
  bool active_=false; uint32_t end_ms_=0;
};
static Buzzer g_buzz;

// ============ MOTOR OUTPUT ============
static bool g_pwm_init=false;
static void motor_init(){
  if (g_pwm_init) return;
  analogWriteFreq(400);
  analogWriteRange(2500);
  for (int i=0;i<4;i++){
    pinMode(PIN_MOTOR[i], OUTPUT);
    analogWrite(PIN_MOTOR[i], 1000);
  }
  g_pwm_init=true;
}
static void motor_write(const float out[4],bool enabled){
#if NEXEND_BENCH_MODE
  (void)out; (void)enabled;
  enabled = false;
#endif
  for (int i=0;i<4;i++){
    uint32_t us=1000;
    if (enabled){
      float v = clampf(out[i],0.f,1.f);
      if (!isfinite(v)) v = 0.f;
      us = 1000 + (uint32_t)(v*1000.f);
    }
    if (us<1000) us=1000;
    if (us>2000) us=2000;
    analogWrite(PIN_MOTOR[i], (int)us);
  }
}

// ============ SENSOR CACHE ============
struct SensCache {
  float gyro[3]={0,0,0}, accel[3]={0,0,1}, temp_c=0;
  float baro_alt=0, baro_alt_last=0; uint32_t baro_last_ms=0;
  float mag[3]={0,0,0}; uint32_t mag_last_ms=0;
  float batt_v=0,batt_a=0; uint32_t batt_last_ms=0;
  double lat=0,lon=0; float gs_mps=0; uint8_t sats=0,fix=0;
  float hdop=99.9f; uint32_t gps_last_ms=0;
  float tof_m=NAN; uint32_t tof_last_ms=0;
};
static SensCache g_sens;

static Adafruit_BMP280 g_baro;
static Adafruit_INA219  g_ina(ADDR_INA);
static QMC5883LCompass  g_mag;
static TinyGPSPlus      g_gps;
#define g_gps_ser Serial2

// ============ SENSOR READERS ============
static void read_imu(uint32_t now){
  float g[3],a[3],t;
  if (!g_imu.read(g,a,t)){ g_health.bad(S_IMU,now,false); i2c_note_error(now); return; }
  if (!is_finite_arr(g,3) || !is_finite_arr(a,3)){ g_health.bad(S_IMU,now,false); return; }

  if (g_cfg.valid()){
    a[0] -= g_cfg.c().accel_bias[0];
    a[1] -= g_cfg.c().accel_bias[1];
    a[2] -= g_cfg.c().accel_bias[2];
  }

  g_sens.gyro[0]=g[0]; g_sens.gyro[1]=g[1]; g_sens.gyro[2]=g[2];
  g_sens.accel[0]=a[0]; g_sens.accel[1]=a[1]; g_sens.accel[2]=a[2];
  g_sens.temp_c=t;
  float m=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  if (m>0.5f && m<2.5f){
    g_health.ok(S_IMU,now,100);
    g_health.ok(S_ACCEL,now,100);
    g_health.ok(S_GYRO,now,100);
    g_i2c_err_streak = (g_i2c_err_streak>0)?g_i2c_err_streak-1:0;
  } else g_health.bad(S_ACCEL,now,false);
}
static void read_baro(uint32_t now){
  float p=g_baro.readPressure();
  float t=g_baro.readTemperature();
  if (isfinite(p) && isfinite(t) && p>30000 && p<110000 && t>-40 && t<85){
    float alt=g_baro.readAltitude(1013.25f);
    if (isfinite(alt)){
      float dt = 0.02f;
      if (g_sens.baro_last_ms){
        uint32_t dms=now-g_sens.baro_last_ms;
        if (dms>0 && dms<500) dt = dms*0.001f;
      }
      if (g_sens.baro_last_ms){
        uint32_t dms=now-g_sens.baro_last_ms;
        if (dms>0) g_alt.st().vs_mps=(alt-g_sens.baro_alt_last)/(dms*0.001f);
      }
      g_sens.baro_alt_last=alt; g_sens.baro_alt=alt; g_sens.baro_last_ms=now;
      g_alt.update_baro(alt, dt);
      g_health.ok(S_BARO,now,100);
      return;
    }
  }
  g_health.bad(S_BARO,now,false);
}
static void read_mag(uint32_t now,QMC5883LCompass& q){
  q.read();
  float mx=(float)q.getX(), my=(float)q.getY(), mz=(float)q.getZ();
  if (g_cfg.valid()){
    mx=(mx-g_cfg.c().mag_offset[0])*g_cfg.c().mag_scale[0];
    my=(my-g_cfg.c().mag_offset[1])*g_cfg.c().mag_scale[1];
    mz=(mz-g_cfg.c().mag_offset[2])*g_cfg.c().mag_scale[2];
  }
  float tmp[3]={mx,my,mz};
  if (!is_finite_arr(tmp,3)){ g_health.bad(S_MAG,now,false); return; }
  g_sens.mag[0]=mx; g_sens.mag[1]=my; g_sens.mag[2]=mz; g_sens.mag_last_ms=now;
  float m=sqrtf(mx*mx+my*my+mz*mz);
  if (m>10.f && m<200.f) g_health.ok(S_MAG,now,90);
  else g_health.bad(S_MAG,now,false);
}
static void read_batt(uint32_t now){
  float v=g_ina.getBusVoltage_V();
  float a=g_ina.getCurrent_mA()/1000.f;
  if (isfinite(v) && isfinite(a) && v>3.f && v<60.f){
    g_sens.batt_v=v; g_sens.batt_a=a; g_sens.batt_last_ms=now;
    g_health.ok(S_BATT,now,100);
    g_batt.set_current(a);
    g_batt.update(v,a,now,true);
  } else g_health.bad(S_BATT,now,true);
}
static void read_gps(uint32_t now,TinyGPSPlus& gps){
  while (g_gps_ser.available()) gps.encode(g_gps_ser.read());
  if (gps.location.isUpdated() && gps.location.isValid() && gps.location.age()<lim::GPS_AGE_MS){
    g_sens.lat=gps.location.lat(); g_sens.lon=gps.location.lng();
    g_sens.sats=gps.satellites.value();
    g_sens.fix=(gps.location.age()<1000)?3:0;
    g_sens.gs_mps=gps.speed.mps();
    g_sens.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9f;
    g_sens.gps_last_ms=now;
    uint8_t conf = (uint8_t)min(100, (int)g_sens.sats*5);
    if (g_sens.hdop > lim::HDOP_MAX) conf = conf/2;
    g_health.ok(S_GPS,now,conf);
  } else if (gps.location.age() > 3000) {
    g_health.bad(S_GPS,now,false);
  }
}
#if NEXEND_HAVE_TOF
static void read_tof(uint32_t now){
  static uint32_t s_last_ms = 0;
  if (!g_tof_ready) return;   // [BOOTFIX] sensor tidak ada/gagal init -> jangan dibaca (bisa nunggu tanpa batas)
  if (now - s_last_ms < TOF_READ_MIN_MS) return;
  s_last_ms = now;
  uint16_t mm = g_tof.readRangeContinuousMillimeters();
  if (g_tof.timeoutOccurred()){ g_health.bad(S_TOF,now,false); return; }
  if (mm < TOF_MM_MIN || mm > TOF_MM_MAX){ g_health.bad(S_TOF,now,false); return; }
  float m = mm / 1000.0f;
  if (!isfinite(m) || m < (TOF_MM_MIN/1000.0f) || m > (TOF_MM_MAX/1000.0f)){
    g_health.bad(S_TOF,now,false); return;
  }
  g_sens.tof_m = m; g_sens.tof_last_ms = now;
  g_alt.update_tof(m);
  g_health.ok(S_TOF,now,100);
}
#endif

// ============ EVENT QUEUE ============
struct EventQ { PktEvent ev[16]; uint8_t head=0,tail=0; };
static EventQ g_evq;
static void post_event(uint8_t sev,uint8_t id,uint16_t arg){
  g_bb.event(sev,id,arg);
  PktEvent e{};
  e.h.s0=NEX_SYNC0; e.h.s1=NEX_SYNC1; e.h.ver=NEX_PROTO_VER; e.h.type=PKT_EVENT;
  static uint8_t seq=0; e.h.seq=seq++;
  e.t_ms=(uint32_t)millis(); e.sev=sev; e.ev=id; e.arg=arg;
  nex_seal(e);
  g_evq.ev[g_evq.head]=e;
  g_evq.head=(g_evq.head+1)&15;
  if (g_evq.head==g_evq.tail) g_evq.tail=(g_evq.tail+1)&15;
}

// ============ COMMAND HANDLER ============
static void send_ack(uint8_t cmd,uint8_t result){
  PktAck a{};
  a.h.s0=NEX_SYNC0; a.h.s1=NEX_SYNC1; a.h.ver=NEX_PROTO_VER; a.h.type=PKT_ACK;
  static uint8_t seq=0; a.h.seq=seq++;
  a.acked=cmd; a.result=result;
  nex_seal(a); g_radio.send(&a,sizeof(a));
}
static void calib_wizard_start(uint8_t cmd);
static void apply_active_profile_to_runtime();

static GeoPoint g_home{};
static bool     g_home_valid=false;
static float    g_home_dist=0;

static void handle_cmd(const PktCmd& c,QMC5883LCompass& q){
  if (!nex_verify(c)) return;
  bool armed = g_arming.st() != AS_DISARMED;

  switch (c.cmd){
    case CMD_CALIB_GYRO:
    case CMD_CALIB_ACCEL:
    case CMD_CALIB_MAG:
      if (armed){ send_ack(c.cmd, (uint8_t)CAL_REJECT_ARMED); break; }
      if (!g_calwiz.busy()) { calib_wizard_start(c.cmd); send_ack(c.cmd,0); }
      else send_ack(c.cmd,1);
      break;
    case CMD_CALIB_ABORT:
      g_calwiz.reset(); post_event(WL_INFO,EV_CALIB,(uint8_t)CAL_REJECT_ABORT); send_ack(c.cmd,0); break;
    case CMD_CALIB_INA: {
      uint16_t meas=(uint16_t)((c.arg>>16)&0xFFFF), rep=(uint16_t)(c.arg&0xFFFF);
      float meas_a=meas/100.f, rep_a=rep/100.f;
      uint16_t reg=0;
      CalibResult r=g_calib.ina_cal(meas_a,rep_a,reg);
      if (r==CAL_OK && g_cfg.valid()){
        g_cfg.c().ina_cal_value=reg; g_cfg.c().ina_cal_ok=1;
        g_cfg.save(g_cfg.c());
        g_batt.set_cal(true); g_warn.clear(W_INA_UNCAL);
        Wire.beginTransmission(ADDR_INA);
        Wire.write(0x05);
        Wire.write((uint8_t)((reg>>8)&0xFF));
        Wire.write((uint8_t)(reg&0xFF));
        Wire.endTransmission();
      }
      post_event(WL_INFO,EV_CALIB,(uint8_t)r); send_ack(c.cmd,(uint8_t)r); break;
    }
    case CMD_SET_BATT_CAP: {
      if (g_cfg.valid()){
        uint16_t cap=(uint16_t)clampf((float)c.arg,200.f,20000.f);
        g_cfg.c().batt_capacity_mah=cap; g_cfg.save(g_cfg.c()); send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_SET_CHEM: {
      if (g_cfg.valid()){
        uint8_t chem=(uint8_t)clampf((float)c.arg,0.f,4.f);
        g_cfg.c().batt_chem=chem; g_cfg.save(g_cfg.c());
        g_batt.set_chemistry((BattChem)chem);
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_RESET_HOME: {
      g_home_valid = false;
      g_home = GeoPoint{};
      g_home_dist = 0;
      g_pos.reset();
      post_event(WL_INFO, EV_HOME_RESET, 0);
      send_ack(c.cmd,0);
      break;
    }
    case CMD_SET_PROFILE: {
      if (c.arg>=0 && c.arg<3 && g_cfg.valid()){
        g_cfg.c().active_profile=(uint8_t)c.arg;
        g_cfg.c().pid=g_cfg.c().pid_profiles[c.arg];
        g_cfg.save(g_cfg.c());
        apply_active_profile_to_runtime();
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_SET_PID: {
      if (armed){ send_ack(c.cmd,1); break; }
      if (g_cfg.valid()){
        if (c.idx==0) g_cfg.c().pid.rate_roll.kp  = clampf((float)c.arg/1000.f,0.4f,6.f);
        if (c.idx==1) g_cfg.c().pid.rate_roll.kd  = clampf((float)c.arg/1000.f,0.f,0.2f);
        if (c.idx==2) g_cfg.c().pid.rate_pitch.kp = clampf((float)c.arg/1000.f,0.4f,6.f);
        if (c.idx==3) g_cfg.c().pid.rate_pitch.kd = clampf((float)c.arg/1000.f,0.f,0.2f);
        g_cfg.save(g_cfg.c());
        apply_active_profile_to_runtime();
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_AUTOTUNE_START: {
      if (g_arming.st()==AS_DISARMED && !g_motordiag.fault()){
        uint8_t axis=(c.arg>=0 && c.arg<=TA_ALL)?(uint8_t)c.arg:TA_ROLL;
        g_autotune.start(axis,g_ctrl.profile(),(uint32_t)millis());
        post_event(WL_INFO,EV_AUTOTUNE_START,axis);
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_AUTOTUNE_ABORT:
      g_autotune.abort(); post_event(WL_INFO,EV_AUTOTUNE_ABORT,0); send_ack(c.cmd,0); break;
    case CMD_ADAPTIVE_TOGGLE:
      if (g_cfg.valid()){
        g_cfg.c().auto_stab_enabled = g_cfg.c().auto_stab_enabled?0:1;
        g_cfg.c().adaptive_enabled  = g_cfg.c().auto_stab_enabled;
        g_cfg.save(g_cfg.c());
        g_adapt.enabled = g_cfg.c().auto_stab_enabled!=0;
      }
      send_ack(c.cmd,0); break;
    case CMD_SAVE_CFG:
      if (g_cfg.valid()){ g_cfg.save(g_cfg.c()); send_ack(c.cmd,0); } else send_ack(c.cmd,1);
      break;
    case CMD_LOAD_CFG:
      g_cfg.begin();
      if (g_cfg.valid()){ apply_active_profile_to_runtime(); send_ack(c.cmd,0); } else send_ack(c.cmd,1);
      break;
    case CMD_ERASE_BLACKBOX:
      if (g_bbread.erase()) send_ack(c.cmd,0); else send_ack(c.cmd,1);
      break;
    case CMD_FACTORY_RESET:
      g_cfg.factory_reset(); g_stats.reset(); send_ack(c.cmd,0); break;
    case CMD_CLEAR_LOCKS:
      g_arb.clear_latches();
      g_crash.clear(); g_motordiag.clear(); g_inv.clear(); g_landing.clear();
      g_takeoff.clear();
      send_ack(c.cmd,0); break;
    case CMD_SET_STABILIZER: {
      if (g_cfg.valid()){
        g_cfg.c().stabilizer_enabled = (c.arg!=0)?1:0;
        g_cfg.c().pid.stabilizer_enabled = g_cfg.c().stabilizer_enabled;
        g_cfg.save(g_cfg.c());
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_SET_AUTO_STAB: {
      if (g_cfg.valid()){
        g_cfg.c().auto_stab_enabled = (c.arg!=0)?1:0;
        g_cfg.c().pid.auto_stab_enabled = g_cfg.c().auto_stab_enabled;
        g_cfg.c().adaptive_enabled = g_cfg.c().auto_stab_enabled;
        g_cfg.save(g_cfg.c());
        g_adapt.enabled = g_cfg.c().auto_stab_enabled!=0;
        if (!g_adapt.enabled){ g_adapt.set_baseline(g_ctrl.profile()); }
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_SET_AUTO_STRENGTH: {
      if (g_cfg.valid()){
        uint8_t s=(uint8_t)clampf((float)c.arg,0.f,100.f);
        g_cfg.c().auto_stab_strength = s;
        g_cfg.c().pid.auto_stab_strength = s;
        g_cfg.save(g_cfg.c());
        g_adapt.strength = s;
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_PID_RESET: {
      if (armed){ send_ack(c.cmd,1); break; }
      if (g_cfg.valid()){
        PidGains d_roll {2.4f,0.06f,0.020f,0.f};
        PidGains d_pitch{2.4f,0.06f,0.020f,0.f};
        PidGains d_yaw  {3.2f,0.03f,0.000f,0.f};
        uint8_t idx=(uint8_t)c.idx;
        if (idx==0 || idx==3) g_cfg.c().pid.rate_roll  = d_roll;
        if (idx==1 || idx==3) g_cfg.c().pid.rate_pitch = d_pitch;
        if (idx==2 || idx==3) g_cfg.c().pid.rate_yaw   = d_yaw;
        g_cfg.save(g_cfg.c());
        apply_active_profile_to_runtime();
        g_adapt.set_baseline(g_ctrl.profile());
        send_ack(c.cmd,0);
      } else send_ack(c.cmd,1);
      break;
    }
    case CMD_PID_SAVE_PROFILE: {
      if (armed){ send_ack(c.cmd,1); break; }
      if (g_cfg.valid()){
        int p=(int)c.arg;
        if (p>=0 && p<3){
          g_cfg.c().pid_profiles[p] = g_cfg.c().pid;
          g_cfg.c().active_profile = (uint8_t)p;
          g_cfg.save(g_cfg.c());
          send_ack(c.cmd,0);
        } else send_ack(c.cmd,1);
      } else send_ack(c.cmd,1);
      break;
    }
    default: send_ack(c.cmd,2); break;
  }
  (void)q;
}

static void apply_active_profile_to_runtime(){
  if (!g_cfg.valid()) return;
  g_ctrl.set_profile(g_cfg.c().pid);
  g_adapt.set_baseline(g_cfg.c().pid);
  g_adapt.enabled  = g_cfg.c().auto_stab_enabled != 0;
  g_adapt.strength = g_cfg.c().auto_stab_strength;
  g_adapt.max_adjust_pct = g_cfg.c().auto_pid_max_adjust;
  g_adapt.update_ms  = g_cfg.c().auto_pid_update_ms;
  g_adapt.smoothing_ms = g_cfg.c().auto_pid_smoothing_ms;
}

// ============ CALIB WIZARD RUNNER ============
static void calib_wizard_start(uint8_t cmd){
  g_calwiz.reset();
  g_calwiz.which = cmd;
  g_calwiz.state = CS_REQ;
  g_calwiz.t_start = (uint32_t)millis();
  post_event(WL_INFO,EV_CALIB, (uint16_t)cmd);
}
static void calib_wizard_tick(){
  if (g_calwiz.state == CS_IDLE) return;
  if (g_arming.st() != AS_DISARMED){
    g_calwiz.reset();
    return;
  }
  uint32_t now = (uint32_t)millis();
  if (g_calwiz.state == CS_REQ){
    if (now - g_calwiz.t_start > 500){ g_calwiz.state = CS_IN_PROGRESS; g_calwiz.t_start = now; }
    return;
  }
  if (g_calwiz.state != CS_IN_PROGRESS) return;
  CalibResult r = CAL_OK;
  if (g_calwiz.which == CMD_CALIB_GYRO){
    float b[3]; bool hp=g_cfg.valid();
    const float* prev = hp?g_cfg.c().gyro_bias:nullptr;
    r = g_calib.gyro(b, prev, hp);
    if (r==CAL_OK){ g_ekf.set_bias(b); if (g_cfg.valid()){ memcpy(g_cfg.c().gyro_bias,b,sizeof(b)); g_cfg.save(g_cfg.c()); } }
  } else if (g_calwiz.which == CMD_CALIB_ACCEL){
    float b[3]; bool hp=g_cfg.valid();
    const float* prev = hp?g_cfg.c().accel_bias:nullptr;
    r = g_calib.accel(b, prev, hp);
    if (r==CAL_OK && g_cfg.valid()){ memcpy(g_cfg.c().accel_bias,b,sizeof(b)); g_cfg.save(g_cfg.c()); }
  } else if (g_calwiz.which == CMD_CALIB_MAG){
    float off[3],sc[3]; bool hp=g_cfg.valid();
    const float* poff = hp?g_cfg.c().mag_offset:nullptr;
    const float* psc  = hp?g_cfg.c().mag_scale :nullptr;
    r = g_calib.mag(off,sc,g_mag,poff,psc,hp);
    if (r==CAL_OK && g_cfg.valid()){
      memcpy(g_cfg.c().mag_offset,off,sizeof(off));
      memcpy(g_cfg.c().mag_scale,sc,sizeof(sc));
      g_cfg.save(g_cfg.c());
    }
  } else r = CAL_REJECT_ABORT;
  g_calwiz.result = r;
  g_calwiz.state = (r==CAL_OK) ? CS_SUCCESS : CS_FAILED;
  g_calwiz.progress = 100;
  g_calwiz.t_result = now;
  post_event(WL_INFO,EV_CALIB,(uint8_t)r);
  if (r == CAL_REJECT_DELTA_TOO_BIG) g_warn.raise(W_CAL_BAD_DELTA,WL_WARNING);
}
static void calib_wizard_autoclear(){
  if (g_calwiz.state==CS_SUCCESS || g_calwiz.state==CS_FAILED || g_calwiz.state==CS_ABORTED){
    if (g_calwiz.t_result == 0) g_calwiz.t_result = (uint32_t)millis();
    if ((uint32_t)millis() - g_calwiz.t_result > 3000) g_calwiz.reset();
  }
}

// ============ CONTROL TICK ============
static uint32_t g_t=0;
static PktRc    g_rc{};
static bool     g_have_rc=false;
static uint32_t g_last_rc_ms=0;
static bool     g_arm_edge=false,g_disarm_edge=false;
static bool     g_arm_latch=false,g_disarm_latch=false;
static float    g_alt_sp=0;
static bool     g_alt_hold=false, g_pos_hold=false;
static bool     g_loiter_active=false;
static FlightMode g_mode=FM_STABILIZE;
static ArbiterOut g_arb_out{};
static ArbiterIn  g_arb_in{};
static FlightState g_last_st=FS_BOOT;
static bool        g_lock_clear_pending=false;
static bool        g_abort_flag=false;
static bool        g_lowbatt_rth_latch=false;
static bool        g_auto_disarm_armed=false;
static uint32_t    g_auto_disarm_t0=0;

// ============================================================================
// [NXL] NEXEND_LINK_V1 — jalur radio ke NEXEND Remote RP2040 (ACK payload)
//  Remote -> FC : paket dinamis (magic 0xA7, hdr 6, payload<=24, CRC16-CCITT LE)
//  FC -> Remote : HANYA lewat ACK payload (FC tidak memancar sendiri)
//  Satu radio nRF24 (ch76, 1Mbps, alamat "NXFC1"). [1R] Radio B sudah dihapus.
//  Dipasang SEBELUM control_tick() agar bisa melihat g_arming/g_fsm/g_ctrl/dll.
// ============================================================================
#define NXL_MAGIC 0xA7
#define NXL_VER   1
#define NXL_HDR   6
#define NXL_MAXPL 24
#define NXL_FC_ID 1

enum : uint8_t {
  NT_HELLO=0x01, NT_HEARTBEAT=0x03, NT_CONTROL=0x10, NT_ARM_REQUEST=0x11, NT_ARM_STATUS=0x12,
  NT_RTH_REQUEST=0x13, NT_RTH_STATUS=0x14,
  NT_TBASIC=0x20, NT_TGPS=0x21, NT_TBATT=0x22, NT_TSENS=0x23, NT_TFLIGHT=0x24, NT_TLINK=0x25,
  NT_CMD_REQ=0x30, NT_CMD_RESP=0x31,
  NT_PID_RD=0x40, NT_PID_RD_R=0x41, NT_PID_WR=0x42, NT_PID_WR_R=0x43,
  NT_CAL_REQ=0x44, NT_CAL_ST=0x45, NT_LOG_REQ=0x46, NT_ERR=0x7F
};
enum : uint32_t {
  NXC_ARM=1UL<<0, NXC_DISARM=1UL<<1, NXC_RTH=1UL<<2, NXC_PID_R=1UL<<3, NXC_PID_W=1UL<<4,
  NXC_CALIB=1UL<<5, NXC_HOME=1UL<<8, NXC_RTH_ALT=1UL<<14,
  NXC_T_BATT=1UL<<9, NXC_T_GPS=1UL<<10, NXC_T_SENS=1UL<<11, NXC_T_FLIGHT=1UL<<12, NXC_T_LINK=1UL<<13
};
// Fitur yang BELUM diimplementasikan di FC sengaja tidak diiklankan (LOG_READ/LOG_CLEAR).
static constexpr uint32_t NXL_CAPS = NXC_ARM|NXC_DISARM|NXC_RTH|NXC_PID_R|NXC_PID_W|NXC_CALIB|NXC_HOME|NXC_RTH_ALT|
                                     NXC_T_BATT|NXC_T_GPS|NXC_T_SENS|NXC_T_FLIGHT|NXC_T_LINK;

#pragma pack(push,1)
struct XBasic  { uint8_t fcState,flightMode,preflight,failsafe,rth,homeSet; uint16_t flightTimeS,errFlags; uint8_t armReject,fwMaj,fwMin; };
struct XBatt   { uint16_t mV; int16_t mA; uint16_t mAh; uint8_t pct,status; };
struct XGps    { uint8_t fix,sats; int32_t lat,lon; uint16_t hdop; int16_t altDm; uint16_t speedCms; };
struct XSens   { uint16_t present,healthy,calibrated,errors,dataAgeMs; };
struct XFlight { int16_t altDm; uint8_t altSrc; uint16_t speedCms,homeDistM; int16_t headingDeg,rollDd,pitchDd,yawDd; uint16_t thrCmd; };
struct XLink   { uint16_t rxOk,rxBad,ctrlGap; uint8_t linkState,failsafe; };
#pragma pack(pop)
static_assert(sizeof(XBasic)==13 && sizeof(XBatt)==8 && sizeof(XGps)==16 && sizeof(XSens)==10 &&
              sizeof(XFlight)==17 && sizeof(XLink)==8, "ukuran struct telemetri harus sama dengan remote");

struct NxlResp { uint8_t type,req,len; uint8_t pl[NXL_MAXPL]; };
struct NxlState {
  NxlResp q[4]; uint8_t qh=0,qt=0;
  uint8_t txSeq=0,lastRxSeq=0; bool haveRx=false;
  uint8_t want=NT_TBASIC;
  uint16_t rxOk=0,rxBad=0,ctrlGap=0; uint32_t lastCtrlMs=0;
  int16_t ax[4]={0,0,0,0}; uint8_t flags=0;
  bool armPend=false; uint8_t armReq=0; uint32_t armT0=0;
  uint32_t disarmUntil=0,rthUntil=0;
};
static NxlState g_nxl;
static ArmChecks g_last_ck;   // diisi control_tick() -> dipakai untuk flag "preflight"

static void nxl_push(uint8_t type,uint8_t req,const void* pl,uint8_t len){
  if (len>NXL_MAXPL) return;
  uint8_t nh=(g_nxl.qh+1)&3; if (nh==g_nxl.qt) return;        // antrean penuh: remote akan retry/timeout
  NxlResp& r=g_nxl.q[g_nxl.qh]; r.type=type; r.req=req; r.len=len; if (len) memcpy(r.pl,pl,len);
  g_nxl.qh=nh;
}
static void nxl_err(uint8_t req,uint8_t type,uint8_t code,uint8_t reason){
  uint8_t p[3]={type,code,reason}; nxl_push(NT_ERR,req,p,3);   // code: 1=UNSUPPORTED 2=REJECTED 3=BUSY 4=BAD_PAYLOAD 5=NOT_READY
}
static uint8_t nxl_frame(uint8_t* b,uint8_t type,uint8_t req,const void* pl,uint8_t len){
  b[0]=NXL_MAGIC; b[1]=NXL_VER; b[2]=type; b[3]=++g_nxl.txSeq; b[4]=req; b[5]=len;
  if (len) memcpy(b+NXL_HDR,pl,len);
  uint16_t c=nex_crc16(b,NXL_HDR+len); b[NXL_HDR+len]=c&0xFF; b[NXL_HDR+len+1]=c>>8;
  return NXL_HDR+len+2;
}
static uint8_t nxl_rej(ArmReject r){   // ArmReject FC -> kode remote (rejName)
  switch (r){
    case AR_OK: return 0;
    case AR_IMU: case AR_ACCEL: case AR_GYRO: return 1;
    case AR_CFG: return 2;
    case AR_BATT: return 3;
    case AR_RADIO_A: case AR_RSV_RADIO_B: case AR_RC: case AR_NO_SYNC: return 4;
    case AR_THROTTLE: return 5;
    case AR_BARO: case AR_MAG: case AR_GPS: case AR_TOF: case AR_ESC: case AR_MOTOR: return 6;
    default: return 8;
  }
}
static uint8_t nxl_fcstate(){          // 1=DISARMED 2=ARMING 3=ARMED 4=FAILSAFE 5=ERROR
  FlightState s=g_fsm.st();
  if (s==FS_FAILSAFE || s==FS_FAILSAFE_DESCEND) return 4;
  if (s==FS_CRITICAL_SENSOR || s==FS_MOTOR_FAULT || s==FS_CRASH || s==FS_INVERTED) return 5;
  if (g_arming.armed()) return 3;
  if (g_arming.st()==AS_PREARM || g_arming.st()==AS_GESTURE) return 2;
  return 1;
}
// [DIR] Tanda arah keluaran tampilan (USB Configurator 0x10/0x11/0x17 + telemetri NXL TFLIGHT).
// TIDAK mempengaruhi kontrol terbang. -1 = dibalik, +1 = normal; ubah per sumbu bila masih terbalik.
static constexpr float USB_SGN_ROLL  = -1.f;
static constexpr float USB_SGN_PITCH = -1.f;
static constexpr float USB_SGN_YAW   = -1.f;
static int16_t nxl_dd(float rad){ float d=rad*RAD_TO_DEG*10.f; if(!isfinite(d)) return 0x7FFF; return (int16_t)clampf(d,-32000.f,32000.f); }

static uint8_t nxl_build_tel(uint8_t type,uint8_t* out){   // isi payload, kembalikan panjang (0 = tidak didukung)
  uint32_t now=(uint32_t)millis();
  switch (type){
    case NT_TBASIC: {
      static uint32_t armT=0; bool a=g_arming.armed(); if (a && !armT) armT=now; if (!a) armT=0;
      XBasic t{}; t.fcState=nxl_fcstate(); t.flightMode=(uint8_t)g_mode;
      t.preflight=(precheck(g_last_ck)==AR_OK)?1:0; t.failsafe=(t.fcState==4)?1:0;
      t.rth=(g_fsm.st()==FS_RTH)?2:0; t.homeSet=g_home_valid?1:0;
      t.flightTimeS=armT?(uint16_t)((now-armT)/1000UL):0; t.errFlags=(uint16_t)g_warn.top_id();
      t.armReject=nxl_rej(g_arming.reject()); t.fwMaj=NEXEND_FW_MAJOR; t.fwMin=NEXEND_FW_MINOR;
      memcpy(out,&t,sizeof(t)); return sizeof(t); }
    case NT_TBATT: {
      XBatt t{}; t.mV=(uint16_t)clampf(g_sens.batt_v*1000.f,0.f,65000.f);
      t.mA=(int16_t)clampf(g_sens.batt_a*1000.f,-32000.f,32000.f); t.mAh=g_batt.mah_used(); t.pct=g_batt.pct();
      t.status=(g_batt.pct()<5)?3:((g_batt.pct()<20)?2:1);
      memcpy(out,&t,sizeof(t)); return sizeof(t); }
    case NT_TGPS: {
      XGps t{}; t.fix=g_sens.fix; t.sats=g_sens.sats; t.lat=(int32_t)(g_sens.lat*1e7); t.lon=(int32_t)(g_sens.lon*1e7);
      t.hdop=(g_sens.hdop<600.f)?(uint16_t)(g_sens.hdop*100.f):0xFFFF; t.altDm=0x7FFF;   // FC tidak menyimpan alt GPS
      t.speedCms=(uint16_t)clampf(g_sens.gs_mps*100.f,0.f,65000.f);
      memcpy(out,&t,sizeof(t)); return sizeof(t); }
    case NT_TSENS: {
      XSens t{}; t.present=0x00DF;                       // imu,baro,mag,gps,tof,(flow tidak ada),current,voltage
      static const SubsysId m[8]={S_IMU,S_BARO,S_MAG,S_GPS,S_TOF,S_COUNT,S_BATT,S_BATT};
      for (int i=0;i<8;i++) if (m[i]!=S_COUNT && g_health.usable(m[i])) t.healthy|=(1<<i);
      if (g_batt.cal_ok()) t.calibrated|=(1<<6);
      memcpy(out,&t,sizeof(t)); return sizeof(t); }
    case NT_TFLIGHT: {
      XFlight t{}; const AltEst& al=g_alt.st(); const EkfState& e=g_ekf.state();
      t.altDm=(int16_t)clampf(al.alt_m*10.f,-3000.f,32000.f); t.altSrc=al.tof_ok?3:(al.baro_ok?1:0);
      t.speedCms=(uint16_t)clampf(g_sens.gs_mps*100.f,0.f,65000.f);
      t.homeDistM=g_home_valid?(uint16_t)clampf(g_home_dist,0.f,65000.f):0xFFFF;
      float h=USB_SGN_YAW*e.yaw*RAD_TO_DEG; while(h<0) h+=360.f; while(h>=360.f) h-=360.f;
      t.headingDeg=(int16_t)h; t.rollDd=nxl_dd(USB_SGN_ROLL*e.roll); t.pitchDd=nxl_dd(USB_SGN_PITCH*e.pitch); t.yawDd=nxl_dd(USB_SGN_YAW*e.yaw);
      t.thrCmd=g_have_rc?g_rc.throttle:0xFFFF;
      memcpy(out,&t,sizeof(t)); return sizeof(t); }
    case NT_TLINK: {
      XLink t{}; t.rxOk=g_nxl.rxOk; t.rxBad=g_nxl.rxBad; t.ctrlGap=g_nxl.ctrlGap;
      t.linkState=(uint8_t)g_radio.st(); t.failsafe=(nxl_fcstate()==4)?1:0;
      memcpy(out,&t,sizeof(t)); return sizeof(t); }
  }
  return 0;
}

static uint16_t nxl_cfgcrc(){ return g_cfg.valid()?nex_crc16((const uint8_t*)&g_cfg.c().pid,sizeof(PidProfile)):0; }
static PidGains* nxl_axis(uint8_t a){
  if (!g_cfg.valid()) return nullptr;
  PidProfile& p=g_cfg.c().pid;
  switch (a){ case 0: return &p.rate_roll; case 1: return &p.rate_pitch; case 2: return &p.rate_yaw; case 3: return &p.alt_vel; }
  return nullptr;
}

static uint16_t nxl_us(int16_t v){ if (v>1000) v=1000; if (v<-1000) v=-1000; return (uint16_t)(1500+v/2); }
// Terjemahkan keadaan link remote -> PktRc internal FC (logika kontrol FC tidak diubah).
static void nxl_make_rc(PktRc& rc,uint32_t now){
  memset(&rc,0,sizeof(rc));
  rc.h.s0=NEX_SYNC0; rc.h.s1=NEX_SYNC1; rc.h.ver=NEX_PROTO_VER; rc.h.type=PKT_RC; rc.h.seq=g_nxl.lastRxSeq;
  uint16_t thr=lim::RC_MIN, roll=lim::RC_MID, pit=lim::RC_MID, yaw=lim::RC_MID;
  // flag bit0 = "control valid" dari remote. Bila tidak valid, remote mengirim sumbu 0 (= tengah!) -> JANGAN dipakai.
  if (g_nxl.flags & 1){ thr=nxl_us(g_nxl.ax[0]); yaw=nxl_us(g_nxl.ax[1]); roll=nxl_us(g_nxl.ax[2]); pit=nxl_us(g_nxl.ax[3]); }
  uint8_t fl=0;
  if (g_nxl.flags & 2) fl|=RC_F_RTH;
  if ((int32_t)(g_nxl.rthUntil-now)>0) fl|=RC_F_RTH;
  if ((int32_t)(g_nxl.disarmUntil-now)>0) fl|=RC_F_DISARM;
  if (g_nxl.armPend){   // gesture sudah diverifikasi remote (ditahan 1.5 s) -> suapkan gesture ke state machine Arming FC
    uint16_t lo=lim::RC_MIN, hi=lim::RC_MAX;
    if (g_cfg.valid()){ uint16_t a=g_cfg.c().rc_min,b=g_cfg.c().rc_max; if (a>=lim::RC_SANITY_MIN && b<=lim::RC_SANITY_MAX && b>a+200){ lo=a; hi=b; } }
    thr=lim::RC_MIN; yaw=hi; roll=lo; pit=lo;
    if (now-g_nxl.armT0<80) fl|=RC_F_ARM_REQ;       // satu tepi naik saja
  }
  rc.throttle=thr; rc.roll=roll; rc.pitch=pit; rc.yaw=yaw; rc.h.flags=fl;
}
static void nxl_arm_progress(uint32_t now){
  if (!g_nxl.armPend) return;
  uint8_t o[2];
  if (g_arming.armed()){ o[0]=3; o[1]=0; nxl_push(NT_ARM_STATUS,g_nxl.armReq,o,2); g_nxl.armPend=false; return; }
  uint32_t dt=now-g_nxl.armT0;
  if (dt>150 && g_arming.st()==AS_DISARMED && g_arming.reject()!=AR_OK){ o[0]=1; o[1]=nxl_rej(g_arming.reject()); nxl_push(NT_ARM_STATUS,g_nxl.armReq,o,2); g_nxl.armPend=false; return; }
  if (dt>2000){ o[0]=1; o[1]=8; nxl_push(NT_ARM_STATUS,g_nxl.armReq,o,2); g_nxl.armPend=false; }
}

static void nxl_request(uint8_t type,uint8_t req,const uint8_t* p,uint8_t l,uint32_t now){
  switch (type){
    case NT_HELLO: {
      uint8_t h[7]={NEXEND_FW_MAJOR,NEXEND_FW_MINOR,0,0,0,0,NXL_FC_ID}; uint32_t c=NXL_CAPS; memcpy(h+2,&c,4);
      nxl_push(NT_HELLO,0,h,7); break; }
    case NT_ARM_REQUEST: {
      if (g_arming.armed()){ uint8_t o[2]={3,0}; nxl_push(NT_ARM_STATUS,req,o,2); }
      else if (g_nxl.armPend){ g_nxl.armReq=req; }
      else { g_nxl.armPend=true; g_nxl.armReq=req; g_nxl.armT0=now; }
      break; }
    case NT_RTH_REQUEST: {
      uint8_t o[2];
      if (!g_arming.armed()){ o[0]=1; o[1]=8; }
      else if (g_fsm.rth_gate()){ o[0]=1; o[1]=g_fsm.rth_gate(); }     // bit: 1=baterai 2=GPS 4=home
      else { g_nxl.rthUntil=now+400; o[0]=2; o[1]=0; }
      nxl_push(NT_RTH_STATUS,req,o,2); break; }
    case NT_CMD_REQ: {
      if (l<1){ nxl_err(req,type,4,0); break; }
      if (p[0]==1){ g_nxl.disarmUntil=now+120; uint8_t o[3]={1,0,0}; nxl_push(NT_CMD_RESP,req,o,3); }
      else if (p[0]==2){
        uint8_t o[11]; o[0]=2; o[1]=0; o[2]=g_home_valid?1:0; int32_t la=(int32_t)(g_home.lat*1e7), lo=(int32_t)(g_home.lon*1e7);
        memcpy(o+3,&la,4); memcpy(o+7,&lo,4); nxl_push(NT_CMD_RESP,req,o,11);
      } else if (p[0]==3){   // [RA] ketinggian RTH maksimal. Req: [3][alt lo][alt hi] = set, [3] saja = baca. Resp: [3][status][0][alt lo][alt hi]
        uint8_t st=0;        // status: 0=OK 2=drone tidak DISARM 3=di luar rentang 4=gagal simpan
        if (l>=3){
          uint16_t a=(uint16_t)p[1]|((uint16_t)p[2]<<8);
          if (g_arming.st()!=AS_DISARMED) st=2;
          else if (a<lim::RTH_MAX_ALT_MIN || a>lim::RTH_MAX_ALT_MAX) st=3;
          else {
            uint16_t old=g_cfg.c().rth_max_alt_m;
            g_cfg.c().rth_max_alt_m=a;
            if (!g_cfg.save(g_cfg.c())){ g_cfg.c().rth_max_alt_m=old; st=4; }   // gagal simpan -> kembalikan nilai lama
          }
        }
        uint16_t cur=rth_max_alt_effective();   // read-back nilai sebenarnya yang dipakai RTH
        uint8_t o[5]={3,st,0,(uint8_t)(cur&0xFF),(uint8_t)(cur>>8)};
        nxl_push(NT_CMD_RESP,req,o,5);
      } else nxl_err(req,type,1,0);
      break; }
    case NT_PID_RD: {
      if (l<1){ nxl_err(req,type,4,0); break; }
      PidGains* g=nxl_axis(p[0]); if (!g){ nxl_err(req,type,5,0); break; }
      uint8_t o[16]; o[0]=p[0]; o[1]=0; memcpy(o+2,&g->kp,4); memcpy(o+6,&g->ki,4); memcpy(o+10,&g->kd,4);
      uint16_t c=nxl_cfgcrc(); memcpy(o+14,&c,2); nxl_push(NT_PID_RD_R,req,o,16); break; }
    case NT_PID_WR: {
      if (l<13){ nxl_err(req,type,4,0); break; }
      float kp,ki,kd; memcpy(&kp,p+1,4); memcpy(&ki,p+5,4); memcpy(&kd,p+9,4);
      uint8_t ax=p[0]; PidGains* g=nxl_axis(ax);
      uint8_t res=0, why=0;           // res: 0=ditolak 1=RAM saja 2=tersimpan
      if (!g){ why=5; }
      else if (g_arming.st()!=AS_DISARMED){ why=CAL_REJECT_ARMED; }
      else if (ax>2){ why=2; }        // altitude: baca saja
      else if (!isfinite(kp)||!isfinite(ki)||!isfinite(kd)||kp<0.4f||kp>6.f||kd<0.f||kd>0.2f||ki<0.f||ki>6.f){ why=3; } // batas sama dgn CMD_SET_PID FC
      else { g->kp=kp; g->ki=ki; g->kd=kd; g_cfg.save(g_cfg.c()); apply_active_profile_to_runtime(); res=2; }
      uint8_t o[17]; o[0]=ax; o[1]=res; o[2]=why;
      float rk=g?g->kp:0.f, ri=g?g->ki:0.f, rd=g?g->kd:0.f;   // readback sesungguhnya
      memcpy(o+3,&rk,4); memcpy(o+7,&ri,4); memcpy(o+11,&rd,4); uint16_t c=nxl_cfgcrc(); memcpy(o+15,&c,2);
      nxl_push(NT_PID_WR_R,req,o,17); break; }
    case NT_CAL_REQ: {
      if (l<2){ nxl_err(req,type,4,0); break; }
      uint8_t t=p[0], a=p[1], cmd=(t==1)?CMD_CALIB_ACCEL:(t==2)?CMD_CALIB_GYRO:(t==3)?CMD_CALIB_MAG:0;
      uint8_t o[4]={t,4,0,0};         // state: 0 idle 1 running 2 done 3 failed 4 unsupported 5 rejected
      if (cmd){
        if (a==1){
          if (g_arming.st()!=AS_DISARMED){ o[1]=5; o[3]=CAL_REJECT_ARMED; }
          else if (g_calwiz.busy()){ o[1]=5; o[3]=CAL_REJECT_BUSY; }
          else { calib_wizard_start(cmd); o[1]=1; }
        } else if (a==2){ g_calwiz.reset(); o[1]=0; }
        else {
          switch (g_calwiz.state){ case CS_IN_PROGRESS: case CS_REQ: o[1]=1; break; case CS_SUCCESS: o[1]=2; break;
                                   case CS_FAILED: case CS_ABORTED: o[1]=3; break; default: o[1]=0; }
        }
        o[2]=g_calwiz.progress;
      }
      nxl_push(NT_CAL_ST,req,o,4); break; }
    default: nxl_err(req,type,1,0); break;   // LOG_REQ dll: tidak didukung
  }
}

static bool nxl_handle(const uint8_t* b,uint8_t n,uint32_t now){
  if (n<NXL_HDR+2 || n>32 || b[0]!=NXL_MAGIC || b[1]!=NXL_VER){ g_nxl.rxBad++; return false; }
  uint8_t l=b[5]; if (l>NXL_MAXPL || (uint8_t)(NXL_HDR+l+2)!=n){ g_nxl.rxBad++; return false; }
  uint16_t c=nex_crc16(b,NXL_HDR+l), r=b[NXL_HDR+l]|((uint16_t)b[NXL_HDR+l+1]<<8);
  if (c!=r){ g_nxl.rxBad++; return false; }
  if (g_nxl.haveRx && b[3]==g_nxl.lastRxSeq) return false;              // duplikat
  g_nxl.haveRx=true; g_nxl.lastRxSeq=b[3]; g_nxl.rxOk++;
  const uint8_t type=b[2], req=b[4]; const uint8_t* p=b+NXL_HDR;
  switch (type){
    case NT_HEARTBEAT: if (l>=3){ g_nxl.flags=0; g_nxl.want=p[2]; g_nxl.lastCtrlMs=0; } break;   // tanpa sumbu: throttle min, lain tengah
    case NT_CONTROL:
      if (l>=10){
        memcpy(g_nxl.ax,p,8); g_nxl.flags=p[8]; g_nxl.want=p[9];
        if (g_nxl.lastCtrlMs && now-g_nxl.lastCtrlMs>60) g_nxl.ctrlGap++;
        g_nxl.lastCtrlMs=now;
      } break;
    default: nxl_request(type,req,p,l,now); break;
  }
  return true;
}
static void nxl_load_ack(RF24& r){
  uint8_t fr[32],n=0;
  if (g_nxl.qh!=g_nxl.qt){ NxlResp& q=g_nxl.q[g_nxl.qt]; n=nxl_frame(fr,q.type,q.req,q.pl,q.len); g_nxl.qt=(g_nxl.qt+1)&3; }
  else {
    uint8_t pl[NXL_MAXPL]; uint8_t k=nxl_build_tel(g_nxl.want,pl);
    if (!k) k=nxl_build_tel(NT_TBASIC,pl), g_nxl.want=NT_TBASIC;
    n=nxl_frame(fr,g_nxl.want,0,pl,k);
  }
  if (!r.writeAckPayload(1,fr,n)){ r.flush_tx(); r.writeAckPayload(1,fr,n); }   // FIFO ACK penuh -> buang yang basi
}
static void nxl_poll(RF24& r,RadioStats& s,uint32_t now){
  bool got=false; uint8_t guard=4;
  while (guard-- && r.available()){
    uint8_t n=r.getDynamicPayloadSize();
    if (n<1 || n>32){ r.flush_rx(); g_nxl.rxBad++; break; }
    uint8_t b[32]; r.read(b,n);
    if (nxl_handle(b,n,now)){
      got=true; nxl_make_rc(s.pkt,now); s.have=true; s.last_ms=now; s.last_good_ms=now; s.last_seq=g_nxl.lastRxSeq;
      if (s.streak<255) s.streak++;
      s.q=(s.q<95)?s.q+5:100;
      nxl_arm_progress(now);
      nxl_make_rc(s.pkt,now);                       // segarkan setelah arm window berubah
      nxl_load_ack(r);
    } else { s.q=(s.q>3)?s.q-3:0; if (s.streak) s.streak--; }
  }
  if (!got){ s.q=(s.q>1)?s.q-1:0; nxl_arm_progress(now); }
  // RC sintetis ikut kedaluwarsa dgn get_rc() (RC_AGE_MS) bila remote berhenti mengirim -> failsafe FC normal
}

static void control_tick(float dt,uint32_t now_ms,QMC5883LCompass& q){
  g_radio.tick(now_ms);
  PktRc rc_pkt;
  if (g_radio.get_rc(rc_pkt,now_ms)){
    g_rc=rc_pkt; g_have_rc=true; g_last_rc_ms=now_ms;
    g_health.ok(S_RADIO_A,now_ms,g_radio.q());
  }
  if (g_have_rc && (now_ms-g_last_rc_ms)>lim::RC_AGE_MS) g_have_rc=false;

  bool arm_req=g_have_rc && (g_rc.h.flags & RC_F_ARM_REQ);
  bool dis_req=g_have_rc && (g_rc.h.flags & RC_F_DISARM);
  g_arm_edge    = arm_req && !g_arm_latch;
  g_disarm_edge = dis_req && !g_disarm_latch;
  g_arm_latch    = arm_req;
  g_disarm_latch = dis_req;

  g_health.tick(now_ms);
  g_warn.tick(now_ms);

  ArmChecks ck{};
  ck.imu_ok=g_health.usable(S_IMU);
  ck.accel_ok=g_health.usable(S_ACCEL);
  ck.gyro_ok=g_health.usable(S_GYRO);
  ck.baro_ok=g_health.usable(S_BARO);
  ck.mag_ok=g_health.usable(S_MAG);
  ck.gps_ok=g_health.usable(S_GPS);
  ck.tof_ok=true;
  ck.batt_ok=g_health.usable(S_BATT) && g_sens.batt_v>g_batt.crit_v()*g_batt.cells();
  ck.rA_ok=g_radio.st()==LS_ACTIVE;
  ck.esc_ok=true;
  ck.motor_ok=!g_motordiag.fault();
  ck.rc_valid=g_have_rc;
  ck.no_inverted=!g_inv.triggered();
  ck.no_crash=!g_crash.triggered();
  ck.no_motor_lock=!g_motordiag.fault();
  ck.no_post_lock=g_fsm.st()!=FS_POST_LANDING_LOCK;
  ck.synced=ck.rA_ok;
  ck.cfg_ok=g_cfg.valid();
  ck.fw_ok = g_fw_ok;
  g_last_ck = ck; // [NXL]
  g_arming.tick(now_ms,g_arm_edge,g_disarm_edge,
                g_have_rc?g_rc.throttle:lim::RC_MIN,
                g_have_rc?g_rc.roll:lim::RC_MID,
                g_have_rc?g_rc.pitch:lim::RC_MID,
                g_have_rc?g_rc.yaw:lim::RC_MID,
                ck);
  if (g_disarm_edge && g_arming.armed()){
    post_event(WL_CRITICAL,EV_DISARM_MANUAL,0);
    g_buzz.disarm();
    g_auto_disarm_armed=false;
  }

  g_ekf.predict(g_sens.gyro,g_sens.accel,dt);
  g_ekf.update_accel(g_sens.accel);
  float decl = g_cfg.valid() ? g_cfg.c().declination_rad : 0.f;
  if (g_sens.mag_last_ms && (now_ms-g_sens.mag_last_ms)<500) g_ekf.update_mag(g_sens.mag,decl);
  const EkfState& ekf=g_ekf.state();

  if (g_home_valid) g_pos.update(g_sens.lat,g_sens.lon,g_sens.sats,g_sens.fix,now_ms);
  g_inv.tick(ekf.roll,ekf.pitch,now_ms);

  float gm=sqrtf(ekf.gx*ekf.gx+ekf.gy*ekf.gy+ekf.gz*ekf.gz);
  g_landing.tick(g_sens.tof_m,
#if NEXEND_HAVE_TOF
    g_health.usable(S_TOF),
#else
    false,
#endif
    g_alt.st().alt_m,g_alt.st().vs_mps,ekf.roll,ekf.pitch,gm,
    g_have_rc?g_rc.throttle:lim::RC_MIN,now_ms);

  float am[3]={ekf.ax,ekf.ay,ekf.az};
  float gm3[3]={ekf.gx,ekf.gy,ekf.gz};
  g_crash.tick(am,gm3,g_alt.st().vs_mps,g_alt.st().alt_m,g_arb_out.motors_enabled,now_ms);

  RcInput rin{};
  rin.valid=g_have_rc;
  if (g_have_rc){
    uint16_t lo = lim::RC_MIN, hi = lim::RC_MAX, mid = lim::RC_MID;
    uint16_t tmid = lim::RC_MID;
    if (g_cfg.valid()){
      uint16_t c_lo = g_cfg.c().rc_min;
      uint16_t c_hi = g_cfg.c().rc_max;
      uint16_t c_mid = g_cfg.c().rc_center[2];
      uint16_t c_tmid = g_cfg.c().thr_center_us;
      if (c_lo>=lim::RC_SANITY_MIN && c_hi<=lim::RC_SANITY_MAX && c_hi>c_lo+200){
        lo = c_lo; hi = c_hi;
      }
      if (c_mid>=lo+100 && c_mid<=hi-100) mid = c_mid;
      if (c_tmid>=lo+100 && c_tmid<=hi-100) tmid = c_tmid;
    }
    auto dz=[](int v,int m,int dead)->float{ int d=v-m; if (abs(d)<dead) return 0.f; return (float)d/500.f; };
    rin.roll  = dz(g_rc.roll,  mid, lim::RC_DZ);
    rin.pitch = dz(g_rc.pitch, mid, lim::RC_DZ);
    rin.yaw   = dz(g_rc.yaw,   mid, lim::RC_DZ);
    rin.thr   = (g_rc.throttle-(float)lo)/1000.f;
    if (hi>lo+200) rin.thr = (g_rc.throttle-(float)lo)/(float)(hi-lo);
    rin.thr=clampf(rin.thr,0.f,1.f);
    (void)tmid;
  }

  FlightState st_prev = g_fsm.st();
  bool fsm_rth     = (st_prev == FS_RTH);
  bool fsm_descend = (st_prev == FS_FAILSAFE_DESCEND);
  bool fsm_autonomous = fsm_rth || fsm_descend;

  bool loiter_enabled = g_cfg.valid() ? (g_cfg.c().loiter_enabled!=0) : true;
  bool arming_ok = g_arming.armed() && !g_arb.has_latch();
  bool baro_ok = g_alt.st().baro_ok;
  int thr_err = g_have_rc ? ((int)g_rc.throttle - (int)lim::RC_MID) : 9999;
  int thr_abs = abs(thr_err);

  bool loiter_forbidden = fsm_autonomous;

  if (!arming_ok || !loiter_enabled || !g_have_rc || loiter_forbidden){
    g_loiter_active=false;
    if (!arming_ok) { g_alt_hold=false; g_pos_hold=false; }
  } else {
    if (!g_loiter_active){
      if (thr_abs <= lim::LOITER_DZ_ENTER && baro_ok){
        g_loiter_active=true; g_alt_hold=true;
        g_pos_hold = g_pos.have() && g_health.usable(S_GPS);
        g_alt_sp = g_alt.st().alt_m;
        post_event(WL_INFO,EV_LOITER_ENTER,(uint16_t)(g_alt.st().alt_m*10));
        g_buzz.loiter();
      }
    } else {
      if (thr_abs > lim::LOITER_DZ_EXIT){
        g_loiter_active=false; g_alt_hold=false; g_pos_hold=false;
        post_event(WL_INFO,EV_LOITER_EXIT,0);
      }
    }
  }
  if (g_loiter_active && thr_abs > lim::LOITER_DZ_ENTER){
    float span = 500.f - lim::LOITER_DZ_ENTER;
    float k = clampf((float)(thr_abs - lim::LOITER_DZ_ENTER) / span, 0.f, 1.f);
    if (thr_err > 0) g_alt_sp += k * lim::LOITER_MAX_CLIMB * dt;
    else             g_alt_sp -= k * lim::LOITER_MAX_DESC  * dt;
    g_alt_sp = clampf(g_alt_sp, 0.3f, 200.f);
  }

  if (fsm_rth) {
    g_mode = FM_RTH; g_alt_hold = true; g_pos_hold = true;
  } else if (fsm_descend) {
    g_mode = FM_ALTHOLD; g_alt_hold = true; g_pos_hold = false;
  } else if (g_loiter_active) {
    g_mode = g_pos_hold ? FM_LOITER : FM_ALTHOLD;
  } else {
    g_mode = FM_STABILIZE;
  }

  float vn_sp=0,ve_sp=0,rth_alt_sp=0,yaw_sp=0;
  bool use_rth_yaw = false;
  // [LP] Loiter GPS ala DJI: stick roll/pitch = perintah KECEPATAN; stick dilepas -> rem lalu kunci titik GPS
  // dan ditarik kembali ke titik itu (loop posisi). Memakai posisi N/E dari g_pos dan ekf.yaw (heading terfusi kompas).
  {
    static bool lp_lock=false; static float lp_n=0.f, lp_e=0.f;
    if (g_loiter_active && g_pos_hold && !g_rth.active()){
      const PosEst& ps = g_pos.st();
      bool gps_fresh = ps.last_ms && (uint32_t)(now_ms - ps.last_ms) < 1000;
      float cy=cosf(ekf.yaw), sy=sinf(ekf.yaw);
      if (!gps_fresh){
        lp_lock=false;                                  // data GPS basi: jangan menahan posisi, kecepatan target 0
      } else if (fabsf(rin.roll) > 0.02f || fabsf(rin.pitch) > 0.02f){
        lp_lock=false;                                  // pilot menggerakkan stick: kecepatan dari stick (maju=pitch negatif, kanan=roll positif)
        float vf = -rin.pitch * lim::LOITER_VMAX;
        float vr =  rin.roll  * lim::LOITER_VMAX;
        vn_sp = cy*vf - sy*vr;
        ve_sp = sy*vf + cy*vr;
      } else {
        if (!lp_lock && hypotf(ps.vn, ps.ve) < lim::LOITER_LATCH_V){ lp_n=ps.n; lp_e=ps.e; lp_lock=true; }
        if (lp_lock){
          float en=lp_n-ps.n, ee=lp_e-ps.e;
          if (hypotf(en,ee) > lim::LOITER_LOST_M){ lp_lock=false; }
          else {
            vn_sp = clampf(lim::LOITER_POS_KP*en, -lim::LOITER_VPOS_MAX, lim::LOITER_VPOS_MAX);
            ve_sp = clampf(lim::LOITER_POS_KP*ee, -lim::LOITER_VPOS_MAX, lim::LOITER_VPOS_MAX);
          }
        }
      }
    } else {
      lp_lock=false;
    }
  }
  if (g_rth.active()){
    g_rth.tick(g_pos.st(),g_alt.st().alt_m,ekf.yaw,dt,yaw_sp,vn_sp,ve_sp,rth_alt_sp);
    g_alt_sp=rth_alt_sp;
    use_rth_yaw = true;
  }

  if (st_prev == FS_FAILSAFE_DESCEND){
    float rate = 1.0f;
    if (g_cfg.valid()) rate = g_cfg.c().failsafe_desc_rate;
    if (rate <= 0.f) rate = 0.5f;
    if (g_alt_sp > 0.05f) g_alt_sp = fmaxf(0.f, g_alt_sp - rate*dt);
    else g_alt_sp = 0.f;
    g_alt_hold = true;
    g_pos_hold = false;
  }

  bool stabilizer_on = true;
  if (g_cfg.valid()) stabilizer_on = (g_cfg.c().stabilizer_enabled!=0);
  if (g_cfg.valid()) stabilizer_on = (g_cfg.c().pid.stabilizer_enabled!=0) && stabilizer_on;

  bool imu_ok = g_health.usable(S_IMU) && g_health.usable(S_GYRO) && g_health.usable(S_ACCEL);
  bool est_ok = isfinite(ekf.roll)&&isfinite(ekf.pitch)&&isfinite(ekf.yaw)
             && isfinite(ekf.gx)&&isfinite(ekf.gy)&&isfinite(ekf.gz);
  bool gyro_sat = (fabsf(ekf.gx)>15.f)||(fabsf(ekf.gy)>15.f)||(fabsf(ekf.gz)>15.f);
  bool adapt_safe_env = imu_ok && est_ok && !gyro_sat && !g_crash.triggered()
                     && g_health.aggregate()!=SS_CRITICAL;

  if (g_cfg.valid()){
    g_adapt.enabled = g_cfg.c().auto_stab_enabled != 0;
    g_adapt.strength = g_cfg.c().auto_stab_strength;
    g_adapt.max_adjust_pct = g_cfg.c().auto_pid_max_adjust;
    g_adapt.update_ms = g_cfg.c().auto_pid_update_ms;
    g_adapt.smoothing_ms = g_cfg.c().auto_pid_smoothing_ms;
  }

  g_ctrl.set_profile(g_adapt.enabled ? g_adapt.profile() : g_ctrl.profile());
  if (!g_adapt.enabled){
    g_ctrl.set_profile(g_cfg.valid()? g_cfg.c().pid : g_ctrl.profile());
  }

  g_ctrl.tick(ekf,g_alt.st(),g_pos.st(),rin,dt,g_mode,
              g_arming.ok(),g_alt_sp,g_alt_hold || g_rth.active(),
              g_pos_hold || g_rth.active(),vn_sp,ve_sp,
              use_rth_yaw, yaw_sp, stabilizer_on);

  float mix[4];
  for (int i=0;i<4;i++) mix[i]=g_ctrl.last_motor()[i];
  if (!is_finite_arr(mix,4)){ post_event(WL_CRITICAL,EV_MOTOR_CUT_NAN,0); for (int i=0;i<4;i++) mix[i]=0.f; }

  if (g_arming.idle_stage()){
    for (int i=0;i<4;i++) if (mix[i] < lim::IDLE_SPIN) mix[i] = lim::IDLE_SPIN;
  }

  if (g_alt_hold && g_arming.armed() && fabsf(rin.thr-0.5f)<0.1f)
    g_ctrl.learn_hover(rin.thr,g_alt.st().vs_mps,dt);

  bool landing_ctx = (g_fsm.st()==FS_LANDING) || g_landing.near_ground();
  bool thr_low=g_have_rc && g_rc.throttle<lim::RC_MIN+40;
  g_motordiag.tick(mix,gm3,am,ekf.roll,ekf.pitch,landing_ctx,thr_low,now_ms);

  bool batt_ok_rth;
  if (g_batt.cal_ok()) batt_ok_rth = (g_batt.cell_rest_v() > g_batt.rth_v());
  else                 batt_ok_rth = (g_sens.batt_v > (g_batt.rth_v()+0.15f) * g_batt.cells());
  bool rth_ok_gps = (g_sens.fix>=2) && (g_sens.sats>=lim::SATS_MIN) && (g_sens.hdop<=lim::HDOP_MAX);
  bool rth_ok_home = g_home_valid;

  if (g_batt.cal_ok()){
    float cr = g_batt.cell_rest_v();
    if (cr>0.f && cr < g_batt.rth_v() && g_arming.armed() && !g_lowbatt_rth_latch && g_fsm.st()==FS_FLIGHT){
      g_lowbatt_rth_latch=true;
      g_warn.raise(W_LOWBATT_RTH, WL_WARNING);
      post_event(WL_WARNING,EV_LOWBATT_RTH,(uint16_t)(cr*1000.f));
    }
  }
  if (g_fsm.st()==FS_DISARMED) g_lowbatt_rth_latch=false;

  bool rth_btn_raw = g_have_rc && (g_rc.h.flags & RC_F_RTH);
  bool rth_btn = false;
  static bool rth_raw_prev = false; // [NXR] event hanya pada tepi naik (hindari banjir event tiap tick 1 kHz)
  bool rth_edge = rth_btn_raw && !rth_raw_prev;
  rth_raw_prev = rth_btn_raw;
  if (rth_btn_raw){
    bool gate_batt = batt_ok_rth;
    bool gate_gps  = rth_ok_gps;
    bool gate_home = rth_ok_home;
    if (!gate_batt){ g_warn.raise(W_RTH_BLOCKED_BATT, WL_WARNING); if (rth_edge) post_event(WL_WARNING, EV_RTH_BLOCK, 0); }
    else if (!gate_gps){ g_warn.raise(W_RTH_BLOCKED_GPS, WL_WARNING); if (rth_edge) post_event(WL_WARNING, EV_RTH_BLOCK, 1); }
    else if (!gate_home){ g_warn.raise(W_RTH_BLOCKED_HOME, WL_WARNING); if (rth_edge) post_event(WL_WARNING, EV_RTH_BLOCK, 2); }
    else { rth_btn = true; }
  }

  FsmIn fi{};
  fi.now=now_ms;
  fi.arb=&g_arb_out;
  fi.link_lost=g_radio.lost();
  fi.arm_ready=g_arming.takeoff_ready();
  fi.arm_ok   =g_arming.ok();
  fi.disarm_edge=g_disarm_edge;
  fi.rth_btn  =rth_btn;
  fi.rth_ok_batt=batt_ok_rth;
  fi.rth_ok_gps =rth_ok_gps;
  fi.rth_ok_home=rth_ok_home;
  fi.landing_detected=g_landing.landed();
  fi.gps_fix_ok = rth_ok_gps;
  fi.home_valid = g_home_valid;
  fi.lock_cleared = g_lock_clear_pending;
  fi.failsafe_action = g_cfg.valid() ? g_cfg.c().failsafe_action : 0;
  fi.lowbatt_rth_trigger = g_lowbatt_rth_latch;
  g_fsm.tick(fi);
  if (g_lock_clear_pending) g_lock_clear_pending = false;

  FlightState st=g_fsm.st();
  bool in_lock = (st==FS_POST_LANDING_LOCK)||(st==FS_INVERTED)||(st==FS_CRASH)||(st==FS_MOTOR_FAULT)||(st==FS_CRITICAL_SENSOR);
  if (in_lock && !g_arming.armed()){
    if (!g_arb.has_latch() && !g_lock_clear_pending) g_lock_clear_pending = true;
  }

  if ((st==FS_LANDED || st==FS_POST_LANDING_LOCK) && g_arming.armed()){
    if (!g_auto_disarm_armed){ g_auto_disarm_armed=true; g_auto_disarm_t0=now_ms; post_event(WL_INFO,EV_RTH_AUTOLAND,0); }
    else if (now_ms-g_auto_disarm_t0 > lim::LAND_DISARM_MS){
      g_arming.force_disarm(); g_auto_disarm_armed=false; g_lock_clear_pending=true;
      post_event(WL_INFO,EV_RTH_AUTODISARM,0); g_buzz.disarm();
    }
  } else g_auto_disarm_armed=false;

  if (g_last_st!=FS_TAKEOFF && st==FS_TAKEOFF){
    g_takeoff.arm(now_ms,g_alt.st().alt_m);
    g_stats.on_flight_start(); g_buzz.arm();
  }
  g_takeoff.tick(ekf,g_alt.st(),mix,g_arb_out.motors_enabled,now_ms);
  g_abort_flag = g_takeoff.aborted();
  if (g_abort_flag && g_last_st==FS_TAKEOFF){
    post_event(WL_CRITICAL,EV_TAKEOFF_ABORT,0);
    g_stats.on_takeoff_abort(); g_buzz.crit();
  }
  g_last_st=st;

  if (!g_home_valid && rth_ok_gps && g_sens.lat && g_sens.lon){
    g_home.lat=g_sens.lat; g_home.lon=g_sens.lon; g_home_valid=true;
    g_pos.begin(g_home);
    post_event(WL_INFO,EV_HOME_SET,0);
  }
  if (g_home_valid){
    GeoPoint n{g_sens.lat,g_sens.lon};
    g_home_dist=geo_dist_m(n,g_home);
  }

  if (st==FS_RTH && !g_rth.active()){
    g_rth.start(g_home,g_pos.st(),g_alt.st().alt_m);
    post_event(WL_INFO,EV_RTH_START,0); g_stats.on_rth();
  }
  if (st!=FS_RTH && g_rth.active()) g_rth.stop();

  static bool last_landed=false;
  if (g_landing.landed() && !last_landed){
    post_event(WL_INFO,EV_LANDING,0);
    g_stats.on_landing(); g_stats.on_flight_end(now_ms);
    g_buzz.disarm();
  }
  last_landed=g_landing.landed();

  static bool ev_inv=false,ev_crash=false,ev_mot=false;
  if (g_inv.triggered() && !ev_inv){ post_event(WL_CRITICAL,EV_INVERTED,0); ev_inv=true; g_buzz.crit(); }
  if (g_crash.triggered() && !ev_crash){ post_event(WL_CRITICAL,EV_CRASH,0); g_stats.on_crash(); ev_crash=true; g_buzz.crit(); }
  if (g_motordiag.fault() && !ev_mot){ post_event(WL_CRITICAL,EV_MOTOR_FAULT,0); g_stats.on_motor_fault(); ev_mot=true; g_buzz.crit(); }

  float cell_rest = g_batt.cell_rest_v();
  if (cell_rest > 0.f && g_batt.cal_ok()){
    if (cell_rest < g_batt.crit_v()) g_warn.raise(W_BATT_CRIT,WL_CRITICAL);
    else if (cell_rest < g_batt.warn_v()) g_warn.raise(W_BATT_LOW,WL_WARNING);
    else { g_warn.clear(W_BATT_CRIT); g_warn.clear(W_BATT_LOW); }
  } else {
    float cell=g_sens.batt_v/g_batt.cells();
    if (cell<g_batt.crit_v()) g_warn.raise(W_BATT_CRIT,WL_CRITICAL);
    else if (cell<g_batt.warn_v()) g_warn.raise(W_BATT_LOW,WL_WARNING);
    else { g_warn.clear(W_BATT_CRIT); g_warn.clear(W_BATT_LOW); }
  }
  if (g_batt.sag_active()){ g_warn.raise(W_BATT_SAG,WL_WARNING); }
  else g_warn.clear(W_BATT_SAG);

  static bool last_link_lost=false;
  static bool radio_lost_while_disarmed_ev=false;
  if (g_radio.lost() && !last_link_lost){
    if (g_arming.armed()){
      post_event(WL_CRITICAL,EV_FAILSAFE,0);
      g_stats.on_failsafe(); g_warn.raise(W_LINK_LOST,WL_CRITICAL); g_buzz.warn();
    } else if (!radio_lost_while_disarmed_ev){
      post_event(WL_INFO,EV_RADIO_LOST_SAFE,0);
      radio_lost_while_disarmed_ev=true;
      g_warn.raise(W_LINK_LOST,WL_WARNING);
    }
  } else if (!g_radio.lost() && last_link_lost){
    g_warn.clear(W_LINK_LOST); radio_lost_while_disarmed_ev=false;
  }
  last_link_lost=g_radio.lost();

  bool in_descend = (st==FS_FAILSAFE_DESCEND);
  if (in_descend && !g_alt_hold){
    g_alt_hold = true; g_alt_sp = 0.f;
    post_event(WL_WARNING,EV_FAILSAFE_DESCEND,0);
  }

  bool airborne = g_arming.armed() &&
    (st==FS_TAKEOFF || st==FS_FLIGHT || st==FS_LANDING || st==FS_RTH || st==FS_FAILSAFE || st==FS_FAILSAFE_DESCEND);

  g_arb_in=ArbiterIn{};
  g_arb_in.disarm_request   = g_disarm_edge;
  g_arb_in.arm_ok           = g_arming.ok() && st!=FS_POST_LANDING_LOCK;
  g_arb_in.inverted         = g_inv.triggered();
  g_arb_in.crash            = g_crash.triggered();
  g_arb_in.motor_fault      = g_motordiag.fault();
  g_arb_in.takeoff_abort    = g_abort_flag;
  g_arb_in.landing_prot     = landing_ctx;
  g_arb_in.failsafe         = (st==FS_FAILSAFE) && !in_descend;
  g_arb_in.rth_active       = (st==FS_RTH);
  g_arb_in.failsafe_descend = in_descend;
  g_arb_in.low_batt         = (cell_rest<g_batt.warn_v() && cell_rest>0.f);
  g_arb_in.crit_batt        = (cell_rest<g_batt.crit_v() && cell_rest>0.f);
  g_arb_in.post_landing_lock= (st==FS_POST_LANDING_LOCK);
  g_arb_in.airborne         = airborne;
  for (int i=0;i<4;i++) g_arb_in.mix[i]=mix[i];
  g_arb_in.sensor_st        = g_health.aggregate();
  if (in_descend) g_arb_in.arm_ok = true;
  g_arb_out=g_arb.evaluate(g_arb_in);
  motor_write(g_arb_out.out,g_arb_out.motors_enabled);

  if (g_arb_out.latch_lock) g_arming.force_disarm();

  bool bench_safe=(g_arming.st()==AS_DISARMED);
  bool tuner_done = g_autotune.tick(ekf,dt,now_ms,bench_safe);
  if (tuner_done && g_autotune.phase()==AT_DONE){
    g_ctrl.set_profile(g_autotune.result());
    g_adapt.set_baseline(g_autotune.result());
    post_event(WL_INFO,EV_AUTOTUNE_DONE,0);
    if (g_cfg.valid()){ g_cfg.c().pid=g_autotune.result(); g_cfg.save(g_cfg.c()); }
  } else if (tuner_done && g_autotune.phase()==AT_ABORTED){
    post_event(WL_INFO,EV_AUTOTUNE_ABORT,0);
  }

  float er=rin.roll*lim::MAX_TILT-ekf.roll;
  float ep=rin.pitch*lim::MAX_TILT-ekf.pitch;
  float ey=rin.yaw*3.f-ekf.gz;
  bool adapt_safe = g_arming.armed() && st!=FS_LANDED && !g_arb_out.latch_lock && adapt_safe_env;
  g_adapt.tick(er,ep,ey,now_ms,adapt_safe);

  // --- rev 6.5: auto PID audit + save-if-improved ---
  static uint32_t last_adapt_log = 0;
  if (g_adapt.enabled && (now_ms - last_adapt_log >= g_adapt.update_ms)){
    last_adapt_log = now_ms;
    adapt_log_to_sd(g_adapt.snapshot());
  }
  if (g_adapt.improvement_ready() && !g_arming.armed() && g_cfg.valid()){
    PidProfile new_pid = g_adapt.profile();
    g_cfg.c().pid = new_pid;
    g_cfg.save(g_cfg.c());
    g_adapt.set_baseline(new_pid);
    g_adapt.clear_improvement();
    post_event(WL_INFO, EV_ADAPT_SAVED, (uint16_t)g_adapt.last_adjust_pct());
    adapt_log_to_sd(g_adapt.snapshot());
  }

  g_stats.live(g_alt.st().alt_m,g_home_dist,g_sens.gs_mps,g_sens.batt_v,g_sens.batt_a);

  static uint32_t last_bb_ms=0;
  if (now_ms - last_bb_ms >= lim::BB_LOG_PERIOD_MS){
    last_bb_ms = now_ms;
    BbFrame bf{};
    bf.t_ms=now_ms;
    bf.throttle=g_have_rc?g_rc.throttle:lim::RC_MIN;
    bf.rc_r=g_have_rc?(int16_t)g_rc.roll:1500;
    bf.rc_p=g_have_rc?(int16_t)g_rc.pitch:1500;
    bf.rc_y=g_have_rc?(int16_t)g_rc.yaw:1500;
    for (int i=0;i<3;i++){ bf.gyro[i]=g_sens.gyro[i]; bf.accel[i]=g_sens.accel[i]; }
    bf.roll=ekf.roll; bf.pitch=ekf.pitch; bf.yaw=ekf.yaw;
    for (int i=0;i<4;i++) bf.out[i]=g_arb_out.out[i];
    bf.alt_dm=(int16_t)(g_alt.st().alt_m*10);
    bf.tof_mm=g_sens.tof_m>0?(int16_t)(g_sens.tof_m*1000):-1;
    bf.batt_mv=(uint16_t)(g_sens.batt_v*1000);
    bf.batt_ca=(int16_t)(g_sens.batt_a*100);
    bf.batt_pct=g_batt.pct();
    bf.fsm=(uint8_t)st;
    bf.arm=(uint8_t)g_arming.st();
    bf.cause=(uint8_t)g_arb_out.cause;
    bf.sys=(uint8_t)g_arb_out.status;
    bf.rA_q=g_radio.q(); bf.rB_q=0; // [1R] rB_q dicadangkan (0)
    bf.stab_on = stabilizer_on?1:0;
    bf.auto_stab_on = g_adapt.enabled?1:0;
    bf.auto_strength = g_adapt.strength;
    bf.active_profile = g_cfg.valid()?g_cfg.c().active_profile:0;
    bf.adaptive_adjust_pct = g_adapt.last_adjust_pct();
    bf.roll_sp = g_ctrl.last_roll_sp();
    bf.pitch_sp = g_ctrl.last_pitch_sp();
    bf.yaw_rate_sp = g_ctrl.last_yaw_rate_sp();
    g_bb.frame(bf);
  }

  g_buzz.tick(now_ms);
  (void)q;
}

// ============ SETUP ============
// [BOOTFIX] Watchdog dinyalakan DI AWAL setup() (bukan di akhir) supaya hang saat init sensor/storage/radio
// berujung reset otomatis, bukan freeze permanen. Tiap tahap dicatat ke scratch register watchdog (selamat dari
// reset WDT) sehingga setelah reset LED berkedip N kali = tahap N yang hang.
#define NEXEND_BOOT_WDT_MS 8000

// USER button: single=IMU gyro, double=compass, triple=sensor check,
// quadruple=gyro then accel then compass. Only works while disarmed.
static uint8_t g_user_clicks = 0;
static uint32_t g_user_last_edge = 0, g_user_click_deadline = 0;
static bool g_user_prev = HIGH;
static uint8_t g_user_cal_step = 0; // 1=gyro->accel->mag, 2=single IMU, 3=mag
static uint32_t g_user_cal_watch = 0;
static uint32_t g_sensor_check_until = 0;

static void user_button_tick(uint32_t now){
  bool cur = digitalRead(PIN_USER_BUTTON);
  if (g_user_prev == HIGH && cur == LOW && (uint32_t)(now-g_user_last_edge) > 35){
    g_user_last_edge = now;
    if (!g_arming.armed() && !g_calwiz.busy()){
      if (g_user_clicks < 4) g_user_clicks++;
      g_user_click_deadline = now + 450;
    }
  }
  g_user_prev = cur;

  if (g_user_clicks && (int32_t)(now-g_user_click_deadline) >= 0){
    uint8_t n = g_user_clicks; g_user_clicks = 0;
    if (g_arming.armed() || g_calwiz.busy()) return;
    if (n == 1){
      g_user_cal_step = 4; // IMU: gyro then accelerometer
      calib_wizard_start(CMD_CALIB_GYRO);
    } else if (n == 2){
      g_user_cal_step = 3;
      calib_wizard_start(CMD_CALIB_MAG);
    } else if (n == 3){
      g_sensor_check_until = now + 1800; // indikasi pemeriksaan sensor
    } else if (n >= 4){
      g_user_cal_step = 1;
      calib_wizard_start(CMD_CALIB_GYRO);
    }
  }

  // Run next calibration stage only after the previous stage finishes.
  if (g_user_cal_step && !g_calwiz.busy() && g_user_cal_watch != now){
    if (g_calwiz.state == CS_SUCCESS){
      if (g_user_cal_step == 1 && g_calwiz.which == CMD_CALIB_GYRO){
        g_user_cal_watch = now; g_calwiz.reset(); calib_wizard_start(CMD_CALIB_ACCEL); return;
      }
      if ((g_user_cal_step == 1 || g_user_cal_step == 4) &&
          g_calwiz.which == CMD_CALIB_ACCEL){
        if (g_user_cal_step == 1){
          g_user_cal_watch = now; g_calwiz.reset(); calib_wizard_start(CMD_CALIB_MAG); return;
        }
        g_user_cal_step = 0; // IMU calibration complete
      }
    }
    if (g_calwiz.state == CS_FAILED || g_calwiz.state == CS_SUCCESS ||
        g_calwiz.state == CS_ABORTED){
      g_user_cal_step = 0;
    }
  }
}

static void status_rgb_tick(uint32_t now){
  static uint32_t last = 0;
  static uint32_t last_init = 0;
  static bool init_done = false;
  if ((uint32_t)(now-last) < 40) return;
  last = now;

  uint8_t r=0, g=0, b=0;
  FlightState st = g_fsm.st();
  bool error = st==FS_CRITICAL_SENSOR || st==FS_MOTOR_FAULT ||
               st==FS_CRASH || st==FS_INVERTED ||
               st==FS_FAILSAFE || st==FS_FAILSAFE_DESCEND;
  if (!init_done && (uint32_t)(now-g_boot_ms) < 3000) {
    r=255; g=110; // yellow: initializing
  } else {
    init_done = true;
    if (error) { // red: error/failsafe
      bool on = ((now/180)&1); if (on) r=255;
    } else if (g_arming.st()==AS_PREARM || g_arming.st()==AS_GESTURE ||
               g_fsm.st()==FS_TAKEOFF) { // purple: arming
      r=180; b=255;
    } else if (g_arming.armed()) { // green blink: in flight
      if ((now/350)&1) g=255;
    } else if (g_remote_heard_ms && (uint32_t)(now-g_remote_heard_ms)<800) {
      b=255; // blue: remote connected
    } else if (st==FS_READY && precheck(g_last_ck)==AR_OK) {
      g=255; // green: ready
    } else if ((int32_t)(g_sensor_check_until-now)>0) {
      if ((now/100)&1) { r=255; g=255; b=255; } // white pulse: sensor check
    } else {
      r=255; g=110; // yellow: initializing / waiting for readiness
    }
  }
  g_status_rgb.setPixelColor(0, g_status_rgb.Color(r,g,b));
  g_status_rgb.show();
}

static inline void boot_stage(uint32_t n){
  watchdog_hw->scratch[0] = n;
  rp2040.wdt_reset();
}
void setup(){
  // [BOOTFIX] BARIS PERTAMA: watchdog jalan dulu, sebelum pin/Serial/sensor/apa pun diinisialisasi.
  rp2040.wdt_begin(NEXEND_BOOT_WDT_MS);
  uint32_t last_stage = watchdog_hw->scratch[0];
  bool wdt_boot = watchdog_caused_reboot();
  watchdog_hw->scratch[0] = 1;
  pinMode(PIN_LED_STATUS,OUTPUT); digitalWrite(PIN_LED_STATUS,LOW);
  Serial.begin(115200);
  if (wdt_boot && last_stage > 1 && last_stage < 30){
    for (uint32_t k=0;k<last_stage;k++){
      rp2040.wdt_reset();
      digitalWrite(PIN_LED_STATUS,HIGH); delay(150); digitalWrite(PIN_LED_STATUS,LOW); delay(250);
    }
    delay(800);
  }
  boot_stage(1);
  delay(150);
  pinMode(PIN_ESC_LED,OUTPUT); digitalWrite(PIN_ESC_LED,ESC_LED_OFF);
  pinMode(PIN_USER_BUTTON, INPUT_PULLUP);
  g_status_rgb.begin();
  g_status_rgb.setBrightness(32);
  g_status_rgb.setPixelColor(0, g_status_rgb.Color(255, 110, 0)); // yellow boot
  g_status_rgb.show();

  Wire.setSDA(PIN_I2C_SDA); Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  boot_stage(2);
  motor_init();
  const float zero[4]={0,0,0,0}; motor_write(zero,false);

  g_buzz.begin(); g_buzz.boot();

  boot_stage(3);
  storage_begin_all();
  boot_stage(4);
  g_health.begin(); g_bb.begin(); g_bbread.begin();
  g_warn.begin(); g_stats.begin(); g_ekf.begin(); g_alt.begin();
  g_arb.clear_latches();
  g_crash.clear(); g_motordiag.clear(); g_inv.clear();
  g_landing.clear(); g_takeoff.clear();
  g_arming.force_disarm();

  if (g_lfs_ok) g_health.ok(S_CFG,(uint32_t)millis(),100);
  else          g_health.bad(S_CFG,(uint32_t)millis(),true);
  if (g_sd_ok){ g_health.ok(S_SD,(uint32_t)millis(),100); g_buzz.sd_ok(); }
  else { g_health.bad(S_SD,(uint32_t)millis(),false); g_warn.raise(W_SD_MISSING, WL_INFO); g_buzz.sd_fail(); }
  if (g_bb.healthy()) g_health.ok(S_BLACKBOX,(uint32_t)millis(),100);
  else                g_health.bad(S_BLACKBOX,(uint32_t)millis(),false);

  boot_stage(5);
  g_fw_ok = fw_check_integrity();
  boot_stage(6);
  if (g_fw_ok) g_health.ok(S_FW, (uint32_t)millis(), 100);
  else { g_health.bad(S_FW, (uint32_t)millis(), true); g_warn.raise(W_FW_INTEGRITY, WL_CRITICAL); }

  bool cfg_ok = g_cfg.begin();
  if (!cfg_ok){
    g_cfg.make_defaults();
    g_cfg.save(g_cfg.c());
    cfg_ok = true;
  }
  {
    float b0[3] = {0.f, 0.f, 0.f};
    if (g_cfg.valid()){
      for (int i = 0; i < 3; i++){
        float v = g_cfg.c().gyro_bias[i];
        if (isfinite(v) && fabsf(v) < 0.1745f) b0[i] = v;   // 10 deg/s; lebih besar dianggap data sampah
      }
    }
    g_ekf.set_bias(b0);
  }
  boot_stage(7);
  g_ctrl.begin(g_cfg.c().pid);
  g_adapt.begin(g_cfg.c().pid);
  g_adapt.enabled = g_cfg.c().auto_stab_enabled != 0;
  g_adapt.strength = g_cfg.c().auto_stab_strength;
  g_adapt.max_adjust_pct = g_cfg.c().auto_pid_max_adjust;
  g_adapt.update_ms = g_cfg.c().auto_pid_update_ms;
  g_adapt.smoothing_ms = g_cfg.c().auto_pid_smoothing_ms;

  uint8_t b_cells = NEXEND_BATT_CELLS; // [1S] dipaksa dari macro, bukan dari config tersimpan
  uint16_t b_cap = g_cfg.c().batt_capacity_mah ? g_cfg.c().batt_capacity_mah : 3000;
  uint16_t b_ina = g_cfg.c().ina_cal_value ? g_cfg.c().ina_cal_value : 4096;
  bool b_cal = g_cfg.c().ina_cal_ok != 0;
  BattChem b_chem = (BattChem)clampf((float)g_cfg.c().batt_chem, 0.f, 4.f);
  g_batt.begin(b_cells, b_cap, b_ina, b_cal, b_chem);
  if (!b_cal) g_warn.raise(W_INA_UNCAL,WL_INFO);

  boot_stage(8);
  if (!g_imu.begin()) g_health.bad(S_IMU,(uint32_t)millis(),true);
  else { g_health.ok(S_IMU,(uint32_t)millis(),100); g_health.ok(S_GYRO,(uint32_t)millis(),100); g_health.ok(S_ACCEL,(uint32_t)millis(),100); }

  boot_stage(9);
  if (!g_baro.begin(ADDR_BMP)) g_health.bad(S_BARO,(uint32_t)millis(),true);
  else {
    g_baro.setSampling(Adafruit_BMP280::MODE_NORMAL,
                       Adafruit_BMP280::SAMPLING_X2,
                       Adafruit_BMP280::SAMPLING_X16,
                       Adafruit_BMP280::FILTER_X16,
                       Adafruit_BMP280::STANDBY_MS_1);
    g_health.ok(S_BARO,(uint32_t)millis(),100);
  }

  boot_stage(10);
  g_mag.init(); delay(20);
  g_health.ok(S_MAG,(uint32_t)millis(),80);

  boot_stage(11);
  g_gps_ser.setRX(PIN_GPS_RX); g_gps_ser.setTX(PIN_GPS_TX);
  g_gps_ser.begin(9600);

  boot_stage(12);
  if (!g_ina.begin()) g_health.bad(S_BATT,(uint32_t)millis(),true);
  else {
    g_ina.setCalibration_32V_2A();
    if (b_cal){
      Wire.beginTransmission(ADDR_INA);
      Wire.write(0x05);
      Wire.write((uint8_t)((b_ina>>8)&0xFF));
      Wire.write((uint8_t)(b_ina&0xFF));
      Wire.endTransmission();
    }
    g_health.ok(S_BATT,(uint32_t)millis(),100);
  }

#if NEXEND_HAVE_TOF
  boot_stage(13);
  Wire.beginTransmission(TOF_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    // [BOOTFIX] timeout HARUS di-set sebelum init(): io_timeout default 0 = tunggu selamanya di dalam init()
    g_tof.setTimeout(TOF_TIMEOUT_MS);
    if (g_tof.init()) {
      rp2040.wdt_reset();
      g_tof.setMeasurementTimingBudget(TOF_TIMING_US);
      g_tof.startContinuous(TOF_PERIOD_MS);
      g_tof_ready = true;
      g_health.ok(S_TOF, (uint32_t)millis(), 100);
    } else {
      g_health.bad(S_TOF, (uint32_t)millis(), false);
    }
  } else {
    g_health.bad(S_TOF, (uint32_t)millis(), false);
  }
#endif

  boot_stage(14);
  if (!g_radio.begin()){ g_health.bad(S_RADIO_A,(uint32_t)millis(),true); }

  boot_stage(15);
  g_fsm.begin((uint32_t)millis());
  g_selftest.evaluate(g_health,g_radio,cfg_ok,g_fw_ok);
  post_event(WL_INFO,EV_SELFTEST,g_selftest.result);
  post_event(g_sd_ok?WL_INFO:WL_WARNING, g_sd_ok?EV_SD_MOUNT:EV_SD_FAIL, 0);
  post_event(g_fw_ok?WL_INFO:WL_CRITICAL, g_fw_ok?EV_FW_OK:EV_FW_BAD, 0);

  boot_stage(0);
  rp2040.wdt_begin(NEXEND_WDT_TIMEOUT_S * 1000UL);   // timeout ketat 3 dtk untuk loop() normal

  digitalWrite(PIN_LED_STATUS,HIGH);
  g_boot_ms = millis();
}

// ============ LOOP ============
static constexpr uint32_t CTRL_US=1000;
static constexpr uint32_t MED_DIV=4;
static constexpr uint32_t SLOW_DIV=10;
static constexpr uint32_t GPS_DIV=5;
static uint32_t last_ctrl_us=0;
static uint32_t ctrl_count=0;

// ---- Identitas USB: MSG_HELLO peran 2 (= FC langsung) untuk APK NEXEND FLY ----
// Frame APK: AA 55 | msgId | seq | len | payload | CRC16-CCITT(init FFFF, poly 1021) high,low
static uint16_t usb_crc16(const uint8_t* d, uint8_t n){
  uint16_t crc = 0xFFFF;
  for (uint8_t i=0;i<n;i++){
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t j=0;j<8;j++) crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
  }
  return crc;
}
static void usb_hello_send(){
  static uint8_t seq = 0;
  if (!Serial || Serial.availableForWrite() < 16) return;   // tidak boleh memblok loop terbang
  uint8_t p[8] = {0xAA, 0x55, 0x01, seq++, 0x01, 0x02, 0, 0};  // MSG_HELLO, len 1, peran 2 = FC
  uint16_t c = usb_crc16(p+2, 4);
  p[6] = (uint8_t)(c >> 8); p[7] = (uint8_t)(c & 0xFF);
  Serial.write(p, 8);
}
// [USB] Telemetri FC -> APK lewat kabel (Nexend Configurator). Format frame sama dgn APK:
// AA 55 | msg | seq | len | payload (little-endian) | CRC16-CCITT (big-endian) atas msg..payload.
static inline int16_t usb_i16(float v){
  if (!isfinite(v)) return 0;
  if (v >  32767.f) return  32767;
  if (v < -32768.f) return -32768;
  return (int16_t)lroundf(v);
}
static void usb_send_frame(uint8_t msg, const uint8_t* pl, uint8_t n){
  static uint8_t seq = 0;
  if (n > 32) return;
  if ((int)Serial.availableForWrite() < (int)(7 + n)) return;   // jangan pernah memblokir loop kontrol
  uint8_t b[7 + 32];
  b[0]=0xAA; b[1]=0x55; b[2]=msg; b[3]=seq++; b[4]=n;
  memcpy(b+5, pl, n);
  uint16_t c = usb_crc16(b+2, (uint8_t)(3 + n));
  b[5+n] = (uint8_t)(c >> 8); b[6+n] = (uint8_t)(c & 0xFF);
  Serial.write(b, 7 + n);
}
static void usb_telem_tick(uint32_t now){
  static uint32_t lt=0, li=0, lg=0, lb=0, lh=0;
  const EkfState& ekf = g_ekf.state();
  if ((uint32_t)(now-lt) >= 100){        // 0x10 TELEMETRY 10 Hz: roll,pitch,yaw,alt,vs,hspd (int16 x0.1) + armed + mode
    lt = now;
    float hs = sqrtf(g_pos.st().vn*g_pos.st().vn + g_pos.st().ve*g_pos.st().ve);
    int16_t v[6] = { usb_i16(USB_SGN_ROLL*ekf.roll*RAD_TO_DEG*10.f), usb_i16(USB_SGN_PITCH*ekf.pitch*RAD_TO_DEG*10.f), usb_i16(USB_SGN_YAW*ekf.yaw*RAD_TO_DEG*10.f),
                     usb_i16(g_alt.st().alt_m*10.f), usb_i16(g_alt.st().vs_mps*10.f), usb_i16(hs*10.f) };
    uint8_t p[14]; memcpy(p, v, 12);
    p[12] = g_arming.armed() ? 1 : 0;
    uint8_t md = 0;                      // peta ke ID mode APK: 2=STABILIZE 3=ALT_HOLD 4=POS_HOLD 5=RTH, lainnya N/A
    switch (g_mode){ case FM_STABILIZE: md=2; break; case FM_ALTHOLD: md=3; break; case FM_LOITER: md=4; break; case FM_RTH: md=5; break; default: md=0; }
    p[13] = md;
    usb_send_frame(0x10, p, 14);
    // 0x17 MAG: heading (int16 x0.1 derajat). Ini yaw hasil fusi EKF, BUKAN heading kompas mentah.
    float h = USB_SGN_YAW*ekf.yaw*RAD_TO_DEG; while (h < 0.f) h += 360.f; while (h >= 360.f) h -= 360.f;
    int16_t hd = usb_i16(h*10.f);
    if (g_health.usable(S_MAG)) usb_send_frame(0x17, (const uint8_t*)&hd, 2);   // hanya kirim bila kompas benar-benar sehat
    // 0x16 RANGE: ToF (cm, 0xFFFF = di luar jangkauan / tidak valid), optical flow tidak ada -> 0
    const AltEst& alt_ = g_alt.st();
    uint16_t tofcm = (alt_.tof_ok && isfinite(alt_.tof_m) && alt_.tof_m > 0.f && g_health.usable(S_TOF)) ? (uint16_t)clampf(alt_.tof_m*100.f, 0.f, 65000.f) : 0xFFFF;
    uint8_t rg[5]; memcpy(rg, &tofcm, 2); rg[2]=0; rg[3]=0; rg[4]=0;
    usb_send_frame(0x16, rg, 5);
  }
  if ((uint32_t)(now-li) >= 50){         // 0x11 IMU 20 Hz: gyro deg/s (x0.1), accel g (x0.001)
    li = now;
    int16_t w[6] = { usb_i16(USB_SGN_ROLL*g_sens.gyro[0]*RAD_TO_DEG*10.f), usb_i16(USB_SGN_PITCH*g_sens.gyro[1]*RAD_TO_DEG*10.f), usb_i16(USB_SGN_YAW*g_sens.gyro[2]*RAD_TO_DEG*10.f),
                     usb_i16(USB_SGN_PITCH*g_sens.accel[0]*1000.f), usb_i16(USB_SGN_ROLL*g_sens.accel[1]*1000.f), usb_i16(g_sens.accel[2]*1000.f) };
    usb_send_frame(0x11, (const uint8_t*)w, 12);
  }
  if ((uint32_t)(now-lg) >= 200){        // 0x13 GPS 5 Hz: lat,lon (int32 x1e7), alt (int16 x0.1), sats, fix, hdop x0.1
    lg = now;
    bool fresh = (g_sens.gps_last_ms != 0) && ((uint32_t)(now - g_sens.gps_last_ms) < 2000);
    int32_t la = (int32_t)(g_sens.lat*1e7), lo = (int32_t)(g_sens.lon*1e7);
    int16_t ga = g_gps.altitude.isValid() ? usb_i16((float)g_gps.altitude.meters()*10.f) : 0;
    uint8_t p[13]; memcpy(p, &la, 4); memcpy(p+4, &lo, 4); memcpy(p+8, &ga, 2);
    p[10] = fresh ? g_sens.sats : 0;
    p[11] = fresh ? g_sens.fix : 0;
    p[12] = (uint8_t)clampf(g_sens.hdop*10.f, 0.f, 255.f);
    usb_send_frame(0x13, p, 13);
  }
  if ((uint32_t)(now-lb) >= 500){        // 0x14 BATTERY 2 Hz: V x0.01, A x0.01, mAh, persen
    lb = now;
    int16_t bv = usb_i16(g_sens.batt_v*100.f), ba = usb_i16(g_sens.batt_a*100.f), bm = usb_i16((float)g_batt.mah_used());
    uint8_t p[7]; memcpy(p, &bv, 2); memcpy(p+2, &ba, 2); memcpy(p+4, &bm, 2); p[6] = g_batt.pct();
    usb_send_frame(0x14, p, 7);
  }
  if ((uint32_t)(now-lh) >= 500){        // 0x15 SENSOR_HEALTH 2 Hz: bit0 IMU,1 Baro,2 Mag,3 GPS,4 ToF,5 Flow (detected, lalu ok)
    lh = now;
    static const SubsysId m[5] = { S_IMU, S_BARO, S_MAG, S_GPS, S_TOF };
    uint8_t det = 0, ok = 0;
    for (int i=0;i<5;i++){
      bool u = g_health.usable(m[i]);
      if (u || g_health.failed(m[i])) det |= (1<<i);
      if (u) ok |= (1<<i);
    }
    uint8_t p[2] = { det, ok };
    usb_send_frame(0x15, p, 2);
  }
}
// [USB-CMD] Penerima perintah dari APK (kabel). Frame: AA 55 | msg | seq | len | payload | CRC16 (BE).
// MSG_COMMAND_REQ 0x40: payload[0] = ID perintah. Balasan MSG_COMMAND_ACK 0x41: [cmd][status], status 0 = OK.
// ID: 0x50 kalibrasi akselerometer, 0x51 gyro, 0x53 kompas. 0x52 (level) belum didukung -> status 5.
// status: 1 = sedang kalibrasi lain, 2 = drone tidak DISARM, 5 = belum didukung.
static void usb_cmd_handle(const uint8_t* pl, uint8_t n){
  if (n < 1) return;
  uint8_t cmd = pl[0], st = 5, wiz = 0;
  if (cmd == 0x50) wiz = CMD_CALIB_ACCEL; else if (cmd == 0x51) wiz = CMD_CALIB_GYRO; else if (cmd == 0x53) wiz = CMD_CALIB_MAG;
  if (wiz){
    if (g_arming.st() != AS_DISARMED) st = 2;
    else if (g_calwiz.busy()) st = 1;
    else { calib_wizard_start(wiz); st = 0; }
  }
  uint8_t a[2] = { cmd, st };
  usb_send_frame(0x41, a, 2);
}
static void usb_rx_tick(uint32_t now){
  static uint8_t buf[40]; static uint8_t idx = 0; static uint32_t last_ms = 0;
  if (idx && (uint32_t)(now - last_ms) > 200) idx = 0;
  int guard = 64;
  while (guard-- > 0 && Serial.available()){
    uint8_t c = (uint8_t)Serial.read(); last_ms = now;
    if (idx == 0 && c != 0xAA) continue;
    if (idx == 1 && c != 0x55){ idx = (c == 0xAA) ? 1 : 0; continue; }
    buf[idx++] = c;
    if (idx >= 5){
      uint8_t len = buf[4];
      if (len > 32){ idx = 0; continue; }
      if (idx >= (uint8_t)(7 + len)){
        uint16_t crc = usb_crc16(buf + 2, (uint8_t)(3 + len));
        if (buf[5+len] == (uint8_t)(crc >> 8) && buf[6+len] == (uint8_t)(crc & 0xFF) && buf[2] == 0x40) usb_cmd_handle(buf + 5, len);
        idx = 0;
      }
    }
  }
}
static void usb_id_tick(uint32_t now){
  static uint32_t last = 0;
  usb_rx_tick(now);   // [USB-CMD] terima perintah dari APK
  if ((uint32_t)(now-last) >= 1000){ last = now; usb_hello_send(); }
  usb_telem_tick(now);   // [USB] data IMU/attitude ke Configurator
}

static void esc_led_update(uint32_t now){
  static int last = -1;
  bool on;
  if ((uint32_t)(now-g_boot_ms) < 3000) on = false;
  else {
    FlightState s = g_fsm.st();
    if (s==FS_CRITICAL_SENSOR || s==FS_MOTOR_FAULT || s==FS_CRASH || s==FS_INVERTED ||
        s==FS_FAILSAFE || s==FS_FAILSAFE_DESCEND) on = ((now/50)&1);
    else if (g_arming.armed()) on = true;
    else if (s==FS_READY && precheck(g_last_ck)==AR_OK) on = ((now/125)&1);
    else on = ((now/500)&1);
  }
  if ((int)on != last){
    last = (int)on;
    digitalWrite(PIN_ESC_LED, on ? ESC_LED_ON : ESC_LED_OFF);
  }
}

static void handle_other_pkt(const uint8_t* buf){ // [NXR]
  if (nex_check_header(buf, PKT_CMD)){
    PktCmd c; memcpy(&c,buf,sizeof(c)); handle_cmd(c,g_mag);
  } else if (nex_check_header(buf, PKT_BB_REQ)){
    PktBbReq r; memcpy(&r,buf,sizeof(r));
    if (nex_verify(r)){
      PktBbDat d{};
      d.h.s0=NEX_SYNC0; d.h.s1=NEX_SYNC1; d.h.ver=NEX_PROTO_VER; d.h.type=PKT_BB_DAT;
      static uint8_t seq=0; d.h.seq=seq++;
      d.off=r.off;
      d.len=g_bbread.read_chunk(r.off,d.data,sizeof(d.data));
      nex_seal(d); g_radio.send(&d,sizeof(d));
    }
  }
}

void loop(){
  rp2040.wdt_reset();
  uint32_t now_us=(uint32_t)micros();
  g_t=(uint32_t)millis();

  if ((uint32_t)(now_us-last_ctrl_us)>=CTRL_US){
    float dt=(now_us-last_ctrl_us)*1e-6f;
    last_ctrl_us=now_us;
    if (!isfinite(dt) || dt>0.01f) dt=0.01f;
    if (dt<=0.f) dt = 0.001f;
    ctrl_count++;
    if ((ctrl_count % LED_FPS_DIV)==0 && (uint32_t)(g_t-g_boot_ms)>=3000)
      digitalWrite(PIN_LED_STATUS,!digitalRead(PIN_LED_STATUS));

    read_imu(g_t);
    if ((ctrl_count%MED_DIV)==0)  read_mag(g_t,g_mag);
    if ((ctrl_count%SLOW_DIV)==0){
      read_baro(g_t);
      read_batt(g_t);
#if NEXEND_HAVE_TOF
      read_tof(g_t);
#endif
    }
    if ((ctrl_count%GPS_DIV)==0) read_gps(g_t,g_gps);

    control_tick(dt,g_t,g_mag);
  }

  calib_wizard_tick();
  user_button_tick(g_t);
  status_rgb_tick(g_t);
  calib_wizard_autoclear();

  { // [NXR] paket CMD/BB_REQ diambil dari antrean LinkRadio (bukan membaca FIFO langsung)
    uint8_t obuf[NEX_PKT_SIZE];
    while (g_radio.pop_other(obuf)) handle_other_pkt(obuf);
  }

  // [NXR] telemetri dijadwalkan 1 paket per 10 ms (A,B,EV,C,D,EV,E,EV) agar blocking radio tidak menumpuk
  static uint32_t last_telem=0; static uint8_t telem_slot=0;
  if (g_t-last_telem>=10){
    last_telem=g_t;
    uint8_t slot=telem_slot; telem_slot=(telem_slot+1)&7;
    switch (slot){
      case 0: {
    PktTelemA a{};
    a.h.s0=NEX_SYNC0; a.h.s1=NEX_SYNC1; a.h.ver=NEX_PROTO_VER; a.h.type=PKT_TELEM_A;
    static uint8_t sa=0; a.h.seq=sa++;
    a.fsm=(uint8_t)g_fsm.st(); a.arm=(uint8_t)g_arming.st();
    a.sys=(uint8_t)g_arb_out.status; a.warn=(uint8_t)g_arming.reject();
    a.batt_mv=(uint16_t)(g_sens.batt_v*1000);
    a.batt_ca=(int16_t)(g_sens.batt_a*100);
    a.alt_dm=(int16_t)(g_alt.st().alt_m*10);
    float hd = g_home_dist; if (hd<0) hd=0; if (hd>65000.f) hd=65000.f;
    a.home_dist_m=(uint16_t)hd;
    a.gs_cms=(uint16_t)(g_sens.gs_mps*100);
    a.sats=g_sens.sats; a.fix=g_sens.fix;
    nex_seal(a); g_radio.send(&a,sizeof(a));

        break; }
      case 1: {
    PktTelemB b{};
    b.h.s0=NEX_SYNC0; b.h.s1=NEX_SYNC1; b.h.ver=NEX_PROTO_VER; b.h.type=PKT_TELEM_B;
    static uint8_t sb=0; b.h.seq=sb++;
    b.heading_cdeg=(int16_t)(g_ekf.state().yaw*RAD_TO_DEG*100.f);
    b.rA_q=g_radio.q(); b.rB_q=0;
    b.rA_st=(uint8_t)g_radio.st(); b.rB_st=(uint8_t)LS_INIT; // [1R] field B dicadangkan agar layout paket tetap
    b.motor_mask=g_arb_out.motors_enabled?0x0F:0x00;
    b.tof_st=g_sens.tof_m>0?1:0;
    b.tof_mm=g_sens.tof_m>0?(uint16_t)(g_sens.tof_m*1000):0;
    b.mode=(uint8_t)g_mode;
    b.rth_gate=g_fsm.rth_gate();
    nex_seal(b); g_radio.send(&b,sizeof(b));

        break; }
      case 3: {
    PktTelemC c{};
    c.h.s0=NEX_SYNC0; c.h.s1=NEX_SYNC1; c.h.ver=NEX_PROTO_VER; c.h.type=PKT_TELEM_C;
    static uint8_t sc=0; c.h.seq=sc++;
    g_health.pack(c.health);
    c.warn_top_lvl=(uint8_t)g_warn.top_level();
    c.warn_top=g_warn.top_id();
    c.calib_ok=g_cfg.valid()?1:0;
    c.calib_state=(uint8_t)g_calwiz.state;
    c.calib_progress=g_calwiz.progress;
    c.fw_flags = (g_fw_ok?1:0) | ((g_cfg.valid()?g_cfg.c().failsafe_action:0)<<1);
    c.fw_flags |= (g_home_valid?(1<<6):0) | (g_have_rc?(1<<7):0); // [NXR] bit6=home valid, bit7=RC aktif
    nex_seal(c); g_radio.send(&c,sizeof(c));

        break; }
      case 4: {
    PktTelemD d{};
    d.h.s0=NEX_SYNC0; d.h.s1=NEX_SYNC1; d.h.ver=NEX_PROTO_VER; d.h.type=PKT_TELEM_D;
    static uint8_t sd=0; d.h.seq=sd++;
    d.lat_e7=(int32_t)(g_sens.lat*1e7);
    d.lon_e7=(int32_t)(g_sens.lon*1e7);
    d.fw_packed=NEXEND_FW_PACKED;
    d.mah_used=g_batt.mah_used();
    d.wh_centi=g_batt.wh_centi();
    d.pct=g_batt.pct();
    d.batt_flags=0;
    if (g_batt.sag_active())    d.batt_flags|=1<<0;
    if (g_batt.pct()<20)        d.batt_flags|=1<<1;
    if (g_batt.pct()<5)         d.batt_flags|=1<<2;
    if (g_batt.cal_ok())        d.batt_flags|=1<<3;
    if (g_sens.batt_a<-0.2f)    d.batt_flags|=1<<4;
    if (g_sd_ok)                d.batt_flags|=1<<5;
    for (int i=0;i<4;i++) d.motor_pct[i]=(uint8_t)(clampf(g_arb_out.out[i],0.f,1.f)*100.f);
    d.cell_mv_min=(int16_t)(g_batt.cell_load_v()*1000.f);
    nex_seal(d); g_radio.send(&d,sizeof(d));

        break; }
      case 6: {
    PktTelemE e{};
    e.h.s0=NEX_SYNC0; e.h.s1=NEX_SYNC1; e.h.ver=NEX_PROTO_VER; e.h.type=PKT_TELEM_E;
    static uint8_t se=0; e.h.seq=se++;
    const PidProfile& pp = g_ctrl.profile();
    e.stab_on = (g_cfg.valid()?g_cfg.c().stabilizer_enabled:1);
    e.auto_stab_on = g_adapt.enabled?1:0;
    e.auto_strength = g_adapt.strength;
    e.active_profile = g_cfg.valid()?g_cfg.c().active_profile:0;
    e.roll_kp_x100  = (int16_t)clampf(pp.rate_roll.kp *100.f, -32768.f, 32767.f);
    e.roll_ki_x100  = (int16_t)clampf(pp.rate_roll.ki *100.f, -32768.f, 32767.f);
    e.roll_kd_x100  = (int16_t)clampf(pp.rate_roll.kd *100.f, -32768.f, 32767.f);
    e.pitch_kp_x100 = (int16_t)clampf(pp.rate_pitch.kp*100.f, -32768.f, 32767.f);
    e.pitch_ki_x100 = (int16_t)clampf(pp.rate_pitch.ki*100.f, -32768.f, 32767.f);
    e.pitch_kd_x100 = (int16_t)clampf(pp.rate_pitch.kd*100.f, -32768.f, 32767.f);
    e.yaw_kp_x100   = (int16_t)clampf(pp.rate_yaw.kp *100.f, -32768.f, 32767.f);
    e.yaw_ki_x100   = (int16_t)clampf(pp.rate_yaw.ki *100.f, -32768.f, 32767.f);
    e.yaw_kd_x100   = (int16_t)clampf(pp.rate_yaw.kd *100.f, -32768.f, 32767.f);
    e.adaptive_adjust_pct = g_adapt.last_adjust_pct();
    nex_seal(e); g_radio.send(&e,sizeof(e));

        break; }
      default:
        if (g_evq.tail != g_evq.head){
          g_radio.send(&g_evq.ev[g_evq.tail],sizeof(PktEvent));
          g_evq.tail=(g_evq.tail+1)&15;
        }
        break;
    }
  }

  esc_led_update(g_t);
  usb_id_tick(g_t);

  static uint32_t last_cfg_flush=0;
  if (g_t-last_cfg_flush>5000 && !g_arming.armed()){
    last_cfg_flush=g_t;
    if (g_cfg.dirty()) g_cfg.flush_if_dirty();
  }
}