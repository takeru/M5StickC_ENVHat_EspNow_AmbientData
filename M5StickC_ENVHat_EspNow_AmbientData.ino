#include <esp_now.h>
#include <WiFi.h>
#include <M5StickC.h>

#include "DHT12.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>

#define SUPER_SLEEP 0
#if SUPER_SLEEP
#include <esp_wifi.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <driver/adc.h>
#endif

#include <Syslog.h>
#include <WiFiUdp.h>
#include <Preferences.h>

#define SERIAL_LOG 1
#if SERIAL_LOG
#define LOGF(...) Serial.printf("[%6d] ", millis()); Serial.printf(__VA_ARGS__)
#else
#define LOGF(...)
#endif

RTC_DATA_ATTR uint32_t boot_counter    = 0;
RTC_DATA_ATTR uint32_t last_elapsed_ms = 0;
RTC_DATA_ATTR uint32_t error_count     = 0;

struct Profile {
  int index;
  uint8_t* mac;
  unsigned int channelId;
  char* writeKey;
};

#include "config.h"

struct Profile *profile = NULL;
struct Profile *server_profile = NULL;

#define LED_BUILTIN 10
#define ESPNOW_CHANNEL 3
#define PROFILE_INDEX_SERVER 2

enum role_t{
  ROLE_NULL   = 0,
  ROLE_SERVER = 1,
  ROLE_CLIENT = 2
};
role_t role = ROLE_NULL;

#define DataPacket_SIGNATURE 0x27F51F4C
#define DataPacket_VERSION   1
struct DataPacket {
  uint32_t signature;
  uint32_t version;
  uint32_t boot_counter;
  uint32_t last_elapsed_ms;
  int32_t v1_temperature_x100;
  int32_t v2_humidity_x100;
  int32_t v3_pressure_x100;
  int32_t v4_vbatAxp_x100;
  int32_t v5_tempAxp_x100;
  int32_t v6_vusbinAxp_x100;
  int32_t v7;
  int32_t v8;
};
DataPacket dp_recv[PROFILE_COUNT];

#define AckPacket_SIGNATURE 0x1D7A10FE
#define AckPacket_VERSION   1
struct AckPacket {
  uint32_t signature;
  uint32_t version;
  int32_t sleep_ms;
};

static unsigned long curr_ms = 0;
#define next_ms (curr_ms+60000)
void server_setup(){
  memset(&dp_recv, 0, sizeof(DataPacket)*PROFILE_COUNT);
  _esp_now_init();
  esp_now_register_recv_cb(server_data_recv_cb);

  RTC_DateTypeDef date;
  M5.Rtc.GetData(&date);
  RTC_TimeTypeDef time;
  M5.Rtc.GetTime(&time);
  curr_ms = millis() + (60-time.Seconds) * 1000;

  char datetime[20];
  sprintf(datetime, "%04d-%02d-%02d %02d:%02d:%02d", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
  LOGF("datetime=%s curr_ms=%d next_ms=%d\n", datetime, curr_ms, next_ms);
}

#define MARGIN_SEC 2
#define WINDOW_PER_CLIENT_MS 500
void server_loop(){
  static int state = 0;

  switch(state){
  case 0:
    state = 1;
    break;
  case 1:
    if((curr_ms + MARGIN_SEC*1000 + PROFILE_COUNT*WINDOW_PER_CLIENT_MS) < millis()){
      state = 2;
    }
    break;
  case 2:
    {
      Preferences preferences;
      char wifi_ssid[33];
      char wifi_key[65];
      preferences.begin("Wi-Fi", true);
      preferences.getString("ssid", wifi_ssid, sizeof(wifi_ssid));
      preferences.getString("key", wifi_key, sizeof(wifi_key));
      preferences.end();
      WiFi.begin(wifi_ssid, wifi_key);
      unsigned long wifi_start_ms = millis();
      bool wifiOK = false;
      while(1){
        delay(1);
        wifiOK = WiFi.status()==WL_CONNECTED;
        if(wifiOK || wifi_start_ms+10*1000 < millis()){ break; }
      }

      if(wifiOK){
        WiFiUDP udpClient;
        Syslog syslog(udpClient, SYSLOG_PROTO_IETF);
        syslog.server(SYSLOG_SERVER, SYSLOG_PORT);
        static String deviceHostname;
        deviceHostname = String("M5StickC-")+profile->index;
        syslog.deviceHostname(deviceHostname.c_str());
        syslog.appName("EnvNowC");
        syslog.defaultPriority(LOG_KERN);
        syslog.logMask(LOG_UPTO(LOG_DEBUG));  
        for(int i=0; i<PROFILE_COUNT; i++){
          DataPacket *dp = &dp_recv[i];
          Profile *pf = &profiles[i];
          if(pf==profile){ make_DataPacket(dp); }
          if(dp->signature == 0){ continue; }
          String s = DataPacket_string(dp);
          LOGF("Syslog: %s\n", s.c_str());
          syslog.logf(LOG_INFO, "src=%s %s", mac_string(pf->mac).c_str(), s.c_str());
          // TODO: Send to AmbientData HERE...
        }
        delay(50);
        udpClient.stop();

        LOGF("ntp_to_rtc begin.\n");
        ntp_to_rtc();
        LOGF("ntp_to_rtc done.\n");
      }else{
        LOGF("No WiFi connection.\n");
      }
    }
    state = 3;
    break;
  case 3:
    {
      int32_t sleep_ms = next_ms - millis() - (MARGIN_SEC*1000);
      LOGF("super_sleep millis=%d sleep_ms=%d\n", millis(), sleep_ms);
      if(sleep_ms <= 0){ sleep_ms = 1; }
      super_sleep(sleep_ms*1000);
    }
    break;
  }
}

void server_data_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  LOGF("Recv from: %s len=%d\n", mac_string(mac_addr).c_str(), data_len);
  if(sizeof(DataPacket) != data_len){ LOGF("invalid data_len.\n"); return; }
  if(((DataPacket*)data)->signature != DataPacket_SIGNATURE){ LOGF("invalid SIGNATURE.\n"); return; }
  if(((DataPacket*)data)->version   != DataPacket_VERSION){   LOGF("invalid VERSION.\n");   return; }

  Profile *source_profile = NULL;
  DataPacket *dp = NULL;
  for(int i=0; i<PROFILE_COUNT; i++){
    if(memcmp(mac_addr, profiles[i].mac, 6)==0){
      source_profile = &profiles[i];
      dp = &dp_recv[i];
      break;
    }
  }
  if(source_profile == NULL){ LOGF("invalid source.\n"); return; }

  memcpy(dp, data, sizeof(DataPacket));
  String s = DataPacket_string(dp);
  LOGF("DataPacket: %s\n", s.c_str());

  // send back ack
  send_ack(source_profile);
}

String DataPacket_string(DataPacket *dp){
  char tmp[256];
  snprintf(tmp, 256, "boot_counter=%d last_elapsed_ms=%d v1_temperature_x100=%d v2_humidity_x100=%d v3_pressure_x100=%d v4_vbatAxp_x100=%d v5_tempAxp_x100=%d v6_vusbinAxp_x100=%d v7=%d v8=%d",
    dp->boot_counter,
    dp->last_elapsed_ms,
    dp->v1_temperature_x100,
    dp->v2_humidity_x100,
    dp->v3_pressure_x100,
    dp->v4_vbatAxp_x100,
    dp->v5_tempAxp_x100,
    dp->v6_vusbinAxp_x100,
    dp->v7,
    dp->v8
  );
  return String(tmp);
}

void send_ack(const Profile *source_profile){
  _esp_now_add_peer(source_profile->mac);
  uint8_t data[250];
  size_t len = make_ack(data, source_profile);
  //LOGF("data=%s\n", formatHex(data, len));
  esp_err_t code = esp_now_send(source_profile->mac, data, len);
  LOGF("esp_now_send len=%d %s(%d)\n", len, esp_err_to_name(code), code);
}

size_t make_ack(uint8_t *data, const Profile *source_profile){
  int32_t sleep_ms = (next_ms - millis()) + source_profile->index * WINDOW_PER_CLIENT_MS;

  struct AckPacket *ap = (AckPacket*)data;
  ap->signature = AckPacket_SIGNATURE;
  ap->version   = AckPacket_VERSION;
  ap->sleep_ms  = sleep_ms;
  return sizeof(AckPacket);
}

void _esp_now_add_peer(const uint8_t *mac){
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = 0;
  memcpy(peer.peer_addr, mac, 6);

  bool exists = esp_now_is_peer_exist(peer.peer_addr);
  if(!exists){
    esp_err_t code = esp_now_add_peer(&peer);
    LOGF("esp_now_add_peer %s(%d)\n", esp_err_to_name(code), code);
  }
}


int32_t client_sleep_ms = 0;

void client_setup(){
}

void client_loop(){
  static int state = 0;
  static unsigned long state1_time_limit = 0;
  
  switch(state){
  case 0:
    {
      uint8_t data[250];
      size_t len = make_data(data);
      //LOGF("data=%s\n", formatHex(data, len));

      _esp_now_init();
      _esp_now_add_peer(server_profile->mac);
      esp_now_register_send_cb(client_data_sent_cb);
      esp_now_register_recv_cb(client_data_recv_cb);
      esp_err_t code = esp_now_send(server_profile->mac, data, len);
      LOGF("esp_now_send len=%d %s(%d)\n", len, esp_err_to_name(code), code);
      state = 1;
      state1_time_limit = millis() + 100;
    }
    break;
  case 1:
    if(0 < client_sleep_ms){
      state = 2;
      error_count = 0;
    }
    if(state1_time_limit < millis()){
      error_count++;
      if(error_count <= 60/MARGIN_SEC){
        client_sleep_ms = MARGIN_SEC*1000;
      }else{
        client_sleep_ms = (60-MARGIN_SEC)*1000;
      }
    }
    break;
  case 2:
    LOGF("super_sleep millis=%d client_sleep_ms=%d\n", millis(), client_sleep_ms);
    last_elapsed_ms = millis();
    super_sleep(client_sleep_ms*1000);
    break;
  }
}

void client_data_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  LOGF("Sent to: %s %s\n",
    mac_string(mac_addr).c_str(),
    status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail"
  );
}

void client_data_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  LOGF("Recv from: %s len=%d\n", mac_string(mac_addr).c_str(), data_len);
  if(sizeof(AckPacket) != data_len){ LOGF("invalid data_len.\n"); return; }
  if(((AckPacket*)data)->signature != AckPacket_SIGNATURE){ LOGF("invalid SIGNATURE.\n"); return; }
  if(((AckPacket*)data)->version   != AckPacket_VERSION){   LOGF("invalid VERSION.\n");   return; }

  Profile *source_profile = NULL;
  for(int i=0; i<PROFILE_COUNT; i++){
    if(memcmp(mac_addr, profiles[i].mac, 6)==0){
      source_profile = &profiles[i];
      break;
    }
  }
  if(source_profile == NULL){ LOGF("invalid source.\n"); return; }
  if(source_profile != server_profile){ LOGF("invalid server.\n"); return; }

  AckPacket ap;
  memcpy(&ap, data, sizeof(AckPacket));
  LOGF("sleep_ms=%d\n",
    ap.sleep_ms
  );
  client_sleep_ms = ap.sleep_ms;
}

size_t make_data(uint8_t *data){
  struct DataPacket *dp = (DataPacket*)data;
  make_DataPacket(dp);
  return sizeof(DataPacket);
}

void make_DataPacket(struct DataPacket *dp){
  Wire.begin(0,26); // ENV Hat : dht12 & bme
  DHT12 dht12;
  Adafruit_BMP280 bme;
  float  temperature = dht12.readTemperature(); // 50ms
  float  humidity    = dht12.readHumidity(); // 50ms
  float  pressure    = bme.begin(0x76) ? bme.readPressure() / 100.0 : 0; // 100ms
  double vbatAxp     = M5.Axp.GetVbatData() * 1.1 / 1000;
  float  tempAxp     = -144.7 + M5.Axp.GetTempData() * 0.1;
  double vusbinAxp   = M5.Axp.GetVusbinData() * 1.7 / 1000;

  dp->signature           = DataPacket_SIGNATURE;
  dp->version             = DataPacket_VERSION;
  dp->v1_temperature_x100 = 100 * temperature; 
  dp->v2_humidity_x100    = 100 * humidity;
  dp->v3_pressure_x100    = 100 * pressure;
  dp->v4_vbatAxp_x100     = 100 * vbatAxp;
  dp->v5_tempAxp_x100     = 100 * tempAxp;
  dp->v6_vusbinAxp_x100   = 100 * vusbinAxp;
  dp->v7                  = 0;
  dp->v8                  = 0;
  dp->boot_counter        = boot_counter;
  dp->last_elapsed_ms     = last_elapsed_ms;
}

void _esp_now_init(void){
  esp_err_t code;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  code = esp_now_init();
  LOGF("esp_now_init %s(%d)\n", esp_err_to_name(code), code);
}

void setup() {
  boot_counter++;
#if SERIAL_LOG
  Serial.begin(115200);
#endif
  LOGF("boot_counter=%d\n", boot_counter);
  bool LCDEnable    = false;
  bool PowerEnable  = true;
  bool SerialEnable = false;
  M5.begin(LCDEnable, PowerEnable, SerialEnable);
  M5.Axp.ScreenBreath(0);
  //M5.Lcd.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  LOGF("mac=%s\n", mac_string(mac).c_str());
  for(int i=0; i<PROFILE_COUNT; i++){
    if(memcmp(mac, profiles[i].mac, 6)==0){
      profile = &profiles[i];
    }
    if(PROFILE_INDEX_SERVER==profiles[i].index){
      server_profile = &profiles[i];
    }
  }
  if(profile){
    LOGF("Profile index=%d\n", profile->index);
  }else{
    LOGF("Profile not found.\n");
    super_sleep(60*1000*1000);
    return;
  }

  if(profile == server_profile){
    role = ROLE_SERVER;
  }else{
    role = ROLE_CLIENT;
  }

  switch(role){
  case ROLE_SERVER: server_setup(); break;
  case ROLE_CLIENT: client_setup(); break;
  }
}

void loop() {
  switch(role){
  case ROLE_SERVER: server_loop(); break;
  case ROLE_CLIENT: client_loop(); break;
  }
}

String mac_string(const uint8_t *mac)
{
  char macStr[18] = { 0 };
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

void super_sleep(uint64_t time_in_us){
#if SUPER_SLEEP
  esp_wifi_stop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
  adc_power_off();
#endif
  M5.Axp.DeepSleep(time_in_us);
}

#define FORMAT_HEX_MAX_BYTES 50
static char formatHexBuffer[FORMAT_HEX_MAX_BYTES*3+3+1];
static char* formatHex(uint8_t* data, uint16_t len){
    for(uint16_t i=0; i<len && i<FORMAT_HEX_MAX_BYTES; i++){
        sprintf(formatHexBuffer+3*i, "%02X ", data[i]);
    }
    if(FORMAT_HEX_MAX_BYTES<len){
        sprintf(formatHexBuffer+3*FORMAT_HEX_MAX_BYTES, "...");
    }
    return formatHexBuffer;
}

bool ntp_to_rtc(){
  configTime(9 * 3600, 0, "ntp.nict.jp");
  struct tm i;
  if (getLocalTime(&i)) {
    RTC_TimeTypeDef t;
    t.Hours   = i.tm_hour;
    t.Minutes = i.tm_min;
    t.Seconds = i.tm_sec;
    M5.Rtc.SetTime(&t);

    RTC_DateTypeDef d;
    d.Year    = i.tm_year + 1900;
    d.Month   = i.tm_mon + 1;
    d.Date    = i.tm_mday;
    d.WeekDay = i.tm_wday;
    M5.Rtc.SetData(&d);

    return true;
  }else{
    return false;
  }
}
