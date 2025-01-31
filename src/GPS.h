#pragma once

class GPSDateTime {
 public:
  GPSDateTime(Stream *gpsUart): gpsUart_(gpsUart) {  };

  bool is_fixed(void);
  String getLine();

  uint16_t hour();
  uint16_t minute();
  uint16_t second();

  uint16_t day(void);
  uint16_t month(void);
  uint16_t year(void);

  DateTime GPSnow();

  int poll_serial();

 protected:
  uint32_t newTime_;
  uint16_t newYear_, newMonth_, newDay_;

  uint32_t time_;
  uint16_t year_, month_, day_;

  DateTime getZDA();
  int process_line();

 private:
  void set_time(String s_token);
  void set_date(String s_token);
  bool crc_check();

  Stream *gpsUart_;

  uint8_t count_;
  uint8_t parity_;

  bool isNotChecked;
  bool validCode;
  bool validString;
  bool isGpsFix;
  bool isGpsFix_;
  bool getFlag_;
  bool debug_;

  String serial_line;

  DateTime now(void);
};
