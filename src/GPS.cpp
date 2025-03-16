// from https://raw.githubusercontent.com/DennisSc/PPS-ntp-server/master/src/GPS.cpp

#include <Arduino.h>
#include <cstddef>
#include <string.h>
#include "DateTime.h"
#include "GPS.h"

#define gpsTimeOffset 0 //centisecond raw offset, compared to known-good stratum 1 server

void GPSDateTime::set_time(String time) {
  time_ = time.toFloat() * 100;
}

uint16_t GPSDateTime::hour() {
  return time_ / 1000000;
}

uint16_t GPSDateTime::minute() {
  return (time_ / 10000) % 100;
}

uint16_t GPSDateTime::second() {
  return (time_ / 100) % 100;
}

void GPSDateTime::set_date(String s_token) {
  // s_token = "310125"
  day_ = s_token.substring(0, 2).toInt();
  month_ = s_token.substring(2, 4).toInt();
  year_ = s_token.substring(4, 6).toInt();
  if (year_ < 100)  // in the case we got 2 digits only
    year_ += 2000;
}

uint16_t GPSDateTime::day(void) { return day_; };
uint16_t GPSDateTime::month(void) { return month_; };
uint16_t GPSDateTime::year(void) { return year_; };

bool GPSDateTime::crc_check() {
  // XOR of all characters from $ to *
  uint8_t xor_sum = 0;
  unsigned i;
  for (i=1; i<serial_line.length(); i++) {
    char c = serial_line[i];
    if (c == '*')
      break;
    xor_sum = xor_sum ^ c;
  }
  String s_rx = serial_line.substring(i + 1, i + 3);
  if (s_rx.length() < 2)
    return false;
  long i_rx = strtol(s_rx.c_str(), NULL, 16);
  // Serial.printf("crc: %02x vs %s (%02lx, %d)\n", xor_sum, s_rx.c_str(), i_rx, i_rx == xor_sum);
  return i_rx == xor_sum;
}

int GPSDateTime::process_line() {
  // The last received line is now in serial_line
  // $GPGSV,1,1,01,19,,,21*73
  // $GPGGA,193903.00,,,,,0,00,99.99,,,,,,*67
  // $GPGSV,1,1,01,16,,,30*7C
  if (!serial_line.startsWith("$GPRMC,") && !serial_line.startsWith("$GPGSV,"))
    return 1;

  if (!crc_check())
    return 1;

  if (serial_line.startsWith("$GPRMC,")) {
    // $GPRMC,,V,,,,,,,,,,N*53
    // $GPRMC,193903.00,V,,,,,,,300125,,,N*79
    // $GPRMC,003934.00,V,,,,,,,310125,,,N*74

    int i_start = 0;
    for (int i_token=0; i_token <= 9; i_token++) {
      int i_end = serial_line.indexOf(",", i_start);
      if (i_end < 0)
        return 1;

      String s_token = serial_line.substring(i_start, i_end);

      // Serial.printf("token %d: %s\n", i_token, s_token.c_str());
      // token 0: $GPRMC
      // token 1: 010340.00
      // token 2: V
      // token 9: 310125
      switch (i_token) {
      case 1:
        set_time(s_token);
        break;

      case 2:
        isGpsFix = s_token[0] == 'A';
        break;

      case 9:
        set_date(s_token);
        return 0b11;
      }

      i_start = i_end + 1;
    }

    return 1;
  } else if (serial_line.startsWith("$GPGSV,")) {
    // $GPGSV,2,1,05,02,25,258,,10,63,118,20,23,39,053,,27,86,061,16*7D
    int i_start = 0;
    for (int i_token=0; i_token <= 3; i_token++) {
      int i_end = serial_line.indexOf(",", i_start);
      if (i_end < 0)
        return 1;

      String s_token = serial_line.substring(i_start, i_end);

      // Serial.printf("token %d: %s\n", i_token, s_token.c_str());
      // token 0: $GPGSV
      // token 1: 2
      // token 2: 1
      // token 3: 05
      if (i_token == 3) {
        n_sats = atoi(s_token.c_str());
      }

      i_start = i_end + 1;
    }
  }
  return 1;
}

/**
 * Decode NMEA line to date and time
 * $GPZDA,174304.36,24,11,2015,00,00*66
 * $0    ,1        ,2 ,3 ,4   ,5 ,6 *7  <-- pos
 * @return
 *  0: nothing to do,
 *  1: gps message received, fetch it wit get_message()
 *  3: new gps time message decoded
 */
int GPSDateTime::poll_serial() {
  static String tmp_line;

  while (gpsUart_->available()) {
    char c = gpsUart_->read();
    tmp_line += c;
    if (c == '\n') {
      serial_line = tmp_line;
      tmp_line = "";
      return process_line();
    }
  }
  return 0;
}

int GPSDateTime::get_n_sats(void) {
  return n_sats;
}

bool GPSDateTime::is_fixed(void) {
  return isGpsFix;
}

String GPSDateTime::getLine() {
  return serial_line;
}

/**
 * Return instance of DateTime class
 * @return DateTime
 */
DateTime GPSDateTime::GPSnow() {
  return DateTime(this->year(), this->month(), this->day(), this->hour(), this->minute(), this->second());
}
