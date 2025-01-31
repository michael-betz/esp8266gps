#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <string>
#include <thread>
#include "DateTime.h"
#include "GPS.h"
#include "NTPClock.h"
#include "ClockPID.h"
#include "NTPServer.h"
#include "settings.h"
#include "platform-clock.h"

#define BAUD_LOGGER 921600
#define BAUD_GPS 9600
#define PPSPIN 12

#define LOG_PORT 1234  // Remote Port

SoftwareSerial gps_serial(13, 4);  // RX, TX
GPSDateTime gps(&gps_serial);
NTPClock localClock;
WiFiUDP ntpSocket, logSocket;
IPAddress logDestination;
NTPServer server(&ntpSocket, &localClock);

uint32_t lastPPS = 0;
uint8_t lastLed = 0;

//ISR for PPS interrupt
void IRAM_ATTR handleInterrupt() {
  lastPPS = COUNTERFUNC();
}

uint32_t compileTime;
void setup() {
  DateTime compile = DateTime(__DATE__, __TIME__);

  Serial.begin(BAUD_LOGGER);
  gps_serial.begin(BAUD_GPS);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PPSPIN, INPUT);
  digitalWrite(PPSPIN, LOW);
  attachInterrupt(digitalPinToInterrupt(PPSPIN), handleInterrupt, RISING);

  compileTime = compile.ntptime();
  localClock.setTime(COUNTERFUNC(), compileTime);
  // allow for compile timezone to be 12 hours ahead
  compileTime -= 12*60*60;

  Serial.print("\nAP mode at: ");
  Serial.print(ssid);
  Serial.print(", ");
  WiFi.hostname(ssid);
  WiFi.softAP(ssid, ssidPass);
  WiFi.setSleepMode(WIFI_NONE_SLEEP, 0);  // no sleeping for minimum latency
  Serial.println(WiFi.softAPIP());

  logDestination.fromString(logDestinationIP);
  ntpSocket.begin(123);  // local port 123
  logSocket.begin(LOG_PORT);

  while(gps_serial.available()) { // throw away all the text received while starting up
    gps_serial.read();
  }
}

uint8_t median(int64_t one, int64_t two, int64_t three) {
  if(one > two) {
    if(one > three) {
      if(two > three) {
        // 1 2 3
        return 2-1;
      } else {
        // 1 3 2
        return 3-1;
      }
    } else {
      // 3 1 2
      return 1-1;
    }
  } else {
    if(two > three) {
      if(one > three) {
        // 2 1 3
        return 1-1;
      } else {
        // 2 3 1
        return 3-1;
      }
    } else {
      // 3 2 1
      return 2-1;
    }
  }
}

#define WAIT_COUNT 3
uint8_t settime = 0;
uint8_t wait = WAIT_COUNT-1;
struct {
  int64_t offset;
  uint32_t pps;
  uint32_t gpstime;
} samples[WAIT_COUNT];

void updateTime(uint32_t gpstime) {
  if(lastPPS == 0) {
    return;
  }

  if(settime) {
    int64_t offset = localClock.getOffset(lastPPS, gpstime, 0);
    samples[wait].offset = offset;
    samples[wait].pps = lastPPS;
    samples[wait].gpstime = gpstime;
    if(ClockPID.full() && wait) {
      wait--;
    } else {
      uint8_t median_index = wait;
      if(wait == 0) {
        median_index = median(samples[0].offset, samples[1].offset, samples[2].offset);
      }
      ClockPID.add_sample(samples[median_index].pps, samples[median_index].gpstime, samples[median_index].offset);
      localClock.setRefTime(samples[median_index].gpstime);
      localClock.setPpb(ClockPID.out() * 1000000000.0);
      wait = WAIT_COUNT-1; // (2+1)*16=48s, 80MHz wraps at 53s

      double offsetHuman = samples[median_index].offset / (double)4294967296.0;
      logSocket.beginPacket(logDestination, LOG_PORT);
      logSocket.print(samples[median_index].pps);
      logSocket.print(" ");
      logSocket.print(offsetHuman, 9);
      logSocket.print(" ");
      logSocket.print(ClockPID.d(), 9);
      logSocket.print(" ");
      logSocket.print(ClockPID.d_chi(), 9);
      logSocket.print(" ");
      logSocket.print(localClock.getPpb());
      logSocket.print(" ");
      logSocket.println(samples[median_index].gpstime);
      logSocket.endPacket();
    }
  } else {
    localClock.setTime(lastPPS, gpstime);
    ClockPID.add_sample(lastPPS, gpstime, 0);
    settime = 1;
    logSocket.beginPacket(logDestination, LOG_PORT);
    logSocket.print("S "); // clock set message
    logSocket.print(lastPPS);
    logSocket.print(" ");
    logSocket.println(gpstime);
    logSocket.endPacket();
  }
  lastPPS = 0;
}

// Receive UDP packets

bool forward_nema = false;

void udp_rx_poll()
{
  static char packetBuffer[255];  // buffer to hold incoming UDP packet

  int packetSize = logSocket.parsePacket();

  if (packetSize) {
    // We send the log to whatever IP pinged us last
    logDestination = logSocket.remoteIP();

    // read the packet into packetBufffer
    int len = logSocket.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len <= 0)
      return;
    packetBuffer[len] = 0;

    logSocket.beginPacket(logDestination, LOG_PORT);
    logSocket.print(">>> ");
    logSocket.print(packetBuffer);
    logSocket.endPacket();

    Serial.print(">>> ");
    Serial.print(packetBuffer);

    if (len <= 3) {
      switch (packetBuffer[0]) {
      case 'v':
        forward_nema = false;
        break;

      case 'V':
        forward_nema = true;
        break;
      }
      return;
    }

    gps_serial.print(packetBuffer);
  }
}

uint32_t lastUpdate = 0;

void loop() {
  static char buf[128];

  uint32_t cyclesNow = COUNTERFUNC();
  if((cyclesNow - lastUpdate) >= COUNTSPERSECOND) {
    uint32_t s, s_fb;
    // update the local clock's cycle count
    localClock.getTime(cyclesNow,&s,&s_fb);
    lastUpdate = cyclesNow;
  }

  udp_rx_poll();

  int ret = gps.poll_serial();
  if (ret & 0b01) {
    // new line was received (raw NEMA messages)
    // send it out to serial and UDP
    if (forward_nema) {
      logSocket.beginPacket(logDestination, LOG_PORT);
      logSocket.print(gps.getLine());
      logSocket.endPacket();
      Serial.print(gps.getLine());
    }
  }
  if (ret & 0b10) {  // new time has been received
    if ((gps.second() % 10) == 0) {
      snprintf(buf, sizeof(buf),
        "gps-time: %02d:%02d:%02d  %02d.%02d.%02d  (%d)\n",
        gps.hour(),
        gps.minute(),
        gps.second(),
        gps.day(),
        gps.month(),
        gps.year(),
        gps.is_fixed()
      );
      Serial.print(buf);
      logSocket.beginPacket(logDestination, LOG_PORT);
      logSocket.print(buf);
      logSocket.endPacket();
    }

    uint32_t gpstime = gps.GPSnow().ntptime();
    if(gpstime > compileTime) {  // && gps.is_fixed()) {
      updateTime(gpstime);
    }
  }

  server.poll();
}
