/* The following is based on https://lloydm.net/Demos/GPS-NTP.html
 *  which itself was based on NTP_Server_01.ino, downloaded from
 * https://forum.arduino.cc/index.php?topic=197870.0
 *
 * This project includes further modifications in order to support
 * the ESP32 and ethernet libraries from LilyGo. In particular, this code is
 * running on a T-Internet-POE board from LilyGo.
 */

#include "ulog.h"
#include <Arduino.h>
#include <ETHClass2.h>
#include <RTClib.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#define vers "NTP GPS V03"

// required for ETHClass2
#define ETH ETH2

// pins for the ethernet port on the T-Internet-POE
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_ADDR 0
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_RESET_PIN 5
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18

// PINS
//  GPS
const u_int8_t GPS_RX_PIN = 36;
const u_int8_t GPS_TX_PIN = 4;
const u_int8_t GPS_PPS_PIN = 33;

//  RTC
const u_int8_t RTC_SDA_PIN = 32;
const u_int8_t RTC_SCL_PIN = 16;
const u_int8_t RTC_SQW_PIN = 14;

// GPS serial
const u_int16_t GPS_BAUD = 9600;
HardwareSerial GPSSerial(2);

// time server port assigned to NTP by IANA, referred to in RFC 5905
#define NTP_PORT 123

// Keep track of how off is the increment on the RTC module compared to the GPS
// pulse. This is used to keep consistency when the GPS module is lost It is set
// automatically when the GPS and RTC modules are synced. Stored in milliseconds
short rtcOffset = 4; // this SHOULD be updated with a difference of when the two
                     // pulses take place, but currently does not work that way.
// Size of a version 4 NTP packet
static const int NTP_PACKET_SIZE = 48;
// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];

// Ethernet UDP instance
// the class naming is due to how the ETHClass2 library is implemented
WiFiUDP ethUdp;
WiFiUDP wifiUdp;

// GPS instance
// uses the GPSSerial as defined above
TinyGPSPlus gps;

// RTC instance
RTC_DS3231 rtc;        // Oscillator i2c address is 0x68 (can't be changed)
DateTime rtcNow;       // From rtc.now()
boolean rtcON = false; // Flag indicating RTC present and initialized
boolean rtcPulseAlive = false;

// RTC update interval from GPS, in milliseconds
long updateInterval = 3600000;

// STATE VARIABLES

// current timestamp from the source that we've last read
volatile uint32_t timestamp;
// last time we got an update from our GPS source.
uint32_t referenceTimestamp;
// set to millis() when the last PPS pulse was received
volatile uint32_t startofSec = 0;
volatile uint32_t startofRTCSec = 0;
// if the source is present and active
volatile bool gpsActive = false;
bool rtcActive = false; // set to the inverse of rtc.lostPower()
bool gpsNeedsSettings = true;

uint32_t tempval;

// Time of last RTC update by Arduino run-time clock
unsigned long lastGPSsync = 0;

// https://arduino.stackexchange.com/questions/49567/synching-local-clock-usign-ntp-to-milliseconds
// Do it backwards (milliseconds to fractional second)
int milliseconds;

uint32_t fractionalSecond;
#define MAXUINT32 4294967295. // i.e. (float) 2^32 - 1

const uint8_t daysInMonth[] PROGMEM = {
    31, 28, 31, 30, 31, 30,
    31, 31, 30, 31, 30, 31}; // const or compiler complains

const unsigned long seventyYears =
    2208988800UL; // 1970 - 1900 in seconds (Unix to Epoch)

// Ethernet

// Flag indicating whether the ethernet is connected
bool eth_connected = false;
// Flag indicating whether the wifi is connected
bool wifi_connected = false;

enum class Source { GPS, RTC, NONE };

// Function prototypes

void WiFiEvent(arduino_event_id_t event);
void processNTP(WiFiUDP &udp);
bool updateRTC();
boolean rtcUpdateDue();
Source readTime();
Source readTime(Source source);
bool getgps();
void displayTime();
String formatMS(int ms);

int calculateChecksum(const char *msg);
void nmeaMsgSend(const char *msg);
int nmeaMsgDisable(const char *nmea);
int nmeaMsgEnable(const char *nmea);

void syncToPPS();
void syncToRTCPPS();
void setGPSSettings();
void setRTCSettings();
// logging

void console_logger(ulog_level_t severity, char *msg) {
  Serial.printf("[%s]: %s\n", ulog_level_name(severity), msg);
}

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPSSerial.onReceive([]() {
    while (GPSSerial.available()) {
      gps.encode(GPSSerial.read());
    }
  });
  pinMode(GPS_PPS_PIN, INPUT_PULLDOWN);
  pinMode(RTC_SQW_PIN, INPUT_PULLDOWN);

  ULOG_INIT();

  while (millis() < 1000 && !Serial) {
    delay(500);
  } // wait for Arduino Serial Monitor

#ifdef LOGGER_LEVEL
  ULOG_SUBSCRIBE(console_logger, LOGGER_LEVEL);
#else
  ULOG_SUBSCRIBE(console_logger, ULOG_INFO_LEVEL);
#endif // LOGGER_LEVEL

  WiFi.onEvent(WiFiEvent);

  while (millis() < 1500 && gps.charsProcessed() < 60 &&
         !GPSSerial.availableForWrite()) {
    ULOG_INFO("Waiting for GPS Serial Monitor...");
    // getgps();
    delay(500);
  } // wait for GPS Serial Monitor

  // start Ethernet and UDP:
  if (!ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_RESET_PIN,
                 ETH_CLK_MODE)) {
    ULOG_ERROR("ETH start Failed!");
  }

#if defined(WIFI_SSID) && defined(WIFI_PASS)
  ULOG_INFO("Connecting to WiFi network %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
#elif defined(WIFI_SSID)
  ULOG_INFO("Connecting to WiFi network %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID);
#else
  ULOG_INFO("No WiFi network specified");
#endif

  ULOG_INFO("Version: %s", std::string(vers).c_str());

  setGPSSettings();

  Wire1.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  if (rtc.begin(&Wire1)) {
    rtcON = true;
    ULOG_INFO("RTC is connected!");
  } else {
    ULOG_INFO("Couldn't find RTC");
  }

  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), syncToPPS, RISING);

  uint32_t lastStart = startofSec;
  if (startofSec == 0) {
    ULOG_INFO("No PPS pulse received, waiting for a pulse.");
    delay(1000); // wait for the first PPS pulse
  }
  if (lastStart == startofSec) {
    ULOG_WARNING("No PPS pulse detected.");
    gpsActive = false;
  } else {
    ULOG_INFO("PPS pulse detected.");
  }

  // If RTC is present and functional, update it from GPS (if possible)
  rtcActive = !rtc.lostPower();

  if (rtcON) {
#ifdef ULOG_ENABLED
    ULOG_DEBUG("Real-time clock (before startup sync): ");
    readTime(Source::RTC);
    displayTime();
#endif
    if (!updateRTC()) {
      ULOG_ERROR("RTC update failed.");
#ifdef ULOG_ENABLED
    } else {
      ULOG_INFO("Real-time clock (after startup sync): ");
      readTime(Source::RTC);
      displayTime();
#endif
    }

    // enable the SQW pin to output a 1Hz signal
    setRTCSettings();
    attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), syncToRTCPPS, FALLING);
  };
}

void loop() { // Original loop received data from GPS continuously (i.e. per
              // second) and called processNTP() when valid data were received
              // (i.e. on the second). This revision monitors NTP port
              // continuously and attempts to retrieve GPS data whenever an NTP
              // request is received.
  if (rtcUpdateDue()) {
    bool lostPower = rtc.lostPower();
    if (!(!rtcActive && lostPower) && gpsActive) {
      if (lostPower) {
        ULOG_WARNING("RTC lost power, updating RTC...");
      } else {
        ULOG_INFO("RTC update due, updating RTC...");
      }
    }
    rtcActive &= !lostPower;
    if (gpsActive) {
      if (!updateRTC()) {
        ULOG_ERROR("RTC update failed.");
#ifdef ULOG_ENABLED
      } else {
        ULOG_INFO("Updated RTC time.");
        displayTime();
#endif
      }
    }

    if (lostPower) {
      detachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN));
      setRTCSettings();
      attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), syncToRTCPPS,
                      FALLING);
    }
  }

  // process any NTP requests
  if (eth_connected) {
    processNTP(ethUdp);
  }
#ifdef WIFI_SSID
  if (wifi_connected) {
    processNTP(wifiUdp);
  }
#endif

  if (gpsActive && millis() - startofSec > 2000) {
    gpsActive = false;
    ULOG_WARNING("GPS signal lost.");
    gpsNeedsSettings = true;
  }
  if (gpsActive && millis() - startofSec < 1000 && gpsNeedsSettings) {
    ULOG_INFO("GPS Signal Restored");
    setGPSSettings();
    gpsNeedsSettings = false;
  }

  if (rtcActive && millis() - startofRTCSec > 2000) {
    rtcPulseAlive = false;
    rtcActive = false;
    ULOG_WARNING("RTC signal lost.");
  }
  if (!rtcPulseAlive && millis() - startofRTCSec < 1000) {
    ULOG_INFO("RTC Signal Restored");
    rtcPulseAlive = true;
    rtcActive = true;
  }
}

// Ran on ETHERNET events, pardon the naming.
// the ETHClass2 library is based on the Wifi library...
void WiFiEvent(arduino_event_id_t event) {
  switch (event) {
  case ARDUINO_EVENT_ETH_START:
    ULOG_INFO("ETH Started");
    // set eth hostname here
    ETH.setHostname("esp32-ethernet");
    break;
  case ARDUINO_EVENT_WIFI_STA_START:
    ULOG_INFO("WiFi Started");
    WiFi.setHostname("esp32-wifi");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    ULOG_INFO("ETH Connected");
    break;
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    ULOG_INFO("WiFi Connected");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    ULOG_INFO("ETH MAC: %s, IPv4: %s, %s, %uMbps", ETH.macAddress().c_str(),
              ETH.localIP().toString().c_str(),
              ETH.fullDuplex() ? "FULL_DUPLEX" : "HALF_DUPLEX",
              ETH.linkSpeed());

    ethUdp.begin(ETH.localIP(), NTP_PORT);
    eth_connected = true;
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    ULOG_INFO("WiFi Connected, IP: %s", WiFi.localIP().toString().c_str());
    wifiUdp.begin(WiFi.localIP(), NTP_PORT);
    wifi_connected = true;
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    ULOG_INFO("ETH Disconnected");
    eth_connected = false;
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    ULOG_INFO("WiFi Disconnected");
    wifi_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    ULOG_INFO("ETH Stopped");
    eth_connected = false;
    break;
  case ARDUINO_EVENT_WIFI_STA_STOP:
    ULOG_INFO("WiFi Stopped");
    wifi_connected = false;
    break;
  default:
    break;
  }
}

// nmea message processing

/// @brief calculate the checksum of a NMEA message
/// @param msg the message to calculate, missing its checksum.
/// @return the checksum of the message
int calculateChecksum(const char *msg) {
  int checksum = 0;
  for (int i = 0; msg[i] && i < 32; i++)
    checksum ^= (unsigned char)msg[i];

  return checksum;
}

/// @brief Send an NMEA message to the GPS module
/// @param msg The message to send, minus checksum and start character.
void nmeaMsgSend(const char *msg) {
  char checksum[8];
  snprintf(checksum, sizeof(checksum) - 1, "*%.2X", calculateChecksum(msg));
  GPSSerial.print("$");
  GPSSerial.print(msg);
  GPSSerial.println(checksum);
  ULOG_DEBUG("$%s%s", msg, checksum);
}

/// @brief Disable an NMEA message from the GPS module
/// @param nmea the NMEA message to disable
/// @return whether or not the message was successfully disabled
int nmeaMsgDisable(const char *nmea) {
  if (strlen(nmea) != 3)
    return 0;

  char tmp[32];
  snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,0,0,0", nmea);
  // snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,0,0,0,0,0", nmea);
  nmeaMsgSend(tmp);

  return 1;
}

/// @brief Enable an NMEA message from the GPS module
/// @param nmea the NMEA message to enable
/// @return whether or not the message was successfully enabled
int nmeaMsgEnable(const char *nmea) {
  if (strlen(nmea) != 3)
    return 0;

  char tmp[32];
  // snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,1,0,0", nmea);
  snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,1,0,0,0,0", nmea);
  nmeaMsgSend(tmp);

  return 1;
}

/// @brief Set the GPS module to only output $GPRMC messages
/// This is done to reduce the amount of data that needs to be processed
void setGPSSettings() {
  if (!gpsNeedsSettings) {
    return;
  }
  // Disable everything but $GPRMC
  nmeaMsgDisable("GLL");
  nmeaMsgDisable("VTG");
  nmeaMsgDisable("GSV");
  nmeaMsgDisable("GGA");
  nmeaMsgDisable("GSA");
  // enable only $GPRMC
  nmeaMsgEnable("RMC");
  gpsNeedsSettings = false;
}

/// @brief Set the RTC module to output a 1Hz signal at the start of every
/// second
void setRTCSettings() {
  // must be ran after the RTC clock loses power
  // set the RTC to output a 1Hz signal on the SQW pin
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  rtc.disable32K();
}

// PPS processing

/// @brief Sync to the PPS pulse
/// Runs on every pulse from the GPS module when it has a lock
/// Will not be triggered if the GPS module doesn't have a signal
/// Hence, we can also use PPS pulses to know if the GPS is active
void syncToPPS() {
  startofSec = millis();
  gpsActive = true;
}

/// @brief Sync to the RTC PPS pulse
/// Runs on every pulse from the RTC module
/// We assume the RTC module is always active
void syncToRTCPPS() { startofRTCSec = millis(); }

////////////////////////////////////////

/// @brief Process NTP requests
/// This function is called whenever an NTP request is received
/// There is not much checking of whether or not a packet is valid
void processNTP(WiFiUDP &udp) {

  // if there's data available, read a packet
  int packetSize = udp.parsePacket();
  if (packetSize) {
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = udp.remoteIP();
    int PortNum = udp.remotePort();

    packetBuffer[0] = 0b00100100; // LI, Version, Mode

    // stratum is set later

    packetBuffer[2] = 6;    // polling minimum (64 seconds - default)
    packetBuffer[3] = 0xF7; // precision (2^-9 ~2 milliseconds)

    packetBuffer[4] = 0; // root delay
    packetBuffer[5] = 0;
    packetBuffer[6] = 0;
    packetBuffer[7] = 0;

    packetBuffer[8] = 0; // root dispersion
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;
    packetBuffer[11] = 0;

    Source source = readTime();

    if (source == Source::GPS) {
      ULOG_INFO("GPS source");
      packetBuffer[1] = 1;   // stratum 1 because we're using the GPS
      packetBuffer[12] = 71; //"G";
      packetBuffer[13] = 80; //"P";
      packetBuffer[14] = 83; //"S";
      packetBuffer[15] = 0;  //" ";
    } else if (source == Source::RTC) {
      ULOG_INFO("RTC source");
      packetBuffer[1] = 2; // stratum 2 because we're using the RTC
      packetBuffer[12] = 127;
      packetBuffer[13] = 0;
      packetBuffer[14] = 0;
      packetBuffer[15] = 1;
    } else {
      ULOG_INFO("No source");
      packetBuffer[1] =
          16; // stratum 16 because our time is now invalid to provide
    }

    if (source == Source::GPS || source == Source::RTC) {
      // Reference timestamp
      tempval = referenceTimestamp + seventyYears;
      packetBuffer[16] = (tempval >> 24) & 0XFF;
      packetBuffer[17] = (tempval >> 16) & 0xFF;
      packetBuffer[18] = (tempval >> 8) & 0xFF;
      packetBuffer[19] = (tempval) & 0xFF;

      tempval = 0;
      packetBuffer[20] = (tempval >> 24) & 0xFF;
      packetBuffer[21] = (tempval >> 16) & 0xFF;
      packetBuffer[22] = (tempval >> 8) & 0xFF;
      packetBuffer[23] = (tempval) & 0xFF;

      // Originate timestamp from incoming UDP transmit timestamp
      packetBuffer[24] = packetBuffer[40];
      packetBuffer[25] = packetBuffer[41];
      packetBuffer[26] = packetBuffer[42];
      packetBuffer[27] = packetBuffer[43];
      packetBuffer[28] = packetBuffer[44];
      packetBuffer[29] = packetBuffer[45];
      packetBuffer[30] = packetBuffer[46];
      packetBuffer[31] = packetBuffer[47];

      // Receive timestamp
      tempval = timestamp; // when we received this packet and its timestamp
      packetBuffer[32] = (tempval >> 24) & 0XFF;
      packetBuffer[33] = (tempval >> 16) & 0xFF;
      packetBuffer[34] = (tempval >> 8) & 0xFF;
      packetBuffer[35] = (tempval) & 0xFF;

      tempval = fractionalSecond;
      packetBuffer[36] = (tempval >> 24) & 0xFF;
      packetBuffer[37] = (tempval >> 16) & 0xFF;
      packetBuffer[38] = (tempval >> 8) & 0xFF;
      packetBuffer[39] = (tempval) & 0xFF;

      // Transmit timestamp
      tempval = timestamp;
      packetBuffer[40] = (tempval >> 24) & 0XFF;
      packetBuffer[41] = (tempval >> 16) & 0xFF;
      packetBuffer[42] = (tempval >> 8) & 0xFF;
      packetBuffer[43] = (tempval) & 0xFF;

      tempval = fractionalSecond;
      packetBuffer[44] = (tempval >> 24) & 0xFF;
      packetBuffer[45] = (tempval >> 16) & 0xFF;
      packetBuffer[46] = (tempval >> 8) & 0xFF;
      packetBuffer[47] = (tempval) & 0xFF;
    } else {
      // source is invalid
      for (int i = 16; i < 48; i++) {
        packetBuffer[i] = 0;
      }
    }

    // Omit optional fields

    // Reply to the IP address and port that sent the NTP request

    udp.beginPacket(Remote, PortNum);
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
    udp.flush();
#ifdef ULOG_ENABLED

    ULOG_INFO("Received a request from %s:%i", Remote.toString().c_str(),
              PortNum);
    displayTime();
#endif
  }
}

// RTC support
bool updateRTC() { // From GPS
  if (!gpsActive)
    return false;
  TinyGPSDate date = gps.date;
  TinyGPSTime time = gps.time;
  if (!date.isValid() || !time.isValid() || date.age() > 1000 ||
      time.age() > 1000)
    return false;
  DateTime ut = DateTime(date.year(), date.month(), date.day(), time.hour(),
                         time.minute(), time.second());

  // sync the RTC clock to the start of the second.
  uint32_t pulseWait = millis();
  uint32_t now = millis();
  while (true) {
    if (!(now - startofSec > 3))
      break;
    if (now - pulseWait > 1000) {
      ULOG_WARNING("No PPS pulse detected.");
      return false;
    }
    // ULOG_INFO("millis: %u, startofSec: %u", millis(), startofSec);
    delay(1);
    now = millis();
  }
  rtcOffset = millis() - startofSec;
  ut = ut + TimeSpan(1 + rtcOffset / 1000);
  rtc.adjust(ut);
  timestamp = ut.unixtime() + seventyYears;
  rtcOffset = rtcOffset % 1000;
  ULOG_DEBUG("RTC updated.");
  lastGPSsync = now; // For subsequent updates (and milliseconds computation)

  rtcActive = true;
  return true;
}

/// @brief Check if an RTC update is due based on when the last update was.
/// @return whether the RTC update is due
boolean rtcUpdateDue() { // Convenience test
  if (rtcON) {
    if (!rtcActive)
      return true;
    if (lastGPSsync == 0)
      return true;
    if (millis() < lastGPSsync)
      return true; // Arduino millis() rollover
    if (lastGPSsync + updateInterval < millis())
      return true;
    if (rtc.lostPower())
      return true;
  }
  return false;
}

/// @brief Read time from RTC or GPS
/// The time is read into the global timestamp object.
/// @return Which source the time was read from.
Source readTime(Source source) { // Read time from RTC or GPS
  rtcNow = rtc.now();
  // DS3231 does not have milliseconds! Need a separate millisecond clock -
  // ouch! RTC is synched to GPS at least hourly. Arduino's millisecond counter
  // is good for that range.
  uint_fast32_t secStart;
  long delta;
  switch (source) {
  case Source::GPS:
    if (!gpsActive)
      return Source::NONE;
    secStart = startofSec;
    delta = millis() - secStart;
    break;
  case Source::RTC:
    if (!rtcActive)
      return Source::NONE;
    secStart = startofRTCSec;
    delta = millis() - secStart + rtcOffset;
    break;
  default:
    if (gpsActive)
      return readTime(Source::GPS);
    if (rtcActive)
      return readTime(Source::RTC);
    return Source::NONE;
    break;
  };
  // To do: Handle rollover rigorously - Next is placeholder
  //        Or power-cycle the Arduino occasionally (before 50 days)
  timestamp = rtcNow.unixtime() + seventyYears; // 1900 Epoch
  if (delta < 0) {                              // Rollover has occurred, maybe

    if (source == Source::RTC && delta + rtcOffset >= rtcOffset / 2) {
      // its likely not a rollover but the offset is poor
      delta = 1000 + delta;
    } else {
      // Constant below is 2^32 - 1
      delta = millis() + (4294967295 - secStart) +
              (source == Source::RTC ? rtcOffset : 0);
    }
  }
  milliseconds = delta % 1000;
  // Compute fractional seconds
  fractionalSecond = ((double)milliseconds / 1000.) * MAXUINT32;
  return source;
}

Source readTime() { return readTime(Source::NONE); }

/// @brief  Display the current time
/// This function is used for debugging purposes
void displayTime() {
  std::string buffer;
  const DateTime now = DateTime(timestamp - seventyYears);
  buffer.append(formatMS(now.month()).c_str());
  buffer += "/";
  buffer.append(formatMS(now.day()).c_str());
  buffer += "/";
  buffer.append(String(now.year()).c_str());
  buffer += "  ";
  buffer.append(formatMS(now.hour()).c_str());
  buffer += ":";
  buffer.append(formatMS(now.minute()).c_str());
  buffer += ":";
  buffer.append(formatMS(now.second()).c_str());
  buffer += ".";
  String sMS = String(milliseconds);
  while (sMS.length() < 3) {
    sMS = "0" + sMS;
  }
  buffer.append(sMS.c_str());
  buffer.append("  [");
  buffer.append(String(fractionalSecond).c_str());
  buffer.append("/(2^32 - 1)]");
  ULOG_INFO(buffer.c_str());
}

/// @brief Format milliseconds
/// @param ms The milliseconds to format
/// @return A string representation of the milliseconds, zero padded
String formatMS(int ms) {
  // pad minute or second with leading 0
  String result = "";
  if (ms < 10)
    result = "0";
  return result + String(ms);
}
