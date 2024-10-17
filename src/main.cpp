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

#define vers "NTP GPS V02A (Rev.RTC)"

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

// GPS serial
const u_int16_t GPS_BAUD = 9600;
HardwareSerial GPSSerial(2);

// Time Server Port according to the standard
#define NTP_PORT 123

// Size of a version 4 NTP packet
static const int NTP_PACKET_SIZE = 48;
// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];

// GPS message parsing
const char EOL = 10;   // End-of-line
const int MSGLEN = 66; // GNRMC message length, 66 printable characters + \r
String tmpMsg = "";
String gnrmcMsg = "";

// LM: Date/time handling
String sUTD = "";         // UT Date
String sUTC = "";         // UT Time
const int CENTURY = 2000; // current century
// NOTE needs to be updated in 2100

// An Ethernet UDP instance
// the class is due to how the ETHClass2 library is implemented
WiFiUDP udp;

// GPS instance
TinyGPSPlus gps;

// RTC instance
RTC_DS3231 rtc;        // Oscillator i2c address is 0x68 (can't be changed)
DateTime rtcNow;       // From rtc.now()
boolean rtcON = false; // Flag indicating RTC present and initialized

// RTC update interval from GPS, in seconds
long updateInterval = 3600000;

// STATE VARIABLES

// current timestamp from the source that we've last read
// ppsTimestamp is the value of timestamp at the last PPS pulse
// both are updated in the syncToPPS() function so they are volatile
volatile uint32_t timestamp, ppsTimestamp;
// set to millis() when the last PPS pulse was received
volatile uint32_t startofSec = 0;
// when we receive a pulse this is set to true so our main loop can react to the
// pulse.
volatile bool unprocessedPulse = false;

uint32_t tempval;

// Time of last RTC update by Arduino run-time clock
unsigned long lastGPSsync = 0;

// https://arduino.stackexchange.com/questions/49567/synching-local-clock-usign-ntp-to-milliseconds
// Do it backwards (milliseconds to fractional second)
int milliseconds;

uint32_t fractionalSecond;
#define MAXUINT32 4294967295. // i.e. (float) 2^32 - 1

// Ethernet

// Flag indicating whether the ethernet is connected
static bool eth_connected = false;

// Function prototypes

void WiFiEvent(arduino_event_id_t event);
void processNTP();
static unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m,
                                                       uint8_t d, uint8_t h,
                                                       uint8_t mm, uint8_t s);
boolean getGPSdata();
DateTime crack(String sDate, String sTime);
void updateRTC();
boolean rtcUpdateDue();
void readRTC();
bool getgps();
void displayTime();
String formatMS(int ms);

int calculateChecksum(const char *msg);
void nemaMsgSend(const char *msg);
int nemaMsgDisable(const char *nema);
int nemaMsgEnable(const char *nema);

void syncToPPS();
// logging

void console_logger(ulog_level_t severity, char *msg) {
  Serial.printf("[%s]: %s\n", ulog_level_name(severity), msg);
}

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

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

  while (millis() < 5000 && gps.charsProcessed() < 60 &&
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

  udp.begin(NTP_PORT);

  ULOG_INFO("Version: %s", std::string(vers).c_str());

  // Disable everything but $GPRMC
  nemaMsgDisable("GLL");
  nemaMsgDisable("VTG");
  nemaMsgDisable("GSV");
  nemaMsgDisable("GGA");
  nemaMsgDisable("GSA");

  Wire1.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  delay(250);

  if (rtc.begin(&Wire1)) {
    rtcON = true;
    ULOG_INFO("RTC is connected!");
  } else {
    ULOG_INFO("Couldn't find RTC");
  }

  if (rtcON && ULOG_ENABLED) {
    ULOG_DEBUG("Real-time clock (before startup sync): ");
    readRTC();
    displayTime();
  }
  // If RTC is present and functional, update it from GPS (if possible)
  delay(3000);
  if (rtcON) {
    // empty the serial buffer
    while (GPSSerial.available()) {
      GPSSerial.read();
    }
    updateRTC();
    displayTime();
  };

  if (rtcON && ULOG_ENABLED) {
    ULOG_INFO("Real-time clock (after startup sync): ");
    readRTC();
    displayTime();
  }

  attachInterrupt(GPS_PPS_PIN, syncToPPS, RISING);
}

void loop() { // Original loop received data from GPS continuously (i.e. per
              // second) and called processNTP() when valid data were received
              // (i.e. on the second). This revision monitors NTP port
              // continuously and attempts to retrieve GPS data whenever an NTP
              // request is received.\
  // empty the serial buffer
  while (GPSSerial.available()) {
    GPSSerial.read();
  }

  if (unprocessedPulse) {
    readRTC();
    unprocessedPulse = false;
  }

  if (eth_connected) {
    processNTP();
  }
  if (rtcUpdateDue()) {
    ULOG_DEBUG("Attempting to update RTC..");

    updateRTC();

    if (ULOG_ENABLED) {
      ULOG_INFO("Updated RTC time.");
      readRTC();
      displayTime();
    }
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
  case ARDUINO_EVENT_ETH_CONNECTED:
    ULOG_INFO("ETH Connected");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    ULOG_INFO("ETH MAC: %s, IPv4: %s, %s, %uMbps", ETH.macAddress().c_str(),
              ETH.localIP().toString().c_str(),
              ETH.fullDuplex() ? "FULL_DUPLEX" : "HALF_DUPLEX",
              ETH.linkSpeed());

    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    ULOG_INFO("ETH Disconnected");
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    ULOG_INFO("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}

// NEMA message processing

int calculateChecksum(const char *msg) {
  int checksum = 0;
  for (int i = 0; msg[i] && i < 32; i++)
    checksum ^= (unsigned char)msg[i];

  return checksum;
}

void nemaMsgSend(const char *msg) {
  char checksum[8];
  snprintf(checksum, sizeof(checksum) - 1, "*%.2X", calculateChecksum(msg));
  GPSSerial.print("$");
  GPSSerial.print(msg);
  GPSSerial.println(checksum);
  ULOG_DEBUG("$%s%s", msg, checksum);
}

int nemaMsgDisable(const char *nema) {
  if (strlen(nema) != 3)
    return 0;

  char tmp[32];
  snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,0,0,0", nema);
  // snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,0,0,0,0,0", nema);
  nemaMsgSend(tmp);

  return 1;
}

int nemaMsgEnable(const char *nema) {
  if (strlen(nema) != 3)
    return 0;

  char tmp[32];
  // snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,1,0,0", nema);
  snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,1,0,0,0,0", nema);
  nemaMsgSend(tmp);

  return 1;
}

// PPS processing

void syncToPPS() {
  startofSec = millis();
  unprocessedPulse = true;
  ppsTimestamp = timestamp + 1;
  timestamp++;
}

////////////////////////////////////////

void processNTP() {

  // if there's data available, read a packet
  int packetSize = udp.parsePacket();
  if (packetSize) {
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = udp.remoteIP();
    int PortNum = udp.remotePort();

    packetBuffer[0] = 0b00100100; // LI, Version, Mode
    // Have to spoof stratum 1 because Galleon test client interprets stratum 2
    // ID as a date/time (not IP)
    packetBuffer[1] = 1;    // stratum (GPS)
                            //  packetBuffer[1] = 2 ;   // stratum (RTC)
    packetBuffer[2] = 6;    // polling minimum (64 seconds - default)
    packetBuffer[3] = 0xFA; // precision (reference sketch - ~15 milliseconds)
    packetBuffer[3] = 0xF7; // precision (2^-9 ~2 milliseconds)
    packetBuffer[3] = 0x09; // precision (2^9 Testing)

    packetBuffer[7] = 0; // root delay
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;

    packetBuffer[11] = 0; // root dispersion
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;

    readRTC(); // Assume succeeds
    // readRTC() has cracked date/time and timestamp
    // No additional parsing required here

    // Reference identifier (for Stratum 1 type)
    packetBuffer[12] = 82; //"R";
    packetBuffer[13] = 84; //"T";
    packetBuffer[14] = 67; //"C";
    packetBuffer[15] = 0;  //"0";
                           /*
                               packetBuffer[12] = 192; // IP address of synchronization source
                               packetBuffer[13] = 168; // (Test client from Galleon Systems interprets
                              as date/time)                        packetBuffer[14] = 1;   //
                              packetBuffer[15] = 225; //
                           */
    // Reference timestamp
    tempval = timestamp;
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    packetBuffer[19] = (tempval) & 0xFF;

    tempval = fractionalSecond;
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
    tempval = timestamp; // Same as reference timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval) & 0xFF;

    tempval = fractionalSecond;
    packetBuffer[36] = (tempval >> 24) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[37] = (tempval >> 16) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[38] = (tempval >> 8) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[39] = (tempval) & 0xFF;

    // Transmit timestamp
    tempval = timestamp;
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval) & 0xFF;

    // LM: Fractional second - Test NTP clients accept the following, but
    // Windows does not
    tempval = fractionalSecond;
    packetBuffer[44] = (tempval >> 24) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[45] = (tempval >> 16) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[46] = (tempval >> 8) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[47] = (tempval) & 0xFF;
    // ;

    // Omit optional fields

    // Reply to the IP address and port that sent the NTP request

    udp.beginPacket(Remote, PortNum);
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
    udp.flush();
    //   if (ULOG_ENABLED)
    //     displayTime();
  }
}

////////////////////////////////////////

// static bool getgps()

// LM: The library crack method always returned an invalid age. I don't know
// why.
//     Nor did gps.encode(c). So I modified this function to construct a string
//     from the $GNRMC message, and return true on detecting EOL.

bool getgps() {
  char c;
  while (GPSSerial.available()) {
    c = GPSSerial.read();
    Serial.print(c);
    if (gps.encode(c)) {
      return true;
    }
    // LM -
    if (c == EOL)
      return true;
    if (c == '$') {
      tmpMsg = c;
    } else if (tmpMsg.length() > 0 && tmpMsg.length() < 80) {
      tmpMsg += c;
    }
  }
  return false;
}

const uint8_t daysInMonth[] PROGMEM = {
    31, 28, 31, 30, 31, 30,
    31, 31, 30, 31, 30, 31}; // const or compiler complains

const unsigned long seventyYears =
    2208988800UL; // 1970 - 1900 in seconds (Unix to Epoch)

// NTP since 1900/01/01
static unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m,
                                                       uint8_t d, uint8_t h,
                                                       uint8_t mm, uint8_t s) {

  uint16_t days = d;

  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
  int x = 1970;
  for (; x < y; ++x) {
    if ((x % 400) == 0) {
      ++days;
    } else if ((x % 100) == 0) {
      days = days;
    } else if ((x % 4) == 0) {
      ++days;
    }
  }

  if (m > 2) {
    if ((x % 400) == 0) {
      ++days;
    } else if ((x % 100) == 0) {
      days = days;
    } else if ((x % 4) == 0) {
      ++days;
    }
  }

  days += 365 * y + (y + 3) / 4 - 1;
  return days * 24L * 3600L + h * 3600L + mm * 60L + s + seventyYears;
}
////////////////////////////////////////

// LM: Poll GPS when NTP request received. Time out if GPS does not return valid
// data.

boolean getGPSdata() {
  long startTime = millis();
  bool validDataReceived = false;
  const long TIMEOUT = 5000;
  while (millis() < startTime + TIMEOUT) {
    if (getgps()) {
      gnrmcMsg = tmpMsg;
      ULOG_INFO(gnrmcMsg.c_str());
      // $GNRMC message length is 67, including EOL - Ensure full length
      // message
      ULOG_INFO("gnrmcMsg length: %i", gnrmcMsg.length());
      // support the length being slightly different
      if (gnrmcMsg.charAt(17) == 'A' && gnrmcMsg.length() == MSGLEN) {
        sUTC = gnrmcMsg.substring(7, 13);
        sUTD = gnrmcMsg.substring(53, 59);
        timestamp = crack(sUTD, sUTC).unixtime();
        tmpMsg = "";
        validDataReceived = true;
        ULOG_INFO("Screaming Success!");
        break;
      }
    }
  }

  ULOG_INFO(validDataReceived ? "GPS data retrieval complete."
                              : "GPS data retrieval EPIC FAIL");
  return validDataReceived;
}

// My message utilities

DateTime crack(String sDate, String sTime) {
  // sDate = ddmmyy
  // sTime = hhmmss
  return DateTime(sDate.substring(4).toInt() + CENTURY,
                  sDate.substring(2, 4).toInt(), sDate.substring(0, 2).toInt(),
                  sTime.substring(0, 2).toInt(), sTime.substring(2, 4).toInt(),
                  sTime.substring(4).toInt());
}
// RTC support
void updateRTC() { // From GPS
  bool dataValid = false;
  for (int i = 0; i < 2; i++) {
    if (getGPSdata()) {
      dataValid = true;
      break;
    }
  }
  if (!dataValid) {
    ULOG_INFO("GPS data retrieval failed.");
    return;
  }

  DateTime ut = DateTime(timestamp);
  rtc.adjust(ut);
  ULOG_DEBUG("RTC updated.");
  lastGPSsync =
      millis(); // For subsequent updates (and milliseconds computation)
}

boolean rtcUpdateDue() { // Convenience test
  if (rtcON) {
    if (millis() < lastGPSsync)
      return true; // Arduino millis() rollover
    if (lastGPSsync + updateInterval < millis())
      return true;
  }
  return false;
}

void readRTC() { // Read time from RTC in same format as GPS crack()
  rtcNow = rtc.now();
  // DS3231 does not have milliseconds! Need a separate millisecond clock -
  // ouch! RTC is synched to GPS at least hourly. Arduino's millisecond counter
  // is good for that range.
  long delta = millis() - startofSec;
  // To do: Handle rollover rigorously - Next is placeholder
  //        Or power-cycle the Arduino occasionally (before 50 days)
  timestamp = rtcNow.unixtime() + seventyYears; // 1900 Epoch
  if (ppsTimestamp - timestamp == 1) {
    // don't change the timestamp if the PPS timestamp is in the future by just
    // one second
    timestamp = ppsTimestamp;
  }
  if (delta < 0) { // Rollover has occurred
                   // Constant below is 2^32 - 1
    delta = millis() + (4294967295 - startofSec);
  }
  milliseconds = delta % 1000;
  // Compute fractional seconds
  fractionalSecond = ((double)milliseconds / 1000.) * MAXUINT32;
}

// My debug

void displayTime() {
  std::string buffer;
  const DateTime now = DateTime(timestamp - seventyYears);
  buffer.append(String(now.month()).c_str());
  buffer += "/";
  buffer.append(formatMS(now.day()).c_str());
  buffer += "/";
  buffer.append(String(now.year()).c_str());
  buffer += "  ";
  buffer.append(String(now.hour()).c_str());
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

String formatMS(int ms) {
  // pad minute or second with leading 0
  String result = "";
  if (ms < 10)
    result = "0";
  return result + String(ms);
}
