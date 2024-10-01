/* LM: The following is based on NTP_Server_01.ino, downloaded from:
 *     https://forum.arduino.cc/index.php?topic=197870.0
 *
 *     Original source is named NTP_Server_01.
 *     This sketch includes modifications as summarized below.
 *
 *                LM:   GPS disciplined real-time clock adaptation
 *                      ------------------------------------------
 *     Modifications:   Update RTC from GPS and satisfy NTP requests from RTC
 *                      Platform Arduion Uno + u-blox NEO-M8N
 *                      GPS com via _software_ serial (D5 and D6)
 *                      Fix leap year bug for year > 1970
 *                      Substitute date/time crack() function for GPS library
 * method Minor trivial changes
 *
 */

#include <Arduino.h>
#include <ETHClass2.h>
#include <RTClib.h>
#include <SPI.h> // needed for Arduino versions later than 0018
#include <TinyGPS++.h>

#define ETH ETH2

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_ADDR 0
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_RESET_PIN 5
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18

#define FOR_WINDOWS                                                            \
  false // true if the NTP client is Windows w32tm
        // Omit (true) or include (false) fractional seconds
#define vers "NTP GPS V02A (Rev.RTC)"

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

// Time Server Port
#define NTP_PORT 123

static const int NTP_PACKET_SIZE = 48;

// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];

// An Ethernet UDP instance
WiFiUDP udp;

// GPS instance
TinyGPSPlus gps;

int year;
byte month, day, hour, minute, second, hundredths;
unsigned long age;
uint32_t timestamp, tempval;

////////////////////////////////////////

// LM: GPS message parsing
const char EOL = 10;   // End-of-line
const int MSGLEN = 67; // GNRMC message length, 66 printable characters + \r
String tmpMsg = "";
String gnrmcMsg = "";

// LM: Date/time handling
String sUTD = ""; // UT Date
String sUTC = ""; // UT Time
const int CENTURY = 2000;

// LM: Real-time clock
RTC_DS3231 rtc;        // Oscillator i2c address is 0x68 (can't be changed)
DateTime rtcNow;       // From rtc.now()
boolean rtcON = false; // Flag indicating RTC present and initialized
// Time of last RTC update by Arduino run-time clock
unsigned long lastGPSsync = 0;
long updateInterval = 3600000;
// https://arduino.stackexchange.com/questions/49567/synching-local-clock-usign-ntp-to-milliseconds
// Do it backwards (milliseconds to fractional second)
int milliseconds;
uint32_t fractionalSecond;
#define MAXUINT32 4294967295. // i.e. (float) 2^32 - 1

// LM: Alternative debug (Replaces #define)
#define MYDEBUG true

static bool eth_connected = false;

char daysOfTheWeek[7][12] = {"Sunday",   "Monday", "Tuesday", "Wednesday",
                             "Thursday", "Friday", "Saturday"};

void processNTP();
static unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m,
                                                       uint8_t d, uint8_t h,
                                                       uint8_t mm, uint8_t s);
boolean getGPSdata();
void crack(String sDate, String sTime);
void updateRTC();
boolean rtcUpdateDue();
void readRTC();
void displayTime();
String formatMS(int ms);

void WiFiEvent(arduino_event_id_t event) {
  switch (event) {
  case ARDUINO_EVENT_ETH_START:
    Serial.println("ETH Started");
    // set eth hostname here
    ETH.setHostname("esp32-ethernet");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    if (ETH.fullDuplex()) {
      Serial.print(", FULL_DUPLEX");
    }

    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps");
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected");
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    Serial.println("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}

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
  Serial.print("$");
  GPSSerial.print(msg);
  Serial.print(msg);
  GPSSerial.println(checksum);
  Serial.println(checksum);
}

int nemaMsgDisable(const char *nema) {
  if (strlen(nema) != 3)
    return 0;

  char tmp[32];
  // snprintf(tmp, sizeof(tmp)-1, "PUBX,40,%s,0,0,0,0", nema);
  snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,0,0,0,0,0", nema);
  nemaMsgSend(tmp);

  return 1;
}

int nemaMsgEnable(const char *nema) {
  if (strlen(nema) != 3)
    return 0;

  char tmp[32];
  // snprintf(tmp, sizeof(tmp) - 1, "PUBX,40,%s,0,1,0,0", nema);
  snprintf(tmp, sizeof(tmp)-1, "PUBX,40,%s,0,1,0,0,0,0", nema);
  nemaMsgSend(tmp);

  return 1;
}

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  while (millis() < 2000 && !Serial.availableForWrite()) {
    Serial.println("Waiting for Serial Monitor...");
    delay(500);
  } // wait for Arduino Serial Monitor

  WiFi.onEvent(WiFiEvent);

  while (millis() < 3000 && gps.charsProcessed() < 10) {
  } // wait for GPS Serial Monitor

  // start Ethernet and UDP:
  if (!ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_RESET_PIN,
                 ETH_CLK_MODE)) {
    Serial.println("ETH start Failed!");
  }

  udp.begin(NTP_PORT);

  if (MYDEBUG) {
    Serial.print("Version:");
    Serial.println(vers);
  }

  // Disable everything but $GPRMC
  // Note the following sentences are for UBLOX NEO6MV2 GPS
  nemaMsgDisable("GLL");
  nemaMsgDisable("VTG");
  nemaMsgDisable("GSV");
  nemaMsgDisable("GGA");
  nemaMsgDisable("GSA");

  Wire1.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  delay(250);

  if (rtc.begin(&Wire1)) {
    rtcON = true;
    Serial.println("RTC is connected!");
  } else {
    Serial.println("Couldn't find RTC");
  }

  if (rtcON && MYDEBUG) {
    Serial.print("Real-time clock (before startup sync): ");
    readRTC();
    displayTime();
  }
  // If RTC is present and functional, update it from GPS (if possible)
  if (rtcON) {
    updateRTC();
    displayTime();
  };

  if (rtcON && MYDEBUG) {
    Serial.print("Real-time clock (after startup sync): ");
    readRTC();
    displayTime();
    Serial.println();
  }
}

/*
  NTP Time Server:

 This code is in the public domain.
 */

////////////////////////////////////////

void loop() { // Original loop received data from GPS continuously (i.e. per
              // second) and called processNTP() when valid data were received
              // (i.e. on the second). This revision monitors NTP port
              // continuously and attempts to retrieve GPS data whenever an NTP
              // request is received.
  if (eth_connected) {
    processNTP();
  }
  if (rtcUpdateDue()) {
    if (MYDEBUG)
      Serial.println("Attempting to update RTC..");

    updateRTC();

    if (MYDEBUG) {
      Serial.println("Updated RTC time.");
      readRTC();
      displayTime();
      Serial.println();
    }
  }
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
    packetBuffer[1] = 2; // stratum (GPS)
                         //  packetBuffer[1] = 2 ;   // stratum (RTC)
    packetBuffer[2] = 6; // polling minimum (64 seconds - default)
    //  packetBuffer[3] = 0xFA; // precision (reference sketch - ~15
    //  milliseconds)
    packetBuffer[3] = 0xF7; // precision (2^-9 ~2 milliseconds)
    //  packetBuffer[3] = 0x09; // precision (2^9 Testing)

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
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval) & 0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

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

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // Transmit timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval) & 0xFF;

#if FOR_WINDOWS
    // LM: Fractional second - Use 0 with Windows client until issue resolved
    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;
#else
    // LM: Fractional second - Test NTP clients accept the following, but
    // Windows does not
    tempval = fractionalSecond;
    packetBuffer[44] = (tempval >> 24) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[45] = (tempval >> 16) & 0xFF;
    ;
    tempval = fractionalSecond;
    packetBuffer[46] = (tempval >> 8) & 0xFF;
    ;
    tempval = fractionalSecond;
    packetBuffer[47] = (tempval) & 0xFF;
    ;
#endif

    // Omit optional fields

    // Reply to the IP address and port that sent the NTP request

    udp.beginPacket(Remote, PortNum);
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
    if (MYDEBUG)
      displayTime();
  }
}

////////////////////////////////////////

// static bool getgps()

// LM: The library crack method always returned an invalid age. I don't know
// why.
//     Nor did gps.encode(c). So I modified this function to construct a string
//     from the $GNRMC message, and return true on detecting EOL.

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
  uint16_t timeout = 5000;
  do {
    while (GPSSerial.available()) {
      // Serial.write(GPSSerial.read());
      gps.encode(GPSSerial.read());
    }
  } while (millis() - startTime < timeout && !gps.time.isValid());
  return gps.time.isValid();
}

// My message utilities

void crack(String sDate, String sTime) {
  // sDate = ddmmyy
  // sTime = hhmmss
  // hundredths = 00
  year = sDate.substring(4).toInt() + CENTURY;
  month = sDate.substring(2, 4).toInt();
  day = sDate.substring(0, 2).toInt();
  hour = sTime.substring(0, 2).toInt();
  minute = sTime.substring(2, 4).toInt();
  second = sTime.substring(4).toInt();
  hundredths = 0; // LM: GPS time is always acquired on the second (not used)
  age = 0;        //     Not used in this adaptation
}

// RTC support

void updateRTC() { // From GPS
  if (!getGPSdata())
    return;
  auto time = gps.time;
  auto date = gps.date;
  year = date.year();
  month = date.month();
  day = date.day();
  hour = time.hour();
  minute = time.minute();
  second = time.second();
  DateTime ut(year, month, day, hour, minute, second);
  rtc.adjust(ut);
#if MYDEBUG
  Serial.println("RTC updated.");
#endif
  lastGPSsync =
      millis(); // For subsequent updates (and milliseconds computation)
}

boolean rtcUpdateDue() { // Convenience test
  if (rtcON) {
    if (lastGPSsync + updateInterval < millis())
      return true;
    if (millis() < lastGPSsync)
      return true; // Arduino millis() rollover
  }
  return false;
}

void readRTC() { // Read time from RTC in same format as GPS crack()
  rtcNow = rtc.now();
  year = rtcNow.year();
  month = rtcNow.month();
  day = rtcNow.day();
  hour = rtcNow.hour();
  minute = rtcNow.minute();
  second = rtcNow.second();
  // DS3231 does not have milliseconds! Need a separate millisecond clock -
  // ouch! RTC is synched to GPS at least hourly. Arduino's millisecond counter
  // is good for that range.
  long delta = millis() - lastGPSsync;
  // To do: Handle rollover rigorously - Next is placeholder
  //        Or power-cycle the Arduino occasionally (before 50 days)
  if (delta < 0) { // Rollover has occurred
                   // Constant below is 2^32 - 1
    delta = millis() + (4294967295 - lastGPSsync);
  }
  milliseconds = delta % 1000;
  // I can't see where hundredths were used in the reference sketch
  hundredths = milliseconds / 10;
  timestamp = rtcNow.unixtime() + seventyYears; // 1900 Epoch
  // Compute fractional seconds
  fractionalSecond = ((double)milliseconds / 1000.) * MAXUINT32;
}

// My debug

void displayTime() {
  Serial.print(month);
  Serial.print("/");
  Serial.print(day);
  Serial.print("/");
  Serial.print(year);
  Serial.print("  ");
  Serial.print(hour);
  Serial.print(":");
  Serial.print(formatMS(minute));
  Serial.print(":");
  Serial.print(formatMS(second));
  Serial.print(".");
  String sMS = String(milliseconds);
  while (sMS.length() < 3) {
    sMS = "0" + sMS;
  }
  Serial.print(sMS);
  Serial.print("  [");
  Serial.print(fractionalSecond);
  Serial.print("/(2^32 - 1)]");
  Serial.println();
}

String formatMS(int ms) {
  // pad minute or second with leading 0
  String result = "";
  if (ms < 10)
    result = "0";
  return result + String(ms);
}
