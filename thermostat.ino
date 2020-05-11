//#define BREADBOARD
//#define BUILD_ONE        // master bedroom
#define BUILD_TWO        // gameroom


/*
 * module is a WeMos D1 R1
 * flash size set to 4M (1M SPIFFS) or 4MB (FS:1MB OTA:~1019KB) (latest esp board sw uses this)
 * OR
 * module is a esp12e
 * flash size set to 4M (1M SPIFFS) or 4MB (FS:1MB OTA:~1019KB) (latest esp board sw uses this)
 * OR
 * module is a esp7
 * flash size set to 1M (128KB SPIFFS) or 1MB (FS:128KB OTA:~438KB) (latest esp board sw uses this)
 * if this enough to do ota?  (esp-07s has 4MB of memory)
 */


#ifdef BREADBOARD
#define DISPLAY_22
#endif
#ifdef BUILD_ONE
#define LATCH_RELAYS
#define DISPLAY_14
#endif
#ifdef BUILD_TWO
#define DISPLAY_18
#endif

/*
 * todo:
 *  - fix mfgfx (getstringwidth and getfont height not working)
 *  - test with a variety of displays
 *  - add ota
 *  - add mqtt
 */

/*
 * library sources:
 * ESP8266WiFi, ESP8266WebServer, FS, DNSServer, Hash, EEPROM, ArduinoOTA - https://github.com/esp8266/Arduino
 * WebSocketsServer - https://github.com/Links2004/arduinoWebSockets (git)
 * WiFiManager - https://github.com/tzapu/WiFiManager (git)
 * ESPAsyncTCP - https://github.com/me-no-dev/ESPAsyncTCP (git)
 * ESPAsyncUDP - https://github.com/me-no-dev/ESPAsyncUDP (git)
 * PubSub - https://github.com/knolleary/pubsubclient (git)
 * TimeLib - https://github.com/PaulStoffregen/Time (git)
 * Timezone - https://github.com/JChristensen/Timezone (git)
 * ArduinoJson - https://github.com/bblanchon/ArduinoJson  (git)
 * 
 * Adafruit_mfGFX - https://github.com/pkourany/Arduino_Adafruit_mfGFX_Library (git)  (not working?)
 * Adafruit_GFX.h - https://github.com/adafruit/Adafruit-GFX-Library (git)
 * 
 * Adafruit_ILI9341 - https://github.com/adafruit/Adafruit_ILI9341 (git)
 * TFT_ILI9163C.h - https://github.com/PaulStoffregen/TFT_ILI9163C
 * Adafruit_ST7735.h - https://github.com/adafruit/Adafruit-ST7735-Library
 */

/*
 * if LATCH_RELAYS is defined, the thermostat is using latch relays to drive the compressor, fan, and reversing valve
 * latch relays need a short pulse to turn them on or off
 * 3 gpios are used to turn each item on, 1 gpio is used to turn them all off
 * 
 * if it isn't defined, then the thermostat is using either relays or thristors for control
 * 3 gpios are used, and the state must be left on to turn the item on
 */
//#define LATCH_RELAYS


/*
 * if DISPLAY_14 is defined, the 1.4" 128x128 tft ILI9163C display is used
 * if DISPLAY_18 is defined, the 1.8" 128x160 tft ST7735 display is used
 * if DISPLAY_22 is defined, the 2.2" 240x320 tft ILI9340 display is used (uses the ILI9341 library)
 * if DISPLAY_28 is defined, the 2.8" 240x320 tft ILI9341 display is used
 *   this display has a touch screen, but it doesn't seem to be very good.
 * we're not using the extra space on the taller display, just centering what we have on it.
 */
//#define DISPLAY_14
//#define DISPLAY_18
//#define DISPLAY_22
//#define DISPLAY_28


#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <TimeLib.h> 
#include <Timezone.h>

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

// --------------------------------------------

// web server library includes
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// --------------------------------------------

// file system (spiffs) library includes
#include <FS.h>

// --------------------------------------------

// wifi manager library includes
#include <DNSServer.h>
#include <WiFiManager.h>

WiFiManager wifiManager;
String ssid;

// --------------------------------------------

// aync library includes
#include <ESPAsyncTCP.h>
#include <ESPAsyncUDP.h>

// --------------------------------------------

// mqtt library includes
#include <PubSubClient.h>

// --------------------------------------------

// arduino ota library includes
#include <ArduinoOTA.h>

#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

// --------------------------------------------

#include <Wire.h>

// --------------------------------------------

const char *weekdayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// --------------------------------------------

// display includes

// cpd...these need to mfgfx library to work...can't get it to compile
//#define FONTS
//#define font_arial_8   1
//#define font_arial_16  2
//#define font_arial_24  3

#include <SPI.h>
#include <Adafruit_GFX.h>
// cpd...working? #include <Adafruit_mfGFX.h>
#ifdef DISPLAY_14
#include <TFT_ILI9163C.h>
#define GREEN 0x07E0
#define RED   0xF800
#endif
#ifdef DISPLAY_18
#include <Adafruit_ST7735.h>
#define BLACK ST7735_BLACK
#define GREEN ST7735_GREEN
#define RED   ST7735_RED
#endif
#ifdef DISPLAY_22
#include <Adafruit_ILI9341.h>
#define BLACK ILI9341_BLACK
#define GREEN ILI9341_GREEN
#define RED   ILI9341_RED
#endif
#ifdef DISPLAY_28
#include <Adafruit_ILI9341.h>
#define BLACK ILI9341_BLACK
#define GREEN ILI9341_GREEN
#define RED   ILI9341_RED
#endif

// --------------------------------------------

// temperature sensor includes
#include <OneWire.h>
#include <DallasTemperature.h>

// --------------------------------------------

#define WU_API_KEY  "1a4e9749a84f549f" // thermostat key
//#define WU_LOCATION "34241"
#define WU_LOCATION "pws:MD0683"       // weather station in serenoa
#define WUNDERGROUND "api.wunderground.com"
const char WUNDERGROUND_REQ1[] = "GET /api/";
const char WUNDERGROUND_REQ2[] = "/conditions/q/";
const char WUNDERGROUND_REQ3[] = ".json HTTP/1.1\r\n"
    "User-Agent: ESP8266/0.1\r\n"
    "Accept: */*\r\n"
    "Host: " WUNDERGROUND "\r\n"
    "Connection: close\r\n"
    "\r\n";

#define TEMP_IP_ADDR "192.168.1.16"
#define TEMP_IP_PORT 9090

#define TEMP_ERROR -999

// --------------------------------------------

/*
 * esp-12 pin usage: 
 * gpio5 - spi - d/c
 * gpio4 - 1-wire - temp sensor
 * gpio0 - i2c - sda
 * gpio2 - i2c - scl
 * gpio15 - spi - cd
 * gpio16 - relays reset
 * gpio14 - spi - sck
 * gpio12 - spi - miso (not used)
 * gpio13 - spi - mosi
 * 
 * mux usage:
 * 0 - up button
 * 1 - enter button
 * 2 - down button
 * 3 - reversing valve relay
 * 4 - fan relay
 * 5 - compressor relay
 * 6 - display backlight led
 * 7 - motion sensor
 * int - not used
 */

// esp8266 i2c mux
#define SDA 0
#define SCL 2

#ifdef BREADBOARD
#define MUX 0x24    // mux board with switch 1 to on, 2&3 to off
#else
#ifdef BUILD_ONE
// 1st gen thermostat uses the a version
// using the PCF8674A which has a fixed address of 111.
//            00111000
#define MUX 0x38    // PCF8574A, A0, A1, and A2 to GND
#endif

#ifdef BUILD_TWO
// 2nd gen thermostat uses the normal version
// using the PCF8574 which has a fixed address of 100.
//            00100000
#define MUX 0x20    // PCF8574, A0, A1, and A2 to GND
#endif
#endif

byte muxState;

// hardware SPI pins
#define __CS  15
#define __DC   5
// MOSI(SDA)  13
// RST wired to VCC
// SCLK(SCK)  14
// MISO       12

// connected via the i2c mux
#define LED    B01000000

#ifdef DISPLAY_14
TFT_ILI9163C display = TFT_ILI9163C(__CS, __DC);
#endif
#ifdef DISPLAY_18
Adafruit_ST7735 display = Adafruit_ST7735(__CS, __DC, -1);
#endif
#ifdef DISPLAY_22
Adafruit_ILI9341 display = Adafruit_ILI9341(__CS, __DC, -1);
#endif
#ifdef DISPLAY_28
Adafruit_ILI9341 display = Adafruit_ILI9341(__CS, __DC);
#endif

// --------------------------------------------

// gpio 4 is available on the esp-12, but it blinks the blue led when used....weird
// so use gpio16 instead...tried this, and didn't work...is gpio 1 not usable?
#define ONE_WIRE_BUS 4

#define TimeBetweenMotionChecks       1000
#define MenuIdleTime                 10000

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// device address
DeviceAddress thermometer1;
DeviceAddress thermometer2;
int numDevices;

#define resolution 12
unsigned long lastTempRequest = 0;
int delayInMillis = 0;


int lastTargetTemperature = TEMP_ERROR;
bool isTargetCool;
float lastTemp;
float lastOutsideTemp = TEMP_ERROR;
unsigned long lastMotionReadTime;
unsigned long lastButtonPressTime;
int targetCoolTemp;
int targetHeatTemp;
boolean isForcePrint = false;
bool isDisplayOn;
byte last_p_program = 0;
byte last_p_time = 0;

// --------------------------------------------

// these relays are connected thru a i2c mux
#define FAN_PIN             B00010000
#define COMPRESSOR_PIN      B00100000
#define REVERSING_VALVE_PIN B00001000
#ifdef LATCH_RELAYS
#define RESET_PIN           16    // not on the mux
#endif

#define FAN             0
#define COMPRESSOR      1
#define REVERSING_VALVE 2

const int relayPins[] = {FAN_PIN, COMPRESSOR_PIN, REVERSING_VALVE_PIN};

// --------------------------------------------

// these buttons are connected thru a i2c mux
#define UP         B00000001
#define ENTER      B00000010
#define DOWN       B00000100

const int buttonPins[] = {UP, ENTER, DOWN};
const int defaultButtonState = LOW;

int numButtons;
int *buttonStates;
int *lastButtonStates;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long *lastDebounceTimes;  // the last time the output pin was toggled
const long debounceDelay = 50;    // the debounce time

// --------------------------------------------

// the motion sensor module was modified to bypass the 3.3v regulator
// since we're providing it 3.3v already, it wouldn't work.
// an extra wire was soldered from VCC to the reguator output
// the difusing window was also removed since we're not interested in wide
// angle motion
#define MOTION B10000000

// --------------------------------------------

bool isSetup = false;

#define OFF  0
#define ON   1
#define TEMP 2
#define HOLD 3
const char *modeNames[] = { "Off    ", "On     ", "Temp", "Hold  " };
#define COOL 0
#define HEAT 1
const char *modeStateNames[] = { "Cool", "Heat" };
byte mode;

#define AUTO 0
const char *fanNames[] = { "Auto", "On    " };

// actions
#define SYSTEM_OFF    'o'
#define SYSTEM_FAN    'f'
#define SYSTEM_HEAT   'h'
#define SYSTEM_COOL   'c'

bool isFanOn;
bool isCompressorOn;
bool isReversingValueOn;

enum screenState { MAIN, MENU };
screenState screen;
int selectedMenuItem;

unsigned long lastMinutes;

#define MODE_BUTTON    0
#define FAN_BUTTON     1
#define EXIT_BUTTON    2

// --------------------------------------------

// program defination

#define NUM_PROGRAMS   4
#define NUM_TIMES      4
#define PROGRAM_NAME_LENGTH 5

typedef struct {
  byte startTime;
  byte coolTemp;
  byte heatTemp;
} timeType;

typedef struct {
  byte isEnabled;
  byte dayMask;
  char name[PROGRAM_NAME_LENGTH];
  timeType time[NUM_TIMES];
} programType;

programType program[NUM_PROGRAMS];


void saveProgramConfig(void);

void initProgram(void) {
/*
each day is a bit: sun is lsb, sat is msb
ex: sat and tue: B01000100

time starts at 12am = 0, and goes in 15 minute increments
ex: 3:30am: 14

default program:
                       cool  heat
SS    1      6:00am    78    72
SS    2      6:30am    78    72
SS    3      4:00pm    78    72
SS    4     10:00pm    78    72
MT    1      6:00am    78    72
MT    2      8:30am    85    60
MT    3      3:45pm    78    72
MT    4     10:00pm    78    72
F     1      6:00am    78    72
F     2      6:30am    85    60
F     3      1:45pm    78    72
F     4     10:00pm    78    72
*/
  for (int i=0; i < NUM_PROGRAMS; ++i) {
    program[i].isEnabled = false;
    program[i].dayMask = B00000000;
    program[i].name[0] = '\0';
    for (int j=0; j < NUM_TIMES; ++j) {
      program[i].time[j].startTime = 0;
      program[i].time[j].coolTemp = 78;
      program[i].time[j].heatTemp = 72;
    }
  }

  // sat-sun program
  program[0].isEnabled = true;
  program[0].dayMask = B01000001;
  strcpy(program[0].name, "S-S");
  program[0].time[0].startTime = 6*4;    //  6:00a
  program[0].time[1].startTime = 6*4+2;  //  6:30a
  program[0].time[2].startTime = 15*4+3; //  3:45p
  program[0].time[3].startTime = 22*4;   // 10:00p

  // mon-thu program
  program[1].isEnabled = true;
  program[1].dayMask = B00011110;
  strcpy(program[1].name, "M-T");
  program[1].time[0].startTime = 6*4;    //  6:00a
  program[1].time[1].startTime = 6*4+2;  //  6:30a
  program[1].time[2].startTime = 15*4+3; //  3:45p
  program[1].time[3].startTime = 22*4;   // 10:00p
  program[1].time[1].coolTemp = 85;
  program[1].time[1].heatTemp = 60;

  // fri program
  program[2].isEnabled = true;
  program[2].dayMask = B00100000;
  strcpy(program[2].name, "F");
  program[2].time[0].startTime = 6*4;    //  6:00a
  program[2].time[1].startTime = 6*4+2;  //  6:30a
  program[2].time[2].startTime = 13*4+3; //  1:45p
  program[2].time[3].startTime = 22*4;   // 10:00p
  program[2].time[1].coolTemp = 85;
  program[2].time[1].heatTemp = 60;
  
  saveProgramConfig(); 
}

// --------------------------------------------


ESP8266WebServer server(80);
File fsUploadFile;

WebSocketsServer webSocket = WebSocketsServer(81);
int webClient = -1;
int programClient = -1;
int setupClient = -1;
int testClient = -1;

int testAddr = -1;
int testValue = -1;
unsigned int testHeap = 0;

int indent = 10;

bool isTimeSet = false;

WiFiClient espClient;
PubSubClient client(espClient);

#define HOST_NAME "THERMOSTAT"
#define MQTT_IP_ADDR "192.168.1.210"
#define MQTT_IP_PORT 1883

bool isPromModified;
bool isMemoryReset = false;
//bool isMemoryReset = true;

typedef struct {
  char host_name[17];
  char mqtt_ip_addr[17];
  int mqtt_ip_port;
  byte use_mqtt;
  byte mode;
  byte fan;
  byte temperature_span;
  byte display_timeout;
  byte out_temp;
  byte swap_sensors;
  byte use_temp;
  char temp_ip_addr[17];
  int  temp_ip_port;
  char wu_key[20];
  char wu_location[20];
} configType;

configType config;


void print(const char *str);
void println(const char *str);
void print(const __FlashStringHelper *str);
void println(const __FlashStringHelper *str);

void setupDisplay(void);
void loadConfig(void);
bool setupWifi(void);
void requestOutsideTemperature(void);
void gotOutsideTemperature(float tempF);
void setupWebServer(void);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght);
void setupTime(void);
void setupButtons(void);
void setupRelays(void);
void setupMotion(void);
bool setupTempSensor(void);
void drawMainScreen(void);
void displayBacklight(bool);
void loadProgramConfig(void);
unsigned long sendNTPpacket(IPAddress& address);
void printTime(bool isCheckProgram, bool isDisplay, bool isTest);
void printCurrentTemperature(bool);
void printTargetTemperature(bool);
void eraseOutsideTemperature(bool);
void printOutsideTemperature(bool);
void printModeState(bool);
void printFanState(bool);
void printRunState(bool);
void printProgramState(byte p_program, byte p_time, bool isDisplay);
void setTargetTemp(int newTemp);
void saveConfig(void);
void relaysOff(void);
void checkMotion(unsigned long time);
void checkTime(unsigned long time);
void checkTemperature(void);
void checkTemperature(unsigned long time);
void checkTemperature(DeviceAddress deviceAddress);
void flashTime(unsigned long time);
void checkButtons(void);
void eraseTime(void);
void everyFiveMinutes(void);
void logTemperature(float inside, float outside);
void checkUnit(void);
void checkProgram(int day, int h, int m);
void logTarget(int temperature);
void doButton(int pin);
void doUp(void);
void doDown(void);
void doEnter(void);
void setSelectedMenuItem(int item);
void doMenuEnter(void);
void drawMenu(void);
void drawButton(const char *s1, const char *s2, int i, int sel);
void drawButton(const char *s, int i, int sel, int row, int col, int width);
void logAction(char state);
void update(int addr, byte data);
int showWeather(char *json);
void setCursor(int x, int y);
void setTextSize(int size);
int getTargetTemperature(void);
void checkOutsideTemperature(void);
void sendWeb(const char *command, const char *value);
void setUpAirTemp(void);
void checkClientConnection(void);


void send(byte addr, byte b) {
  Wire.beginTransmission(addr);
  Wire.write(b);
  Wire.endTransmission();
  muxState = b; 
}


void setup(void) {
  // start serial port
  Serial.begin(115200);
  Serial.print(F("\n\n"));

  Wire.begin(SDA,SCL);
  send(MUX, 0xFF);

  setupDisplay();

#ifdef BREADBOARD
  println(F("BREADBOARD CONFIG"));
#endif  
#ifdef BUILD_ONE
  println(F("BUILD_ONE CONFIG"));
#endif  
#ifdef BUILD_TWO
  println(F("BUILD_TWO CONFIG"));
#endif  

  println(F("esp8266 thermostat"));
  println(F("compiled:"));
  print(F(__DATE__));
  print(F(","));
  println(F(__TIME__));
  
  print(F("MUX at "));
  Serial.println(MUX, HEX);
  display.println(MUX, HEX);

  if (!setupWifi())
    return;
    
  // 512 bytes is not exact, but is more than enough
  EEPROM.begin(512);

  loadConfig();
  loadProgramConfig();
  isMemoryReset = false;
  
  setupTime();
  setupWebServer();
  setupMqtt();
  setupOta();
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  setupButtons();
  setupRelays();
  setupMotion();

  if (!setupTempSensor())
    return;

  lastMinutes = 0;
  lastMotionReadTime = -TimeBetweenMotionChecks;

  lastButtonPressTime = millis();

  isSetup = true;
  drawMainScreen();
}


#define TIME_BETWEEN_CONNECTS 1000*60
unsigned long lastAttempt;
AsyncClient* airTempClient = NULL;


void setUpAirTemp(void) {
  Serial.println(F("setting up air temp socket client"));

  lastAttempt = 0;

  airTempClient = new AsyncClient();
      
  airTempClient->onConnect([](void *obj, AsyncClient* c) {
    Serial.printf("airClient: [A-TCP] onConnect\n");
  }, NULL);

  airTempClient->onDisconnect([](void *obj, AsyncClient* c) {
    Serial.printf("airClient: [A-TCP] onDisconnect\n");

    float airTemp = TEMP_ERROR;
    if ( airTemp != lastOutsideTemp ) {
      lastOutsideTemp = airTemp;
      eraseOutsideTemperature(true);
    }
  }, NULL);

  airTempClient->onError([](void *obj, AsyncClient* c, int8_t err) {
    Serial.printf("airClient: [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  airTempClient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("airClient: [A-TCP] onData: %s %d\n", buf, len);
    char *ptr = (char *)buf;
    float airTemp = strtod(ptr, &ptr);
//    Serial.println(airTemp);
 
    if ( airTemp != lastOutsideTemp ) {
      lastOutsideTemp = airTemp;
  
      if (screen == MAIN)
        printOutsideTemperature(true);
    }
  }, NULL);


  // ack received
//  airTempClient->onAck([](void *obj, AsyncClient* c, size_t len, uint32_t time) {
//    Serial.printf("airClient: [A-TCP] onAck\n");
//  }, NULL);

  // ack timeout
//  airTempClient->onTimeout([](void *obj, AsyncClient* c, uint32_t time) {
//    Serial.printf("airClient: [A-TCP] onTimeout\n");
//  }, NULL);


  // every 125ms when connected
//  airTempClient->onPoll([](void *obj, AsyncClient* c) {
//    Serial.printf("airClient: [A-TCP] onPoll\n");
//  }, NULL);
}


void clearScreen(void) {
#ifdef DISPLAY_14
  display.clearScreen();
#endif
#ifdef DISPLAY_18
  display.fillScreen(BLACK);
#endif
#ifdef DISPLAY_22
  display.fillScreen(BLACK);
#endif
#ifdef DISPLAY_28
  display.fillScreen(BLACK);
#endif
}


void setCursor(int x, int y) {
#ifdef DISPLAY_18
  y += 16;
#endif
#ifdef DISPLAY_22
#endif
#ifdef DISPLAY_28
#endif
  display.setCursor(x, y);
}


void setupDisplay(void) {
#ifdef DISPLAY_14
  Serial.println(F("1.4\" tft display"));
  display.begin();
  display.setBitrate(16000000);
#endif
#ifdef DISPLAY_18
  Serial.println(F("1.8\" tft display"));
  display.initR(INITR_BLACKTAB);
#endif
#ifdef DISPLAY_22
  Serial.println(F("2.2\" tft display"));
  display.begin();
#endif
#ifdef DISPLAY_28
  Serial.println(F("2.8\" tft display"));
  display.begin();
#endif

  clearScreen();
  setCursor(0, 0);
  display.setTextColor(GREEN);
  display.setTextSize(1);
#ifdef FONTS
  display.setFont(font_arial_8);
#endif

#ifdef BREADBOARD
#ifdef DISPLAY_14
  display.setRotation(2);
#endif
#endif

  displayBacklight(true);
}


void displayBacklight(bool isOn) {
//  Serial.printf("backlight %d\n", isOn);
  if (isSetup && !isDisplayOn && isOn) {
    // redraw the display before turning it on, so we don't see it flash
    isDisplayOn = true;
    drawMainScreen();
  }

  if (isOn)
    send(MUX, muxState & ~LED);
  else
    send(MUX, muxState | LED);

  isDisplayOn = isOn;
}


void print(const char *str) {
  Serial.print(str);
  display.print(str);
}


void print(const __FlashStringHelper *str) {
  Serial.print(str);
  display.print(str);
}


void println(const char *str) {
  Serial.println(str);
  display.println(str);
}


void println(const __FlashStringHelper *str) {
  Serial.println(str);
  display.println(str);
}


bool setupTempSensor(void) {
  // locate devices on the bus
  print(F("temp sensor: "));

  sensors.begin();
  numDevices = sensors.getDeviceCount();
  if (numDevices != 1 && numDevices != 2) {
    println(F("not found"));
    return false;
  }
  char buf[2];
  sprintf(buf, "%d", numDevices);
  print(buf);
  println(F(" found"));

  if (!sensors.getAddress(thermometer1, 0)) {
    println(F("unable to find address for temp sensor"));
    return false;
  }
  sensors.setResolution(thermometer1, resolution);

  if (numDevices == 2) {
    if (!sensors.getAddress(thermometer2, 1)) {
      println(F("unable to find address for outside temp sensor"));
      return false;
    }
    sensors.setResolution(thermometer2, resolution);
  }
  
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - resolution)); 
  lastTempRequest = millis(); 

  return true;
}


void printName() {
  if (webClient == -1)
    return;
    
  sendWeb("name", config.host_name);
}


void configModeCallback(WiFiManager *myWiFiManager) {
  // this callback gets called when the enter AP mode, and the users
  // need to connect to us in order to configure the wifi
  clearScreen();
  setCursor(0, 0);
  display.setTextColor(GREEN);
  setTextSize(1);
  println(F("WiFi Not Configured"));
  println(F("Join this network:"));
  setTextSize(2);
  println(config.host_name);
  setTextSize(1);
  println(F("And open a browser to:"));
  Serial.println(WiFi.softAPIP());
  setTextSize(2);
  display.println(WiFi.softAPIP());
}


bool setupWifi(void) {
  WiFi.hostname(config.host_name);
  
//  wifiManager.setDebugOutput(false);
  
  //reset settings - for testing
  //wifiManager.resetSettings();

  ssid = WiFi.SSID();
  if (ssid.length() > 0) {
    print(F("Connecting to "));
    Serial.println(ssid);
    display.println(ssid);
  }
  
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  if(!wifiManager.autoConnect(config.host_name)) {
    Serial.println(F("failed to connect and hit timeout"));
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  return true;
}


void setupTime(void) {
  if (!isSetup)
    println(F("Getting time"));
  else
    Serial.println(F("Getting time"));

  AsyncUDP* udp = new AsyncUDP();

  // time.nist.gov NTP server
  // NTP requests are to port 123
  if (udp->connect(IPAddress(129,6,15,28), 123)) {
//    Serial.println("UDP connected");
    
    udp->onPacket([](void *arg, AsyncUDPPacket packet) {
//      Serial.println(F("received NTP packet"));
      byte *buf = packet.data();
      
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
    
      // convert four bytes starting at location 40 to a long integer
      unsigned long secsSince1900 =  (unsigned long)buf[40] << 24;
      secsSince1900 |= (unsigned long)buf[41] << 16;
      secsSince1900 |= (unsigned long)buf[42] << 8;
      secsSince1900 |= (unsigned long)buf[43];
      time_t utc = secsSince1900 - 2208988800UL;
    
      TimeChangeRule *tcr;
      time_t local = myTZ.toLocal(utc, &tcr);
      Serial.printf("\ntime zone %s\n", tcr->abbrev);
    
      setTime(local);
    
      // just print out the time
      printTime(false, false, true);
    
      isTimeSet = true;

      free(arg);
    }, udp);
    
//    Serial.println(F("sending NTP packet"));

    const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
    byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold outgoing packet

    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    
    // all NTP fields have been given values, now
    // send a packet requesting a timestamp:
    udp->write(packetBuffer, NTP_PACKET_SIZE);
  }
  else {
    free(udp);
    if (!isSetup)
      print(F("\nWiFi:time failed...will retry in a minute\n"));
    else
      Serial.println(F("\nWiFi:time failed...will retry in a minute"));
  }
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      if (num == webClient)
        webClient = -1;
      else if (num == programClient)
        programClient = -1;
      else if (num == setupClient)
        setupClient = -1;
      else if (num == testClient) {
        testClient = -1;
        testAddr = -1;
        testValue = -1;
        testHeap = 0;
      }
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      
      if (strcmp((char *)payload,"/") == 0) {
        webClient = num;
      
        // send the current state
        printName();
        printCurrentTemperature(false);
        printTargetTemperature(false);
        printOutsideTemperature(false);
        printModeState(false);
        printFanState(false);
        printRunState(false);
        isForcePrint = true;
        printProgramState(last_p_program, last_p_time, false);
        printTime(false, false, false);
        isForcePrint = false;
      }
      else if (strcmp((char *)payload,"/program") == 0) {
        programClient = num;

        // send programs
        char json[265];
        strcpy(json, "{\"command\":\"program\",\"value\":[");
        for (int i=0; i < NUM_PROGRAMS; ++i) {
          sprintf(json+strlen(json), "%s[%d,%d,\"%s\",[",
            (i==0)?"":",",
            program[i].isEnabled,
            program[i].dayMask,
            program[i].name);
          for (int j=0; j < NUM_TIMES; ++j) {
            sprintf(json+strlen(json), "%d,%d,%d%s",
              program[i].time[j].startTime,
              program[i].time[j].coolTemp,
              program[i].time[j].heatTemp,
              (j==NUM_TIMES-1)?"]]":",");
          }
        }
        strcpy(json+strlen(json), "]}");
        //Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(programClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/setup") == 0) {
        setupClient = num;

        char json[512];
        strcpy(json, "{");
        sprintf(json+strlen(json), "\"date\":\"%s\"", __DATE__);
        sprintf(json+strlen(json), ",\"time\":\"%s\"", __TIME__);
        sprintf(json+strlen(json), ",\"host_name\":\"%s\"", config.host_name);
        sprintf(json+strlen(json), ",\"use_mqtt\":\"%d\"", config.use_mqtt);
        sprintf(json+strlen(json), ",\"mqtt_ip_addr\":\"%s\"", config.mqtt_ip_addr);
        sprintf(json+strlen(json), ",\"mqtt_ip_port\":\"%d\"", config.mqtt_ip_port);
        sprintf(json+strlen(json), ",\"ssid\":\"%s\"", ssid.c_str());
        sprintf(json+strlen(json), ",\"span\":\"%d\"", config.temperature_span);
        sprintf(json+strlen(json), ",\"timeout\":\"%d\"", config.display_timeout);
        sprintf(json+strlen(json), ",\"out_temp\":\"%d\"", config.out_temp);
        sprintf(json+strlen(json), ",\"swap\":\"%d\"", config.swap_sensors);
        sprintf(json+strlen(json), ",\"use_temp\":\"%d\"", config.use_temp);
        sprintf(json+strlen(json), ",\"temp_ip_addr\":\"%s\"", config.temp_ip_addr);
        sprintf(json+strlen(json), ",\"temp_ip_port\":\"%d\"", config.temp_ip_port);
        sprintf(json+strlen(json), ",\"key\":\"%s\"", config.wu_key);
        sprintf(json+strlen(json), ",\"location\":\"%s\"", config.wu_location);
        strcpy(json+strlen(json), "}");
//        Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(setupClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/test") == 0) {
        testClient = num;
        testAddr = MUX;

        // send the addessses of the muxes we are using
        char json[128];
        sprintf(json, "{\"msg\":\"MUX at %d\",\"addr\":\"%d\"}", MUX, MUX);
//        Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(testClient, json, strlen(json));
      }
      else {
        Serial.printf("unknown call %s\n", payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      
      if (num == webClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"up",2) == 0) {
          setTargetTemp(lastTargetTemperature+1);
        }
        else if (strncmp(ptr,"down",4) == 0) {
          setTargetTemp(lastTargetTemperature-1);
        }        
        else if (strncmp(ptr,"mode",4) == 0) {
          target ="value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          config.mode = strtol(ptr, &ptr, 10);
          saveConfig();
          drawMainScreen();
        }        
        else if (strncmp(ptr,"fan",3) == 0) {
          target ="value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          config.fan = strtol(ptr, &ptr, 10);
          saveConfig();
          drawMainScreen();
        }        
      }
      else if (num == programClient) {
        Serial.println(F("save programs"));
        char *ptr = strchr((char *)payload, '[')+2;
        for (int i=0; i < NUM_PROGRAMS; ++i) {
          program[i].isEnabled = strtol(ptr, &ptr, 10);
          ptr += 1;

          program[i].dayMask = strtol(ptr, &ptr, 10);
          ptr += 2;

          char *end = strchr(ptr, '\"');
          memcpy(program[i].name, ptr, (end-ptr));
          program[i].name[end-ptr] = '\0';

//          Serial.printf("num %d,isEnabled %d,dayMask %d,name %s\n", i, program[i].isEnabled, program[i].dayMask, program[i].name);
          ptr = end + 3;
          for (int j=0; j < NUM_TIMES; ++j, ++ptr) {
            program[i].time[j].startTime = strtol(ptr, &ptr, 10);
            program[i].time[j].coolTemp = strtol(ptr+1, &ptr, 10);
            program[i].time[j].heatTemp = strtol(ptr+1, &ptr, 10);
//            Serial.printf("startTime %d, coolTemp %d, heatTemp %d\n",
//              program[i].time[j].startTime,
//              program[i].time[j].coolTemp,
//              program[i].time[j].heatTemp);
          }
          ptr += 3;
        }      

        saveProgramConfig();
        drawMainScreen();
      }
      else if (num == setupClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"reboot",6) == 0) {
          ESP.restart();
        }
        else if (strncmp(ptr,"save",4) == 0) {
          Serial.println(F("save setup"));
          
          const char *target = "host_name";
          char *ptr = strstr((char *)payload, target) + strlen(target)+3;
          char *end = strchr(ptr, '\"');
          memcpy(config.host_name, ptr, (end-ptr));
          config.host_name[end-ptr] = '\0';

          target = "use_mqtt";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.use_mqtt = strtol(ptr, &ptr, 10);

          target = "mqtt_ip_addr";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.mqtt_ip_addr, ptr, (end-ptr));
          config.mqtt_ip_addr[end-ptr] = '\0';

          target = "mqtt_ip_port";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.mqtt_ip_port = strtol(ptr, &ptr, 10);

          target = "span";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.temperature_span = strtol(ptr, &ptr, 10);

          target = "timeout";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.display_timeout = strtol(ptr, &ptr, 10);
  
          target = "out_temp";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.out_temp = strtol(ptr, &ptr, 10);

          target = "swap";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.swap_sensors = strtol(ptr, &ptr, 10);

          target = "use_temp";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.use_temp = strtol(ptr, &ptr, 10);

          target = "temp_ip_addr";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.temp_ip_addr, ptr, (end-ptr));
          config.temp_ip_addr[end-ptr] = '\0';

          target = "temp_ip_port";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.temp_ip_port = strtol(ptr, &ptr, 10);
  
          target = "key";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.wu_key, ptr, (end-ptr));
          config.wu_key[end-ptr] = '\0';
  
          target = "location";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.wu_location, ptr, (end-ptr));
          config.wu_location[end-ptr] = '\0';

//    Serial.printf("host_name %s\n", config.host_name);
//    Serial.printf("use_mqtt %d\n", config.use_mqtt);
//    Serial.printf("mqtt_ip_addr %s\n", config.mqtt_ip_addr);
//    Serial.printf("mqtt_ip_port %d\n", config.mqtt_ip_port);
//    Serial.printf("temperature_span %d\n", config.temperature_span);
//    Serial.printf("display_timeout %d\n", config.display_timeout);
//    Serial.printf("out_temp %d\n", config.out_temp);
//    Serial.printf("swap_sensors %d\n", config.swap_sensors);
//    Serial.printf("use_temp %d\n", config.use_temp);
//    Serial.printf("temp_ip_addr %s\n", config.temp_ip_addr);
//    Serial.printf("temp_ip_port %d\n", config.temp_ip_port);
//    Serial.printf("wu_key %s\n", config.wu_key);
//    Serial.printf("wu_location %s\n", config.wu_location);
          saveConfig();
        }
      }
      else if (num == testClient) {
        Serial.printf("test %s\n", payload);
        const char *target = "addr";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        int addr = strtol(ptr, &ptr, 10);
        target = "value";
        ptr = strstr(ptr, target) + strlen(target)+3;
        int value = strtol(ptr, &ptr, 10);
        Serial.printf("%d %d\n", addr, value);
        send(addr, value);
        testAddr = addr;
      }
      break;
  }
}


void sendWeb(const char *command, const char *value) {
  char json[128];
  sprintf(json, "{\"command\":\"%s\",\"value\":\"%s\"}", command, value);
  webSocket.sendTXT(webClient, json, strlen(json));
}


void setupButtons(void) {
  numButtons = sizeof(buttonPins)/sizeof(buttonPins[0]);
  buttonStates = new int[numButtons];
  lastButtonStates = new int[numButtons];
  lastDebounceTimes = new long[numButtons];

  // initialize the pushbutton pins as inputs
  for (int i=0; i < numButtons; ++i) {
    buttonStates[i] = defaultButtonState;
    lastButtonStates[i] = defaultButtonState;
    lastDebounceTimes[i] = 0;
  }
  println(F("buttons: setup"));
}


void setupRelays(void) {
#ifdef LATCH_RELAYS
  println(F("using LATCH RELAYS"));
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
#endif
  
  relaysOff();
  isFanOn = false;
  isCompressorOn = false;
  isReversingValueOn = false;

  println(F("relays: setup"));
}


void relaysOff(void) {
#ifdef LATCH_RELAYS
  // pulse the reset line low for 5ms to turn all the latch relays off
  digitalWrite(RESET_PIN, LOW);
  delay(5);
  digitalWrite(RESET_PIN, HIGH);
#else
  byte value = muxState;
  for (int i=0; i < 3; ++i)
    value |= relayPins[i];
  send(MUX, value);
#endif
  
  Serial.println(F("relaysOff"));
}


void setupMotion(void) {
  println(F("motion: setup"));
}


void loop(void) {
  if (!isSetup)
    return;

  unsigned long time = millis();

  // only check the motion sensor every so often
  if ( time - lastMotionReadTime > TimeBetweenMotionChecks ) {
    checkMotion(time);
    lastMotionReadTime = time;
  }
  
  if (screen == MAIN) {
    // turn off the display if idle
    if (isDisplayOn && ((time - lastButtonPressTime) > (config.display_timeout*1000)) ) {
      displayBacklight(false);
    }

    checkTime(time);

    checkTemperature(time);

    // flash the time if its not set
    if (!isTimeSet)
      flashTime(time);
  }
  else {
    // go back to main screen if idle
    if (time - lastButtonPressTime > MenuIdleTime ) {
      drawMainScreen();
    }
  }
  
  checkButtons();

  webSocket.loop();
  server.handleClient();
  MDNS.update();
  
  // mqtt
  if (config.use_mqtt) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }

  ArduinoOTA.handle();

  if (config.out_temp) {
    if (numDevices == 1) {
      if (config.use_temp)
        checkClientConnection();
    }
  }
}


void checkClientConnection(void) {
  if (airTempClient == NULL)
    setUpAirTemp();

  // try and connect every minute if not already connected
  if (!airTempClient->connected()) {
    if (millis() > lastAttempt + TIME_BETWEEN_CONNECTS) {
      Serial.printf("connecting to socket server\n");
      airTempClient->connect(config.temp_ip_addr, config.temp_ip_port);
      lastAttempt = millis();
    }
  }
}


#define TimeBetweenFlashes 500
unsigned long lastFlashTime = 0;
bool isFlash = false;


void flashTime(unsigned long time) {
  // the temperature sensor blocks the app from running while it is reading.
  // so, it may make the flash look off every time it is being checked
  if (time - lastFlashTime > TimeBetweenFlashes) {
    lastFlashTime = time;
    isFlash = !isFlash;
    if (isFlash)
      eraseTime();
    else
      printTime(false, true, false);
  }
}


void checkMotion(unsigned long time) {
  Wire.requestFrom(MUX, 1); 
  if (!Wire.available())
    return;
    
  byte value = Wire.read();
  if ((value & MOTION) != 0) {
//    Serial.println(F("motion"));
    lastButtonPressTime = time;
  
    if (!isDisplayOn) {
      displayBacklight(true);
    }
  }
}


void checkTime(unsigned long time) {
  int minutes = minute();

  if (minutes == lastMinutes)
    return;

  // resync time at 3am every morning
  // this also catches daylight savings time changes which happen at 2am
  if (minutes == 0 && hour() == 3)
    isTimeSet = false;

  if (!isTimeSet)
    setupTime();
 
  lastMinutes = minutes;
  printTime(true, true, false);

  if (minutes % 5 == 0)
    everyFiveMinutes();
}


void everyFiveMinutes(void) {
  if (config.out_temp) {
    // get the outside temperature, unless we are using a sensor outside
    // or a remote air temperature sensor
    if (!config.use_temp && numDevices == 1) {
      requestOutsideTemperature();
      return;
    }
  }
      
  // log the temperatures to the database
  if (lastTemp != TEMP_ERROR && lastOutsideTemp != TEMP_ERROR)
    logTemperature(lastTemp, lastOutsideTemp);
}


void checkTemperature(unsigned long time) {
  if (time - lastTempRequest >= delayInMillis) // waited long enough??
  {
//  Serial.println(F("checking temperature"));
    checkTemperature();

    if (config.out_temp) {
      // if we're not using a remote air temperature sensor, and have two connected sensors,
      // then read the second sensor now
      if (!config.use_temp && numDevices == 2)
        checkOutsideTemperature();
    }
        
    sensors.requestTemperatures(); 
    lastTempRequest = millis(); 
  }
}


int lastSentTargetTemperature = TEMP_ERROR;


void checkTemperature(void) {
  float tempF = sensors.getTempF((config.swap_sensors == 0) ? thermometer1 : thermometer2);
  tempF = (float)round(tempF*10)/10.0;
  if ( tempF == lastTemp )
    return;
    
  lastTemp = tempF;

  if (screen == MAIN) {
    printCurrentTemperature(true);

    int temp = getTargetTemperature();
    if (temp != lastSentTargetTemperature ) {
      lastSentTargetTemperature = temp;
      logTarget(temp);
      printTargetTemperature(true);
    }
  }
  
  checkUnit();
}


void checkOutsideTemperature(void) {
  float tempF = sensors.getTempF((config.swap_sensors == 0) ? thermometer2 : thermometer1);
  tempF = (float)round(tempF*10)/10.0;
  if ( tempF == lastOutsideTemp )
    return;
    
  lastOutsideTemp = tempF;
  printOutsideTemperature(true);
}


void setTextSize(int size) {
#ifdef FONTS
  if (size == 2)
    display.setFont(font_arial_16);
  else if (size == 3)
    display.setFont(font_arial_24);
  else
    display.setFont(font_arial_8);
#else
  display.setTextSize(size);
#endif
}


void eraseTime(void) {
  if (isDisplayOn) {
    setTextSize(2);
    char *txt = (char *)"         ";
    setCursor(128-getStringWidth(txt), 25);
    display.print(txt);
    txt = (char *)"              ";
    setCursor(128-getStringWidth(txt), 45);
    display.print(txt);
  }
}


void printTime(bool isCheckProgram, bool isDisplay, bool isTest) {
  int dayOfWeek = weekday()-1;
  int hours = hour();
  int minutes = minute();
  const char *ampm = "a";
  int h = hours;
  if (hours == 0)
    h = 12;
  else if (h == 12)
    ampm = "p";
  else if (hours > 12) {
    h -= 12;
    ampm = "p";
  }
  char buf[10];
  sprintf(buf, "  %2d:%02d%s", h, minutes, ampm); 
//  Serial.println(buf);

  if (isDisplay && isDisplayOn) {
    display.setTextColor(GREEN, BLACK);
    setTextSize(2);
    setCursor(128-getStringWidth(buf), 45);
    display.print(buf);
    setCursor(128-getStringWidth(weekdayNames[dayOfWeek]), 25);
    display.print(weekdayNames[dayOfWeek]);
  }
  
  if (webClient != -1 || isTest) {
    char msg[6+1+4];
    sprintf(msg, "%s %s", buf, weekdayNames[dayOfWeek]); 
    if (webClient != -1)
      sendWeb("time", msg);
    if (isTest)
      Serial.printf("time is %s\n", msg);
  }
  
  if (isCheckProgram)
    checkProgram(dayOfWeek, hours, minutes);
}


byte getProgramForDay(int day) {
  // day: Sun = 0, Mon = 1, etc
  // convert day to a dayMask
  byte mask = 1 << day;
//  Serial.print("mask ");
//  Serial.println(mask, BIN);

  for (int i=0; i < NUM_PROGRAMS; ++i) {
    if ( program[i].isEnabled ) {
      if ( mask & program[i].dayMask ) {
//        Serial.printf("day match with %s\n", program[i].name);
        return i;
      }
    }
  }

  // we should never get here, this means they didn't configure
  // one of the days of the week in the programs
  Serial.print(F("error - no program defined for day")); 
  Serial.println(day);
  return 0;  
}


void checkProgram(int day, int h, int m) {
  if (config.mode == OFF) {
    printProgramState(0,0,true);
    return;
  }

//  Serial.println(F("check program"));
  byte ctime = h*4+(m/15);
//  Serial.printf("ctime %d day %d\n", ctime, day);

  byte p_program = getProgramForDay(day);

  // if the current time is before the first program time of the day,
  // then use the last program time/temp of the previous day
  byte p_time;
  if (ctime < program[p_program].time[0].startTime) {  
//    Serial.println("using last program from previous day");
    if (day == 0)
      day = 6;
    else
      --day;
    p_program = getProgramForDay(day);
    p_time = NUM_TIMES-1;
  }
  else {
    // locate which time slot we are in
    p_time = NUM_TIMES-1;
    for (int i=1; i < NUM_TIMES; ++i) {
      if (ctime < program[p_program].time[i].startTime) {
        p_time = i-1;
        break;  
      }
    }
  }
//  Serial.printf("time match with %d\n", p_time);

  byte p_ctemp = program[p_program].time[p_time].coolTemp;
  byte p_htemp = program[p_program].time[p_time].heatTemp;
//  Serial.printf("target temp %d $d\n", p_ctemp, p_htemp);

  // if we just started a new program, then cancel the temp hold
  // cpd...perm hold should be checked here
  if (last_p_time != p_time) {
    if (config.mode == TEMP) {
      config.mode = ON;
      printModeState(true);
    }
  }
  printProgramState(p_program, p_time, true);
  
  if (config.mode == ON && (targetCoolTemp != p_ctemp || targetHeatTemp != p_htemp)) {
    targetCoolTemp = p_ctemp;
    targetHeatTemp = p_htemp;
    logTarget(getTargetTemperature());
    printTargetTemperature(true);
    checkUnit();
  }
}


int outsideTempWidth = 0;


void eraseOutsideTemperature(bool isDisplay) {
  if (isDisplay && isDisplayOn) {
    display.setTextColor(GREEN, BLACK);
    setTextSize(2);

    int width = 0;
//    Serial.printf("outside temp %d %d\n", width, outsideTempWidth);
    if (width < outsideTempWidth) {
      // erase some more
      display.fillRect(128-outsideTempWidth, 0, outsideTempWidth-width, getFontHeight(), BLACK);
    }
    outsideTempWidth = width;
  }

  if (webClient != -1) {
    sendWeb("outsideTemp", "");  
  }
}


void printOutsideTemperature(bool isDisplay) {
  if (!config.out_temp)
    return;

  if (lastOutsideTemp == TEMP_ERROR)
    return;
    
  if (isDisplay && isDisplayOn) {
    display.setTextColor(GREEN, BLACK);
    setTextSize(2);

    // weather underground only gives us integer precision for the temperature
    // the temp sensor or the remote air sensor gives us decimal precision
    // so only show the precision we have on the display
    char buf[8];
    if (config.use_temp || numDevices == 2)
      dtostrf(lastOutsideTemp, 4, 1, buf);
    else
      sprintf(buf, "%d", (int)lastOutsideTemp);
#ifdef FONTS
    char *ptr = buf + strlen(buf);
    *ptr++ = (char)176;  // degree symbol
    *ptr = '\0';
#endif
    int width = getStringWidth(buf);
//    Serial.printf("outside temp %d %d\n", width, outsideTempWidth);
    if (width < outsideTempWidth) {
      // erase some more
      display.fillRect(128-outsideTempWidth, 0, outsideTempWidth-width, getFontHeight(), BLACK);
    }
    outsideTempWidth = width;

    setCursor(128-width, 0);
    display.print(buf);
  }

  if (webClient != -1) {
    char buf[7];
    if (config.use_temp || numDevices == 2)
      dtostrf(lastOutsideTemp, 4, 1, buf);
    else
      sprintf(buf, "%d", (int)lastOutsideTemp);
    sendWeb("outsideTemp", buf);  
  }
}


int currentTempWidth = 0;


void printCurrentTemperature(bool isDisplay) {
  if (isDisplay && isDisplayOn) {
    display.setTextColor(GREEN, BLACK);
    setTextSize(3);

    char buf[8];
    dtostrf(lastTemp, 4, 1, buf);
#ifdef FONTS
    char *ptr = buf + strlen(buf);
    *ptr++ = (char)176;
    *ptr = '\0';
#endif
    int width = getStringWidth(buf);
    if (width < currentTempWidth) {
      // erase some more
      display.fillRect(width, 0, currentTempWidth-width, getFontHeight(), BLACK);
    }
    currentTempWidth = width;

    setCursor(0, 0);
    display.print(buf);

//    display.print(lastTemp,1);
//#ifdef FONTS
//    display.print((char)176);  // degree symbol
//#endif
//    display.print(" ");
  }

  if (webClient != -1) {
    char buf[7];
    dtostrf(lastTemp, 4, 1, buf);
    sendWeb("currentTemp", buf);  
  }
}


int getTargetTemperature(void) {
  if (lastTemp == TEMP_ERROR )
    return TEMP_ERROR;
  else if (lastTargetTemperature == 0 || (lastTargetTemperature != targetCoolTemp && lastTargetTemperature != targetHeatTemp)) {
    /*
     * the target tempersature is either the target cool or heat terperature
     * whichever one the current temperature is closer to
     */
    float midTemp = (targetCoolTemp - targetHeatTemp) / 2 + targetHeatTemp;
    if ( lastTemp >= midTemp ) {
      lastTargetTemperature = targetCoolTemp;
      isTargetCool = true;
    }
    else {
      lastTargetTemperature = targetHeatTemp;
      isTargetCool = false;
    }
  }
  else if (lastTargetTemperature == targetCoolTemp) {
    if ( lastTemp < targetHeatTemp ) {
      lastTargetTemperature = targetHeatTemp;
      isTargetCool = false;
    }
  }
  else if (lastTargetTemperature == targetHeatTemp) {
    if ( lastTemp > targetCoolTemp ) {
      lastTargetTemperature = targetCoolTemp;
      isTargetCool = true;
    }
  }
  return lastTargetTemperature;
}


void printTargetTemperature(bool isDisplay) {
  if (lastTargetTemperature == TEMP_ERROR)
    return;

  if (isDisplay && isDisplayOn) {
    setCursor(0, 34);
    display.setTextColor(GREEN, BLACK);
    setTextSize(3);
    if (config.mode == OFF)
      display.print("    ");
    else {
      display.print(lastTargetTemperature);
#ifdef FONTS
      display.print((char)176);  // degree symbol
#endif
      display.print(" ");
    }
  }

  if (webClient != -1) {
    if (config.mode == OFF)
      sendWeb("targetTemp", "");  
    else {
      char buf[5];
      sprintf(buf, "%d", lastTargetTemperature);
      sendWeb("targetTemp", buf);  
    }
  }
}


void printModeState(bool isDisplay) {
  if (isDisplay && isDisplayOn) {
    display.setTextColor(GREEN, BLACK);
    setTextSize(2);
    setCursor(0, 70);
    display.print("Mode:");
    setCursor(128-getStringWidth(modeNames[config.mode]), 70);
    display.print(modeNames[config.mode]);
  }

  if (webClient != -1) {
    char buf[3];
    sprintf(buf, "%d", config.mode);    
    sendWeb("mode", buf);
  }
}


void printFanState(bool isDisplay) {
  if (isDisplay && isDisplayOn) {
    display.setTextColor(GREEN, BLACK);
    setTextSize(2);
    setCursor(0, 90);
    display.print("Fan:");
    setCursor(128-getStringWidth(fanNames[config.fan]), 90);
    display.print(fanNames[config.fan]);
  }

  if (webClient != -1) {
    char buf[3];
    sprintf(buf, "%d", config.fan);    
    sendWeb("fan", buf);
  }
}


void printRunState(bool isDisplay) {
  if (isDisplay && isDisplayOn) {
    setCursor(0, 110);
    display.setTextColor(GREEN, BLACK);
    setTextSize(2);
    if (isCompressorOn) 
      display.print(modeStateNames[mode]);
    else if (isFanOn)
      display.print("Fan  ");
    else
      display.print("          ");
  }

  if (webClient != -1) {
    if (isCompressorOn) 
      sendWeb("run", modeStateNames[mode]);
    else if (isFanOn)
      sendWeb("run", "Fan");
    else
      sendWeb("run", "");
  }
}


int getStringWidth(const char* buf) {
  // cpd...not working
//  return display.getStringWidth(buf);
  return 10;
}


int getFontHeight() {
  // cpd...not working
//  return display.getFontHeight();
  return 8;
}


void printProgramState(byte p_program, byte p_time, bool isDisplay) {
  if (last_p_program == p_program && last_p_time == p_time && !isForcePrint)
    return;
  
  last_p_program = p_program;  
  last_p_time = p_time;  

  if (isDisplay && isDisplayOn) {
    display.setTextColor(GREEN, BLACK);
    setTextSize(2);
    if (config.mode != ON) {
      setCursor(65, 110);
      display.print("        ");
    }
    else {
      char buf[6];
      sprintf(buf, "%s %d", program[p_program].name, p_time+1);
      setCursor(128-getStringWidth(buf), 110);
      display.print(buf);
    }
  }

  if (webClient != -1) {
    if (config.mode != ON)
      sendWeb("program", "");
    else {
      char buf[6];
      sprintf(buf, "%s %d", program[p_program].name, p_time+1);
      sendWeb("program", buf);
    }
  }
}


void checkButtons(void) {
  if (testClient != -1 && testAddr != -1) {
    Wire.requestFrom(testAddr, 1); 
    if (!Wire.available())
      return;

    unsigned int heap = ESP.getFreeHeap();
    byte value = Wire.read();
    if (value != testValue || heap != testHeap) {
      testValue = value;
      testHeap = heap;
      char json[128];
      sprintf(json, "{\"command\":\"read\",\"value\":\"%d\",\"heap\":\"%u\"}", value, heap);
      Serial.printf("sending %s\n", json);
      webSocket.sendTXT(testClient, json, strlen(json));
    }
  }

  Wire.requestFrom(MUX, 1); 
  if (!Wire.available())
    return;
    
  byte value = Wire.read();
  for (int i=0; i < numButtons; ++i) {
    int reading = ((value & buttonPins[i]) == 0) ? HIGH : LOW;
//    if (i == 1)
//      Serial.printf("%d %d %d\n", i, reading, lastButtonStates[i]);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonStates[i]) {
      // reset the debouncing timer
      lastDebounceTimes[i] = millis();
    }

    if ((millis() - lastDebounceTimes[i]) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        if (reading != defaultButtonState)
          doButton(buttonPins[i]);
      }
    }

    // save the reading.  Next time through the loop,
    // it'll be the lastButtonState:
    lastButtonStates[i] = reading;
  }
}


void doButton(int pin) {
  lastButtonPressTime = millis();
  
  if (!isDisplayOn) {
    // just turn on the display, ignore the button press
    displayBacklight(true);
    return;
  }
  
  switch (pin) {
    case UP:
      doUp();
      break;
    case DOWN:
      doDown();
      break;
    case ENTER:
      doEnter();
      break;
  }
}


void doUp(void) {
  switch(screen) {
    case MAIN:
      setTargetTemp(lastTargetTemperature+1);
      break;
    case MENU:
      setSelectedMenuItem(selectedMenuItem-1);
      break;
  }
}


void doDown(void) {
  switch(screen) {
    case MAIN:
      setTargetTemp(lastTargetTemperature-1);
      break;
    case MENU:
      setSelectedMenuItem(selectedMenuItem+1);
      break;
  }
}


void doEnter(void) {
  switch(screen) {
    case MAIN:
      screen = MENU;
      clearScreen();  
      setSelectedMenuItem(EXIT_BUTTON);
      break;
    case MENU:
      doMenuEnter();
      break;
  }
}


void doMenuEnter(void) {
  switch(selectedMenuItem) {
    case MODE_BUTTON:
      ++config.mode;
      if ( config.mode > HOLD )
        config.mode = OFF;
      drawMenu();
      break;
    case FAN_BUTTON:
      ++config.fan;
      if ( config.fan > ON )
        config.fan = AUTO;
      drawMenu();
      break;
    case EXIT_BUTTON:
      saveConfig();
      drawMainScreen();
      break;
  }
}


void drawMainScreen(void) {
  screen = MAIN;

  if (isDisplayOn)
    clearScreen();
  if (lastTemp != TEMP_ERROR)
    printCurrentTemperature(true);
  printTargetTemperature(true);
  printOutsideTemperature(true);
  printModeState(true);
  printFanState(true);
  
  isForcePrint = true;
  printTime(true, true, false);
  isForcePrint = false;
  
  checkUnit();
}


#define BUTTON_WIDTH 128
#define BUTTON_HEIGHT 30


void setSelectedMenuItem(int item) {
  if (item < MODE_BUTTON) item = EXIT_BUTTON;
  else if (item > EXIT_BUTTON ) item = MODE_BUTTON;
  
  selectedMenuItem = item;
  
  drawMenu();
}


void drawMenu(void) {
  // display our ip address
  setCursor(0, 100);
  display.setTextColor(GREEN);
  setTextSize(2);
  display.print(WiFi.localIP());

  drawButton("Mode:", modeNames[config.mode], 0, selectedMenuItem);
  drawButton("Fan:", fanNames[config.fan], 1, selectedMenuItem);
  drawButton("Exit", 2, selectedMenuItem, 2, 0, 12);
}


void drawButton(const char *s, int i, int sel, int row, int col, int width) {
  int color = (i == sel) ? RED : GREEN;
  int x = col * BUTTON_WIDTH/12;
  int y = row * (BUTTON_HEIGHT+1);
  int w = width * BUTTON_WIDTH/12;
  setCursor(x+indent, y+8);
  display.setTextColor(color, BLACK);
  setTextSize(2);
  display.print(s);
#ifdef DISPLAY_18
  y += 16;
#endif
#ifdef DISPLAY_22
#endif
#ifdef DISPLAY_28
#endif
  display.drawRect(x,y,w,BUTTON_HEIGHT,color);
}


void drawButton(const char *s1, const char *s2, int i, int sel) {
  int color = (i == sel) ? RED : GREEN;
  int y = i * (BUTTON_HEIGHT+1);
  setCursor(10, y+8);
  display.setTextColor(color, BLACK);
  setTextSize(2);
  display.print(s1);
  if (s2)
    display.print(s2);
#ifdef DISPLAY_18
  y += 16;
#endif
#ifdef DISPLAY_22
#endif
#ifdef DISPLAY_28
#endif
  display.drawRect(0,y,BUTTON_WIDTH,BUTTON_HEIGHT,color);
}


void setTargetTemp(int newTemp) {
  if (config.mode == OFF)
    return;
    
  if (config.mode == ON) {
    config.mode = TEMP;
    printModeState(true);
  }

  if (isTargetCool)
    targetCoolTemp = newTemp;
  else
    targetHeatTemp = newTemp;
  lastTargetTemperature = newTemp;
  logTarget(newTemp);
  printTargetTemperature(true);
  printProgramState(last_p_program, last_p_time, true);
  checkUnit();
}


void relayOn(int which) {
#ifdef LATCH_RELAYS
  byte value = muxState;
  value &= ~relayPins[which];
  send(MUX, value);
  delay(5);
  value |= relayPins[which];
  send(MUX, value);
#else
  byte value = muxState;
  value &= ~relayPins[which];
  send(MUX, value);
#endif

  Serial.printf("relay on %d\n", which);
}


void checkUnit(void) {
  // turn the unit on or off if needed
  if (config.mode == OFF) {
    if (isCompressorOn || isReversingValueOn) {
      relaysOff();
      logAction(SYSTEM_OFF);
      isCompressorOn = false;
      isReversingValueOn = false;
      isFanOn = false;
    }
    if (config.fan == AUTO) {
      if (isFanOn) {
        relaysOff();
        logAction(SYSTEM_OFF);
        isFanOn = false;
      }
    }
    else {
      if (!isFanOn) {
        relayOn(FAN);
        logAction(SYSTEM_FAN);
        isFanOn = true;
      }
    }
  }
  else {
    // cool or heat
    boolean isOn = false;
    float span = (float)config.temperature_span / 10.0;
    if (isCompressorOn) {
      // to prevent short cycling, don't turn off until we're
      // <span> degrees below target temp (cooling)
      // or <span> degrees above target temp (heating)
      if (lastTemp >= (targetCoolTemp-span)) {
        isOn = true;
        mode = COOL;
      }
      else if (lastTemp <= (targetHeatTemp+span)) {
        isOn = true;
        mode = HEAT;
      }
    }
    else {
      // to prevent short cycling, don't turn on until we're
      // <span> degrees above target temp (cooling)
      // or <span> degrees below target temp (heating)
      if (lastTemp >= (targetCoolTemp+span)) {
        isOn = true;
        mode = COOL;
      }
      else if (lastTemp <= (targetHeatTemp-span)) {
        isOn = true;
        mode = HEAT;
      }
    }

    if (isOn) {
      if (!isCompressorOn) {
        relayOn(COMPRESSOR);
        logAction((mode == HEAT) ? SYSTEM_HEAT : SYSTEM_COOL);
        isCompressorOn = true;
      }
      if (!isFanOn) {
        relayOn(FAN);
        isFanOn = true;
      }
      if (mode == COOL) {
        if (!isReversingValueOn) {
          relayOn(REVERSING_VALVE);
          isReversingValueOn = true;
        }
      }
    }
    else {
      if (isCompressorOn || isReversingValueOn || isFanOn) {
        relaysOff();
        logAction(SYSTEM_OFF);
        isCompressorOn = false;
        isReversingValueOn = false;
        isFanOn = false;
      }
    }
  }
  
  if (screen == MAIN)
    printRunState(true);
}


#define MAGIC_NUM   0xB1

#define MAGIC_NUM_ADDRESS      0
#define CONFIG_ADDRESS         1
#define PROGRAM_ADDRESS        CONFIG_ADDRESS + sizeof(config)


void set(char *name, const char *value) {
  for (int i=strlen(value); i >= 0; --i)
    *(name++) = *(value++);
}


void loadConfig(void) {
  int magicNum = EEPROM.read(MAGIC_NUM_ADDRESS);
  if (magicNum != MAGIC_NUM) {
    Serial.println(F("invalid eeprom data"));
    isMemoryReset = true;
  }
  
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.printf("using default config\n");
    set(config.host_name, HOST_NAME);
    set(config.mqtt_ip_addr, MQTT_IP_ADDR);
    config.mqtt_ip_port = MQTT_IP_PORT;
    config.use_mqtt = 0;
    config.mode = OFF;
    config.fan = AUTO;
    config.temperature_span = 7;
    config.display_timeout = 20;
    config.out_temp = 0;
    config.swap_sensors = 0;
    config.use_temp = 0;
    set(config.temp_ip_addr, TEMP_IP_ADDR);
    config.temp_ip_port = TEMP_IP_PORT;
    set(config.wu_key, WU_API_KEY);
    set(config.wu_location, WU_LOCATION);

    saveConfig();
  }
  else {
    int addr = CONFIG_ADDRESS;
    byte *ptr = (byte *)&config;
    for (int i=0; i < sizeof(config); ++i, ++ptr)
      *ptr = EEPROM.read(addr++);
  }
  lastTemp = TEMP_ERROR;  
  lastOutsideTemp = TEMP_ERROR;  

//  Serial.printf("host_name %s\n", config.host_name);
//  Serial.printf("use_mqtt %d\n", config.use_mqtt);
//  Serial.printf("mqqt_ip_addr %s\n", config.mqtt_ip_addr);
//  Serial.printf("mqtt_ip_port %d\n", config.mqtt_ip_port);
//  Serial.printf("mode %d\n", config.mode);
//  Serial.printf("fan %d\n", config.fan);
//  Serial.printf("temperature_span %d\n", config.temperature_span);
//  Serial.printf("display_timeout %d\n", config.display_timeout);
//  Serial.printf("out_temp %d\n", config.out_temp);
//  Serial.printf("swap_sensors %d\n", config.swap_sensors);
//  Serial.printf("use_temp %d\n", config.use_temp);
//  Serial.printf("temp_ip_addr %s\n", config.temp_ip_addr);
//  Serial.printf("temp_ip_port %d\n", config.temp_ip_port);
//  Serial.printf("wu_key %s\n", config.wu_key);
//  Serial.printf("wu_location %s\n", config.wu_location);
}


void loadProgramConfig(void) {
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.printf("using default programs\n");
    initProgram();  
  }
  else {
    Serial.printf("loading programs from eeprom\n");
    int addr = PROGRAM_ADDRESS;
    byte *ptr = (byte *)&program;
    for (int i = 0; i < sizeof(program); ++i, ++ptr, ++addr)
      *ptr = EEPROM.read(addr);
  }
}


void saveConfig(void) {
  isPromModified = false;
  update(MAGIC_NUM_ADDRESS, MAGIC_NUM);
  
  byte *ptr = (byte *)&config;
  int addr = CONFIG_ADDRESS;
  for (int j=0; j < sizeof(config); ++j, ++ptr)
    update(addr++, *ptr);

  if (isPromModified)
    EEPROM.commit();
}


void saveProgramConfig(void) {
  isPromModified = false;
  Serial.printf("saving programs to eeprom\n");
  int addr = PROGRAM_ADDRESS;
  byte *ptr = (byte *)&program;
  for (int i = 0; i < sizeof(program); ++i, ++ptr, ++addr)
    update(addr, *ptr);
      
  if (isPromModified)
    EEPROM.commit();
}


void update(int addr, byte data) {
  if (EEPROM.read(addr) != data) {
    EEPROM.write(addr, data);
    isPromModified = true;
  }
}


void setupOta(void) {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(config.host_name);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    const char *msg = "Unknown Error";
    if (error == OTA_AUTH_ERROR) {
      msg = "Auth Failed";
    } else if (error == OTA_BEGIN_ERROR) {
      msg = "Begin Failed";
    } else if (error == OTA_CONNECT_ERROR) {
      msg = "Connect Failed";
    } else if (error == OTA_RECEIVE_ERROR) {
      msg = "Receive Failed";
    } else if (error == OTA_END_ERROR) {
      msg = "End Failed";
    }
    Serial.println(msg);
  });
  
  ArduinoOTA.begin();
  Serial.println("Arduino OTA ready");

  char host[20];
  sprintf(host, "%s-webupdate", config.host_name);
  MDNS.begin(host);
  httpUpdater.setup(&server);
  MDNS.addService("http", "tcp", 80);
  Serial.println("Web OTA ready");
}


void setupMqtt() {
  client.setServer(config.mqtt_ip_addr, config.mqtt_ip_port);
  client.setCallback(callback);
}


void logTemperature(float inside, float outside) {
  if (config.use_mqtt) {
    // mqtt
    char topic[30];
    sprintf(topic, "%s/temperature", config.host_name);
    char json[128];
    sprintf(json, "{\"inside\":\"%f\",\"outside\":\"%f\"}",
      inside, outside);
    client.publish(topic, json);
  }
}


void logAction(char action) {
  if (config.use_mqtt) {
    // mqtt
    char topic[30];
    sprintf(topic, "%s/action", config.host_name);
    char buf[10];
    sprintf(buf, "%c", action);
    client.publish(topic, buf);
  }
}


void logTarget(int temperature) {
  if (config.use_mqtt) {
    // mqtt
    char topic[30];
    sprintf(topic, "%s/target", config.host_name);
    char buf[10];
    sprintf(buf, "%d", temperature);
    client.publish(topic, buf);
  }
}


void requestOutsideTemperature(void) {
  AsyncClient* aclient = new AsyncClient();
      
  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    // after connecting, send the request, and wait for a reply
    // Make an HTTP GET request
    c->write(WUNDERGROUND_REQ1);
    c->write(config.wu_key);
    c->write(WUNDERGROUND_REQ2);
    c->write(config.wu_location);
    c->write(WUNDERGROUND_REQ3);
  }, NULL);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
//    Serial.printf("requestOutsideTemperature [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);
    // search for the temperature in the response
    const char *target = "\"temp_f\":";
    char *ptr = strstr((char *)buf, target);
    if (ptr != NULL) {
      ptr += strlen(target);
      int temp = strtol(ptr, &ptr, 10);
      Serial.print("outside temp is ");
      Serial.println(temp);
      gotOutsideTemperature((float)temp);
      c->close(true);  // close right now...no more onData
    }
  }, NULL);
  
//  Serial.printf("request temp\n");
  if (!aclient->connect(WUNDERGROUND, 80)) {
    free(aclient);
  }
}


void gotOutsideTemperature(float tempF) {
  if ( tempF != lastOutsideTemp ) {
    lastOutsideTemp = tempF;
    printOutsideTemperature(true);
  }

  // log the temperatures to the database
  if (lastTemp != TEMP_ERROR && lastOutsideTemp != TEMP_ERROR)
    logTemperature(lastTemp, lastOutsideTemp);
}


//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}


String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}


bool handleFileRead(String path){
  Serial.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}


void handleFileUpload_edit(){
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
}


void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.print(F("handleFileDelete: "));
  Serial.println(path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}


void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.print(F("handleFileCreate: "));
  Serial.println(path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}


void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  Serial.print(F("handleFileList: "));
  Serial.println(path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}


void countRootFiles(void) {
  int num = 0;
  size_t totalSize = 0;
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    ++num;
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    totalSize += fileSize;
    Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
  }
  Serial.printf("FS File: serving %d files, size: %s from /\n", num, formatBytes(totalSize).c_str());
}


void setupWebServer(void) {
  SPIFFS.begin();

  countRootFiles();
  
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  
  //load editor
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload_edit);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([](){
    if(!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  server.begin();

  Serial.println(F("HTTP server started"));
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
//    // Create a random client ID
//    String clientId = "ESP8266Client-";
//    clientId += String(random(0xffff), HEX);
//    // Attempt to connect
//    if (client.connect(clientId.c_str())) {
    if (client.connect(config.host_name)) {
      Serial.println("connected");
      // ... and resubscribe
      char topic[30];
      sprintf(topic, "%s/command", config.host_name);
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  // only topic we get is <host_name>/command
  
  // strip off the hostname from the topic
  topic += strlen(config.host_name) + 1;
    
  char value[12];
  memcpy(value, payload, length);
  value[length] = '\0';
  Serial.printf("Message arrived [%s] %s\n", topic, value);

  if (strcmp(topic, "command") == 0) {
    char action = *value;
    if (action == SYSTEM_OFF) {
      // cpd
//      config.mode = RUN;
//      modeChange(false);
    }
    else if (action == SYSTEM_FAN) {
      // cpd
//      config.mode = OFF;
//      modeChange(false);
    }
    else if (action == SYSTEM_HEAT) {
      // cpd
//      config.mode = OFF;
//      modeChange(false);
    }
    else if (action == SYSTEM_COOL) {
      // cpd
//      config.mode = OFF;
//      modeChange(false);
    }
    else {
      Serial.printf("Unknown action: %c\n", action);
    }
  }
  else {
    Serial.printf("Unknown topic: %s\n", topic);
  }
}


/*
cpd...todo


log to pogoplug database
database is running
httpd is running
php is running

docs at /srv/http
httpd logfile at /var/log/httpd
http://pogoplug2/test.html
rpi temperature sensor page works:   http://pogoplug2/temp
adding data to it: 
insert into temps (inside,outside) values (77.9, 81.0)

insert using a url:
http://pogoplug2/temp/remote/insertTemperature.php?inside=68.1&outside=85.2
esp8266 must use pogoplug2 ip address


create table action (
  ts timestamp DEFAULT CURRENT_TIMESTAMP,
  state char
);
insert into action (state) values ('c')

create table target (
  ts timestamp DEFAULT CURRENT_TIMESTAMP,
  temperature int
);
insert into target (temperature) values (78)



program page, default button doesn't load



outside temp doesn't erase correctly (line 1490)




add wires to header for outside temp plug
add 2nd temp sensor (sw done...just add the sensor)




websockerclient
if server restarts, we don't get a disconnect..just no more updates
todo: implement a ping every 30 seconds.  if no reply, then disconnect, and connect


if server isn't running, we get disconnect messages over and over again...app is basically frozen
when server starts, it connects, and all is well.
after we get the disconnect message,
we set a flag and start a timer
when the flag is set, we don't call loop anymore for the ws client
after 30 seconds, we call begin again....this works
but kinda freezes the app if the connection doesn't happen right away
try the websocket async method.





test mode
00111111 -  63 - all off, no movement, display on
10111111 - 191 - all off, movement, display on
01111111 - 127 - all off, no movement, display off
00101111 -  47 - fan on, no movement, display on
00000111 -   7 - fan on, compressor on, display on

dials on the motion sensor:
left....doesn't seem to matter....set to middle
right..must be turned all the way counter clockwise....otherwise, it false triggers...even thenn it is too sensitive
i think this is caused by running it at 3.3 instead of 5v
*/
