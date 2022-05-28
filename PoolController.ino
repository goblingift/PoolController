#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <PinButton.h> // https://github.com/poelstra/arduino-multi-button
#include <AsyncTCP.h>           // https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
#include "image_processor.h"
#include "webserver_processor.h"
   
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// GPIO connections
const int oneWireBus = 4;
const int ledGreenPin = 12;
const int relayEngineValve = 26;
const int relayPump = 27;
const int setupButtonPin = 32;

PinButton setupButton(setupButtonPin);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// variable to hold device addresses
DeviceAddress Thermometer;

// Manages wifi and connect to local wifi network
WiFiManager wifiManager;
WiFiManagerParameter config_targetTemperature;
WiFiManagerParameter config_definedPumpRuntime;
WiFiManagerParameter config_latestHourToStartPump;
WiFiManagerParameter config_pumpShutdownTime;
WiFiManagerParameter config_temperatureDifference;
WiFiManagerParameter config_engineValveSwitchingTimespanSeconds;


// Prometheus endpoint configuration
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";
AsyncWebServer server(80);

const char state_html[] PROGMEM = R"rawliteral(
%PLACEHOLDER%
)rawliteral";

bool portalRunning = false;

// Time Server config
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, 7200);

// Read and save preferences to EEPROM
Preferences preferences;

// integration system DS18B20s sensor addresses:
uint8_t sensorPool[8] = { 0x28, 0x51, 0x32, 0x2A, 0x0B, 0x00, 0x00, 0xAA };
uint8_t sensorSolarHeater[8] = { 0x28, 0xF9, 0xD8, 0xFD, 0x39, 0x19, 0x01, 0x26 };
// production system DS18B20s sensor addresses:
//uint8_t sensorPool[8] = { 0x28, 0xB3, 0x6D, 0x0d, 0x3A, 0x19, 0x01, 0x94 };
//uint8_t sensorSolarHeater[8] = { 0x28, 0x75, 0xEA, 0xD3, 0x39, 0x19, 0x01, 0xD3 };

// runtime variables
unsigned long lastScreenUpdate;
unsigned long lastMeasurement;
unsigned long lastChangeEngineValve;
int deviceCount = 0;
bool enabledHeater = false;
bool enabledPump = false;
float temperatureHeater;
float temperaturePool;
int lastResetDay = 0;
// daily stats
float morningTemperaturePool;
float dailyTempChange;
// how long was the pump ON today - in milliseconds
long dailyPumpRuntime = 0;
// how long was the heater ON today - in milliseconds
long dailyHeaterRuntime = 0;
// if the latestHourToStartPump has passed, pump should be always on
bool pumpAlwaysOn;
bool dailyPumpRuntimeReached;
bool nightTimeReached;
bool timeDiffEngineValvePassed;
// Contains todays weather. 0 if no data today. 1 for sunny, 2 for sunny/cloudy and 3 for cloudy.
byte todayWeather = 0;
unsigned long lastTimeSetupButtonPressed;
bool manualMode = false;


//stores the current image index- for animation purpose
int imageIndex = 1;

// configuration variables
float targetTemperature = 35.0;
int dailyResetMinute = 00;
int dailyResetHour = 6;
int measurementIntervalMillis = 15000;
int updateScreenIntervalMillis = 200;
// temperature difference of pool & heater which is required to switch engine valve
float definedTemperatureDifference = 5.0;
// defines how long system will wait until switching valve again
int engineValveSwitchingTimespanSeconds = 10;
float definedPumpRuntime = 8.0;
// at which hour shall the pump get started
int latestHourToStartPump = 18;
// until which time the pump is allowed to be powered on
int pumpShutdownTime = 21;
// defines how long the manual started configuration should be visible
int manualConfigTimeout = 60000;

// EEPROM keys
const char* EEPROM_targetTemp = "targetTemp";
const char* EEPROM_pumpRuntime = "pumpRuntime";
const char* EEPROM_pumpStartAt = "pumpStartAt";
const char* EEPROM_tempDiff = "tempDiff";
const char* EEPROM_valveSwitching = "valveSwitching";
const char* EEPROM_pumpShutdown = "pumpShutdown";

// +++++++++++++++++ Mighty setup method ++++++++++++++++
void setup() {
  // Begin Serial on 115200
  Serial.begin(115200);

  readConfigFromEEPROM();
  configureWifiApSetupMenu();

  Serial.println("Start initializing Wifi now");
  wifiManager.autoConnect("ESP32-AP");
  timeClient.begin();   

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", webserver_processor_h::index_html, processorDashboard);
  });

  // Route for prometheus
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain;version=0.0.4;charset=utf-8", state_html, processorPrometheus);
  });

  // Route for action triggered method
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();

      // Handler for manual-mode input
      if (inputMessage1.toInt() == 99) {
        Serial.println("Will set manual-mode to:" + inputMessage2);
        if (inputMessage2.toInt() == 0) {
          manualMode = false;
        } else {
          manualMode = true;
        }
      }
    }
    request->send(200, "text/plain", "OK");
  });

  // Start server
  server.begin();
  
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  
  pinMode(ledGreenPin, OUTPUT);
  pinMode(relayPump, OUTPUT);
  pinMode(relayEngineValve, OUTPUT);
  pinMode(setupButtonPin, INPUT);
  digitalWrite(relayPump, LOW);
  digitalWrite(relayEngineValve, LOW);

  // Start the DS18B20 sensor
  sensors.begin();
  sensors.setResolution(sensorPool, 10);
  sensors.setResolution(sensorSolarHeater, 10);
  Serial.print("Started DS18B20 temperature sensor");

  // locate devices on the bus
  Serial.println("Locating devices...");
  Serial.print("Found ");
  deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");
  
  Serial.println("Printing addresses...");
  for (int i = 0;  i < deviceCount;  i++)
  {
    Serial.print("Sensor ");
    Serial.print(i+1);
    Serial.print(" : ");
    sensors.getAddress(Thermometer, i);
    printAddress(Thermometer);
  }
  
  digitalWrite(ledGreenPin, LOW);

  u8g2.drawStr(0,15,"Starting Goblingift Pool-Controller!");
  u8g2.sendBuffer();
  delay(2000);
    
  lastMeasurement = millis();
  lastScreenUpdate = millis();
}

String processorPrometheus(const String& var){
  if(var == "PLACEHOLDER"){
    String pinStateDecimal = "0";
    if(digitalRead(2)) {
      pinStateDecimal = "1";
    }

    String scrapingText = "# HELP io_temperature_heater temperature of the heater \n";
    scrapingText += "# TYPE io_temperature_heater gauge\n";
    scrapingText += "io_temperature_heater " + convertTemperature(temperatureHeater) + " \n";
    
    scrapingText += "# HELP io_temperature_pool temperature of the pool \n";
    scrapingText += "# TYPE io_temperature_pool gauge\n";
    scrapingText += "io_temperature_pool " + convertTemperature(temperaturePool) + " \n";

    scrapingText += "# HELP io_pump_active boolean if the pump is currently activated \n";
    scrapingText += "# TYPE io_pump_active gauge\n";
    String strPumpActive = enabledPump ? "1" : "0";
    scrapingText += "io_pump_active " + strPumpActive + " \n";

    scrapingText += "# HELP io_heater_active boolean if the heater is currently activated \n";
    scrapingText += "# TYPE io_heater_active gauge\n";
    String strHeaterActive = enabledHeater ? "1" : "0";
    scrapingText += "io_heater_active " + strHeaterActive + " \n";
    
    return scrapingText;
  }
  return String();
}


String processorDashboard(const String& var){
  String isActive = "";
  if (manualMode) {
    isActive = "checked";
  }
  
  if(var == "BUTTONPLACEHOLDER"){
    String buttons = "";
    buttons += "<h4>Manual Mode active:</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"99\" " + isActive + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  return String();
}

// Defines the html view of the configuration setup menu and which values will get stored into EEPROM afterwards
void configureWifiApSetupMenu() {

  const char* input_config_targetTemperature = "<br/><label for='config_targetTemperature'>Target Temperature (celsius)</label><input type='text' name='config_targetTemperature'>";
  new (&config_targetTemperature) WiFiManagerParameter(input_config_targetTemperature);
  
  const char* input_config_definedPumpRuntime = "<br/><label for='config_definedPumpRuntime'>Daily pump-runtime (hours)</label><input type='text' name='config_definedPumpRuntime'>";
  new (&config_definedPumpRuntime) WiFiManagerParameter(input_config_definedPumpRuntime);
  
  const char* input_config_latestHourToStartPump = "<br/><label for='config_latestHourToStartPump'>Latest time to start pump (24h)</label><input type='text' name='config_latestHourToStartPump'>";
  new (&config_latestHourToStartPump) WiFiManagerParameter(input_config_latestHourToStartPump);

  const char* input_config_pumpShutdownTime = "<br/><label for='config_pumpShutdownTime'>At which time the pump will always get shutdown (24h)</label><input type='text' name='config_pumpShutdownTime'>";
  new (&config_pumpShutdownTime) WiFiManagerParameter(input_config_pumpShutdownTime);

  const char* input_config_temperatureDifference = "<br/><label for='config_temperatureDifference'>Temperature difference required to switch</label><input type='text' name='config_temperatureDifference'>";
  new (&config_temperatureDifference) WiFiManagerParameter(input_config_temperatureDifference);
  
  const char* input_config_engineValveSwitchingTimespanSeconds = "<br/><label for='config_engineValveSwitchingTimespanSeconds'>Required seconds between each engine-valve switch</label><input type='text' name='config_engineValveSwitchingTimespanSeconds'>";
  new (&config_engineValveSwitchingTimespanSeconds) WiFiManagerParameter(input_config_engineValveSwitchingTimespanSeconds);
    
  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wifiManager.setMenu(menu);
  wifiManager.addParameter(&config_targetTemperature);
  wifiManager.addParameter(&config_definedPumpRuntime);
  wifiManager.addParameter(&config_latestHourToStartPump);
  wifiManager.addParameter(&config_pumpShutdownTime);
  wifiManager.addParameter(&config_temperatureDifference);
  wifiManager.addParameter(&config_engineValveSwitchingTimespanSeconds);
  wifiManager.setSaveParamsCallback(saveParamCallback);
}

String getParam(String name){
  //read parameter from server, for customhmtl input
  String value;
  if(wifiManager.server->hasArg(name)) {
    value = wifiManager.server->arg(name);
  }
  return value;
}

// Reading preferences from EEPROM (Which were set previous by Wifi-Config Page, or use default
void readConfigFromEEPROM() {
  
  preferences.begin("pool-controller", true);
    
  targetTemperature = preferences.getFloat(EEPROM_targetTemp, targetTemperature);
  Serial.println("Successful read and set targetTemp from EEPROM:" + String(targetTemperature));
  
  definedPumpRuntime = preferences.getFloat(EEPROM_pumpRuntime, definedPumpRuntime);
  Serial.println("Successful read and set pumpRuntime from EEPROM:" + String(definedPumpRuntime));

  latestHourToStartPump = preferences.getInt(EEPROM_pumpStartAt, latestHourToStartPump);
  Serial.println("Successful read and set latestHourToStartPump from EEPROM:" + String(latestHourToStartPump));

  pumpShutdownTime = preferences.getInt(EEPROM_pumpShutdown, pumpShutdownTime);
  Serial.println("Successful read and set pumpShutdownTime from EEPROM:" + String(pumpShutdownTime));

  definedTemperatureDifference = preferences.getFloat(EEPROM_tempDiff, definedTemperatureDifference);
  Serial.println("Successful read and set definedTemperatureDifference from EEPROM:" + String(definedTemperatureDifference));

  engineValveSwitchingTimespanSeconds = preferences.getInt(EEPROM_valveSwitching, engineValveSwitchingTimespanSeconds);
  Serial.println("Successful read and set engineValveSwitchingTimespanSeconds from EEPROM:" + String(engineValveSwitchingTimespanSeconds));
  
  preferences.end();
}

void saveParamCallback(){
  Serial.println("[CALLBACK] saveParamCallback fired");

  float new_config_targetTemperature = getParam("config_targetTemperature").toFloat();
  float new_config_definedPumpRuntime =  getParam("config_definedPumpRuntime").toFloat();
  int new_config_latestHourToStartPump = getParam("config_latestHourToStartPump").toInt();
  int new_config_pumpShutdownTime = getParam("config_pumpShutdownTime").toInt();
  float new_config_temperatureDifference = getParam("config_temperatureDifference").toFloat();
  int new_config_engineValveSwitchingTimespanSeconds = getParam("config_engineValveSwitchingTimespanSeconds").toInt();
  
  Serial.println("Save param to EEPROM: config_targetTemperature = " + String(new_config_targetTemperature));
  Serial.println("Save param to EEPROM: config_definedPumpRuntime = " + String(new_config_definedPumpRuntime));
  Serial.println("Save param to EEPROM: config_latestHourToStartPump = " + String(new_config_latestHourToStartPump));
  Serial.println("Save param to EEPROM: config_pumpShutdownTime = " + String(new_config_pumpShutdownTime));
  Serial.println("Save param to EEPROM: config_temperatureDifference = " + String(new_config_temperatureDifference));
  Serial.println("Save param to EEPROM: config_engineValveSwitchingTimespanSeconds = " + String(new_config_engineValveSwitchingTimespanSeconds));
  
  preferences.begin("pool-controller", false); 
  preferences.putFloat(EEPROM_targetTemp, new_config_targetTemperature);
  preferences.putFloat(EEPROM_pumpRuntime, new_config_definedPumpRuntime);
  preferences.putInt(EEPROM_pumpStartAt, new_config_latestHourToStartPump);
  preferences.putInt(EEPROM_pumpShutdown, new_config_pumpShutdownTime);
  preferences.putFloat(EEPROM_tempDiff, new_config_temperatureDifference);
  preferences.putInt(EEPROM_valveSwitching, new_config_engineValveSwitchingTimespanSeconds);
  preferences.end();

  // Update runtime variables with new values
  targetTemperature = new_config_targetTemperature;
  definedPumpRuntime = new_config_definedPumpRuntime;
  latestHourToStartPump = new_config_latestHourToStartPump;
  pumpShutdownTime = new_config_pumpShutdownTime;
  definedTemperatureDifference = new_config_temperatureDifference;
  engineValveSwitchingTimespanSeconds = new_config_engineValveSwitchingTimespanSeconds;
}

void loop() {

  setupButton.update();

  if (setupButton.isSingleClick()) {
    singleTap();
  } else if (setupButton.isDoubleClick()) {
    doubleTap();
  } else if (setupButton.isLongClick()) {
    hold();
  }
  
  if (millis() - lastMeasurement >= measurementIntervalMillis) {
    timeClient.update();
  
    handleTemperatureMeasurement();
    writeStats();
    
    if (manualMode == false) {
      handlePumpAndValve();
    }
        
    tryToResetDailyStats();
  }

  if (millis() - lastScreenUpdate >= updateScreenIntervalMillis) {
    updateDisplay();
    lastScreenUpdate = millis();
  }
}

void singleTap() {
  Serial.println("SINGLE TAPPED! No action configured!");
}

void doubleTap() {
  Serial.println("Setup button pressed 2x - starting Wifi AP to setup things for x milliseconds:" + String(manualConfigTimeout));

  wifiManager.setConfigPortalTimeout(manualConfigTimeout / 1000);
  displayWifiApInformations();
  wifiManager.startConfigPortal("ESP32-AP");
}

void hold() {
  Serial.println("Button hold! No action configured!");
}

// Triggers temperature measurement
void handleTemperatureMeasurement() {
  digitalWrite(ledGreenPin, HIGH);
  delay(50);
  sensors.requestTemperatures(); 
  temperaturePool = readTemperature(sensorPool);
  temperatureHeater = readTemperature(sensorSolarHeater);
  digitalWrite(ledGreenPin, LOW);

  if (morningTemperaturePool == 0.0) morningTemperaturePool = temperaturePool;
}

// Edits the daily stats of pump and heater runtime
void writeStats() {
  unsigned long millisSinceLastMeasurement = millis() - lastMeasurement;

  if (enabledPump) {
    dailyPumpRuntime += millisSinceLastMeasurement;

    if (enabledHeater) {
      dailyHeaterRuntime += millisSinceLastMeasurement;
    }
  }

  dailyTempChange = temperaturePool - morningTemperaturePool;

  // When the pump run for 2 minutes (and less than 3 minutes), save the daily morning temperature
  if (dailyPumpRuntime >= 120000 && dailyPumpRuntime < 180000) {
    Serial.println("Daily morning temperature was set to current temperature of the pool.");
    morningTemperaturePool = temperaturePool;
  }
  
  lastMeasurement = millis();
}

// Checks if the pump or the valve needs to be switched
void handlePumpAndValve() {

  long definedPumpRuntimeMillis = ((long)(definedPumpRuntime * 3600000));
  dailyPumpRuntimeReached = dailyPumpRuntime > definedPumpRuntimeMillis;
  nightTimeReached = timeClient.getHours() >= pumpShutdownTime;
  
  pumpAlwaysOn = timeClient.getHours() >= latestHourToStartPump && timeClient.getHours()< pumpShutdownTime && !dailyPumpRuntimeReached;
  
  bool targetIsHeating = temperaturePool < targetTemperature;
  unsigned long timeDiffLastEngineValveSwitch = millis() - lastChangeEngineValve;
  timeDiffEngineValvePassed = timeDiffLastEngineValveSwitch >= engineValveSwitchingTimespanSeconds * 1000;
  
  if (targetIsHeating) {
    // Pool needs to be heated up
    if ((temperatureHeater > temperaturePool) && ((temperatureHeater - temperaturePool) >= definedTemperatureDifference)) {
      heaterOn();
      startPump();
    } else {
      heaterOff();
      stopPump();
    }   
  } else {
    // Pool needs to be cooled down
    if ((temperatureHeater < temperaturePool) && ((temperaturePool - temperatureHeater) >= definedTemperatureDifference)) {
      heaterOn();
      startPump();
    } else {
      heaterOff();
      stopPump();
    }
  }

  if (pumpAlwaysOn) {
    Serial.println("Starting pump - because we already passed the latest time for starting pump.");
    startPump();
  }

  if (dailyPumpRuntimeReached || nightTimeReached) {
    Serial.println("Night time or daily pump runtime reached. Shutdown pump now");
    stopPump();
  }
}

void startPump() {
  if ((manualMode == true) || (!enabledPump && !dailyPumpRuntimeReached && !nightTimeReached && timeDiffEngineValvePassed)) {
    Serial.println("Starting pump now");
    digitalWrite(relayPump, HIGH);
    enabledPump = true;
    lastChangeEngineValve = millis();
  }  
}
void stopPump() {
  if (manualMode == false && enabledPump && !pumpAlwaysOn && timeDiffEngineValvePassed) {
    Serial.println("Stop pump now");
    digitalWrite(relayPump, LOW);
    enabledPump = false;
    lastChangeEngineValve = millis();
  }
}
void heaterOff() {
  if (enabledHeater && timeDiffEngineValvePassed) {
    Serial.println("Open engine-valve now");
    digitalWrite(relayEngineValve, LOW);
    enabledHeater = false;
    lastChangeEngineValve = millis();
  }
}
void heaterOn() {
  if ((manualMode == true) || (!enabledHeater && timeDiffEngineValvePassed)) {
    Serial.println("Close engine-valve now");
    digitalWrite(relayEngineValve, HIGH);
    enabledHeater = true;
    lastChangeEngineValve = millis();
  }
}

void displayWifiApInformations() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_HelvetiPixelOutline_te);
  u8g2.setCursor(5, 0);
  u8g2.print("-SETUP MODE-");
  u8g2.setCursor(5, 25);
  u8g2.print("Connect to Wifi:");
  u8g2.setCursor(5, 50);
  u8g2.print("ESP32-AP");
  
  u8g2.sendBuffer(); 
}

void updateDisplay() {

  u8g2.clearBuffer();
  u8g2.setBitmapMode(false /* solid */);
  u8g2.setDrawColor(1); 
   
  u8g2.setCursor(0, 32);
  if (enabledHeater) {
    if (enabledPump) {
      switch(imageIndex) {
        case 1:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_heater_pump_on_1);
          imageIndex++;
          break;
        case 2:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_heater_pump_on_2);
          imageIndex++;
          break;
        case 3:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_heater_pump_on_3);
          imageIndex++;
          break;
        case 4:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_heater_pump_on_4);
          imageIndex = 1;
          break;
      }
    } else {
      u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_heater_pump_off);
    }
  } else {
    if (enabledPump) {
      switch(imageIndex) {
        case 1:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_non_heater_pump_1);
          imageIndex++;
          break;
        case 2:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_non_heater_pump_2);
          imageIndex++;
          break;
        case 3:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_non_heater_pump_3);
          imageIndex++;
          break;
        case 4:
          u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_non_heater_pump_4);
          imageIndex = 1;
          break;
      }
    } else {
      u8g2.drawXBMP(0, 0, 128, 64, image_processor_h::background_non_heater_pump_off);
    }
  }

  u8g2.setFont(u8g2_font_HelvetiPixelOutline_te);
  u8g2.setCursor(1, 2);
  u8g2.print(convertTemperature(temperatureHeater));
  u8g2.setCursor(1, 44);
  u8g2.print(convertTemperature(temperaturePool));


  u8g2.setFont(u8g2_font_t0_12_mr);
  u8g2.setCursor(101, 0);
  u8g2.print(String(targetTemperature, 1));
  
  u8g2.setCursor(96, 18);
  u8g2.print(String(dailyTempChange, 1));
  
  printDailyHeaterRuntime();
  printDailyPumpRuntime();
  calculateWeatherToday();

  if (manualMode == true) {
    u8g2.setCursor(40, 0);
    u8g2.print("MANUAL MODE");
  }
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  switch(todayWeather) {
    case 1:
      u8g2.drawGlyph(65, 2, 0x2600);  /*  hex 2600 sunny */
      break;
    case 2:
      u8g2.drawGlyph(65, 2, 0x2601);  /*  hex 2601 cloudy */
      break;
    case 3:
      u8g2.drawGlyph(65, 2, 0x2602);  /*  hex 2602 rainy */
      break;
  }
   
  u8g2.sendBuffer();  
}

void printDailyHeaterRuntime() {
  u8g2.setCursor(96, 32);

  if (dailyHeaterRuntime <= 0) {
    u8g2.print("OFF");
  } else if (dailyHeaterRuntime < 60000) {
    int dailyHeaterRuntimeSeconds = dailyHeaterRuntime / 1000;
    u8g2.print(String(dailyHeaterRuntimeSeconds) + String("s"));
  } else if (dailyHeaterRuntime < 3600000) {
    int dailyHeaterRuntimeMinutes = (dailyHeaterRuntime / 1000) / 60;
    u8g2.print(String(dailyHeaterRuntimeMinutes) + String("m"));
  } else {
    float dailyHeaterRuntimeHours = ((dailyHeaterRuntime / 1000) / 60) / 60;
    u8g2.print(String(dailyHeaterRuntimeHours, 1) + String("h"));
  }
}

void printDailyPumpRuntime() {
  u8g2.setCursor(96, 50);

  if (dailyPumpRuntime <= 0) {
    u8g2.print("OFF");
  } else if (dailyPumpRuntime < 60000) {
    int dailyPumpRuntimeSeconds = dailyPumpRuntime / 1000;
    u8g2.print(String(dailyPumpRuntimeSeconds) + String("s"));
  } else if (dailyPumpRuntime < 3600000) {
    int dailyPumpRuntimeMinutes = (dailyPumpRuntime / 1000) / 60;
    u8g2.print(String(dailyPumpRuntimeMinutes) + String("m"));
  } else {
    float dailyPumpRuntimeHours = ((dailyPumpRuntime / 1000) / 60) / 60;
    u8g2.print(String(dailyPumpRuntimeHours, 1) + String("h"));
  }
}

// Calculates if todays weather is sunny or cloudy (If heater was used or not)
void calculateWeatherToday() {
  if (dailyPumpRuntime > 0) {
    if (dailyHeaterRuntime <= 0) {
      // heater wasnt enabled today, set to cloudy
      todayWeather = 3;
    } else {
      // heater was enabled today, calculate how many percentage
      float percentageHeated = (float)dailyHeaterRuntime / (float)dailyPumpRuntime;

      if (percentageHeated >= 0.75) {
        // sunny today
        todayWeather = 1;
      } else if (percentageHeated >= 0.25) {
        // sunny/cloudy today
        todayWeather = 2;
      } else {
        // cloudy today
        todayWeather = 3;
      }
    }
  }
}

void printAddress(DeviceAddress deviceAddress) { 
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}

float readTemperature(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  return tempC;
}

String convertTemperature(float tempC) {
  char outstr[15];
  dtostrf(tempC,4, 1, outstr);
  return String(outstr);
}

// This method will reset daily stats (like pump-runtime, temp-difference) at a specific given time
void tryToResetDailyStats() {

  int actDay = timeClient.getDay();
  int actHour = timeClient.getHours();
  int actMinute = timeClient.getMinutes();

  if (lastResetDay != actDay) {
    if ((actHour == dailyResetHour) && (actMinute == dailyResetMinute)) {
      Serial.println("TIMER TRIGGERED - RESET DAILY STATS");
      dailyTempChange = 0.0;
      dailyPumpRuntime = 0;
      dailyHeaterRuntime = 0;
      morningTemperaturePool = temperaturePool;
      dailyPumpRuntimeReached = false;
      nightTimeReached = false;
      pumpAlwaysOn = false;
      timeDiffEngineValvePassed = false;
      todayWeather = 0;
      lastResetDay = actDay;
    }
  }
    
}

void types(String a) { Serial.println("it's a String"); }
void types(int a) { Serial.println("it's an int"); }
void types(char *a) { Serial.println("it's a char*"); }
void types(float a) { Serial.println("it's a float"); }
void types(bool a) { Serial.println("it's a bool"); }
