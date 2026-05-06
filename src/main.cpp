#include <Arduino.h>
#include "config.h"

#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <NewPing.h>
#include "MedianFilter.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include "RTClib.h"
#include "DFRobot_PH.h"
#include <EEPROM.h>

HardwareSerial SerialGSM(2);
TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

NewPing sonar(SR04_TRIGGER_PIN, SR04_ECHO_PIN, SR04_MAX_DISTANCE);
MedianFilter med(MED_WINDOW, 0);
int lastMedian = 0;
uint32_t sr04LastMs = 0;

RTC_DS3231 rtc;
Adafruit_ADS1115 ads;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// dallas temperature sensors
DeviceAddress sensor1 = { 0x28, 0x3A, 0x06, 0x38, 0x00, 0x00, 0x00, 0xF4 };
DeviceAddress sensor2 = { 0x28, 0xAD, 0x9A, 0x38, 0x00, 0x00, 0x00, 0x56 };
DeviceAddress sensor3 = { 0x28, 0x1B, 0x6D, 0x35, 0x00, 0x00, 0x00, 0x89 };

DFRobot_PH ph;

float t1 = NAN, t2 = NAN, t3 = NAN;
float tAvg = 25.0;
float phValue = NAN;
float phVoltage_mV = NAN;
float doVoltage_mV = NAN;
float doValue = 0.0;
unsigned long lastReconnectAttempt = 0;

// dissolved oxygen saturation table by temperature (0-40°C)
static const uint16_t DO_TABLE[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

enum CheckStatus : uint8_t { CHECK_PASS = 0, CHECK_WARN, CHECK_FAIL };

void connectGPRS() {
  Serial.print(F("connecting gprs... "));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(F("fail"));
    return;
  }
  Serial.println(F("ok"));
}

void checkNetwork() {
  if (!mqtt.connected()) {
    if (millis() - lastReconnectAttempt > 5000) {
      Serial.print(F("mqtt reconnecting... "));
      if (!modem.isGprsConnected()) {
        connectGPRS();
      }
      if (mqtt.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
        Serial.println(F("ok"));
        lastReconnectAttempt = 0;
      } else {
        Serial.print(F("fail, rc="));
        Serial.println(mqtt.state());
        lastReconnectAttempt = millis();
      }
    }
  }
}

bool isValidTemp(float t) {
  return (!isnan(t) && t > -100.0 && t < 125.0);
}

static void printCheck(const char* name, CheckStatus status, const char* detail) {
  Serial.print(F("["));
  Serial.print(status == CHECK_PASS ? F("pass") : status == CHECK_WARN ? F("warn") : F("fail"));
  Serial.print(F("] "));
  Serial.print(name);
  if (detail != nullptr && detail[0] != '\0') {
    Serial.print(F(" - "));
    Serial.print(detail);
  }
  Serial.println();
}

void readDallasTemps() {
  sensors.requestTemperatures();
  t1 = sensors.getTempC(sensor1);
  t2 = sensors.getTempC(sensor2);
  t3 = sensors.getTempC(sensor3);

  float sum = 0;
  int n = 0;
  if (isValidTemp(t1)) { sum += t1; n++; }
  if (isValidTemp(t2)) { sum += t2; n++; }
  if (isValidTemp(t3)) { sum += t3; n++; }
  tAvg = (n > 0) ? (sum / n) : 25.0;
}

void readPHfromADS() {
  int16_t adc0 = ads.readADC_SingleEnded(PH_ADC_CHANNEL);
  float volts = ads.computeVolts(adc0);
  phVoltage_mV = volts * 1000.0f;
  phValue = ph.readPH(phVoltage_mV, tAvg);
  // Serial.print(F("pH Value: ")); Serial.println(phValue);
}

static float computeDissolvedOxygenMgL(float voltage_mV, float temperatureC) {
  int tempIndex = (int)roundf(temperatureC);
  if (tempIndex < 0) tempIndex = 0;
  if (tempIndex > 40) tempIndex = 40;

  float saturationVoltage = DO_CAL1_V_MV + 35.0f * (temperatureC - DO_CAL1_T_C);
  if (saturationVoltage <= 0.0f) return NAN;

  float doUgL = (voltage_mV * (float)DO_TABLE[tempIndex]) / saturationVoltage;
  return doUgL / 1000.0f;
}

void readDOfromADS() {
  int16_t adc1 = ads.readADC_SingleEnded(DO_ADC_CHANNEL);
  float volts = ads.computeVolts(adc1);
  doVoltage_mV = volts * 1000.0f;

  float tempForDO = isValidTemp(tAvg) ? tAvg : 25.0f;
  doValue = computeDissolvedOxygenMgL(doVoltage_mV, tempForDO);
}

static void sortSmall(int* a, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (a[j] < a[i]) {
        int t = a[i]; a[i] = a[j]; a[j] = t;
      }
    }
  }
}

static bool readSR04BatchMedian(int& outMedian, int& validCount) {
  int vals[SR04_BATCH];
  validCount = 0;
  for (uint8_t i = 0; i < SR04_BATCH; i++) {
    int v = sonar.ping_cm();
    if (v > 0 && v <= SR04_MAX_DISTANCE) vals[validCount++] = v;
    delay(SR04_INTER_PING_MS);
  }
  if (validCount == 0) return false;
  sortSmall(vals, validCount);
  outMedian = vals[validCount / 2];
  return true;
}

void handleSR04() {
  uint32_t ms = millis();
  if (ms - sr04LastMs < SR04_INTERVAL_MS) return;
  sr04LastMs = ms;

  int batchMedian = 0;
  int validN = 0;
  if (readSR04BatchMedian(batchMedian, validN)) {
    lastMedian = med.in(batchMedian);
  }
}

void runStartupSelfTest(bool rtcOk, bool adsOk) {
  Serial.println(F("=== sensor self test ==="));

  char detail[96];

  printCheck("RTC DS3231", rtcOk ? CHECK_PASS : CHECK_FAIL,
             rtcOk ? "I2C detected" : "Not found");

  printCheck("ADS1115", adsOk ? CHECK_PASS : CHECK_FAIL,
             adsOk ? "ADC ready" : "Not found");

  uint8_t deviceCount = sensors.getDeviceCount();
  snprintf(detail, sizeof(detail), "%u device(s) found", deviceCount);
  printCheck("dallas bus", deviceCount > 0 ? CHECK_PASS : CHECK_WARN, detail);

  bool d1 = sensors.isConnected(sensor1);
  bool d2 = sensors.isConnected(sensor2);
  bool d3 = sensors.isConnected(sensor3);
  printCheck("dallas sensor #1", d1 ? CHECK_PASS : CHECK_WARN, d1 ? "ROM ok" : "ROM fail");
  printCheck("dallas sensor #2", d2 ? CHECK_PASS : CHECK_WARN, d2 ? "ROM ok" : "ROM fail");
  printCheck("dallas sensor #3", d3 ? CHECK_PASS : CHECK_WARN, d3 ? "ROM ok" : "ROM fail");

  if (adsOk) {
    int16_t phRaw = ads.readADC_SingleEnded(PH_ADC_CHANNEL);
    int16_t doRaw = ads.readADC_SingleEnded(DO_ADC_CHANNEL);
    float phmV = ads.computeVolts(phRaw) * 1000.0f;
    float domV = ads.computeVolts(doRaw) * 1000.0f;

    snprintf(detail, sizeof(detail), "raw=%d, %.1f mV", phRaw, phmV);
    CheckStatus phState = (phmV >= 0.0f && phmV <= 3300.0f) ? CHECK_PASS : CHECK_WARN;
    printCheck("pH input (ADS A0)", phState, detail);

    snprintf(detail, sizeof(detail), "raw=%d, %.1f mV", doRaw, domV);
    CheckStatus doState = (domV >= 0.0f && domV <= 3300.0f) ? CHECK_PASS : CHECK_WARN;
    printCheck("DO input (ADS A1)", doState, detail);
  } else {
    printCheck("pH input (ADS A0)", CHECK_FAIL, "ADS belum siap");
    printCheck("DO input (ADS A1)", CHECK_FAIL, "ADS belum siap");
  }

  int sr04Median = 0;
  int sr04Valid = 0;
  bool sr04Ok = readSR04BatchMedian(sr04Median, sr04Valid);
  if (sr04Ok) {
    snprintf(detail, sizeof(detail), "valid=%d, median=%d cm", sr04Valid, sr04Median);
    printCheck("SR04 ultrasonic", CHECK_PASS, detail);
  } else {
    printCheck("SR04 ultrasonic", CHECK_WARN, "no echo, check wiring/position");
  }

  Serial.println(F("=== end self test ==="));
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Init Modem
  Serial.println(F("Init SIM800L..."));
  SerialGSM.begin(9600, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(3000);
  modem.restart();
  connectGPRS();
  mqtt.setServer(mqtt_server, 1883);

  sensors.begin();
  bool rtcOk = rtc.begin();
  bool adsOk = ads.begin();
  if (!rtcOk) Serial.println(F("RTC not found"));
  if (!adsOk) Serial.println(F("ADS1115 fail"));
  EEPROM.begin(32);
  ph.begin();

  runStartupSelfTest(rtcOk, adsOk);

  Serial.println(F("--- Smart Feeder Node Ready ---"));
}

void loop() {
  checkNetwork();
  if (mqtt.connected()) mqtt.loop();

  handleSR04();

  DateTime now = rtc.now();
  static uint8_t lastTriggerSecond = 255;

  if ((now.second() == 0 || now.second() == 20 || now.second() == 40) && now.second() != lastTriggerSecond) {
    lastTriggerSecond = now.second();
    
    Serial.println(F("--- Baca Sensor ---"));
    readDallasTemps();
    readPHfromADS();
    readDOfromADS();
    ph.calibration(phVoltage_mV, tAvg);

    if (mqtt.connected()) {
      char payload[150];
      
      float safeT1 = isValidTemp(t1) ? t1 : 0.0;
      float safeT2 = isValidTemp(t2) ? t2 : 0.0;
      float safeT3 = isValidTemp(t3) ? t3 : 0.0;
      float safePH = isnan(phValue) ? 0.0 : phValue;
      float safeDO = isnan(doValue) ? 0.0 : doValue;

      snprintf(payload, sizeof(payload), 
               "field1=%.2f&field2=%.2f&field3=%.2f&field4=%.2f&field5=%.2f&field6=%d",
           safeT1, safeT2, safeT3, safeDO, safePH, lastMedian);

      Serial.print(F("Publishing: "));
      Serial.println(payload);
      mqtt.publish(publish_topic, payload);
    } else {
      Serial.println(F("MQTT disconnect, data not sent"));
    }
  }
}