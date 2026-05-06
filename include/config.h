#ifndef CONFIG_H
#define CONFIG_H

// gprs & mqtt credentials (telkomsel = "internet", ...)
const char apn[]      = "internet"; 
const char gprsUser[] = "";
const char gprsPass[] = "";

const char* mqtt_server    = "mqtt3.thingspeak.com";
const char* mqtt_client_id = "PCMwATchLjswOw8WFislEjs";
const char* mqtt_username  = "PCMwATchLjswOw8WFislEjs";
const char* mqtt_password  = "aQHwPZk+jmmxX2XbkZB+U1bD";
const char* publish_topic  = "channels/3355601/publish";

// pins: sim800l (uart2), sensors
#define MODEM_RX_PIN 16
#define MODEM_TX_PIN 17
#define SR04_TRIGGER_PIN 5
#define SR04_ECHO_PIN 18
#define SR04_MAX_DISTANCE 200
#define ONE_WIRE_BUS 27

// ads1115: ph (a0), do (a1)
#define PH_ADC_CHANNEL 0
#define DO_ADC_CHANNEL 1

// do calibration (temperature compensated)
const float DO_CAL1_V_MV = 1600.0f;
const float DO_CAL1_T_C = 25.0f;

// sensor timings & filtering
const uint8_t MED_WINDOW = 15;
const uint8_t SR04_BATCH = 15;
const uint16_t SR04_INTER_PING_MS = 50;
const uint32_t PUBLISH_INTERVAL_MS = 20000;

#endif