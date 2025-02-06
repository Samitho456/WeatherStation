/*
Weatherstation By Thomas Larsen
made in 2025

This weatherstation can test for airquality using these reading
1. Humidity and temperature
2. Air Pressure
3. Amount of CO2 in the air
4. Dust Partiacals in the air

Components needed:
1. Esp32
2. DHT22
3. BMP280+AHT20
4. SenseAir S8
5. PMS7003 


How to connect pins:
DHT22:
  GND to Ground
  VCC to 5V
  Data to pin 4 through 10k ohm resistor

BMP280+AHT20:
  GND to Ground
  VCC to 5V
  SDA to GPIO21 / SDA
  SCL to GPIO22 / SCL

SenseAir S8:
  G0 to Ground
  G+ to 5V
  UART_RxD to UART TX / pin 17 
  UART_TxD to UART RX / pin 16

PMS7003:
  VCC to 5V
  GND to Ground
  TX to UART RX / pin 14

ToDo:
  Add touch screen / test if enough pins
  Add battery / charging
  Add Automatic Switch between and plug 
  Make screen only turn on when plugged into outlet
  Remove all serial conncetion and run only on power and api
  Design 3d og box to contain components
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>
#include "s8_uart.h"

const char* ssid = "larsen";
const char* password = "samitho3";

// Set DHT pin and type
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// 
Adafruit_BMP280 bmp;

// Configuration for the RX pin and UART1
#define PMS7003_RX 14
HardwareSerial pms7003(1);
uint8_t buffer[32]; // Buffer to store received data
unsigned long runTime; // PMS7003 sensor runtime

// Configuration for UART and SenseAir S8
#define S8_TX_PIN 17
#define S8_RX_PIN 16
HardwareSerial S8_serial(2);  // Using UART2
S8_UART *sensor_S8;
S8_sensor sensor;

// Variables for calculating average CO2 level
int total_co2 = 0;
unsigned long n_measurements = 0;

// Thresholds for values
#define MAX_TEMP 30.0    // Max temperature in Celsius
#define MAX_CO2 1000     // Max CO2 level
#define MAX_PRESSURE 1015 // Max air pressure in hPa

WebServer server(80);

// Define sleep time in microseconds (30 minutes)
const uint64_t sleepTime = 1 * 60 * 1000000;

// Define server uptime in milliseconds (5 minutes)
const unsigned long serverUptime = 5 * 60 * 1000;

// Function that runs when the /GetInfo api is requested
void handleGetInfo() {
  StaticJsonDocument<500> doc;

  // Read temperature and humidity from DHT22
  float temperatureDHT = dht.readTemperature();
  float humidity = dht.readHumidity();
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(temperatureDHT, humidity, false);

  // Read pressure from BMP280
  float pressure = bmp.readPressure() / 100.0F; // Convert to hPa

  // Read CO2 from S8 sensor
  int co2 = sensor_S8->get_co2();
  if (co2 > 0) {
    total_co2 += co2;
    n_measurements++;
  }

  // Calculate average CO2
  double avg_co2 = (n_measurements > 0) ? total_co2 / (double)n_measurements : 0;

  // Check if DHT22 reading failed
  if (isnan(temperatureDHT) || isnan(humidity) || isnan(hic)) {
    doc["temperature_dht22"] = "Error reading temperature";
    doc["humidity"] = "Error reading humidity";
    doc["HeatIndex"] = "Error calculation Heat Index";
  } else {
    doc["temperature_dht22"] = temperatureDHT;
    doc["humidity"] = humidity;
    doc["HeatIndex"] = hic;
  }

  // Check if BMP280 reading failed
  if (isnan(pressure)) {
    doc["pressure"] = "Error reading pressure";
  } else {
    doc["pressure"] = pressure;
  }

  // Check if S8 CO2 reading failed
  if (co2 <= 0) {
    doc["co2"] = "Error reading CO2 level";
  } else {
    doc["co2"] = co2;
  }

  // Include average CO2 in the response
  doc["average_co2"] = avg_co2;

  // check if the sensor is warmed up 
  if (millis() - runTime > 60000) {
    if (pms7003.available() >= 32) {  // Check if at least 32 bytes are available
      pms7003.readBytes(buffer, 32);
      if (buffer[0] == 0x42 && buffer[1] == 0x4D) { // Verify frame header
        uint16_t pm1_0 = (buffer[10] << 8) | buffer[11];
        uint16_t pm2_5 = (buffer[12] << 8) | buffer[13];
        uint16_t pm10  = (buffer[14] << 8) | buffer[15];  

        doc["pm1.0"] = pm1_0;
        doc["pm2.5"] = pm2_5;
        doc["pm10"] = pm10;
      }
    }
  } 
  else {
    doc["PMS7003"] = "Sensor not warmed up";
  }


  doc["device"] = "ESP32 WeatherStation";
  doc["uptime"] = millis() / 1000;
  doc["status"] = "active";

  String jsonResponse;
  serializeJson(doc, jsonResponse);

  server.send(200, "application/json", jsonResponse);
}

// Starts the server
void startServer() {
  server.on("/GetInfo", handleGetInfo);
  server.begin();
  Serial.println("Server started");

  String ipAddress = WiFi.localIP().toString();
  Serial.print("Server IP address: ");
  Serial.println(ipAddress);

  unsigned long startTime = millis();
  while (millis() - startTime < serverUptime) {
    server.handleClient();
    delay(10);  // Prevents watchdog timer reset
  }
  server.stop();
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize DHT22 sensor
  dht.begin();

  // Initialize BMP280 sensor and if the sensor fails to begin the program crashes
  if (!bmp.begin()) {
    while (1);
  }

  // Initialize SenseAir s8
  S8_serial.begin(S8_BAUDRATE, SERIAL_8N1, S8_RX_PIN, S8_TX_PIN);
  sensor_S8 = new S8_UART(S8_serial);

  // Initialize PMS7003
  pms7003.begin(9600, SERIAL_8N1, PMS7003_RX); // Initialize UART1

  // Initialize WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start timer for PMS7003
  runTime = millis();

  // Start Server
  startServer();

  // Enter Deep Sleep mode for the amount the sleeptime is set to
  esp_sleep_enable_timer_wakeup(sleepTime);
  esp_deep_sleep_start();
}

void loop() {}
