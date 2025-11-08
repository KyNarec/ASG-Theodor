#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>  // built-in ESP32 Wi-Fi library
#include <HTTPClient.h>


const char* ssid = "GYAS-INTRA2";
const char* password = "ldlhbf5iilelljzjxvnr9nzhfla2zt";


#define LED 2
#define PIN1TOSENSOR 21  // SDA // 21
#define PIN2TOSENSOR 22  // SCL // 22


#define GAS_PIN 34  // ADC pin connected to AOUT
#define VCC 5.0     // Sensor supply voltage
#define RL 10.0     // Load resistor in kilo-ohms (check your circuit)
#define R0 10.0     // Sensor resistance in clean air (kOhm) – calibrate manually

// Voltage divider if using 5V output to ESP32 3.3V ADC
#define VOLTAGE_DIVIDER 1.5  // Example: 5V scaled to ~3.3V, adjust if different

#define MQ7_PIN 39   // ADC pin for MQ-7
#define RL_MQ7 10.0  // kΩ
#define R0_MQ7 10.0  // kΩ (calibrate in clean air)
#define VCC_MQ7 5.0

// === Datenstruktur, die vom Sender kommt ===
typedef struct struct_message {
  float temperature;
  //  float humidity;
  float pressure;
  float altitude;
  float coconcentration;
  float o3concentration;
} struct_message;


Adafruit_BMP280 bme;

struct_message values;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;  // wait for serial port to connect
  //Wire.begin(PIN1TOSENSOR, PIN2TOSENSOR);
  Serial.println("Serial Setup");

  if (!bme.begin(0x76)) {
    Serial.println("Could not find BMP280 sensor, check wiring!");
  } else {
    Serial.println("BMP280 Sensor Initialized");
  }

  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);

  // if (hm3301.begin()) {
  //   Serial.println("HM3301 initialized");
  // } else {
  //   Serial.println("HM3301 init failed");
  // }

  connect_to_wifi();

  //upload_to_open_sense_map(20.0F, 50.0F, 300.0F, 400.0F);


  pinMode(LED, OUTPUT);
  Serial.println("Setup completed");
}

void check_bme() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find BMP280 sensor, check wiring!");
  } else {
    Serial.println("BMP280 Sensor Initialized");
  }
}

void loop() {
  check_bme();


  Serial.println("===== New Sensor Values =====");
  read_bme();

  read_MQ131();

  read_CO_sensor();

  read_MQ7();


  upload_to_open_sense_map(values.temperature, values.pressure, values.altitude, values.coconcentration, values.o3concentration);

  Serial.println();
  Serial.println();

  delay(300000);  // pause before next cycle
}

void read_bme() {

  Serial.println();
  Serial.println("=== BME ===");
  values.temperature = bme.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  values.pressure = bme.readPressure() / 100.0F;
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);  // hPa
  Serial.println(" hPa");

  //Serial.print("Humidity = ");
  //Serial.print(bme.readHumidity());
  //Serial.println(" %");

  values.altitude = bme.readAltitude(1013);
  Serial.print("Approx Altitude = ");
  // preassure level from https://zoom.earth/maps/pressure/#view=48.70595,13.381687,11z/model=icon
  Serial.print(bme.readAltitude(1013));  // Adjust for local sea level pressure
  Serial.println(" m");
}

void read_CO_sensor() {
  // Read ADC value
  int adcValue = analogRead(GAS_PIN);                           // ESP32 ADC: 0-4095
  float voltage = adcValue * (3.3 / 4095.0) * VOLTAGE_DIVIDER;  // scaled back to actual sensor voltage

  // Compute sensor resistance Rs
  float Rs = RL * (VCC - voltage) / voltage;

  // Compute ratio Rs/R0
  float ratio = Rs / R0;

  // Estimate CO ppm using approximate curve (from datasheet)
  // Rs/R0 ≈ A * (ppm)^-B -> simplified formula for example
  // Example constants from datasheet: CO curve: log-log slope ~ -0.8
  float CO_ppm = pow(10, (log10(ratio) / -0.8));  // crude estimate

  // Print results
  Serial.println();
  Serial.println("=== CO Sensor ===");
  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print("  Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V");
  Serial.print("  Rs: ");
  Serial.print(Rs, 2);
  Serial.print(" kΩ");
  Serial.print("  Rs/R0: ");
  Serial.print(ratio, 2);
  Serial.print("  CO: ");
  Serial.print(CO_ppm, 2);
  Serial.println(" ppm");
  values.coconcentration = CO_ppm;
}

void read_MQ7() {
  int adcValue = analogRead(MQ7_PIN);
  float voltage = adcValue * (3.3 / 4095.0) * VOLTAGE_DIVIDER;

  float Rs = RL_MQ7 * (VCC_MQ7 - voltage) / voltage;
  float ratio = Rs / R0_MQ7;

  // Approx CO in ppm from datasheet curve (log-log slope ~ -0.8 for MQ-7)
  float CO_ppm = pow(10, (log10(ratio) / -0.8));

  Serial.println();
  Serial.println("=== MQ7 ===");

  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print("  Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V  Rs: ");
  Serial.print(Rs, 2);
  Serial.print(" kΩ  Rs/R0: ");
  Serial.print(ratio, 2);
  Serial.print("  CO: ");
  Serial.print(CO_ppm, 2);
  Serial.println(" ppm");
}

void read_MQ131() {
  const int adcPin = 36;
  const float RLl = 10000.0;  // load resistor on module
  const float Vc = 5.0;       // sensor supply voltage
  const float scale = 1.51;   // divider compensation (5.1k/10k)
  const float Ro = 78000.0;   // clean-air baseline resistance, adjust as measured

  // Constants for Rs/Ro → ppm
  const float A = 0.21;
  const float B = -2.0;

  int val = analogRead(adcPin);
  float Vadc = val * 3.3 / 4095.0;
  float Vsensor = Vadc * scale;

  if (Vsensor > 5.0) Vsensor = 5.0;  // clamp to max sensor voltage

  float Rs = RLl * (Vc / Vsensor - 1.0);
  float ratio = Rs / Ro;

  // approximate ozone concentration in ppm
  float ppm = A * pow(ratio, B);

  values.o3concentration = ppm;

  Serial.println();
  Serial.println("=== MQ131 ===");

  Serial.print("ADC: ");
  Serial.print(val);
  Serial.print("  Vsensor: ");
  Serial.print(Vsensor, 3);
  Serial.print(" V  Rs: ");
  Serial.print(Rs, 0);
  Serial.print(" ohms  Rs/Ro: ");
  Serial.print(ratio, 3);
  Serial.print("  O3 (ppm): ");
  Serial.println(ppm, 3);
}



void connect_to_wifi() {
  Serial.println("\nConnecting to WiFi...");

  // Start connecting
  WiFi.begin(ssid, password);

  // Wait until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // When connected, print IP address
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// === openSenseMap Zugangsdaten ===
const char* senseBoxID = "67f91210806ae7000735c019";

const char* sensorID_Temperature = "67f91210806ae7000735c01b";
const char* sensorID_Humidity = "67f91210806ae7000735c01asd";
const char* sensorID_Pressure = "67f91210806ae7000735c01a";
const char* sensorID_Altitude = "67f91210806ae7000735c01c";
const char* sensorID_COConcentration = "690e11d982e4270007d5faed";
const char* sensorID_O3Concentration = "690f1d1d332444000805fb9d";



// === Funktion zum Hochladen auf openSenseMap ===
void upload_to_open_sense_map(
  float temperature,
  //float humidity,
  float pressure,
  float altitude,
  float coconcentration,
  float o3concentration) {
  WiFi.mode(WIFI_STA);

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.print("Verbinde mit WLAN");

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 60) {  // bis 30 Sekunden warten
      delay(500);
      Serial.print(".");
      retry++;

      // Debug Infos
      wl_status_t status = WiFi.status();
      switch (status) {
        case WL_NO_SSID_AVAIL:
          Serial.print(" [SSID nicht verfügbar]");
          break;
        case WL_CONNECT_FAILED:
          Serial.print(" [Verbindung fehlgeschlagen]");
          break;
        case WL_IDLE_STATUS:
          Serial.print(" [Idle]");
          break;
        case WL_DISCONNECTED:
          Serial.print(" [Getrennt]");
          break;
        default:
          break;
      }
    }
    Serial.println();
  }

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String serverPath;

    // Temperatur
    serverPath = "https://api.opensensemap.org/boxes/" + String(senseBoxID) + "/" + String(sensorID_Temperature) + "?value=" + String(temperature, 2);
    http.begin(serverPath);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST("");
    Serial.print("Temperatur Upload: ");
    Serial.println(httpCode);
    http.end();

    // Luftfeuchtigkeit
    //serverPath = "https://api.opensensemap.org/boxes/" + String(senseBoxID) +
    //             "/" + String(sensorID_Humidity) + "?value=" + String(humidity, 2);
    //http.begin(serverPath);
    //http.addHeader("Content-Type", "application/json");
    //httpCode = http.POST("");
    //Serial.print("Luftfeuchtigkeit Upload: "); Serial.println(httpCode);
    //http.end();

    // Luftdruck
    serverPath = "https://api.opensensemap.org/boxes/" + String(senseBoxID) + "/" + String(sensorID_Pressure) + "?value=" + String(pressure, 2);
    http.begin(serverPath);
    http.addHeader("Content-Type", "application/json");
    httpCode = http.POST("");
    Serial.print("Luftdruck Upload: ");
    Serial.println(httpCode);
    http.end();

    // Altitude
    serverPath = "https://api.opensensemap.org/boxes/" + String(senseBoxID) + "/" + String(sensorID_Altitude) + "?value=" + String(altitude, 2);
    http.begin(serverPath);
    http.addHeader("Content-Type", "application/json");
    httpCode = http.POST("");
    Serial.print("Altitude Upload: ");
    Serial.println(httpCode);
    http.end();

    // COConcentration
    serverPath = "https://api.opensensemap.org/boxes/" + String(senseBoxID) + "/" + String(sensorID_COConcentration) + "?value=" + String(coconcentration, 2);
    http.begin(serverPath);
    http.addHeader("Content-Type", "application/json");
    httpCode = http.POST("");
    Serial.print("COConcentration Upload: ");
    Serial.println(httpCode);
    http.end();

    // O3Concentration
    serverPath = "https://api.opensensemap.org/boxes/" + String(senseBoxID) + "/" + String(sensorID_O3Concentration) + "?value=" + String(o3concentration, 2);
    http.begin(serverPath);
    http.addHeader("Content-Type", "application/json");
    httpCode = http.POST("");
    Serial.print("O3Concentration Upload: ");
    Serial.println(httpCode);
    Serial.print("03Concentration Uploaded: ");
    Serial.println(o3concentration);
    http.end();


  } else {
    Serial.println("⚠️ WLAN-Verbindung fehlgeschlagen!");
  }
}
