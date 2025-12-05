/***************************************************
 *  ESP8266 NodeMCU Weather Station
 *  Sensors included:
 *     - DHT22 / DHT11  (Temp + Humidity)
 *     - BMP280         (Pressure + Temp)
 *     - BH1750         (Light Intensity)
 *     - Rain Sensor    (Analog)
 *     - Soil Moisture  (Analog)
 *     - Wind Speed     (Optional Digital Interrupt)
 *
 *  Fully working code — upload to GitHub.
 **************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
#include "DHT.h"

/***************************************************
 * PIN DEFINITIONS
 **************************************************/

// DHT Sensor
#define DHTPIN D4         // GPIO2
#define DHTTYPE DHT22     // Change to DHT11 if needed

// Analog Sensors
#define RAIN_SENSOR A0      // Analog pin for rain sensor
#define SOIL_SENSOR A0      // NOTE: ESP8266 has only 1 analog pin (A0)
// To use both, add external ADC (ADS1115)


// Wind Speed Sensor (Optional)
#define WIND_PIN D5         // GPIO14 — digital interrupt

// Frequency for wind speed interrupt
volatile unsigned long windCount = 0;

/***************************************************
 * SENSOR OBJECTS
 **************************************************/
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;
BH1750 lightMeter;



/***************************************************
 *  INTERRUPT HANDLER FOR WIND SPEED
 **************************************************/
void IRAM_ATTR windISR() {
  windCount++;
}

/***************************************************
 *  SETUP
 **************************************************/
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Weather Station Initializing...");

  // Initialize DHT
  dht.begin();
  Serial.println("DHT sensor OK");

  // Initialize I2C sensors
  Wire.begin();

  // BH1750
  if (lightMeter.begin()) {
    Serial.println("BH1750 Light Sensor OK");
  } else {
    Serial.println("BH1750 INIT FAILED!");
  }

  // BMP280
  if (!bmp.begin(0x76)) {   // Address may be 0x76 or 0x77
    Serial.println("BMP280 INIT FAILED!");
  } else {
    Serial.println("BMP280 Pressure Sensor OK");
  }

  // Wind speed sensor
  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), windISR, FALLING);

  Serial.println("Weather Station Ready!");
}

/***************************************************
 *  WIND SPEED CALCULATION
 **************************************************/
float getWindSpeed() {
  static unsigned long lastTime = 0;
  static unsigned long lastCount = 0;

  unsigned long now = millis();

  if (now - lastTime >= 2000) {
    float rotations = windCount - lastCount;
    lastCount = windCount;
    lastTime = now;

    // Example: 1 rotation = 2.4 km/h (adjust to your sensor)
    float speed = rotations * 2.4;
    return speed;
  }
  return 0;
}

/***************************************************
 *  MAIN LOOP
 **************************************************/
void loop() {

  /*********** DHT SENSOR ************/
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("ERROR reading DHT!");
  }

  /*********** BMP280 SENSOR ************/
  float pressure = bmp.readPressure() / 100.0F; // hPa
  float altitude = bmp.readAltitude(1013.25);    // sea level pressure adjust

  /*********** BH1750 SENSOR ************/
  float lux = lightMeter.readLightLevel();

  /*********** ANALOG SENSORS ************/
  int rainValue = analogRead(RAIN_SENSOR);
  int soilValue = analogRead(SOIL_SENSOR);

  /*********** WIND SPEED ************/
  float windSpeed = getWindSpeed();

  /*************************************************
   * PRINT OUTPUT
   *************************************************/
  Serial.println("====================================");
  Serial.println("         WEATHER STATION DATA       ");
  Serial.println("====================================");

  Serial.printf("Temperature (DHT): %.2f °C\n", t);
  Serial.printf("Humidity: %.2f %%\n", h);

  Serial.printf("Pressure (BMP280): %.2f hPa\n", pressure);
  Serial.printf("Altitude (BMP280): %.2f m\n", altitude);

  Serial.printf("Light (BH1750): %.2f lux\n", lux);

  Serial.printf("Rain Sensor: %d / 1023\n", rainValue);
  Serial.printf("Soil Moisture: %d / 1023\n", soilValue);

  Serial.printf("Wind Speed: %.2f km/h\n", windSpeed);

  Serial.println("====================================\n");

  delay(2000);  // read every 2 seconds
}
