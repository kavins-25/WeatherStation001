# ESP8266 Weather Station

This project is a complete weather-monitoring system using **NodeMCU ESP8266**.

## Included Sensors
- DHT22 / DHT11 – Temperature & Humidity  
- BMP280 – Pressure & Temperature  
- BH1750 – Ambient Light  
- Rain Sensor – Analog  
- Soil Moisture Sensor – Analog  
- Wind Speed Sensor (Optional)

## Pins Used
D4 – DHT22  
D5 – Wind Speed Sensor (interrupt)  
A0 – Rain Sensor / Soil Sensor  
I2C (D1 = SCL, D2 = SDA) – BMP280 & BH1750  

## Libraries Needed
Install via Arduino IDE:
- DHT sensor library
- Adafruit BMP280
- BH1750
- Adafruit Unified Sensor
- Wire

## Upload
Select board: **NodeMCU 1.0 (ESP8266)**  
Upload speed: 115200  
Flash size: 4MB
