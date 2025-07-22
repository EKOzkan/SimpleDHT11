#include "DHT11.h"

DHT11::DHT11(uint8_t pin) {
  _pin = pin;
  _lastReadTime = 0;
}

void DHT11::setPin(uint8_t pin) {
  _pin = pin;
}

bool DHT11::isReady() {
  return (millis() - _lastReadTime) >= MIN_READ_INTERVAL;
}

void DHT11::dec2bin(int n) {
  int c, k;
  for (c = 15; c >= 0; c--) {
    k = n >> c;
    if (k & 1)
      Serial.print("1");
    else
      Serial.print("0");
  }
}

void DHT11::dec2bin8(int n) {
  int c, k;
  for (c = 7; c >= 0; c--) {
    k = n >> c;
    if (k & 1)
      Serial.print("1");
    else
      Serial.print("0");
  }
}

void DHT11::startSignal() {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(18);
  digitalWrite(_pin, HIGH);
  pinMode(_pin, INPUT);
  digitalWrite(_pin, HIGH);
}

uint8_t DHT11::readSensor(uint16_t &rawHumidity, uint16_t &rawTemperature, uint8_t &checkSum) {
  uint16_t data = 0;
  unsigned long startTime;
  
  rawHumidity = 0;
  rawTemperature = 0;
  checkSum = 0;
  
  for (int8_t i = -3; i < 80; i++) {
    byte live;
    startTime = micros();
    do {
      live = (unsigned long)(micros() - startTime);
      if (live > 90) {
        return DHT11_ERROR_TIMEOUT;
      }
    } while (digitalRead(_pin) == (i & 1) ? HIGH : LOW);
    
    if (i >= 0 && (i & 1)) {
      data <<= 1;
      // TON of bit 0 is maximum 30 usecs and of bit 1 is at least 68 usecs.
      if (live > 30) {
        data |= 1; // we got a one
      }
    }
    
    switch (i) {
      case 31:
        rawHumidity = data;
        break;
      case 63:
        rawTemperature = data;
        break;
      case 79:
        checkSum = data;
        data = 0;
        break;
    }
  }
  
  return DHT11_OK;
}

DHT11_Data DHT11::read() {
  DHT11_Data result;
  uint16_t rawHumidity, rawTemperature;
  uint8_t checkSum;
  
  // Check if enough time has passed
  if (!isReady()) {
    result.error = DHT11_ERROR_TIMEOUT;
    result.humidity = 0;
    result.temperature = 0;
    result.checksumValid = false;
    return result;
  }
  
  startSignal();
  result.error = readSensor(rawHumidity, rawTemperature, checkSum);
  
  if (result.error != DHT11_OK) {
    result.humidity = 0;
    result.temperature = 0;
    result.checksumValid = false;
    return result;
  }
  
  // Extract humidity
  uint8_t humi = rawHumidity >> 8;
  uint8_t humd = (rawHumidity << 8) >> 8;
  result.humidity = humi + (humd * 0.1);
  
  // Extract temperature
  uint8_t tempi = rawTemperature >> 8;
  uint8_t tempd = (rawTemperature << 8) >> 8;
  result.temperature = tempi + (tempd * 0.1);
  
  // Verify checksum
  uint8_t calculatedChecksum = tempi + tempd + humi + humd;
  result.checksumValid = (checkSum == calculatedChecksum);
  result.error = result.checksumValid ? DHT11_OK : DHT11_ERROR_CHECKSUM;
  
  _lastReadTime = millis();
  return result;
}

DHT11_Data DHT11::readVerbose() {
  DHT11_Data result;
  uint16_t rawHumidity, rawTemperature;
  uint8_t checkSum;
  
  // Check if enough time has passed
  if (!isReady()) {
    Serial.println("ERROR: Not ready for reading yet. Please wait.");
    result.error = DHT11_ERROR_TIMEOUT;
    result.humidity = 0;
    result.temperature = 0;
    result.checksumValid = false;
    return result;
  }
  
  startSignal();
  result.error = readSensor(rawHumidity, rawTemperature, checkSum);
  
  if (result.error != DHT11_OK) {
    Serial.println("ERROR_TIMEOUT");
    result.humidity = 0;
    result.temperature = 0;
    result.checksumValid = false;
    return result;
  }
  
  // Extract values
  uint8_t humi = rawHumidity >> 8;
  uint8_t humd = (rawHumidity << 8) >> 8;
  uint8_t tempi = rawTemperature >> 8;
  uint8_t tempd = (rawTemperature << 8) >> 8;
  
  result.humidity = humi + (humd * 0.1);
  result.temperature = tempi + (tempd * 0.1);
  
  // Print detailed output
  Serial.println("Humidity: ");
  dec2bin(rawHumidity);
  Serial.print("\t");
  dec2bin8(humi);
  Serial.print("\t");
  dec2bin8(humd);
  Serial.print("\t");
  Serial.print(humi);
  Serial.print(".");
  Serial.print(humd);
  Serial.print("%");
  Serial.println("");
  
  Serial.println("Temperature Degree Celcius: ");
  dec2bin(rawTemperature);
  Serial.print("\t");
  dec2bin8(tempi);
  Serial.print("\t");
  dec2bin8(tempd);
  Serial.print("\t");
  Serial.print(tempi);
  Serial.print(".");
  Serial.print(tempd);
  Serial.print("C");
  Serial.println("");
  
  Serial.println("Checksum Byte: ");
  dec2bin8(checkSum);
  Serial.println("");
  dec2bin8(tempi + tempd + humi + humd);
  Serial.println("");
  
  // Verify checksum
  uint8_t calculatedChecksum = tempi + tempd + humi + humd;
  result.checksumValid = (checkSum == calculatedChecksum);
  result.error = result.checksumValid ? DHT11_OK : DHT11_ERROR_CHECKSUM;
  
  if (result.checksumValid) {
    Serial.print("CHECKSUM_OK");
  } else {
    Serial.print("CHECKSUM_ERROR");
  }
  Serial.println("");
  Serial.println("");
  Serial.println("");
  
  _lastReadTime = millis();
  return result;
}
