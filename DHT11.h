#ifndef DHT11_H
#define DHT11_H

#include <Arduino.h>

// Error codes
#define DHT11_OK 0
#define DHT11_ERROR_TIMEOUT 1
#define DHT11_ERROR_CHECKSUM 2

struct DHT11_Data {
  float humidity;
  float temperature;
  uint8_t error;
  bool checksumValid;
};

class DHT11 {
private:
  uint8_t _pin;
  unsigned long _lastReadTime;
  static const unsigned long MIN_READ_INTERVAL = 2000; // Minimum 2 seconds between reads
  
  void dec2bin(int n);
  void dec2bin8(int n);
  void startSignal();
  uint8_t readSensor(uint16_t &rawHumidity, uint16_t &rawTemperature, uint8_t &checkSum);

public:
  DHT11(uint8_t pin);
  DHT11_Data read();
  DHT11_Data readVerbose(); // Includes binary output for debugging
  bool isReady(); // Check if enough time has passed for next reading
  void setPin(uint8_t pin);
};

#endif
