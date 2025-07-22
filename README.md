
# 📡 DHT11 Arduino Library

A lightweight and easy-to-use Arduino library to read temperature and humidity data from the **DHT11 sensor**. Includes checksum validation, read throttling, and verbose debugging with binary output.

---

## 🔧 Features

- ✅ Simple interface to read temperature and humidity
- 🔄 Built-in minimum read interval (2 seconds)
- 🧠 Checksum validation
- 🐞 `readVerbose()` for binary debugging output
- 🔌 Ability to change pin at runtime

---

## 📦 Installation

### Option 1: Manual
1. Download or clone this repository:
   ```bash
   git clone https://github.com/YOUR_USERNAME/DHT11.git


2. Copy the `DHT11.h` and `DHT11.cpp` files into your Arduino project's folder.
3. In your sketch, include the header:

   ```cpp
   #include "DHT11.h"
   ```

---

## ⚙️ Usage

### Example Sketch

```cpp
#include <Arduino.h>
#include "DHT11.h"

DHT11 dht(2); // Use digital pin 2

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (dht.isReady()) {
    DHT11_Data data = dht.read();
    if (data.error == DHT11_OK) {
      Serial.print("Humidity: ");
      Serial.print(data.humidity);
      Serial.println("%");

      Serial.print("Temperature: ");
      Serial.print(data.temperature);
      Serial.println("°C");
    } else {
      Serial.println("Error reading from DHT11 sensor.");
    }
  }

  delay(500); // Optional, since isReady() handles timing
}
```

---

## 📚 API Reference

### Constructor

```cpp
DHT11(uint8_t pin);
```

### Public Methods

| Method                      | Description                                                         |
| --------------------------- | ------------------------------------------------------------------- |
| `DHT11_Data read();`        | Reads the sensor and returns humidity, temperature, and error code. |
| `DHT11_Data readVerbose();` | Same as `read()` but with binary debug output over Serial.          |
| `bool isReady();`           | Returns `true` if 2 seconds have passed since last read.            |
| `void setPin(uint8_t pin);` | Changes the GPIO pin used by the sensor.                            |

### `DHT11_Data` Struct

```cpp
struct DHT11_Data {
  float humidity;
  float temperature;
  uint8_t error;         // DHT11_OK, DHT11_ERROR_TIMEOUT, DHT11_ERROR_CHECKSUM
  bool checksumValid;    // true if checksum is correct
};
```

### Error Codes

| Code                   | Meaning          |
| ---------------------- | ---------------- |
| `DHT11_OK`             | Read successful  |
| `DHT11_ERROR_TIMEOUT`  | Sensor timed out |
| `DHT11_ERROR_CHECKSUM` | Checksum failed  |

---

## 🧪 Debugging Example

```cpp
dht.readVerbose();
```

This will print binary representation of the raw data, extracted bits, and checksum information for in-depth debugging.

---

## 🛠 Hardware Setup

| DHT11 Pin | Arduino                 |
| --------- | ----------------------- |
| VCC       | 3.3V or 5V              |
| GND       | GND                     |
| DATA      | D2 (or your chosen pin) |

**⚠️ Use a 10kΩ pull-up resistor between VCC and DATA for stable communication.**

---

## 📃 License

MIT License

---

## 🙌 Credits

Developed by ekozkan

Contributions welcome!

```

---

Let me know if you'd like me to generate a `library.properties` file to make this installable as an official Arduino library!
```
