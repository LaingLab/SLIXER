// Connects feetech::Bus to an Arduino hardware UART.
#pragma once

#include <Arduino.h>

#include "feetech.h"

template <class SerialT>
class SerialPort : public feetech::Port {
 public:
  explicit SerialPort(SerialT& serial) : serial_(serial) {}

  void write(const uint8_t* data, size_t len) override {
    serial_.write(data, len);
    serial_.flush();  // wait until sent: the servo answers right after, on the same wire
  }

  int readByte(uint32_t timeout_us) override {
    const uint32_t start = micros();
    while (!serial_.available()) {
      if (micros() - start >= timeout_us) return -1;
    }
    return serial_.read();
  }

  void discardInput() override {
    while (serial_.available()) serial_.read();
  }

 private:
  SerialT& serial_;
};
