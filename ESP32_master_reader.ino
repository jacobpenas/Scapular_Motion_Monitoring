#include "Wire.h"

#define I2C_DEV_ADDR 0x02           // first slave address
#define N1 2                        // number of slaves
#define N2 6                        // number of bytes to request

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();
  Wire.setClock(100000);
}

void loop() {
  delay(5000);

  // Write message to each slave
  for (int j = 0; j < N1; j++) {
    Wire.beginTransmission(I2C_DEV_ADDR + j);
    Wire.printf("Hello World! %lu", i++);
    uint8_t error = Wire.endTransmission(true);
    Serial.printf("endTransmission (slave %d): %u\n", j, error);
  }

  delay(1000);
  
  // Read 6 bytes from each slave
  for (int j = 0; j < N1; j++) {
    uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR + j, N2);
    Serial.printf("requestFrom (slave %d): %u\n", j, bytesReceived);
    if ((bool)bytesReceived) {
      uint8_t temp[bytesReceived];
      Wire.readBytes(temp, bytesReceived);
      log_print_buf(temp, bytesReceived);
    }
  }
}
