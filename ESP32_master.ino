#include "Wire.h"
#define I2C_SLAVE_ADDR 0x02
#define N1 1
#define N2 1000
#define trigger 0xFF

enum states {send_trigger, delay_state, receive_data, write_data_file};
states currentState = send_trigger;

char dataBuffer[N1][N2/2];
int bufferLength[N1];

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();
  memset(dataBuffer, 0, sizeof(dataBuffer));
  memset(bufferLength, 0, sizeof(bufferLength));
}

void loop() {
  switch (currentState) {
    case send_trigger:
      sendTrigger();
      currentState = delay_state;
      break;
    case delay_state:
      delay(2000);
      currentState = receive_data;
      break;
    case receive_data:
      receiveData();
      currentState = write_data_file;
      break;
    case write_data_file:
      writeData();
      currentState = send_trigger;
      break;
  }
}

void sendTrigger() {
  for (int i = 0; i < N1; i++) {
    Wire.beginTransmission(I2C_SLAVE_ADDR + i);
    Wire.write(trigger);
    Wire.endTransmission();
    Serial.printf("Sent trigger to slave %d\n", i);
  }
}

void receiveData() {
  for (int i = 0; i < N1; i++) {
    Wire.requestFrom(I2C_SLAVE_ADDR + i, N2/2);
    int idx = 0;
    while (Wire.available()) {
      dataBuffer[i][idx++] = Wire.read();  
    }
    bufferLength[i] = idx;
    Serial.printf("Received %d bytes from slave %d\n", idx, i);
  }
}

void writeData() {
  for (int i = 0; i < N1; i++) {
    if (bufferLength[i] > 0) {
      Serial.write(dataBuffer[i], bufferLength[i]);
      Serial.println();
    }
  }
  Serial.println("Finished writing data\n");
}
