#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33;
#define TA_SHIFT 8
float mlx90640To[768];
paramsMLX90640 mlx90640;

void setup() {
  Wire.begin();
  Wire.setClock(1000000); // Max I2C speed (1MHz)
  Serial.begin(2000000);  // 2Mbps serial

  if (!isConnected()) {
    Serial.println("MLX90640 not detected!");
    while (1);
  }

  uint16_t eeMLX90640[832];
  int status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  status |= MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) Serial.println("EEPROM Error");

  // Set to 16Hz without SynchFrame
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); // 16Hz
  MLX90640_SetResolution(MLX90640_address, 0x03);  // 19-bit resolution
}

void loop() {
  uint16_t mlx90640Frame[834];
  static uint32_t lastSend = 0;
  
  // Remove SynchFrame call
  int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
  
  float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
  MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95, Ta - TA_SHIFT, mlx90640To);

  // Binary output (768 floats × 4 bytes = 3072 bytes)
  Serial.write((byte*)mlx90640To, sizeof(mlx90640To)); 

  // Strict 62.5ms timing for 16Hz
  while (millis() - lastSend < 62); 
  lastSend = millis();
}

bool isConnected() {
  Wire.beginTransmission(MLX90640_address);
  return (Wire.endTransmission() == 0);
}