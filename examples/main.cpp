#include <Arduino.h>
#include <Remond.h>

#define RS485_RO 17  // receiver output (rx of MCU)
#define RS485_DI 4   // device input (tx of MCU)
#define DE_1 16      // device enable for first RS485 modulator
#define MODBUS_ID 1

Remond remond;

void port1setDE();
void port1resetDE();

void setup() {
  pinMode(DE_1, OUTPUT);

  Serial.begin(2000000);
  log_i("Remond pH Test");
  Serial2.begin(9600, SERIAL_8N1, RS485_RO, RS485_DI, false, 20000UL);  // set up Serial2 for RS485 communication
  remond.begin(MODBUS_ID, Serial2, &port1setDE, &port1resetDE);
  log_i("ph Upper Limit: %f: ", remond.getpHUpperLimit());
  log_i("pH Lower Limit: %f", remond.getpHLowerLimit());
  log_i("pH Offset: %f", remond.getpHOffset());
  log_i("Temperature Upper Limit: %f", remond.getTemperatureUpperLimit());
  log_i("Temperature Lower Limit: %f", remond.getTemperatureLowerLimit());
  log_i("Temperature Offset: %f", remond.getTemperatureOffset());
  log_i("ORP Upper Limit: %f", remond.getORPUpperLimit());
  log_i("ORP Lower Limit: %f", remond.getORPLowerLimit());
  log_i("ORP Offset: %f", remond.getORPOffset());
  log_i("Damping Coefficient: %d", remond.getDampingCoefficient());
  log_i("Device Address: %d", remond.getDeviceAddress());
  log_i("Baud Rate: %s", remond.getBaudRate());
  log_i("Mode: %s", remond.getMode());
  log_i("ORP Calibration Value: %f", remond.getORPCalibrationValue());
  log_i("Calibration Slope: %f", remond.getCalibrationSlope());
  log_i("Zero Point Calibration Solution: %s", remond.getZeroPointCalibrationSolution());
  log_i("Slope Calibration Solution: %s", remond.getSlopeCalibrationSolution());
  log_i("Manual Temperature: %f", remond.getManualTemperature());
}

void loop() {
  float pH, temperature, current, temperature_ul = -2, temperature_ll = -3;
  uint16_t warning = remond.readMeasurements();
  if (warning != 0)
    log_w("remond read warning: %s",
          remond.getWarningDescription(warning));  // this will read the pH, temperature, current, and warning code (in one modbus command)
  pH = remond.getpH();
  temperature = remond.getTemperature();
  current = remond.getCurrent();
  log_i("\n\tpH: %f \n\tTemperature: %f \n\tCurrent: %f \n", pH, temperature, current);

  delay(1000);
}

void port1setDE() {
  digitalWrite(DE_1, HIGH);
  delay(2);
}

void port1resetDE() {
  digitalWrite(DE_1, LOW);
  delay(2);
}
