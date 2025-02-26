#include <Arduino.h>
#include <ModbusMaster.h>

class Remond {
 public:
  Remond();
  ~Remond();
  bool begin(int slaveID, Stream &serial, void (*_preTransmission)(), void (*_postTransmission)());
  uint16_t readMeasurements();
  float getpH() { return pH; }
  float getTemperature() { return temperature; }
  float getCurrent() { return current; }
  uint16_t getWarning() { return warning; }

  char *getMode() { return modeDescription[mode]; }
  float getpHUpperLimit() { return pHUpperLimit; }
  float getpHLowerLimit() { return pHLowerLimit; }
  float getpHOffset() { return pHOffset; }
  float getORPLowerLimit() { return ORPLowerLimit; }
  float getORPUpperLimit() { return ORPUpperLimit; }
  float getORPOffset() { return ORPOffset; }
  float getTemperatureUpperLimit() { return temperatureUpperLimit; }
  float getTemperatureLowerLimit() { return temperatureLowerLimit; }
  float getTemperatureOffset() { return temperatureOffset; }
  uint16_t getDampingCoefficient() { return dampingCoefficient; }
  uint16_t getDeviceAddress() { return deviceAddress; }
  char *getBaudRate() { return baudDescription[baudRate]; }

  float getORPCalibrationValue() { return ORPCalibrationValue; }
  float getCalibrationSlope() { return calibrationSlope; }
  char *getZeroPointCalibrationSolution() { return zeroPointCalibrationSolutionDescription[zeroPointCalibrationSolution]; }
  char *getSlopeCalibrationSolution() { return slopeCalibrationSolutionDescription[slopeCalibrationSolution]; }
  float getManualTemperature() { return manualTemperature; }

  void setActive(bool state = true) { ACTIVE = state; }
  uint8_t setMode(uint8_t mode);
  uint8_t setpHUpperLimit(float pH);
  uint8_t setpHLowerLimit(float pH);
  uint8_t setpHOffset(float pH);
  uint8_t setORPUpperLimit(float ORP);
  uint8_t setORPLowerLimit(float ORP);
  uint8_t setORPOffset(float ORP);
  uint8_t setTemperatureUpperLimit(float temperature);
  uint8_t setTemperatureLowerLimit(float temperature);
  uint8_t setTemperatureOffset(float temperature);
  uint8_t setDampingCoefficient(uint16_t dampingCoefficient);
  uint8_t setDeviceAddress(uint16_t deviceAddress);
  uint8_t setBaudRate(uint16_t baudRate);
  uint8_t setFactoryReset();

  uint8_t setORPCalibrationValue(float ORP);
  uint8_t setCalibrationSlope(float calibrationSlope);
  uint8_t setZeroPointCalibrationSolution(uint16_t zeroPointCalibrationSolution);  // 0 = pH 7.00, 1 = pH 6.86
  uint8_t setSlopeCalibrationSolution(uint16_t slopeCalibrationSolution);          // 0 = pH 1.68, 1 = pH 4.01, 2 = 9.18, 3 = 10.01, 4 = 12.45
  uint8_t setManualTemperature(float temperature);

  uint8_t calibrateZeroPoint();
  uint8_t calibrateSlope();

  const char *getModbusErrorDescription(uint8_t errorCode);
  const char *getWarningDescription(uint16_t warningCode);

  bool ACTIVE = true;

  static const uint16_t SUCCESS = 0x00;
  static const uint16_t PH_HIGH = 0x01;
  static const uint16_t PH_LOW = 0x02;
  static const uint16_t TEMP_HIGH = 0x03;
  static const uint16_t TEMP_LOW = 0x04;
  static const uint16_t MODBUS_ERROR = 0x05;

 private:
  uint8_t readHoldingRegisters(uint16_t address, uint16_t quantity, uint16_t *data);
  float readFloat(uint16_t address);
  uint8_t writeFloat(uint16_t address, float value);
  uint16_t readInteger16(uint16_t address);
  uint32_t readInteger32(uint16_t address);
  uint8_t readOtherParams();
  uint8_t readCalibrationParams();

  // uint8_t SLAVE_ID;
  ModbusMaster node;
  float pH = -1.0, ORP = -1.0, temperature = -1.0, current = -1.0;
  float pHUpperLimit = -1.0, pHLowerLimit = -1.0, pHOffset = -1.0, ORPUpperLimit = -1.0, ORPLowerLimit = -1.0, ORPOffset = -1.0, temperatureUpperLimit = -1.0, temperatureLowerLimit = -1.0,
        temperatureOffset = -1.0, ORPCalibrationValue = -1.0, calibrationSlope = 0, manualTemperature = -1.0;
  uint16_t dampingCoefficient = 0, deviceAddress = 0, baudRate = 5, zeroPointCalibrationSolution = 2, slopeCalibrationSolution = 5, measuredAD = 0;
  uint16_t warning, mode = 0;  // 00 = pH, 01 = ORP
  char modeDescription[3][4] = {"pH", "ORP", "???"};
  char baudDescription[6][6] = {"2400", "4800", "9600", "19200", "38400", "???"};
  char zeroPointCalibrationSolutionDescription[3][6] = {"7.00", "6.86", "???"};
  char slopeCalibrationSolutionDescription[6][6] = {"1.68", "4.01", "9.18", "10.1", "12.45", "???"};
  bool otherParamsValid = false;
  char readOtherParamsFirstWarning[50] = "not valid - please call readOtherParams() first";
};