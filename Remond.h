#include <Arduino.h>
#include <ModbusMaster.h>

#define REM_ADDR_OF_PH 0x0001
#define REM_ADDR_OF_TEMPERATURE 0x0003
#define REM_ADDR_OF_CURRENT 0X0005
#define REM_ADDR_OF_WARNING 0x0007          // 00 = no warning
#define REM_ADDR_OF_MODE 0x0008             // 00 = pH, 01 = ORP
#define REM_ADDR_OF_PH_UPPER_LIMIT 0x000A   // corresponds to 20mA upper limit
#define REM_ADDR_OF_PH_LOWER_LIMIT 0x000C   // corresponds to 4mA lower limit
#define REM_ADDR_OF_PH_OFFSET 0x0012        // adjust measured value
#define REM_ADDR_OF_ORP_UPPER_LIMIT 0x000A  // corresponds to 20mA upper limit
#define REM_ADDR_OF_ORP_LOWER_LIMIT 0x000C  // corresponds to 4mA lower limit
#define REM_ADDR_OF_ORP_OFFSET 0x0012       // adjust measured value
#define REM_ADDR_OF_TEMPERATURE_UPPER_LIMIT 0x000E
#define REM_ADDR_OF_TEMPERATURE_LOWER_LIMIT 0x0010
#define REM_ADDR_OF_TEMPERATURE_OFFSET 0x0014
#define REM_ADDR_OF_DAMPING_COEFFICIENT 0x0016
#define REM_ADDR_OF_DEVICE_REM_ADDR 0x0019
#define REM_ADDR_OF_BAUD_RATE 0x001A
#define REM_ADDR_OF_FACTORY_RESET 0x001B
#define REM_ADDR_OF_ORP_CALIBRATION_VALUE 0x0030
#define REM_ADDR_OF_CALIBRATION_SLOPE 0x0034
#define REM_ADDR_OF_ZERO_POINT_CALIBRATION_SOLUTION 0x0036  // 00 = pH 7.00, 01 = pH 6.86
#define REM_ADDR_OF_SLOPE_CALIBRATION_SOLUTION 0x0038       // 00 = pH 1.68, 01 = pH 4.01, 02 = 9.18, 03 = 10.01, 04 = 12.45
#define REM_ADDR_OF_MANUAL_TEMPERATURE 0x003A
#define REM_ADDR_OF_ZERO_CONFIRMATION 0x003E
#define REM_ADDR_OF_SLOPE_CONFIRMATION 0x003F
#define REM_ADDR_OF_MEASURED_AD 0x0066
#define MIN_DELAY_BETWEEN_READS 5

class Remond {
 public:
  Remond();
  ~Remond();
  void begin(int slaveID, Stream &serial, void (*_preTransmission)(), void (*_postTransmission)());
  uint8_t readHoldingRegisters(uint16_t address, uint16_t quantity, uint16_t *data);
  float readFloat(uint16_t address);
  uint8_t writeFloat(uint16_t address, float value);
  uint16_t readInteger16(uint16_t address);
  uint32_t readInteger32(uint16_t address);

  uint16_t readMeasurements();
  float getpH() { return pH; }
  float getTemperature() { return temperature; }
  float getCurrent() { return current; }
  uint16_t getWarning() { return warning; }

  uint16_t readOtherParams();
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

  uint16_t readCalibrationParams();
  float getORPCalibrationValue() { return ORPCalibrationValue; }
  float getCalibrationSlope() { return calibrationSlope; }
  char *getZeroPointCalibrationSolution() { return zeroPointCalibrationSolutionDescription[zeroPointCalibrationSolution]; }
  char *getSlopeCalibrationSolution() { return slopeCalibrationSolutionDescription[slopeCalibrationSolution]; }
  float getManualTemperature() { return manualTemperature; }

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

 private:
  uint8_t SLAVE_ID;
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