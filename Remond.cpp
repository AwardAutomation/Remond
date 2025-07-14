#include <Remond.h>

#define ADDRESS_OF_PH 0x0001
#define ADDRESS_OF_TEMPERATURE 0x0003
#define ADDRESS_OF_CURRENT 0X0005
#define ADDRESS_OF_WARNING 0x0007          // 00 = no warning
#define ADDRESS_OF_MODE 0x0008             // 00 = pH, 01 = ORP
#define ADDRESS_OF_PH_UPPER_LIMIT 0x000A   // corresponds to 20mA upper limit
#define ADDRESS_OF_PH_LOWER_LIMIT 0x000C   // corresponds to 4mA lower limit
#define ADDRESS_OF_PH_OFFSET 0x0012        // adjust measured value
#define ADDRESS_OF_ORP_UPPER_LIMIT 0x000A  // corresponds to 20mA upper limit
#define ADDRESS_OF_ORP_LOWER_LIMIT 0x000C  // corresponds to 4mA lower limit
#define ADDRESS_OF_ORP_OFFSET 0x0012       // adjust measured value
#define ADDRESS_OF_TEMPERATURE_UPPER_LIMIT 0x000E
#define ADDRESS_OF_TEMPERATURE_LOWER_LIMIT 0x0010
#define ADDRESS_OF_TEMPERATURE_OFFSET 0x0014
#define ADDRESS_OF_DAMPING_COEFFICIENT 0x0016
#define ADDRESS_OF_DEVICE_ADDRESS 0x0019
#define ADDRESS_OF_BAUD_RATE 0x001A
#define ADDRESS_OF_FACTORY_RESET 0x001B
#define ADDRESS_OF_ORP_CALIBRATION_VALUE 0x0030
#define ADDRESS_OF_CALIBRATION_SLOPE 0x0034
#define ADDRESS_OF_ZERO_POINT_CALIBRATION_SOLUTION 0x0036  // 00 = pH 7.00, 01 = pH 6.86
#define ADDRESS_OF_SLOPE_CALIBRATION_SOLUTION 0x0038       // 00 = pH 1.68, 01 = pH 4.01, 02 = 9.18, 03 = 10.01, 04 = 12.45
#define ADDRESS_OF_MANUAL_TEMPERATURE 0x003A
#define ADDRESS_OF_ZERO_CONFIRMATION 0x003E
#define ADDRESS_OF_SLOPE_CONFIRMATION 0x003F
#define ADDRESS_OF_MEASURED_AD 0x0066
#define MIN_DELAY_BETWEEN_READS 5

uint16_t Remond::indexCounter = 0;  // initialize the static member variable

float regToFloat(uint16_t AB, uint16_t CD);
uint16_t *floatToReg(float floatValue, uint16_t *reg, size_t len);

Remond::Remond() {
  // constructor
  index = indexCounter++;
}

Remond::~Remond() {
  // destructor
  indexCounter--;
}

bool Remond::begin(int slaveID, Stream &serial, void (*_preTransmission)(), void (*_postTransmission)(), const char *name) {
  // SLAVE_ID = slaveID;
  snprintf(NAME, sizeof(NAME), "%s%u", name, index + 1);  // note index is zero based
  node.begin(slaveID, serial);
  node.preTransmission(_preTransmission);
  node.postTransmission(_postTransmission);
  ACTIVE = true;
  uint8_t mbRet = readOtherParams();
  if (mbRet != node.ku8MBSuccess) {
    log_e("Remond sensor %d slaveID %d failed to initialize. Error: 0x%02X  %s", index + 1, slaveID, mbRet, getModbusErrorDescription(mbRet));
    ACTIVE = false;
    return false;
  }
  log_i("Remond sensor %d initialized successfully", index + 1);
  ACTIVE = true;
  READ_FAIL_COUNT = 0;
  delay(MIN_DELAY_BETWEEN_READS);
  readCalibrationParams();
  delay(MIN_DELAY_BETWEEN_READS);
  readMeasurements();
  delay(MIN_DELAY_BETWEEN_READS);
  return true;
}

uint8_t Remond::readHoldingRegisters(uint16_t address, uint16_t quantity, uint16_t *reg) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.readHoldingRegisters(address, quantity);
  // delay(10);  // wait for the data to be available
  if (mbRet == node.ku8MBSuccess) {
    for (int i = 0; i < quantity; i++) {
      reg[i] = node.getResponseBuffer(i);
      // log_d("Data[%d]: 0x%04X", i, reg[i]);  // print the 16bit reg
    }
    // log_d("holding register 0x%04X: mbRet 0x%02X %s \t Bytes available: %d", address, mbRet, getModbusErrorDescription(mbRet), node.available());  // report the number of bytes available
  }
  else {
    log_e("Failed to read holding register 0x%04X. Error: 0x%02X  %s", address, mbRet, getModbusErrorDescription(mbRet));
  }
  return mbRet;
}

float Remond::readFloat(uint16_t address) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint16_t reg[2] = {0};
  float float_value = -1.0;

  if (readHoldingRegisters(address, 2, reg) == node.ku8MBSuccess) {
    float_value = regToFloat(reg[1], reg[0]);
    // log_d("Value: %f \t reg[0] 0x%02X, reg[1] 0x%02X", float_value, reg[0], reg[1]);
  }
  else {
    log_e("Failed to read holding registers for float. Error: 0x%02X  %s", node.ku8MBSuccess, getModbusErrorDescription(node.ku8MBSuccess));
  }
  return float_value;
}

uint8_t Remond::writeFloat(uint16_t address, float value) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint16_t reg[2] = {0};
  floatToReg(value, reg, 2);
  uint8_t mbRet = node.setTransmitBuffer(0, reg[0]);
  mbRet = node.setTransmitBuffer(1, reg[1]);
  mbRet = node.writeMultipleRegisters(address, 2);
  if (mbRet != node.ku8MBSuccess)
    log_e("Failed to write float to 0x%04X. Error: 0x%02X  %s", address, mbRet, getModbusErrorDescription(mbRet));
  else
    log_d("Float written to 0x%04X: %f", address, value);
  return mbRet;
}

uint16_t Remond::readInteger16(uint16_t address) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint16_t ret[1] = {0};
  readHoldingRegisters(address, 1, ret);
  return ret[0];
}

uint32_t Remond::readInteger32(uint16_t address) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint32_t ret = 0;
  uint16_t reg[2] = {0};
  uint8_t mbRet = readHoldingRegisters(address, 2, reg);
  if (mbRet == node.ku8MBSuccess) ret = (uint32_t)reg[1] << 16 | reg[0];
  return ret;
}

uint16_t Remond::readMeasurements() {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  // read the first 8 registers
  uint16_t reg[8] = {0};
  uint8_t mbRet = readHoldingRegisters(ADDRESS_OF_PH, 8, reg);
  if (mbRet == node.ku8MBSuccess) {
    mode = reg[7];  // copy the mode
    if (mode == 0x00) {
      pH = regToFloat(reg[1], reg[0]);  // convert pH to float
      ORP = 0.0;                        // set ORP to 0 if mode is pH
    }
    if (mode == 0x01) {
      ORP = regToFloat(reg[1], reg[0]);  // convert ORP to float
      pH = 0.0;                          // set pH to 0 if mode is ORP
    }
    temperature = regToFloat(reg[3], reg[2]);  // convert temperature to float
    current = regToFloat(reg[5], reg[4]);      // convert current to float
    warning = reg[6] << 8;                     // copy warning to upper byte of warning variable
    if (warning != 0) log_w("Warning: %s", getWarningDescription(warning));
    // log_d(
    //     "reading successful \n"
    //     "\treg[0] 0x%04X \n"
    //     "\treg[1] 0x%04X \n"
    //     "\treg[2] 0x%04X \n"
    //     "\treg[3] 0x%04X \n"
    //     "\treg[4] 0x%04X \n"
    //     "\treg[5] 0x%04X \n"
    //     "\treg[6] 0x%04X \n"
    //     "\treg[7] 0x%04X",
    //     reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7]);
  }
  else {
    log_e("Modbus error: 0x%02X  %s", mbRet, getModbusErrorDescription(mbRet));
    warning |= mbRet;  // append mbRet to the lower byte of warning
    READ_FAIL_COUNT++;
  }
  return warning;
}

uint8_t Remond::readOtherParams() {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  // read the next 19 registers starting from MODE
  log_d("Remond %u: Reading other parameters from address: 0x%04X", index, ADDRESS_OF_MODE);
  uint16_t reg[19] = {0};
  uint8_t mbRet = readHoldingRegisters(ADDRESS_OF_MODE, 19, reg);
  if (mbRet == node.ku8MBSuccess) {
    otherParamsValid = true;
    mode = reg[0];  // copy mode
    if (mode == 0x00) {
      pHUpperLimit = regToFloat(reg[3], reg[2]);  // convert pH upper limit to float
      pHLowerLimit = regToFloat(reg[5], reg[4]);  // convert pH lower limit to float
      pHOffset = regToFloat(reg[11], reg[10]);    // convert pH offset to float
      ORPUpperLimit = 0.0;                        // set ORP upper limit to 0 if mode is pH
      ORPLowerLimit = 0.0;                        // set ORP lower limit to 0 if mode is pH
      ORPOffset = 0.0;                            // set ORP offset to 0 if mode is pH
    }
    else if (mode == 0x01) {
      pHUpperLimit = 0.0;                          // set pH upper limit to 0 if mode is ORP
      pHLowerLimit = 0.0;                          // set pH lower limit to 0 if mode is ORP
      pHOffset = 0.0;                              // set pH offset to 0 if mode is ORP
      ORPUpperLimit = regToFloat(reg[3], reg[2]);  // convert ORP upper limit to float
      ORPLowerLimit = regToFloat(reg[5], reg[4]);  // convert ORP lower limit to float
      ORPOffset = regToFloat(reg[11], reg[10]);    // convert ORP offset to float
    }
    else {
      log_e("Invalid mode: 0x%02X", mode);
    }
    temperatureUpperLimit = regToFloat(reg[7], reg[6]);  // convert temp upper limit to float
    temperatureLowerLimit = regToFloat(reg[9], reg[8]);  // convert temp lower limit to float
    temperatureOffset = regToFloat(reg[13], reg[12]);    // convert temp offset to float
    dampingCoefficient = reg[14];                        // copy damping coefficient
    deviceAddress = reg[17];                             // copy device address
    baudRate = reg[18];                                  // copy baud rate
  }
  else {
    log_e("Modbus error: 0x%02X  %s", mbRet, getModbusErrorDescription(mbRet));
  }
  return mbRet;
}

uint8_t Remond::readCalibrationParams() {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  // read the next 10 registers starting from ORP calibration value
  uint16_t reg[12] = {0};
  uint8_t mbRet = readHoldingRegisters(ADDRESS_OF_ORP_CALIBRATION_VALUE, 12, reg);
  if (mbRet == node.ku8MBSuccess) {
    ORPCalibrationValue = regToFloat(reg[1], reg[0]);  // convert ORP calibration value to float
    calibrationSlope = regToFloat(reg[5], reg[4]);     // copy calibration slope
    zeroPointCalibrationSolution = reg[6];             // copy zero point calibration solution
    slopeCalibrationSolution = reg[8];                 // copy slope calibration solution
    manualTemperature = regToFloat(reg[11], reg[10]);  // convert manual temperature to float
  }
  else {
    log_e("Modbus error: 0x%02X  %s", mbRet, getModbusErrorDescription(mbRet));
  }
  return mbRet;
}

// 0x00 = pH, 0x01 = ORP
uint8_t Remond::setMode(uint8_t _mode) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.writeSingleRegister(ADDRESS_OF_MODE, mode);
  if (mbRet == node.ku8MBSuccess) mode = _mode;
  return mbRet;
}

uint8_t Remond::setpHUpperLimit(float _pH) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  if (mode != 0x00) {
    log_e("Invalid mode: 0x%02X %s", mode, modeDescription[mode]);
    return node.ku8MBIllegalFunction;
  }
  uint8_t mbRet = writeFloat(ADDRESS_OF_PH_UPPER_LIMIT, _pH);
  if (mbRet == node.ku8MBSuccess) pHUpperLimit = _pH;
  return mbRet;
}

uint8_t Remond::setpHLowerLimit(float _pH) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  if (mode != 0x00) {
    log_e("Invalid mode: 0x%02X %s", mode, modeDescription[mode]);
    return node.ku8MBIllegalFunction;
  }
  uint8_t mbRet = writeFloat(ADDRESS_OF_PH_LOWER_LIMIT, _pH);
  if (mbRet == node.ku8MBSuccess) pHLowerLimit = _pH;
  return mbRet;
}

uint8_t Remond::setpHOffset(float _pH) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  if (mode != 0x00) {
    log_e("Invalid mode: 0x%02X %s", mode, modeDescription[mode]);
    return node.ku8MBIllegalFunction;
  }
  uint8_t mbRet = writeFloat(ADDRESS_OF_PH_OFFSET, _pH);
  if (mbRet == node.ku8MBSuccess) pHOffset = _pH;
  return mbRet;
}

uint8_t Remond::setORPUpperLimit(float _ORP) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  if (mode != 0x01) {
    log_e("Invalid mode: 0x%02X %s", mode, modeDescription[mode]);
    return node.ku8MBIllegalFunction;
  }
  uint8_t mbRet = writeFloat(ADDRESS_OF_ORP_UPPER_LIMIT, _ORP);
  if (mbRet == node.ku8MBSuccess) ORPUpperLimit = _ORP;
  return mbRet;
}

uint8_t Remond::setORPLowerLimit(float _ORP) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  if (mode != 0x01) {
    log_e("Invalid mode: 0x%02X %s", mode, modeDescription[mode]);
    return node.ku8MBIllegalFunction;
  }
  uint8_t mbRet = writeFloat(ADDRESS_OF_ORP_LOWER_LIMIT, _ORP);
  if (mbRet == node.ku8MBSuccess) ORPLowerLimit = _ORP;
  return mbRet;
}

uint8_t Remond::setORPOffset(float _ORP) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  if (mode != 0x01) {
    log_e("Invalid mode: 0x%02X %s", mode, modeDescription[mode]);
    return node.ku8MBIllegalFunction;
  }
  uint8_t mbRet = writeFloat(ADDRESS_OF_ORP_OFFSET, _ORP);
  if (mbRet == node.ku8MBSuccess) ORPOffset = _ORP;
  return mbRet;
}

uint8_t Remond::setTemperatureUpperLimit(float _temperature) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = writeFloat(ADDRESS_OF_TEMPERATURE_UPPER_LIMIT, _temperature);
  if (mbRet == node.ku8MBSuccess) temperatureUpperLimit = _temperature;
  return mbRet;
}

uint8_t Remond::setTemperatureLowerLimit(float _temperature) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = writeFloat(ADDRESS_OF_TEMPERATURE_LOWER_LIMIT, _temperature);
  if (mbRet == node.ku8MBSuccess) temperatureLowerLimit = _temperature;
  return mbRet;
}

uint8_t Remond::setTemperatureOffset(float _temperature) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = writeFloat(ADDRESS_OF_TEMPERATURE_OFFSET, _temperature);
  if (mbRet == node.ku8MBSuccess) temperatureOffset = _temperature;
  return mbRet;
}

uint8_t Remond::setDampingCoefficient(uint16_t _dampingCoefficient) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.writeSingleRegister(ADDRESS_OF_DAMPING_COEFFICIENT, mode);
  if (mbRet == node.ku8MBSuccess) dampingCoefficient = _dampingCoefficient;
  return mbRet;
}

uint8_t Remond::setDeviceAddress(uint16_t _deviceAddress) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.writeSingleRegister(ADDRESS_OF_DEVICE_ADDRESS, mode);
  if (mbRet == node.ku8MBSuccess) deviceAddress = _deviceAddress;
  return mbRet;
}

uint8_t Remond::setBaudRate(uint16_t _baudRate) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.writeSingleRegister(ADDRESS_OF_BAUD_RATE, mode);
  if (mbRet == node.ku8MBSuccess) baudRate = _baudRate;
  return mbRet;
}

uint8_t Remond::setFactoryReset() {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.writeSingleRegister(ADDRESS_OF_FACTORY_RESET, 1);
  if (mbRet == node.ku8MBSuccess) log_w("Factory reset");
  return mbRet;
}

uint8_t Remond::setORPCalibrationValue(float _ORP) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = writeFloat(ADDRESS_OF_ORP_CALIBRATION_VALUE, _ORP);
  if (mbRet == node.ku8MBSuccess) ORPCalibrationValue = _ORP;
  return mbRet;
}

uint8_t Remond::setCalibrationSlope(float _calibrationSlope) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = writeFloat(ADDRESS_OF_CALIBRATION_SLOPE, _calibrationSlope);
  if (mbRet == node.ku8MBSuccess) calibrationSlope = _calibrationSlope;
  return mbRet;
}

uint8_t Remond::setZeroPointCalibrationSolution(uint16_t _zeroPointCalibrationSolution) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.writeSingleRegister(ADDRESS_OF_ZERO_POINT_CALIBRATION_SOLUTION, _zeroPointCalibrationSolution);
  if (mbRet == node.ku8MBSuccess) zeroPointCalibrationSolution = _zeroPointCalibrationSolution;
  return mbRet;
}

uint8_t Remond::setSlopeCalibrationSolution(uint16_t _slopeCalibrationSolution) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = node.writeSingleRegister(ADDRESS_OF_SLOPE_CALIBRATION_SOLUTION, _slopeCalibrationSolution);
  if (mbRet == node.ku8MBSuccess) slopeCalibrationSolution = _slopeCalibrationSolution;
  return mbRet;
}

uint8_t Remond::setManualTemperature(float _temperature) {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint8_t mbRet = writeFloat(ADDRESS_OF_MANUAL_TEMPERATURE, _temperature);
  if (mbRet == node.ku8MBSuccess) manualTemperature = _temperature;
  return mbRet;
}

uint8_t Remond::calibrateZeroPoint() {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint16_t measuredAD;
  uint8_t mbRet = readHoldingRegisters(ADDRESS_OF_MEASURED_AD, 1, &measuredAD);
  if (mbRet == node.ku8MBSuccess) {
    mbRet = node.writeSingleRegister(ADDRESS_OF_ZERO_CONFIRMATION, measuredAD);
    if (mbRet == node.ku8MBSuccess) log_d("Zero point calibrated: %d", measuredAD);
  }
  else
    log_e("Zero calibration failed. Error: 0x%02X  %s", mbRet, getModbusErrorDescription(mbRet));
  return mbRet;
}

uint8_t Remond::calibrateSlope() {
  if (!ACTIVE) return node.ku8MBSlaveDeviceFailure;
  uint16_t measuredAD;
  uint8_t mbRet = readHoldingRegisters(ADDRESS_OF_MEASURED_AD, 1, &measuredAD);
  if (mbRet == node.ku8MBSuccess) {
    mbRet = node.writeSingleRegister(ADDRESS_OF_SLOPE_CONFIRMATION, measuredAD);
    if (mbRet == node.ku8MBSuccess) log_d("Slope calibrated: %d", measuredAD);
  }
  else
    log_e("Slope calibration failed. Error: 0x%02X  %s", mbRet, getModbusErrorDescription(mbRet));
  return mbRet;
}

const char *Remond::getModbusErrorDescription(uint16_t errorCode) {
  switch (errorCode) {
    case node.ku8MBSuccess:
      return "Success";
    case node.ku8MBIllegalFunction:
      return "Illegal Function";
    case node.ku8MBIllegalDataAddress:
      return "Illegal Data Address";
    case node.ku8MBIllegalDataValue:
      return "Illegal Data Value";
    case node.ku8MBSlaveDeviceFailure:
      return "Slave Device Failure";
    case node.ku8MBInvalidSlaveID:
      return "Invalid Slave ID";
    case node.ku8MBInvalidFunction:
      return "Invalid Function";
    case node.ku8MBResponseTimedOut:
      return "Response Timed Out";
    case node.ku8MBInvalidCRC:
      return "Invalid CRC";
    default:
      return "Unknown Error";
  }
}

const char *Remond::getWarningDescription(uint16_t warningCode) {
  switch (warningCode) {
    case SUCCESS:
      return "No warning";
    case PH_HIGH:
      return (mode == 1) ? "ORP upper limit exceeded" : "pH upper limit exceeded";
    case PH_LOW:
      return (mode == 1) ? "ORP lower limit exceeded" : "pH lower limit exceeded";
    case TEMP_HIGH:
      return "temp upper limit exceeded";
    case TEMP_LOW:
      return "temp lower limit exceeded";
    case MODBUS_ERROR:
      return "modbus error";
    default:
      return "Unknown warning";
  }
}

// assuming float consists of 4 bytes, ABCD
float regToFloat(uint16_t AB, uint16_t CD) {
  float floatValue;                           // return value
  uint32_t temp = AB << 16 | CD;              // create a 32bit integer
  memcpy(&floatValue, &temp, sizeof(float));  // copy 1x 32bit to 1x float
  return floatValue;
}

uint16_t *floatToReg(float floatValue, uint16_t *reg, size_t len) {
  if (len < 2) {
    log_e("reg array length must be at least 2");
    return reg;
  }
  uint32_t temp;
  memcpy(&temp, &floatValue, sizeof(float));
  reg[0] = temp & 0xFFFF;
  reg[1] = temp >> 16;
  return reg;
}