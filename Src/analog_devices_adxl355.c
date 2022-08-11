#include "analog_devices_adxl355.h"

void ADXL355I2C_Init(ADXL355Handle* hadxl355, I2C_HandleTypeDef* hi2c, \
                  uint8_t ASEL_PIN_LEVEL)
{
  hadxl355->hi2c = hi2c;
  if (ASEL_PIN_LEVEL == 0)
    hadxl355->i2cAddr = 0x1D << 9;
  else if (ASEL_PIN_LEVEL == 1)
    hadxl355->i2cAddr = 0x53 << 9;
}

void ADXL355SPI_Init(ADXL355Handle* hadxl355, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint32_t cs_pin)
{
  hadxl355->hspi = hspi;
  hadxl355->cs_port = cs_port;
  hadxl355->cs_pin = cs_pin;
  hadxl355->range = ADXL355_MEASURE_RANGE_2G;
  hadxl355->ifNewDataReady = 0;
}

void ADXL355SPI_ReadRegisters(ADXL355Handle* hadxl355, uint8_t start_register, uint8_t num)
{
  if (num > 9)
    return;
  uint8_t tempTx = (start_register << 1) | 0x01;
  HAL_GPIO_WritePin(hadxl355->cs_port, hadxl355->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hadxl355->hspi, &tempTx, 1, 1);
  HAL_SPI_Receive(hadxl355->hspi, hadxl355->tempRx, num, 1);
  HAL_GPIO_WritePin(hadxl355->cs_port, hadxl355->cs_pin, GPIO_PIN_SET);
}

void ADXL355SPI_WriteRegisters(ADXL355Handle* hadxl355, uint8_t start_register, uint8_t* txbuf, uint8_t len)
{
  uint8_t tempTx = start_register << 1;
  HAL_GPIO_WritePin(hadxl355->cs_port, hadxl355->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hadxl355->hspi, &tempTx, 1, 1);
  HAL_SPI_Transmit(hadxl355->hspi, txbuf, len, 1);
  HAL_GPIO_WritePin(hadxl355->cs_port, hadxl355->cs_pin, GPIO_PIN_SET);
}

void ADXL355SPI_ReadAccelerations(ADXL355Handle* hadxl355)
{
  ADXL355SPI_ReadRegisters(hadxl355, ADXL355_REG_XDATA3, 9);
  int32_t rawX, rawY, rawZ;
  rawX = ((int32_t)hadxl355->tempRx[0]) << 12 | ((int32_t)hadxl355->tempRx[1]) << 4 | ((int32_t)hadxl355->tempRx[2]) << 4;
  if (rawX >> 19)
    rawX -= 1048576;
  rawY = ((int32_t)hadxl355->tempRx[3]) << 12 | ((int32_t)hadxl355->tempRx[4]) << 4 | ((int32_t)hadxl355->tempRx[5]) << 4;
  if (rawY >> 19)
    rawY -= 1048576;
  rawZ = ((int32_t)hadxl355->tempRx[6]) << 12 | ((int32_t)hadxl355->tempRx[7]) << 4 | ((int32_t)hadxl355->tempRx[8]) << 4;
  if (rawZ >> 19)
    rawZ -= 1048576;
  if (hadxl355->range == ADXL355_MEASURE_RANGE_2G)
  {
    hadxl355->xAcc.f = (float)rawX * 0.0000039f * ADXL355_GRAVITATIONAL_ACCELERATION;
    hadxl355->yAcc.f = (float)rawY * 0.0000039f * ADXL355_GRAVITATIONAL_ACCELERATION;
    hadxl355->zAcc.f = (float)rawZ * 0.0000039f * ADXL355_GRAVITATIONAL_ACCELERATION;
  }
  else if (hadxl355->range == ADXL355_MEASURE_RANGE_4G)
  {
    hadxl355->xAcc.f = ((float)rawX) * 0.0000078f * ADXL355_GRAVITATIONAL_ACCELERATION;
    hadxl355->yAcc.f = (float)rawY * 0.0000078f * ADXL355_GRAVITATIONAL_ACCELERATION;
    hadxl355->zAcc.f = (float)rawZ * 0.0000078f * ADXL355_GRAVITATIONAL_ACCELERATION;
  }
  else if (hadxl355->range == ADXL355_MEASURE_RANGE_8G)
  {
    hadxl355->xAcc.f = ((float)rawX) * 0.00000156f * ADXL355_GRAVITATIONAL_ACCELERATION;
    hadxl355->yAcc.f = (float)rawY * 0.00000156f * ADXL355_GRAVITATIONAL_ACCELERATION;
    hadxl355->zAcc.f = (float)rawZ * 0.00000156f * ADXL355_GRAVITATIONAL_ACCELERATION;
  }
  
  hadxl355->norm.f = powf(hadxl355->xAcc.f, 2.0f) + pow(hadxl355->yAcc.f, 2.0f) + pow(hadxl355->zAcc.f, 2.0f);
  hadxl355->norm.f = sqrtf(hadxl355->norm.f);
}

void ADXL355SPI_EnterSelfTestMode(ADXL355Handle* hadxl355)
{
  uint8_t tempTx = 0x03;
  ADXL355SPI_WriteRegisters(hadxl355, ADXL355_REG_SELF_TEST, &tempTx, 1);
}

void ADXL355SPI_ExitSelfTestMode(ADXL355Handle* hadxl355)
{
  uint8_t tempTx = 0x00;
  ADXL355SPI_WriteRegisters(hadxl355, ADXL355_REG_SELF_TEST, &tempTx, 1);
}

void ADXL355SPI_EnterStandbyMode(ADXL355Handle* hadxl355)
{
  uint8_t tempTx = 0x00;
  ADXL355SPI_WriteRegisters(hadxl355, ADXL355_REG_POWER_CTL, &tempTx, 1);
}

void ADXL355SPI_EnterMeasurementMode(ADXL355Handle* hadxl355)
{
  uint8_t tempTx = 0x01;
  ADXL355SPI_WriteRegisters(hadxl355, ADXL355_REG_POWER_CTL, &tempTx, 1);
}

void ADXL355SPI_SetI2CSpeed_InterruptPolarity_AccelerationRange(ADXL355Handle* hadxl355, uint8_t i2c_speed, uint8_t polarity, uint8_t range)
{
  uint8_t tempTx = i2c_speed | polarity | range;
  ADXL355SPI_WriteRegisters(hadxl355, ADXL355_REG_Range, &tempTx, 1);
}

void ADXL355SPI_Reset(ADXL355Handle* hadxl355)
{
  uint8_t tempTx = 0x52;
  ADXL355SPI_WriteRegisters(hadxl355, ADXL355_REG_Reset, &tempTx, 1);
}

void ADXL355SPI_ConfigureFilters(ADXL355Handle* hadxl355, uint8_t high_pass_filter_corner, \
     uint8_t output_data_rate_low_pass_filter)
{
  uint8_t tempTx = high_pass_filter_corner | output_data_rate_low_pass_filter;
  ADXL355SPI_WriteRegisters(hadxl355, ADXL355_REG_Filter, &tempTx, 1);
}
