#ifndef ANALOG_DEVICES_ADXL355_H
#define ANALOG_DEVICES_ADXL355_H

#include "main.h"

#ifdef USE_STM32F1_SERIES
#include "stm32f1xx_hal.h"
#endif
#ifdef USE_STM32F4_SERIES
#include "stm32f4xx_hal.h"
#endif

#include "common.h"
#include <math.h>

/** @defgroup ADXL355_Gravitational_Acceleration Local gravitational acceleration value in m/s2
  * @{
  */
#define ADXL355_GRAVITATIONAL_ACCELERATION   9.781f   /*!< Gravitational acceleration in Singapore */

/** @defgroup ADXL355_Register_Address Register addresses of ADXL355
  * @{
  */
#define     ADXL355_REG_DEVID_AD      0x00
#define     ADXL355_REG_DEVID_MST     0x01
#define     ADXL355_REG_PARTID        0x02
#define     ADXL355_REG_REVID         0x03
#define     ADXL355_REG_Status        0x04
#define     ADXL355_REG_FIFO_ENTRIES  0x05
#define     ADXL355_REG_TEMP2         0x06
#define     ADXL355_REG_TEMP1         0x07
#define     ADXL355_REG_XDATA3        0x08
#define     ADXL355_REG_XDATA2        0x09
#define     ADXL355_REG_XDATA1        0x0A
#define     ADXL355_REG_YDATA3        0x0B
#define     ADXL355_REG_YDATA2        0x0C
#define     ADXL355_REG_YDATA1        0x0D
#define     ADXL355_REG_ZDATA3        0x0E
#define     ADXL355_REG_ZDATA2        0x0F
#define     ADXL355_REG_ZDATA1        0x10
#define     ADXL355_REG_FIFO_DATA     0x11
#define     ADXL355_REG_OFFSET_X_H    0x1E
#define     ADXL355_REG_OFFSET_X_L    0x1F
#define     ADXL355_REG_OFFSET_Y_H    0x20
#define     ADXL355_REG_0FFSET_Y_L    0x21
#define     ADXL355_REG_OFFSET_Z_H    0x22
#define     ADXL355_REG_OFFSET_Z_L    0x23
#define     ADXL355_REG_ACT_EN        0x24
#define     ADXL355_REG_ACT_THRESH_H  0x25
#define     ADXL355_REG_ACT_THRESH_L  0x26
#define     ADXL355_REG_ACT_COUNT     0x27
#define     ADXL355_REG_Filter        0x28
#define     ADXL355_REG_FIFO_SAMPLES  0x29
#define     ADXL355_REG_INT_MAP       0x2A
#define     ADXL355_REG_Sync          0x2B
#define     ADXL355_REG_Range         0x2C
#define     ADXL355_REG_POWER_CTL     0x2D
#define     ADXL355_REG_SELF_TEST     0x2E
#define     ADXL355_REG_Reset         0x2F

#define     ADXL355_I2C_HS_HIGH_SPEED 0x80
#define     ADXL355_I2C_HS_FAST_MODE  0x00
#define     ADXL355_INT_POL_LOW       0x00
#define     ADXL355_INT_POL_HIGH      0x40
#define     ADXL355_MEASURE_RANGE_2G  0x01
#define     ADXL355_MEASURE_RANGE_4G  0x02
#define     ADXL355_MEASURE_RANGE_8G  0x03

#define     ADXL355_HPF_CORNER_NONE   0x00
#define     ADXL355_HPF_CORNER_1      0x10
#define     ADXL355_HPF_CORNER_2      0x20
#define     ADXL355_HPF_CORNER_3      0x30
#define     ADXL355_HPF_CORNER_4      0x40
#define     ADXL355_HPF_CORNER_5      0x50
#define     ADXL355_HPF_CORNER_6      0x60
#define     ADXL355_ODR_LPF_CORNER_4000HZ_1000HZ    0x00
#define     ADXL355_ODR_LPF_CORNER_2000HZ_500HZ     0x01
#define     ADXL355_ODR_LPF_CORNER_1000HZ_250HZ     0x02
#define     ADXL355_ODR_LPF_CORNER_500HZ_125HZ      0x03
#define     ADXL355_ODR_LPF_CORNER_250HZ_62P5HZ     0x04
#define     ADXL355_ODR_LPF_CORNER_125HZ_31P25HZ    0x05
#define     ADXL355_ODR_LPF_CORNER_62P5HZ_15P625HZ  0x06
#define     ADXL355_ODR_LPF_CORNER_31P25HZ_7P813HZ  0x07
#define     ADXL355_ODR_LPF_CORNER_15P625HZ_3P906HZ 0x08
#define     ADXL355_ODR_LPF_CORNER_7P813HZ_1P953HZ  0x09
#define     ADXL355_ODR_LPF_CORNER_3P906HZ_0P977HZ  0x0A

#define ADXL355_CS_Pin GPIO_PIN_4
#define ADXL355_CS_GPIO_Port GPIOA

typedef struct
{
  uint8_t status;
}ADXL355RegisterHandle;

typedef struct
{
  /* For using SPI communication */
  SPI_HandleTypeDef*        hspi;
  GPIO_TypeDef*             cs_port;
  uint32_t                  cs_pin;
  /* For using I2C communication */
  I2C_HandleTypeDef*        hi2c;
  uint16_t                  i2cAddr;
  /////////////////////////////////
  ADXL355RegisterHandle     registers;
  uint8_t                   range;
  uint8_t                   ifNewDataReady;
  uint8_t                   tempRx[9];
  union FloatUInt8          xAcc, yAcc, zAcc, norm;
}ADXL355Handle;

void ADXL355I2C_Init(ADXL355Handle* hadxl355, I2C_HandleTypeDef* hi2c, \
                  uint8_t ASEL_PIN_LEVEL);

void ADXL355SPI_Init(ADXL355Handle* hadxl355, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint32_t cs_pin);

void ADXL355SPI_ReadRegisters(ADXL355Handle* hadxl355, uint8_t start_register, uint8_t num);

void ADXL355SPI_WriteRegisters(ADXL355Handle* hadxl355, uint8_t start_register, uint8_t* txbuf, uint8_t len);

void ADXL355SPI_ReadAccelerations(ADXL355Handle* hadxl355);

void ADXL355SPI_EnterSelfTestMode(ADXL355Handle* hadxl355);

void ADXL355SPI_ExitSelfTestMode(ADXL355Handle* hadxl355);

void ADXL355SPI_EnterStandbyMode(ADXL355Handle* hadxl355);

void ADXL355SPI_EnterMeasurementMode(ADXL355Handle* hadxl355);

void ADXL355SPI_SetI2CSpeed_InterruptPolarity_AccelerationRange\
     (ADXL355Handle* hadxl355, uint8_t i2c_speed, uint8_t polarity, uint8_t range);

void ADXL355SPI_Reset(ADXL355Handle* hadxl355);

void ADXL355SPI_ConfigureFilters(ADXL355Handle* hadxl355, uint8_t high_pass_filter_corner, \
     uint8_t output_data_rate_low_pass_filter);
#endif
