#ifndef LC709203F_H
#define LC709203F_H

#include "stm32l4xx_hal.h"

#define I2CHandler hi2c3
extern I2C_HandleTypeDef hi2c3;

#define BATT_I2C_TIMEOUT 100

#define BATT_I2C_DEV_ADDR 0x0b << 1

#define LC709203F_ADDR_SIZE 1
#define LC709203F_REG_SIZE 2

#define LC709203F_IC_VERSION 0x2AFF


#define LC709203F_I2CADDR_DEFAULT 0x0B     ///< LC709203F default i2c address
#define LC709203F_CMD_THERMISTORB 0x06     ///< Read/write thermistor B
#define LC709203F_CMD_INITRSOC 0x07        ///< Initialize RSOC calculation
#define LC709203F_CMD_CELLTEMPERATURE 0x08 ///< Read/write batt temperature
#define LC709203F_CMD_CELLVOLTAGE 0x09     ///< Read batt voltage
#define LC709203F_CMD_APA 0x0B             ///< Adjustment Pack Application
#define LC709203F_CMD_RSOC 0x0D            ///< Read state of charge
#define LC709203F_CMD_CELLITE 0x0F         ///< Read batt indicator to empty
#define LC709203F_CMD_ICVERSION 0x11       ///< Read IC version
#define LC709203F_CMD_BATTPROF 0x12        ///< Set the battery profile
#define LC709203F_CMD_ALARMRSOC 0x13       ///< Alarm on percent threshold
#define LC709203F_CMD_ALARMVOLT 0x14       ///< Alarm on voltage threshold
#define LC709203F_CMD_POWERMODE 0x15       ///< Sets sleep/power mode
#define LC709203F_CMD_STATUSBIT 0x16       ///< Temperature obtaining method
#define LC709203F_CMD_PARAMETER 0x1A       ///< Batt profile code

/*!  Battery temperature source */
typedef enum {
  LC709203F_TEMPERATURE_I2C = 0x0000,
  LC709203F_TEMPERATURE_THERMISTOR = 0x0001,
} lc709203_tempmode_t;

/*!  Chip power state */
typedef enum {
  LC709203F_POWER_OPERATE = 0x0001,
  LC709203F_POWER_SLEEP = 0x0002,
} lc709203_powermode_t;

/*!  Approx battery pack size */
typedef enum {
  LC709203F_APA_100MAH = 0x08,
  LC709203F_APA_200MAH = 0x0B,
  LC709203F_APA_500MAH = 0x10,
  LC709203F_APA_1000MAH = 0x19,
  LC709203F_APA_2000MAH = 0x2D,
  LC709203F_APA_3000MAH = 0x36,
} lc709203_adjustment_t;

typedef enum {BATT_OK, BATT_ERR} LC709203F_status;

LC709203F_status LC709203F_init();
uint16_t LC709203F_get_ic_version();
void LC709203F_set_power_mode(lc709203_powermode_t val);
void LC709203F_set_pack_size(lc709203_adjustment_t val);
void LC709203F_set_profile(uint16_t val);
void LC709203F_send_temp_mode(lc709203_tempmode_t val);
float LC709203F_get_percent();

#endif
