#ifndef BME680_HAL_WRAPPER_H
#define BME680_HAL_WRAPPER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/sensors/bme680/bme680.h"
#include "stm32h7rs_hal_i2c.h"

#include "console.h"

#define BME680_I2C_ADDR_PRIMARY   0x76
#define BME680_I2C_ADDR_SECONDARY 0x77

typedef struct {
    float   temperature;
    float   pressure;
    float   humidity;
    float   gas_resistance;
    uint8_t status;
} BME680_SensorData;

// Sensor state machine states
typedef enum {
    BME680_STATE_IDLE = 0,
    BME680_STATE_TRIGGER_MEASUREMENT,
    BME680_STATE_WAITING_MEASUREMENT,
    BME680_STATE_READING_DATA,
    BME680_STATE_DATA_READY,
    BME680_STATE_ERROR
} BME680_StateTypeDef;

// Sensor handle with state tracking
typedef struct {
    struct bme68x_dev   bme_dev;
    I2C_Handle         *hi2c;
    uint8_t             i2c_addr;
    BME680_StateTypeDef state;
    uint32_t            measurement_start_tick;
    uint32_t            measurement_duration_ms;
    BME680_SensorData   data;
    int8_t              error_code;
    bool                use_gas_sensor;
} BME680_Handle;

bool BME680_Sensor_Init(void);
void BME680_Sensor_Task(void);
void BME680_Read_Trigger(void);

/**
 * @brief Initialize BME68x sensor with interrupt-based I2C
 */
int8_t BME680_HAL_Init(BME680_Handle *handle, I2C_Handle *hi2c, uint8_t i2c_addr);

/**
 * @brief Configure sensor for basic TPH readings
 */
int8_t BME680_HAL_ConfigureBasic(BME680_Handle *handle);

/**
 * @brief Start a non-blocking sensor measurement
 *
 * Call this to trigger a measurement. Then call BME680_HAL_Process()
 * periodically to handle the state machine.
 *
 * @param handle Sensor handle
 * @param with_gas true to include gas sensor reading
 * @return int8_t BME680_OK if measurement started successfully
 */
int8_t BME680_HAL_StartMeasurement(BME680_Handle *handle, bool with_gas);

/**
 * @brief Process sensor state machine (call periodically, e.g. 10ms)
 *
 * This advances the sensor through measurement states without blocking.
 *
 * @param handle Sensor handle
 * @return BME680_StateTypeDef Current state
 */
BME680_StateTypeDef BME680_HAL_Process(BME680_Handle *handle);

/**
 * @brief Get sensor data (only valid when state is DATA_READY)
 *
 * @param handle Sensor handle
 * @param data Output structure for sensor readings
 * @return int8_t BME680_OK if data is valid
 */
int8_t BME680_HAL_GetData(BME680_Handle *handle, BME680_SensorData *data);

/**
 * @brief Get current state
 */
BME680_StateTypeDef BME680_HAL_GetState(BME680_Handle *handle);

/**
 * @brief Check if sensor is busy
 */
bool BME680_HAL_IsBusy(BME680_Handle *handle);

/**
 * @brief Callback for I2C completion (call from HAL_I2C_MemTxCpltCallback)
 */
void BME680_HAL_I2C_TxCpltCallback(I2C_Handle *handle);

/**
 * @brief Callback for I2C completion (call from HAL_I2C_MemRxCpltCallback)
 */
void BME680_HAL_I2C_RxCpltCallback(I2C_Handle *handle);

void BME680_HAL_I2C_ErrorCallback(I2C_Handle *handle);

#endif
