#include "bme680_sensor.h"
#include "system.h"
#include <string.h>

// I2C operation tracking
static volatile bool i2c_tx_complete = false;
static volatile bool i2c_rx_complete = false;
static volatile bool i2c_error       = false;

BME680_Handle bme680_sensor_handle;
I2C_Handle    i2c1_handler;

bool BME680_Sensor_Init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure PB8 and PB9 as I2C */
    GPIO_Init i2c1_gpio;
    i2c1_gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9; // PB8=SCL, PB9=SDA
    i2c1_gpio.Mode      = GPIO_MODE_ALT_FUNC_OD;   // Alternate function, open-drain
    i2c1_gpio.Pull      = GPIO_NOPULL_UP;          // No internal pull-ups (external on BME680 board)
    i2c1_gpio.Speed     = GPIO_SPEED_FREQ_HIGH;    // High speed
    i2c1_gpio.Alternate = ((uint8_t) 0x04);        // AF4 = I2C1
    HAL_GPIO_Init(GPIOB, &i2c1_gpio);

    __HAL_RCC_I2C1_CLK_ENABLE();

    /* Configure I2C1 Handler */
    i2c1_handler.Instance              = I2C1;
    i2c1_handler.Init.Timing           = 0x10707DBC; // 100kHz Standard Mode
    i2c1_handler.Init.OwnAddress1      = 0;
    i2c1_handler.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    i2c1_handler.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    i2c1_handler.Init.OwnAddress2      = 0;
    i2c1_handler.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c1_handler.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    i2c1_handler.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    i2c1_handler.masterTxCpltCallback = BME680_HAL_I2C_TxCpltCallback;
    i2c1_handler.masterRxCpltCallback = BME680_HAL_I2C_RxCpltCallback;

    if (HAL_I2C_Init(&i2c1_handler) != HAL_OK) {
        Error_Handler();
    }

    // 4. Enable I2C interrupts in NVIC
    __NVIC_SetPriority(I2C1_EV_IRQn, 5);
    __NVIC_EnableIRQ(I2C1_EV_IRQn);

    __NVIC_SetPriority(I2C1_ER_IRQn, 5);
    __NVIC_EnableIRQ(I2C1_ER_IRQn);

    if (BME680_HAL_Init(&bme680_sensor_handle, &i2c1_handler, BME680_I2C_ADDR_PRIMARY) != BME680_OK) {
        PRINT_ERROR("BME680 Sensor init failed");
        Error_Handler();
    }

    BME680_HAL_ConfigureBasic(&bme680_sensor_handle);
    PRINT_INFO("BME680 initialized");

    return true;
}

void I2C1_EV_IRHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c1_handler);
}

void I2C1_ER_IRHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c1_handler);
}

void BME680_HAL_I2C_TxCpltCallback(I2C_Handle *handle)
{
    i2c_tx_complete = true;
}

void BME680_HAL_I2C_RxCpltCallback(I2C_Handle *handle)
{
    i2c_rx_complete = true;
}

void BME680_Sensor_Task()
{
    BME680_StateTypeDef state = BME680_HAL_Process(&bme680_sensor_handle);
    BME680_SensorData   data;

    if (state == BME680_STATE_DATA_READY) {
        if (BME680_HAL_GetData(&bme680_sensor_handle, &data) == BME680_OK) {
            PRINT_INFO("Temp: %.2f°C  Press: %.2f hPa  Hum: %.2f%%", data.temperature, data.pressure, data.humidity);
        }
    }
}

void BME680_Read_Trigger()
{
    if (!BME680_HAL_IsBusy(&bme680_sensor_handle)) {
        BME680_HAL_StartMeasurement(&bme680_sensor_handle, false); // false = no gas
    }
}

static BME680_INTF_RET_TYPE bme68x_i2c_read_blocking_it(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    BME680_Handle *handle = (BME680_Handle *) intf_ptr;

    i2c_rx_complete = false;
    i2c_error       = false;

    if (HAL_I2C_Mem_Read_IT(handle->hi2c, handle->i2c_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len) != HAL_OK) {
        return BME680_E_COM_FAIL;
    }

    // Wait for completion (init only happens once, so acceptable)
    uint32_t timeout = HAL_GetTick() + 100;
    while (!i2c_rx_complete && !i2c_error) {
        if (HAL_GetTick() > timeout) {
            return BME680_E_COM_FAIL;
        }
        // Could call your scheduler here if needed
    }

    return i2c_error ? BME680_E_COM_FAIL : BME680_OK;
}

static BME680_INTF_RET_TYPE bme68x_i2c_write_blocking_it(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    BME680_Handle *handle = (BME680_Handle *) intf_ptr;

    i2c_tx_complete = false;
    i2c_error       = false;

    if (HAL_I2C_Mem_Write_IT(handle->hi2c, handle->i2c_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *) reg_data, len) != HAL_OK) {
        return BME680_E_COM_FAIL;
    }

    uint32_t timeout = HAL_GetTick() + 100;
    while (!i2c_tx_complete && !i2c_error) {
        if (HAL_GetTick() > timeout) {
            return BME680_E_COM_FAIL;
        }
    }

    return i2c_error ? BME680_E_COM_FAIL : BME680_OK;
}

/**
 * @brief Non-blocking I2C read (used during measurements)
 */
static int8_t bme68x_i2c_read_async(BME680_Handle *handle, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    i2c_rx_complete = false;
    i2c_error       = false;

    if (HAL_I2C_Mem_Read_IT(handle->hi2c, handle->i2c_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len) == HAL_OK) {
        return BME680_OK;
    }
    return BME680_E_COM_FAIL;
}

/**
 * @brief Non-blocking I2C write (used during measurements)
 */
static int8_t bme68x_i2c_write_async(BME680_Handle *handle, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{
    i2c_tx_complete = false;
    i2c_error       = false;

    if (HAL_I2C_Mem_Write_IT(handle->hi2c, handle->i2c_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *) reg_data, len) == HAL_OK) {
        return BME680_OK;
    }
    return BME680_E_COM_FAIL;
}

static void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t delay_ms = (period / 1000) + ((period % 1000) ? 1 : 0);
    if (delay_ms < 1)
        delay_ms = 1;
    HAL_DelayMS(delay_ms);
}

int8_t BME680_HAL_Init(BME680_Handle *handle, I2C_Handle *hi2c, uint8_t i2c_addr)
{
    int8_t result;

    memset(handle, 0, sizeof(BME680_Handle));

    handle->hi2c     = hi2c;
    handle->i2c_addr = i2c_addr;
    handle->state    = BME680_STATE_IDLE;

    // Use blocking I2C for initialization (one-time setup)
    handle->bme_dev.read     = bme68x_i2c_read_blocking_it;
    handle->bme_dev.write    = bme68x_i2c_write_blocking_it;
    handle->bme_dev.delay_us = bme68x_delay_us;
    handle->bme_dev.intf     = BME680_I2C_INTF;
    handle->bme_dev.intf_ptr = handle; // Pass handle, not just I2C
    handle->bme_dev.amb_temp = 25;

    result = bme68x_init(&handle->bme_dev);
    if (result != BME680_OK) {
        handle->state = BME680_STATE_ERROR;
        return result;
    }

    return BME680_OK;
}

int8_t BME680_HAL_ConfigureBasic(BME680_Handle *handle)
{
    int8_t             result;
    struct bme68x_conf conf;

    result = bme68x_get_conf(&conf, &handle->bme_dev);
    if (result != BME680_OK)
        return result;

    conf.filter  = BME680_FILTER_OFF;
    conf.odr     = BME680_ODR_NONE;
    conf.os_hum  = BME680_OS_2X;
    conf.os_pres = BME680_OS_4X;
    conf.os_temp = BME680_OS_8X;

    result = bme68x_set_conf(&conf, &handle->bme_dev);
    return result;
}

int8_t BME680_HAL_StartMeasurement(BME680_Handle *handle, bool with_gas)
{
    if (handle->state != BME680_STATE_IDLE) {
        return BME680_E_DEV_NOT_FOUND; // Busy
    }

    handle->use_gas_sensor = with_gas;
    handle->state          = BME680_STATE_TRIGGER_MEASUREMENT;

    return BME680_OK;
}

BME680_StateTypeDef BME680_HAL_Process(BME680_Handle *handle)
{
    int8_t                    result;
    uint8_t                   mode_reg_val = BME680_FORCED_MODE;
    struct bme68x_heatr_conf  heatr_conf;
    static struct bme68x_data sensor_data[3];
    static uint8_t            n_fields;

    switch (handle->state) {
    case BME680_STATE_IDLE:
        // Nothing to do
        break;

    case BME680_STATE_TRIGGER_MEASUREMENT:
        // Configure gas heater if needed
        if (handle->use_gas_sensor) {
            heatr_conf.enable     = BME680_ENABLE;
            heatr_conf.heatr_temp = 300;
            heatr_conf.heatr_dur  = 100;
            result                = bme68x_set_heatr_conf(BME680_FORCED_MODE, &heatr_conf, &handle->bme_dev);
            if (result != BME680_OK) {
                handle->error_code = result;
                handle->state      = BME680_STATE_ERROR;
                break;
            }
            // Measurement duration includes heater time
            handle->measurement_duration_ms = 100 + 100; // ~200ms total
        } else {
            // TPH only, much faster
            handle->measurement_duration_ms = 50; // ~50ms for TPH
        }

        // Trigger forced mode via async I2C write
        result = bme68x_i2c_write_async(handle, BME680_REG_CTRL_MEAS, &mode_reg_val, 1);
        if (result == BME680_OK) {
            handle->measurement_start_tick = HAL_GetTick();
            handle->state                  = BME680_STATE_WAITING_MEASUREMENT;
        } else {
            handle->error_code = result;
            handle->state      = BME680_STATE_ERROR;
        }
        break;

    case BME680_STATE_WAITING_MEASUREMENT:
        // Wait for I2C write to complete
        if (i2c_tx_complete) {
            // Check if measurement time has elapsed
            if ((HAL_GetTick() - handle->measurement_start_tick) >= handle->measurement_duration_ms) {
                handle->state = BME680_STATE_READING_DATA;
            }
        } else if (i2c_error) {
            handle->state = BME680_STATE_ERROR;
        }
        break;

    case BME680_STATE_READING_DATA:
        // Read sensor data using blocking API (Bosch API limitation)
        // This is fast (<5ms) so acceptable
        result = bme68x_get_data(BME680_FORCED_MODE, sensor_data, &n_fields, &handle->bme_dev);

        if (result == BME680_OK && n_fields > 0) {
            handle->data.temperature    = sensor_data[0].temperature;
            handle->data.pressure       = sensor_data[0].pressure / 100.0f;
            handle->data.humidity       = sensor_data[0].humidity;
            handle->data.gas_resistance = sensor_data[0].gas_resistance / 1000.0f;
            handle->data.status         = sensor_data[0].status;
            handle->state               = BME680_STATE_DATA_READY;
        } else {
            handle->error_code = result;
            handle->state      = BME680_STATE_ERROR;
        }
        break;

    case BME680_STATE_DATA_READY:
        // Data has been retrieved, waiting for app to read
        break;

    case BME680_STATE_ERROR:
        // Error state, waiting for reset
        break;
    }

    return handle->state;
}

int8_t BME680_HAL_GetData(BME680_Handle *handle, BME680_SensorData *data)
{
    if (handle->state != BME680_STATE_DATA_READY) {
        return BME680_W_NO_NEW_DATA;
    }

    memcpy(data, &handle->data, sizeof(BME680_SensorData));
    handle->state = BME680_STATE_IDLE; // Ready for next measurement

    return BME680_OK;
}

BME680_StateTypeDef BME680_HAL_GetState(BME680_Handle *handle)
{
    return handle->state;
}

bool BME680_HAL_IsBusy(BME680_Handle *handle)
{
    return (handle->state != BME680_STATE_IDLE && handle->state != BME680_STATE_DATA_READY && handle->state != BME680_STATE_ERROR);
}
