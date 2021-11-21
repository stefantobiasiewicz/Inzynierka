#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "pca10056/blank/config/sdk_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_internal.h"

#include "lib/bme680/bme68x.h"
#include "lib/bme680/bme68x_defs.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static struct bme68x_dev bme;
static uint8_t addres = 0x77;
static struct bme68x_conf bme_conf;
static struct bme68x_heatr_conf bme_heatr_conf;
static struct bme68x_data bme_data;

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_config = {
       .scl                = NRF_GPIO_PIN_MAP(0,4),
       .sda                = NRF_GPIO_PIN_MAP(0,29),
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void error_handler(){
    while(1);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:
            NRF_LOG_DEBUG("API name [%s] status OK", api_name);
            NRF_LOG_FLUSH();
            break;
        case BME68X_E_NULL_PTR:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Null pointer", api_name, rslt);
            NRF_LOG_FLUSH();
            error_handler();
            break;
        case BME68X_E_COM_FAIL:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Communication failure", api_name, rslt);
            NRF_LOG_FLUSH();
            error_handler();
            break;
        case BME68X_E_INVALID_LENGTH:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Incorrect length parameter", api_name, rslt);
            NRF_LOG_FLUSH();
            error_handler();
            break;
        case BME68X_E_DEV_NOT_FOUND:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Device not found", api_name, rslt);
            NRF_LOG_FLUSH();
            error_handler();
            break;
        case BME68X_E_SELF_TEST:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Self test error", api_name, rslt);
            NRF_LOG_FLUSH();
            error_handler();
            break;
        case BME68X_W_NO_NEW_DATA:
            NRF_LOG_ERROR("API name [%s]  Warning [%d] : No new data found", api_name, rslt);
            NRF_LOG_FLUSH();
            error_handler();
            break;
        default:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Unknown error code", api_name, rslt);
            NRF_LOG_FLUSH();
            error_handler();
            break;
    }
}

BME68X_INTF_RET_TYPE bme680_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    ret_code_t err_code;
    uint8_t reg = reg_addr;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    err_code = nrf_drv_twi_tx(&m_twi,dev_addr,&reg,1,true);
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("bme680_i2c_read -> nrf_drv_twi_tx fails");
        NRF_LOG_FLUSH();
        return BME68X_E_COM_FAIL;
    }
    err_code = nrf_drv_twi_rx(&m_twi,dev_addr,reg_data,len);
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("bme680_i2c_read -> nrf_drv_twi_rx fails");
        NRF_LOG_FLUSH();
        return BME68X_E_COM_FAIL;
    }
    NRF_LOG_DEBUG("TWI read form regisrer %u, 0x%x, len %u", reg_addr, reg_addr, len);
    NRF_LOG_HEXDUMP_DEBUG(reg_data,len);
    NRF_LOG_FLUSH();
    return BME68X_INTF_RET_SUCCESS;
}


BME68X_INTF_RET_TYPE bme680_i2c_write_single_register(uint8_t reg_addr, const uint8_t data, void *intf_ptr){
    ret_code_t err_code;
    uint8_t data_write[2];
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    data_write[0] = reg_addr;
    data_write[1] = data;
        
    err_code = nrf_drv_twi_tx(&m_twi, dev_addr, data_write, sizeof(data_write), false);  
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("bme680_i2c_write -> nrf_drv_twi_tx fails");
        NRF_LOG_FLUSH();
        return BME68X_E_COM_FAIL;
    }
    return BME68X_INTF_RET_SUCCESS;
}

BME68X_INTF_RET_TYPE bme680_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{  
    for(uint8_t i= 0; i< len; i++){
        if(bme680_i2c_write_single_register(reg_addr,reg_data[i],intf_ptr) != BME68X_INTF_RET_SUCCESS) 
            return BME68X_E_COM_FAIL;
    }

    NRF_LOG_DEBUG("TWI write to register %u, 0x%x, len %u", reg_addr, reg_addr, len);
    NRF_LOG_HEXDUMP_DEBUG(reg_data,len);
    NRF_LOG_FLUSH();
    return BME68X_INTF_RET_SUCCESS; 
}

void bme680_wait_us (uint32_t period, void *intf_ptr)
{
    nrf_delay_us(period);
}

void bme680_init(){
    bme.intf = BME68X_I2C_INTF;
    bme.intf_ptr = &addres;
    bme.read = bme680_i2c_read;
    bme.write = bme680_i2c_write;
    bme.delay_us = bme680_wait_us;
    bme.amb_temp = 25;
    NRF_LOG_INFO("BME680 initialize stared")
    uint8_t ret = bme68x_init(&bme);
    bme68x_check_rslt("bme680_init", ret);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint8_t ret;
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_WARNING("BME680 Application started");
    NRF_LOG_FLUSH();
    twi_init();

    bme680_init();
    nrf_delay_ms(10);
    
    ret = bme68x_set_op_mode(BME68X_FORCED_MODE,&bme);
    ret = bme68x_set_op_mode(BME68X_FORCED_MODE,&bme);
    bme68x_check_rslt("bme68x_set_op_mode", ret);
    

    bme_conf.filter = BME68X_FILTER_OFF;
    bme_conf.odr = BME68X_ODR_NONE;
    bme_conf.os_hum = BME68X_OS_16X;
    bme_conf.os_pres = BME68X_OS_1X;
    bme_conf.os_temp = BME68X_OS_2X;
    ret = bme68x_set_conf(&bme_conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", ret);

   NRF_LOG_INFO("bme_conf.filter: %u\n" \
                "cbme_confonf.odr: %u\n"\
                "bme_conf.os_hum: %u\n" \
                "bme_conf.os_pres: %u\n" \
                "bme_conf.os_temp: %u\n",
                bme_conf.filter,
                bme_conf.odr,
                bme_conf.os_hum,
                bme_conf.os_pres,
                bme_conf.os_temp)

    struct bme68x_conf conf;
    ret = bme68x_get_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_get_conf", ret);

    NRF_LOG_INFO("conf.filter: %u\n" \
                "conf.odr: %u\n"\
                "conf.os_hum: %u\n" \
                "conf.os_pres: %u\n" \
                "conf.os_temp: %u",
                conf.filter,
                conf.odr,
                conf.os_hum,
                conf.os_pres,
                conf.os_temp)
    NRF_LOG_FLUSH();

    bme_heatr_conf.enable = BME68X_ENABLE;
    bme_heatr_conf.heatr_temp = 300;
    bme_heatr_conf.heatr_dur = 100;
    ret = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme_heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", ret);

    NRF_LOG_INFO("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
    NRF_LOG_FLUSH();

    uint32_t del_period;
    uint8_t n_fields;

    for(uint8_t i = 0; i < 3; i++){
        ret = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        bme68x_check_rslt("bme68x_set_op_mode", ret);

        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme_conf, &bme) + (bme_heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);
        NRF_LOG_INFO("BME680 delay period = %dus",del_period);
        NRF_LOG_FLUSH();

        ret = bme68x_get_data(BME68X_FORCED_MODE, &bme_data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", ret);

        NRF_LOG_INFO("temperature: " NRF_LOG_FLOAT_MARKER,
            NRF_LOG_FLOAT(bme_data.temperature));
        NRF_LOG_INFO("pressure: " NRF_LOG_FLOAT_MARKER,
            NRF_LOG_FLOAT(bme_data.pressure));
        NRF_LOG_INFO("humidity: " NRF_LOG_FLOAT_MARKER,
            NRF_LOG_FLOAT(bme_data.humidity));
        NRF_LOG_INFO("gas_resistance: " NRF_LOG_FLOAT_MARKER,
            NRF_LOG_FLOAT(bme_data.gas_resistance));
        NRF_LOG_INFO("status: 0x%x\n",
            bme_data.status);

        NRF_LOG_FLUSH();
    }

    while (true)
    {
       
    }
}

/** @} */
