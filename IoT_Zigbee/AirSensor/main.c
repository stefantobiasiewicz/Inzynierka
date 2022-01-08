
#include "zboss_api.h"
#include "zb_mem_config_med.h"
#include "zb_error_handler.h"
#include "zigbee_helpers.h"
#include "app_timer.h"
#include "bsp.h"
#include "boards.h"
#include "sensorsim.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "zb_multi_sensor.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "lib/bme680/bme68x.h"
#include "lib/bme680/bme68x_defs.h"


#define IEEE_CHANNEL_MASK                  (1l << 11)               /**< Scan only one, predefined channel to find the coordinator. */
#define ERASE_PERSISTENT_CONFIG            ZB_FALSE                             /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define ZIGBEE_NETWORK_STATE_LED           BSP_BOARD_LED_2                      /**< LED indicating that light switch successfully joind Zigbee network. */

#define MIN_TEMPERATURE_VALUE              0                                    /**< Minimum temperature value as returned by the simulated measurement function. */
#define MAX_TEMPERATURE_VALUE              4000                                 /**< Maximum temperature value as returned by the simulated measurement function. */
#define TEMPERATURE_VALUE_INCREMENT        50                                   /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */
#define MIN_PRESSURE_VALUE                 700                                  /**< Minimum pressure value as returned by the simulated measurement function. */
#define MAX_PRESSURE_VALUE                 1100                                 /**< Maximum pressure value as returned by the simulated measurement function. */
#define PRESSURE_VALUE_INCREMENT           5                                    /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile End Device source code.
#endif

#define TWI_INSTANCE_ID     0

#define SCL_TWI_PIN         NRF_GPIO_PIN_MAP(0,4)
#define SDA_TWI_PIN         NRF_GPIO_PIN_MAP(0,29)

#define BOARD_BUTTON_PIN NRF_GPIO_PIN_MAP(0,11)  

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static struct bme68x_dev bme;
static uint8_t addres = 0x77;
static struct bme68x_conf bme_conf;
static struct bme68x_heatr_conf bme_heatr_conf;
static struct bme68x_data bme_data;


static sensor_device_ctx_t m_dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &m_dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
                                     &m_dev_ctx.basic_attr.zcl_version,
                                     &m_dev_ctx.basic_attr.app_version,
                                     &m_dev_ctx.basic_attr.stack_version,
                                     &m_dev_ctx.basic_attr.hw_version,
                                     m_dev_ctx.basic_attr.mf_name,
                                     m_dev_ctx.basic_attr.model_id,
                                     m_dev_ctx.basic_attr.date_code,
                                     &m_dev_ctx.basic_attr.power_source,
                                     m_dev_ctx.basic_attr.location_id,
                                     &m_dev_ctx.basic_attr.ph_env,
                                     m_dev_ctx.basic_attr.sw_ver);


ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(temperature_attr_list, 
                                            &m_dev_ctx.temp_attr.measure_value,
                                            &m_dev_ctx.temp_attr.min_measure_value, 
                                            &m_dev_ctx.temp_attr.max_measure_value, 
                                            &m_dev_ctx.temp_attr.tolerance);

ZB_ZCL_DECLARE_PRES_MEASUREMENT_ATTRIB_LIST(pressure_attr_list, 
                                            &m_dev_ctx.pres_attr.measure_value, 
                                            &m_dev_ctx.pres_attr.min_measure_value, 
                                            &m_dev_ctx.pres_attr.max_measure_value, 
                                            &m_dev_ctx.pres_attr.tolerance);

ZB_ZCL_DECLARE_REL_HUMIDITY_MEASUREMENT_ATTRIB_LIST(humidity_attr_list,
                                                    &m_dev_ctx.humidity_attr.measure_value,
                                                    &m_dev_ctx.humidity_attr.min_measure_value,
                                                    &m_dev_ctx.humidity_attr.max_measure_value);

ZB_DECLARE_MULTI_SENSOR_CLUSTER_LIST(multi_sensor_clusters,
                                     basic_attr_list,
                                     identify_attr_list,
                                     temperature_attr_list,
                                     pressure_attr_list,
                                     humidity_attr_list);

ZB_ZCL_DECLARE_MULTI_SENSOR_EP(multi_sensor_ep,
                               MULTI_SENSOR_ENDPOINT,
                               multi_sensor_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(multi_sensor_ctx, multi_sensor_ep);


static sensorsim_cfg_t   m_temperature_sim_cfg;                                 /**< Temperature sensor simulator configuration. */
static sensorsim_state_t m_temperature_sim_state;                               /**< Temperature sensor simulator state. */
static sensorsim_cfg_t   m_pressure_sim_cfg;                                    /**< Pressure sensor simulator configuration. */
static sensorsim_state_t m_pressure_sim_state;                                  /**< Pressure sensor simulator state. */

APP_TIMER_DEF(zb_app_timer);


void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_TWI_PIN,
       .sda                = SDA_TWI_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
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
            break;
        case BME68X_E_NULL_PTR:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Null pointer", api_name, rslt);
            error_handler();
            break;
        case BME68X_E_COM_FAIL:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Communication failure", api_name, rslt);
            error_handler();
            break;
        case BME68X_E_INVALID_LENGTH:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Incorrect length parameter", api_name, rslt);
            error_handler();
            break;
        case BME68X_E_DEV_NOT_FOUND:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Device not found", api_name, rslt);
            error_handler();
            break;
        case BME68X_E_SELF_TEST:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Self test error", api_name, rslt);
            error_handler();
            break;
        case BME68X_W_NO_NEW_DATA:
            NRF_LOG_ERROR("API name [%s]  Warning [%d] : No new data found", api_name, rslt);
            error_handler();
            break;
        default:
            NRF_LOG_ERROR("API name [%s]  Error [%d] : Unknown error code", api_name, rslt);
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
        return BME68X_E_COM_FAIL;
    }
    err_code = nrf_drv_twi_rx(&m_twi,dev_addr,reg_data,len);
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("bme680_i2c_read -> nrf_drv_twi_rx fails");
        return BME68X_E_COM_FAIL;
    }
    return BME68X_INTF_RET_SUCCESS;
}

BME68X_INTF_RET_TYPE bme680_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{  
    ret_code_t err_code;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    // tu trzeba obejść biblioteke dane juz sa odpowiednio posortowane w tablicy [addres,dane,addres++,dane++]
    err_code = nrf_drv_twi_tx(&m_twi, dev_addr, reg_data-1, len+1, false);  
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("bme680_i2c_write -> nrf_drv_twi_tx fails");
        return BME68X_E_COM_FAIL;
    }
    return BME68X_INTF_RET_SUCCESS; 
}

void bme680_wait_us (uint32_t period, void *intf_ptr)
{
    nrf_delay_us(period);
}

void bme680_init_interface(){
    bme.intf = BME68X_I2C_INTF;
    bme.intf_ptr = &addres;
    bme.read = bme680_i2c_read;
    bme.write = bme680_i2c_write;
    bme.delay_us = bme680_wait_us;
    bme.amb_temp = 25;
}

void bme680_measure(){
    ret_code_t     err_code;
    err_code = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", err_code);

    uint32_t del_period;
    uint8_t n_fields;

    /* Calculate delay period in microseconds */
    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme_conf, &bme) + (bme_heatr_conf.heatr_dur * 1000);
    bme.delay_us(del_period, bme.intf_ptr);
    //NRF_LOG_INFO("BME680 delay period = %dus",del_period);

    err_code = bme68x_get_data(BME68X_FORCED_MODE, &bme_data, &n_fields, &bme);
    bme68x_check_rslt("bme68x_get_data", err_code);
}

static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){

    switch (pin)
    {
    case BOARD_BUTTON_PIN:
        if(action == NRF_GPIOTE_POLARITY_HITOLO){
            NRF_LOG_INFO("Button press");
            //zb_bdb_reset_via_local_action(0);
            NVIC_SystemReset();
        }
        break;
    default:
        NRF_LOG_WARNING("Unknown gpio event pin: %d, action %d", pin, action);
        break;
    }
}

static void gpio_init(void)
{
    ret_code_t err_code;

    // inicjalizacja modułu GPIOTE
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

   // struktura konfiguracyjna wejście - przycisk
    nrf_drv_gpiote_in_config_t button_in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    button_in_config.pull = NRF_GPIO_PIN_PULLUP;
    // inicjalizacja wyścia
    err_code = nrf_drv_gpiote_in_init(BOARD_BUTTON_PIN, &button_in_config, gpio_handler);
    APP_ERROR_CHECK(err_code);
    // właczenie mechanizmu eventow przychodzących od pinu
    nrf_drv_gpiote_in_event_enable(BOARD_BUTTON_PIN, true);

}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing all clusters attributes.
 */
static void multi_sensor_clusters_attr_init(void)
{
    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = SENSOR_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = SENSOR_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = SENSOR_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          SENSOR_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          SENSOR_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          SENSOR_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = SENSOR_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          SENSOR_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = SENSOR_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time        = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

    /* Temperature measurement cluster attributes data */
    m_dev_ctx.temp_attr.measure_value            = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.temp_attr.min_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.temp_attr.max_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.temp_attr.tolerance                = ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_MAX_VALUE;

    /* Pressure measurement cluster attributes data */
    m_dev_ctx.pres_attr.measure_value            = ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.pres_attr.min_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.pres_attr.max_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.pres_attr.tolerance                = ZB_ZCL_ATTR_PRES_MEASUREMENT_TOLERANCE_MAX_VALUE;

    m_dev_ctx.humidity_attr.measure_value        = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.humidity_attr.min_measure_value    = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.humidity_attr.max_measure_value    = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_MAX_VALUE;
}

/**@brief Function for initializing LEDs.
 */
static zb_void_t leds_init(void)
{
    ret_code_t error_code;

    /* Initialize LEDs and buttons - use BSP to control them. */
    error_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(error_code);

    bsp_board_leds_off();
}

/**@brief Function for handling nrf app timer.
 * 
 * @param[IN]   context   Void pointer to context function is called with.
 * 
 * @details Function is called with pointer to sensor_device_ep_ctx_t as argument.
 */
static void zb_app_timer_handler(void * context)
{
    zb_zcl_status_t zcl_status;
    uint16_t new_temp_value, new_pres_value, new_hum_value;
    bme680_measure();
    /* Get new temperature measured value */
    new_temp_value = (zb_int16_t)bme_data.temperature;
    //NRF_LOG_INFO("bme_data.temperature: %d, %x" , new_temp_value, new_temp_value);
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT, 
                                     ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_temp_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set temperature value fail. zcl_status: %d", zcl_status);
    }

   // NRF_LOG_INFO("bme_data.pressure: %d, %x" , bme_data.pressure, bme_data.pressure);
    /* Get new pressure measured value */
    new_pres_value = (int16_t) (((int32_t)bme_data.pressure)/100);
    //NRF_LOG_INFO("bme_data.pressure/100: %d, %x" , new_pres_value, new_pres_value);
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_pres_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set pressure value fail. zcl_status: %d", zcl_status);
    }

    new_hum_value = (int16_t) (((int32_t)bme_data.humidity)/10);
    //sNRF_LOG_INFO("bme_data.humidity/10: %d, %x" , new_hum_value, new_hum_value);
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_hum_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set humidity value fail. zcl_status: %d", zcl_status);
    }
}

/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_temperature_sim_cfg.min          = MIN_TEMPERATURE_VALUE;
    m_temperature_sim_cfg.max          = MAX_TEMPERATURE_VALUE;
    m_temperature_sim_cfg.incr         = TEMPERATURE_VALUE_INCREMENT;
    m_temperature_sim_cfg.start_at_max = false;

    sensorsim_init(&m_temperature_sim_state, &m_temperature_sim_cfg);

    m_pressure_sim_cfg.min          = MIN_PRESSURE_VALUE;
    m_pressure_sim_cfg.max          = MAX_PRESSURE_VALUE;
    m_pressure_sim_cfg.incr         = PRESSURE_VALUE_INCREMENT;
    m_pressure_sim_cfg.start_at_max = false;

    sensorsim_init(&m_pressure_sim_state, &m_pressure_sim_cfg);


}


/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t  * p_sg_p      = NULL;
    zb_zdo_app_signal_type_t   sig         = zb_get_app_signal(bufid, &p_sg_p);
    zb_ret_t                   status      = ZB_GET_APP_SIGNAL_STATUS(bufid);

    /* Update network status LED */
    zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);
    //NRF_LOG_INFO("zboss_signal_handler: %hd", sig);
    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            /* fall-through */
        case ZB_BDB_SIGNAL_STEERING:
            /* Call default signal handler. */
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
            if (status == RET_OK)
            {
                NRF_LOG_INFO("APP timer starting...");
                ret_code_t err_code = app_timer_start(zb_app_timer, APP_TIMER_TICKS(3000), NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case ZB_ZDO_SIGNAL_LEAVE:
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
            if (status == RET_OK )
            {
                NRF_LOG_INFO("APP timer stop");
                ret_code_t err_code = app_timer_stop(zb_app_timer);
                APP_ERROR_CHECK(err_code);
            }
            break;
        
        case ZB_COMMON_SIGNAL_CAN_SLEEP:

            break;

        default:
            /* Call default signal handler. */
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
            break;
    }

    if (bufid)
    {
        zb_buf_free(bufid);
    }
}

static zb_uint8_t zcl_device_cb(zb_bufid_t bufid)
{

    zb_zcl_parsed_hdr_t *cmd_info = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
   

    NRF_LOG_INFO("zcl cluster id: 0x%x", cmd_info->cluster_id);
    NRF_LOG_INFO("zcl cmd_id: 0x%x", cmd_info->cmd_id);

    switch (cmd_info->cmd_id)
    {
        case ZB_ZCL_CMD_CONFIG_REPORT:
            NRF_LOG_INFO("ZB_ZCL_CMD_CONFIG_REPORT");
            break;

        case ZB_ZCL_CMD_READ_ATTRIB:
            {
                zb_zcl_read_attr_req_t * read_attr;
                read_attr = (zb_zcl_read_attr_req_t *) zb_buf_begin(bufid);
                NRF_LOG_INFO("ZB_ZCL_CMD_READ_ATTRIB->attr_id:  0x%x", read_attr->attr_id[0]); 
            }
        break;

        default:

            break;
    }

    //NRF_LOG_INFO("zcl_device_cb status: %hd", p_device_cb_param->status);
    
    return RET_OK;
}

/**@brief Function for application main entry.
 */
int main(void)
{
    zb_ret_t       zb_err_code;
    ret_code_t     err_code;
    zb_ieee_addr_t ieee_addr;

    /* Initialize loging system and GPIOs. */
    timers_init();
    log_init();
    sensor_simulator_init();
    leds_init();

    gpio_init();
    twi_init();
    {
        bme680_init_interface();
        err_code = bme68x_init(&bme);


        bme_conf.filter = BME68X_FILTER_OFF;
        bme_conf.odr = BME68X_ODR_NONE;
        bme_conf.os_hum = BME68X_OS_16X;
        bme_conf.os_pres = BME68X_OS_1X;
        bme_conf.os_temp = BME68X_OS_2X;
        err_code = bme68x_set_conf(&bme_conf, &bme);
        //bme68x_check_rslt("bme68x_set_conf", ret);


        bme_heatr_conf.enable = BME68X_ENABLE;
        bme_heatr_conf.heatr_temp = 300;
        bme_heatr_conf.heatr_dur = 100;
        err_code = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme_heatr_conf, &bme);
        //bme68x_check_rslt("bme68x_set_heatr_conf", ret);

        err_code = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        bme68x_check_rslt("bme68x_set_op_mode", err_code);

        uint32_t del_period;
        uint8_t n_fields;

        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme_conf, &bme) + (bme_heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);
        NRF_LOG_INFO("BME680 delay period = %dus",del_period);

        err_code = bme68x_get_data(BME68X_FORCED_MODE, &bme_data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", err_code);

        NRF_LOG_INFO("temperature: %d", bme_data.temperature);
        NRF_LOG_INFO("pressure: %d", bme_data.pressure);
        NRF_LOG_INFO("humidity: %d", bme_data.humidity);
        NRF_LOG_INFO("gas_resistance: %d", bme_data.gas_resistance);


    }




    /* Create Timer for reporting attribute */
    err_code = app_timer_create(&zb_app_timer, APP_TIMER_MODE_REPEATED, zb_app_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* Set Zigbee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize Zigbee stack. */
    ZB_INIT("multi_sensor");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set static long IEEE address. */
    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    // zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);
    uint32_t pin_state = nrf_gpio_pin_read(BOARD_BUTTON_PIN);
    if (pin_state == 0)
    {
        zb_bool_t erase = ZB_TRUE;
        NRF_LOG_INFO("Forcing flash erasure due to pin state");
        zb_set_nvram_erase_at_start(erase);
    }

    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));
    zb_set_rx_on_when_idle(ZB_FALSE);

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register temperature sensor device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&multi_sensor_ctx);

    /* Register callback for handling ZCL commands. */
    ZB_AF_SET_ENDPOINT_HANDLER(MULTI_SENSOR_ENDPOINT, zcl_device_cb);

    /* Initialize sensor device attibutes */
    multi_sensor_clusters_attr_init();

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start_no_autostart();
    ZB_ERROR_CHECK(zb_err_code);

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}


/**
 * @}
 */
