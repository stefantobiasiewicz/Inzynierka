

#include "sdk_config.h"
#include "zboss_api.h"
#include "zboss_api_addons.h"
#include "zb_mem_config_max.h"
#include "zb_ha_dimmable_light.h"
#include "zb_error_handler.h"
#include "zb_nrf52_internal.h"
#include "zigbee_helpers.h"

#include "bsp.h"
#include "boards.h"
#include "app_pwm.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "Contact.h"
#include "device_factory_settings.h"

#include "nrf_drv_saadc.h"
#include "nrf_drv_gpiote.h"

#define MAX_CHILDREN                      10                                    /**< The maximum amount of connected devices. Setting this value to 0 disables association to this device.  */
#define IEEE_CHANNEL_MASK                 (1l << 11)                            /**< Scan only one, predefined channel to find the coordinator. value for all chanel -> 0x07fff800U*/
#define ERASE_PERSISTENT_CONFIG           ZB_FALSE                             /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define MATCH_DESC_REQ_ROLE                 ZB_NWK_BROADCAST_RX_ON_WHEN_IDLE    /**< Find only non-sleepy device. */
//#define ZB_ED_ROLE

#ifdef  BOARD_PCA10059                                                          /**< If it is Dongle */
#define IDENTIFY_MODE_BSP_EVT             BSP_EVENT_KEY_0                       /**< Button event used to enter the Bulb into the Identify mode. */
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_0                       /**< LED indicating that light switch successfully joind Zigbee network. */
#else
#define IDENTIFY_MODE_BSP_EVT             BSP_EVENT_KEY_3                       /**< Button event used to enter the Bulb into the Identify mode. */
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_2                       /**< LED indicating that light switch successfully joind Zigbee network. */
#endif
#define BULB_LED                          BSP_BOARD_LED_3                       /**< LED immitaing dimmable light bulb. */

#ifndef ZB_ED_ROLE
#define ZB_ED_ROLE
#endif

#define BOARD_LED_PIN NRF_GPIO_PIN_MAP(0,05)  
#define BOARD_BUTTON_PIN  NRF_GPIO_PIN_MAP(0,30)
#define KONTACTOR_PIN  NRF_GPIO_PIN_MAP(0,19)


#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.

// static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];

// TODO ZB_BDB_SIGNAL_DEVICE_FIRST_START -> DEFAULT_ZBOSS_HANDLER ZOBACZ TAM !!! JEST TAM IMPLEMENTACJA REJOINU


/* Main application customizable context. Stores all settings and static values. */
typedef struct
{
    zb_zcl_basic_attrs_ext_t         basic_attr;
    zb_zcl_identify_attrs_t          identify_attr;
    ias_zone_attr_t                  ias_zone_attr;
    power_config_attr_t              power_config_attr;
} bulb_device_ctx_t;

static bulb_device_ctx_t m_dev_ctx;

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

/* Contact cluster attributes additions data */

ZB_ZCL_DECLARE_IAS_ZONE_ATTRIB_LIST_EXT(ias_zone_attr_list,
                                    &m_dev_ctx.ias_zone_attr.zone_state,
                                    &m_dev_ctx.ias_zone_attr.zone_type,
                                    &m_dev_ctx.ias_zone_attr.zone_status,
                                    &m_dev_ctx.ias_zone_attr.number_of_zone_sens_levels_supported,
                                    &m_dev_ctx.ias_zone_attr.current_zone_sens_level,
                                    &m_dev_ctx.ias_zone_attr.ias_cie_address,
                                    &m_dev_ctx.ias_zone_attr.zone_id,
                                    &m_dev_ctx.ias_zone_attr.cie_short_addr,
                                    &m_dev_ctx.ias_zone_attr.cie_ep
                                    );

ZB_ZCL_DECLARE_POWER_CONFIG_ATTRIB_LIST_EXTEND_WITH_PERCENTAGE(power_config_attr_list,
                                             &m_dev_ctx.power_config_attr.voltage,
                                             &m_dev_ctx.power_config_attr.size,
                                             &m_dev_ctx.power_config_attr.quantity,
                                             &m_dev_ctx.power_config_attr.rated_voltage,
                                             &m_dev_ctx.power_config_attr.alarm_mask,
                                             &m_dev_ctx.power_config_attr.voltage_min_threshold,
                                             &m_dev_ctx.power_config_attr.battery_percentage_remaining);

ZB_HA_CONTACT_CLUSTER_LIST(contact_cluster_list,
                            basic_attr_list,
                            identify_attr_list,
                            ias_zone_attr_list,
                            power_config_attr_list);


ZB_HA_CONTACT_EP(contact_ep,
                HA_CONTACT_ENDPOINT,
                contact_cluster_list);


ZB_HA_DECLARE_CONTACT_CTX(cotact_ctx,
                        contact_ep);

                        


/**@brief Function for initializing the application timer.
 */
static void timer_init(void)
{
    uint32_t error_code = app_timer_init();
    APP_ERROR_CHECK(error_code);
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

void saadc_init(void)
{
    ret_code_t err_code;                          

    err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_config;                                                   
        channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;     
        channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;      
        channel_config.gain       = NRF_SAADC_GAIN1_6;                
        channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;     
        channel_config.acq_time   = NRF_SAADC_ACQTIME_10US;          
        channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;      
        channel_config.burst      = NRF_SAADC_BURST_DISABLED;         
        channel_config.pin_p      = NRF_SAADC_INPUT_VDD;  
        channel_config.pin_n      = NRF_SAADC_INPUT_DISABLED;          
    

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

static void contact_sensor_clusters_attr_init(void)
{

    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = CONTACT_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = CONTACT_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = CONTACT_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          CONTACT_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(CONTACT_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          CONTACT_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(CONTACT_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          CONTACT_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(CONTACT_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = CONTACT_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          CONTACT_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(CONTACT_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = CONTACT_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
    

    m_dev_ctx.ias_zone_attr.zone_state = ZB_ZCL_IAS_ZONE_ZONESTATE_NOT_ENROLLED;
    m_dev_ctx.ias_zone_attr.zone_type = ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH;
    m_dev_ctx.ias_zone_attr.zone_status = ZB_ZCL_IAS_ZONE_ZONE_STATUS_DEF_VALUE;
    m_dev_ctx.ias_zone_attr.ias_cie_address = 0x0000000000000000;
    m_dev_ctx.ias_zone_attr.zone_id = ZB_ZCL_IAS_ZONEID_ID_DEF_VALUE;
    m_dev_ctx.ias_zone_attr.number_of_zone_sens_levels_supported = ZB_ZCL_IAS_ZONE_NUMBER_OF_ZONE_SENSITIVITY_LEVELS_SUPPORTED_DEFAULT_VALUE;
    m_dev_ctx.ias_zone_attr.current_zone_sens_level = ZB_ZCL_IAS_ZONE_CURRENT_ZONE_SENSITIVITY_LEVEL_DEFAULT_VALUE;
    m_dev_ctx.ias_zone_attr.cie_short_addr = 0x0000;
    m_dev_ctx.ias_zone_attr.cie_ep = 0;


    m_dev_ctx.power_config_attr.voltage = 29;
    m_dev_ctx.power_config_attr.size = ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_CR2;
    m_dev_ctx.power_config_attr.quantity= 1;
    m_dev_ctx.power_config_attr.rated_voltage = 30;
    m_dev_ctx.power_config_attr.alarm_mask = ZB_ZCL_POWER_CONFIG_BATTERY_ALARM_STATE_DEFAULT_VALUE;
    m_dev_ctx.power_config_attr.voltage_min_threshold = 20;
    m_dev_ctx.power_config_attr.battery_percentage_remaining = 255;

    zb_uint8_t percent =  m_dev_ctx.power_config_attr.battery_percentage_remaining;
    zb_zcl_status_t zcl_status;
    zcl_status = zb_zcl_set_attr_val(HA_CONTACT_ENDPOINT, 
                                     ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, 
                                     (zb_uint8_t *)&percent, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set BATTERY_PERCENTAGE_REMAINING fail. zcl_status: %d", zcl_status);
    }

                           
}

static zb_void_t contact_send_notification_req(zb_bufid_t bufid, zb_uint16_t on_off)
{
    zb_uint16_t cmd;

    if(on_off == true){
        cmd = ZB_ZCL_IAS_ZONE_ZONE_STATUS_ALARM1;
    }
    else{
        cmd = 0;
    }

    NRF_LOG_INFO("Send NOTIFICATION command: %d cie_short_addr = 0x%x, cie_ep = 0x%x ", cmd,
                          m_dev_ctx.ias_zone_attr.cie_short_addr, m_dev_ctx.ias_zone_attr.cie_ep);
    // todo -> w??aczy?? diode i w callbackuy zgasic
    ZB_ZCL_IAS_ZONE_SEND_STATUS_CHANGE_NOTIFICATION_REQ(bufid,
                                                        m_dev_ctx.ias_zone_attr.cie_short_addr,
                                                        ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                                        m_dev_ctx.ias_zone_attr.cie_ep,
                                                        HA_CONTACT_ENDPOINT,
                                                        ZB_AF_HA_PROFILE_ID,
                                                        NULL,
                                                        cmd,
                                                        m_dev_ctx.ias_zone_attr.zone_state,
                                                        m_dev_ctx.ias_zone_attr.zone_id,
                                                        0
    );
}

/**
 *  @return battery voltage in mV 
 */
static void battery_measure(zb_uint8_t *battery, zb_uint8_t *percent){
    saadc_init();

    nrf_saadc_value_t new_voltage_value;
    nrfx_saadc_sample_convert(0, &new_voltage_value);
    NRF_LOG_INFO("nrfx_saadc_sample_convert: %d", new_voltage_value);
    nrf_drv_saadc_uninit();

    *battery = (zb_uint8_t) (new_voltage_value * 0.0351); //3.6 / 1024 * 10
    *percent = (zb_uint8_t) (new_voltage_value * 0.00351 * 66.666); // 3.6 / 1024 * 66.666
}

static void check_contact(){
    zb_ret_t zb_err_code;
    zb_bool_t cmd = false;

    battery_measure(&m_dev_ctx.power_config_attr.voltage, &m_dev_ctx.power_config_attr.battery_percentage_remaining);

    NRF_LOG_INFO("voltage: %d", m_dev_ctx.power_config_attr.voltage);
    NRF_LOG_INFO("battery_percentage_remaining: %d", m_dev_ctx.power_config_attr.battery_percentage_remaining);

    zb_uint8_t percent =  m_dev_ctx.power_config_attr.battery_percentage_remaining;
    zb_zcl_status_t zcl_status;
    zcl_status = zb_zcl_set_attr_val(HA_CONTACT_ENDPOINT, 
                                     ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, 
                                     &percent, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS) {
        NRF_LOG_INFO("Set BATTERY_PERCENTAGE_REMAINING fail. zcl_status: %d", zcl_status);
    }

    if(nrf_drv_gpiote_in_is_set(KONTACTOR_PIN)){
        // kontaktron otwarty
        NRF_LOG_INFO("Contact open");
        cmd = true;
    } else{
        // kontaktron z????czony
        NRF_LOG_INFO("Contact close");
        cmd = false;
    }
    if(m_dev_ctx.ias_zone_attr.zone_state == ZB_ZCL_IAS_ZONE_ZONESTATE_ENROLLED){
        zb_err_code = zb_buf_get_out_delayed_ext(contact_send_notification_req,cmd, 0);
        ZB_ERROR_CHECK(zb_err_code);
    }
}


static void izs_send_enroll_req(zb_bufid_t bufid){
    NRF_LOG_INFO("send ZB_ZCL_IAS_ZONE_SEND_ZONE_ENROLL_REQUEST_REQ");
    ZB_ZCL_IAS_ZONE_SEND_ZONE_ENROLL_REQUEST_REQ(
        bufid,
        m_dev_ctx.ias_zone_attr.cie_short_addr,
        ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        m_dev_ctx.ias_zone_attr.cie_ep,
        HA_CONTACT_ENDPOINT,
        ZB_AF_HA_PROFILE_ID,
        NULL,
        m_dev_ctx.ias_zone_attr.zone_type,
        0);

}

zb_uint8_t izs_zcl_cmd_handler(zb_uint8_t param)
{
  zb_bufid_t zcl_cmd_buf = param;
  zb_zcl_parsed_hdr_t *cmd_info = ZB_BUF_GET_PARAM(zcl_cmd_buf, zb_zcl_parsed_hdr_t);
  zb_uint8_t cmd_processed = ZB_FALSE;
  
    NRF_LOG_INFO("zcl cluster id: 0x%x", cmd_info->cluster_id);
    NRF_LOG_INFO("zcl cmd_id: 0x%x", cmd_info->cmd_id);

  if (cmd_info->cmd_direction == ZB_ZCL_FRAME_DIRECTION_TO_SRV)
  {
   if (cmd_info->cluster_id == ZB_ZCL_CLUSTER_ID_IAS_ZONE &&
             cmd_info->is_common_command)
    {
       switch (cmd_info->cmd_id)
       {
          case ZB_ZCL_CMD_WRITE_ATTRIB:
            {
               zb_zcl_write_attr_req_t *write_attr_req;
               write_attr_req = (zb_zcl_write_attr_req_t*)zb_buf_begin(zcl_cmd_buf);
               if (ZB_ZCL_ATTR_IAS_ZONE_IAS_CIE_ADDRESS_ID == write_attr_req->attr_id)
               {
                m_dev_ctx.ias_zone_attr.cie_short_addr = cmd_info->addr_data.common_data.source.u.short_addr;
                m_dev_ctx.ias_zone_attr.cie_ep = cmd_info->addr_data.common_data.src_endpoint;
                NRF_LOG_INFO("CIE address is updated. New cie_short_addr = 0x%x, cie_ep = 0x%x ",
                          m_dev_ctx.ias_zone_attr.cie_short_addr, m_dev_ctx.ias_zone_attr.cie_ep);
             
                zb_buf_get_out_delayed(izs_send_enroll_req);

                cmd_processed = ZB_FALSE;
               }
             }
             break;
          default:
            break;
       }
    } else
    if (cmd_info->cluster_id == ZB_ZCL_CLUSTER_ID_IAS_ZONE &&
        !cmd_info->is_common_command)
    {
      switch (cmd_info->cmd_id)
      {
        case ZB_ZCL_CMD_IAS_ZONE_ZONE_ENROLL_RESPONSE_ID:
        {
            zb_zcl_ias_zone_enroll_response_value_param_t *response_param;
            response_param = (zb_zcl_ias_zone_enroll_response_value_param_t*)zb_buf_begin(zcl_cmd_buf);
            if(response_param->enroll_response == ZB_ZCL_STATUS_SUCCESS){
                m_dev_ctx.ias_zone_attr.zone_state = ZB_ZCL_IAS_ZONE_ZONESTATE_ENROLLED;
                m_dev_ctx.ias_zone_attr.zone_id = response_param->zone_id;
                m_dev_ctx.ias_zone_attr.cie_short_addr = cmd_info->addr_data.common_data.source.u.short_addr;
                m_dev_ctx.ias_zone_attr.cie_ep = cmd_info->addr_data.common_data.src_endpoint;
                zb_nvram_write_dataset(ZB_NVRAM_HA_DATA);
                check_contact();
            }         
          cmd_processed = ZB_TRUE;
          break;
        }
        default:
          NRF_LOG_WARNING("skip command %hd", cmd_info->cmd_id);
          break;
      }
    }
  }

  return cmd_processed;
}



void zboss_signal_handler(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t  * p_sg_p = NULL;
    zb_zdo_app_signal_type_t   sig    = zb_get_app_signal(bufid, &p_sg_p);
    zb_ret_t                   status = ZB_GET_APP_SIGNAL_STATUS(bufid);

    switch (sig)
    {
        case ZB_BDB_SIGNAL_STEERING:
            if (status == RET_OK)
            {
                NRF_LOG_INFO("Network connected");
            }
        break;

        case ZB_ZDO_SIGNAL_LEAVE:
            m_dev_ctx.ias_zone_attr.zone_state = ZB_ZCL_IAS_ZONE_ZONESTATE_NOT_ENROLLED;
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
        break;

        case ZB_COMMON_SIGNAL_CAN_SLEEP:
            {
//                zb_zdo_signal_can_sleep_params_t *can_sleep_params = ZB_ZDO_SIGNAL_GET_PARAMS(p_sg_p, zb_zdo_signal_can_sleep_params_t);
//                NRF_LOG_INFO("Can sleep for %ld ms", can_sleep_params->sleep_tmo);
                zb_sleep_now();
            }
        break;
        
        default:
            /* No application-specific behavior is required. Call default signal handler. */
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
        break;
    }


    if (bufid)
    {
        zb_buf_free(bufid);
    }
}

static zb_void_t sleepy_device_setup(void)
{
    zb_set_rx_on_when_idle(false);

    if (ZB_PIBCACHE_RX_ON_WHEN_IDLE() == ZB_FALSE)
    {
        zigbee_power_down_unused_ram();
    }
}



static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){

    switch (pin)
    {
    case BOARD_BUTTON_PIN:
        if(action == NRF_GPIOTE_POLARITY_HITOLO){
            // NRF_LOG_INFO("Button press");
            // zb_bdb_reset_via_local_action(0);
            // NVIC_SystemReset();
        }
        break;
    case KONTACTOR_PIN:
        if(action == NRF_GPIOTE_POLARITY_TOGGLE){
            check_contact();
        }
        break;
    default:
        NRF_LOG_WARNING("Unknown gpio event pin: %d, action %d", pin, action);
        break;
    }

    nrf_drv_gpiote_out_toggle(BOARD_LED_PIN);
}

static void gpio_init(void)
{
    ret_code_t err_code;

    // inicjalizacja modu??u GPIOTE
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // struktura konfiguracyjna wyj??ciue - dioda LED
    nrf_drv_gpiote_out_config_t led_out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    // inicjalizacja wyjscia
    err_code = nrf_drv_gpiote_out_init(BOARD_LED_PIN, &led_out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_set(BOARD_LED_PIN);

    // struktura konfiguracyjna wej??cie - przycisk
    nrf_drv_gpiote_in_config_t button_in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    button_in_config.pull = NRF_GPIO_PIN_PULLUP;
    // inicjalizacja wy??cia
    err_code = nrf_drv_gpiote_in_init(BOARD_BUTTON_PIN, &button_in_config, gpio_handler);
    APP_ERROR_CHECK(err_code);
    // w??aczenie mechanizmu eventow przychodz??cych od pinu
    nrf_drv_gpiote_in_event_enable(BOARD_BUTTON_PIN, true);

    // struktura konfiguracyjna wej??cie - kontraktron
    nrf_drv_gpiote_in_config_t contact_in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    contact_in_config.pull = NRF_GPIO_PIN_PULLUP;
    // inicjalizacja wy??cia
    err_code = nrf_drv_gpiote_in_init(KONTACTOR_PIN, &contact_in_config, gpio_handler);
    APP_ERROR_CHECK(err_code);
    // w??aczenie mechanizmu eventow przychodz??cych od pinu
    nrf_drv_gpiote_in_event_enable(KONTACTOR_PIN, true);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    zb_ret_t       zb_err_code;
    zb_ieee_addr_t ieee_addr;

    /* Initialize timer, logging system and GPIOs. */
    timer_init();
    log_init();


    gpio_init();

    /* Set Zigbee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize Zigbee stack. */
    ZB_INIT("Contact Sensor");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set static long IEEE address. */
    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);

    uint32_t pin_state = nrf_gpio_pin_read(BOARD_BUTTON_PIN);
    if (pin_state == 0)
    {
        zb_bool_t erase = ZB_TRUE;
        NRF_LOG_INFO("Forcing flash erasure due to pin state");
        zb_set_nvram_erase_at_start(erase);
    }

    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

    sleepy_device_setup();

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&cotact_ctx);

    ZB_AF_SET_ENDPOINT_HANDLER(HA_CONTACT_ENDPOINT, izs_zcl_cmd_handler);

    contact_sensor_clusters_attr_init();

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
