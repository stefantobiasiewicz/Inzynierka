

#include "sdk_config.h"
#include "zboss_api.h"
#include "zboss_api_addons.h"
#include "zb_mem_config_med.h"
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

#define MAX_CHILDREN                      10                                    /**< The maximum amount of connected devices. Setting this value to 0 disables association to this device.  */
#define IEEE_CHANNEL_MASK                 (1l << 11)                /**< Scan only one, predefined channel to find the coordinator. */
#define HA_DIMMABLE_LIGHT_ENDPOINT        10                                    /**< Device endpoint, used to receive light controlling commands. */
#define ERASE_PERSISTENT_CONFIG           ZB_FALSE                              /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */
#define BULB_PWM_NAME                     PWM1                                  /**< PWM instance used to drive dimmable light bulb. */
#define BULB_PWM_TIMER                    2                                     /**< Timer number used by PWM. */


#ifdef  BOARD_PCA10059                                                          /**< If it is Dongle */
#define IDENTIFY_MODE_BSP_EVT             BSP_EVENT_KEY_0                       /**< Button event used to enter the Bulb into the Identify mode. */
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_0                       /**< LED indicating that light switch successfully joind Zigbee network. */
#else
#define IDENTIFY_MODE_BSP_EVT             BSP_EVENT_KEY_3                       /**< Button event used to enter the Bulb into the Identify mode. */
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_2                       /**< LED indicating that light switch successfully joind Zigbee network. */
#endif
#define BULB_LED                          BSP_BOARD_LED_3                       /**< LED immitaing dimmable light bulb. */


/* Main application customizable context. Stores all settings and static values. */
typedef struct
{
    zb_zcl_basic_attrs_ext_t         basic_attr;
    zb_zcl_identify_attrs_t          identify_attr;
    ias_zone_attr_t                  ias_zone_attr;
    power_config_attr_t              power_config_attr;
} bulb_device_ctx_t;


APP_PWM_INSTANCE(BULB_PWM_NAME, BULB_PWM_TIMER);
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

ZB_ZCL_DECLARE_IAS_ZONE_ATTRIB_LIST(ias_zone_attr_list,
                                    &m_dev_ctx.ias_zone_attr.zone_state,
                                    &m_dev_ctx.ias_zone_attr.zone_type,
                                    &m_dev_ctx.ias_zone_attr.zone_status,
                                    &m_dev_ctx.ias_zone_attr.ias_cie_address,
                                    &m_dev_ctx.ias_zone_attr.cie_short_addr,
                                    &m_dev_ctx.ias_zone_attr.cie_ep
                                    );

ZB_ZCL_DECLARE_POWER_CONFIG_MAINS_ATTRIB_LIST(power_config_attr_list,
                                             &m_dev_ctx.power_config_attr.voltage,
                                             &m_dev_ctx.power_config_attr.frequency,
                                             &m_dev_ctx.power_config_attr.alarm_mask,
                                             &m_dev_ctx.power_config_attr.voltage_min_threshold,
                                             &m_dev_ctx.power_config_attr.voltage_max_threshold,
                                             &m_dev_ctx.power_config_attr.dwell_trip_point);

ZB_HA_CONTACT_CLUSTER_LIST(contact_cluster_list,
                            basic_attr_list,
                            identify_attr_list,
                            ias_zone_attr_list,
                            power_config_attr_list);


ZB_HA_CONTACT_EP(contact_light_ep,
                HA_CONTACT_ENDPOINT,
                contact_cluster_list);


ZB_HA_DECLARE_CONTACT_CTX(cotact_ctx,
                        contact_light_ep);


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

/**@brief Callback for button events.
 *
 * @param[in]   evt      Incoming event from the BSP subsystem.
 */
static void buttons_handler(bsp_event_t evt)
{
    zb_ret_t zb_err_code;

    switch(evt)
    {
        case IDENTIFY_MODE_BSP_EVT:
            /* Check if endpoint is in identifying mode, if not put desired endpoint in identifying mode. */
            if (m_dev_ctx.identify_attr.identify_time == ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE)
            {
                NRF_LOG_INFO("Bulb put in identifying mode");
                zb_err_code = zb_bdb_finding_binding_target(HA_DIMMABLE_LIGHT_ENDPOINT);
                ZB_ERROR_CHECK(zb_err_code);
            }
            else
            {
                NRF_LOG_INFO("Cancel F&B target procedure");
                zb_bdb_finding_binding_target_cancel();
            }
            break;

        case BSP_EVENT_KEY_1:
            
        break;

        default:
            NRF_LOG_INFO("Unhandled BSP Event received: %d", evt);
            break;
    }
}


/**@brief Function for initializing LEDs and a single PWM channel.
 */
static void leds_buttons_init(void)
{
    ret_code_t       err_code;
    app_pwm_config_t pwm_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, bsp_board_led_idx_to_pin(BULB_LED));

    /* Initialize all LEDs and buttons. */
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, buttons_handler);
    APP_ERROR_CHECK(err_code);
    /* By default the bsp_init attaches BSP_KEY_EVENTS_{0-4} to the PUSH events of the corresponding buttons. */

    /* Initialize PWM running on timer 1 in order to control dimmable light bulb. */
    err_code = app_pwm_init(&BULB_PWM_NAME, &pwm_cfg, NULL);
    APP_ERROR_CHECK(err_code);

    app_pwm_enable(&BULB_PWM_NAME);

    while (app_pwm_channel_duty_set(&BULB_PWM_NAME, 0, 99) == NRF_ERROR_BUSY)
    {
    }
}

/**@brief Function for initializing all clusters attributes.
 */
static void bulb_clusters_attr_init(void)
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


    m_dev_ctx.ias_zone_attr.zone_type = ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH;

    // ZB_ZCL_SET_ATTRIBUTE(HA_CONTACT_ENDPOINT,
    //                     ZB_ZCL_CLUSTER_ID_IAS_ZONE,
    //                     ZB_ZCL_CLUSTER_SERVER_ROLE,
    //                     ZB_ZCL_ATTR_IAS_ZONE_ZONETYPE_ID,
    //                     (zb_uint8_t *)&m_dev_ctx.ias_zone_attr.zone_type,
    //                     ZB_FALSE);

    /* On/Off cluster attributes data */
    // m_dev_ctx.on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;

    // m_dev_ctx.level_control_attr.current_level  = ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE;
    // m_dev_ctx.level_control_attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;

    // ZB_ZCL_SET_ATTRIBUTE(HA_DIMMABLE_LIGHT_ENDPOINT, 
    //                      ZB_ZCL_CLUSTER_ID_ON_OFF,    
    //                      ZB_ZCL_CLUSTER_SERVER_ROLE,  
    //                      ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
    //                      (zb_uint8_t *)&m_dev_ctx.on_off_attr.on_off,                        
    //                      ZB_FALSE);                   

    // ZB_ZCL_SET_ATTRIBUTE(HA_DIMMABLE_LIGHT_ENDPOINT,                                       
    //                      ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,            
    //                      ZB_ZCL_CLUSTER_SERVER_ROLE,                 
    //                      ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, 
    //                      (zb_uint8_t *)&m_dev_ctx.level_control_attr.current_level,                                       
    //                      ZB_FALSE);                                  
}

/**@brief Function which tries to sleep down the MCU 
 *
 * Function which sleeps the MCU on the non-sleepy End Devices to optimize the power saving.
 * The weak definition inside the OSIF layer provides some minimal working template
 */
zb_void_t zb_osif_go_idle(zb_void_t)
{
    //TODO: implement your own logic if needed
    zb_osif_wait_for_event();
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer used to pass received data.
 */
static zb_void_t zcl_device_cb(zb_bufid_t bufid)
{
    //zb_uint8_t                       cluster_id;
    //zb_uint8_t                       attr_id;
    zb_zcl_device_callback_param_t * p_device_cb_param = ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

    NRF_LOG_INFO("zcl_device_cb id %hd", p_device_cb_param->device_cb_id);

    /* Set default response value. */
    p_device_cb_param->status = RET_OK;

    switch (p_device_cb_param->device_cb_id)
    {
        case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
            NRF_LOG_INFO("Level control setting to %d", p_device_cb_param->cb_param.level_control_set_value_param.new_value);
            break;

        default:
            p_device_cb_param->status = RET_ERROR;
            break;
    }

    NRF_LOG_INFO("zcl_device_cb status: %hd", p_device_cb_param->status);
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    /* Update network status LED */
    zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

    /* No application-specific behavior is required. Call default signal handler. */
    ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

    if (bufid)
    {
        zb_buf_free(bufid);
    }
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
    leds_buttons_init();

    /* Set Zigbee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize Zigbee stack. */
    ZB_INIT("led_bulb");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set static long IEEE address. */
    zb_set_network_router_role(IEEE_CHANNEL_MASK);
    zb_set_max_children(MAX_CHILDREN);
    zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register callback for handling ZCL commands. */
    ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&cotact_ctx);

    bulb_clusters_attr_init();


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
