
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

#include "nrf_drv_gpiote.h"
#include "zb_ha_on_off_output.h"

#define MAX_CHILDREN                      10                                    /**< The maximum amount of connected devices. Setting this value to 0 disables association to this device.  */
#define IEEE_CHANNEL_MASK                 (1l << 11)                /**< Scan only one, predefined channel to find the coordinator. */
#define HA_ON_OFF_ENDPOINT                  10                                    /**< Device endpoint, used to receive light controlling commands. */
#define ERASE_PERSISTENT_CONFIG           ZB_FALSE                              /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */


/* Basic cluster attributes initial values. */
#define BULB_INIT_BASIC_APP_VERSION       02                                    /**< Version of the application software (1 byte). */
#define BULB_INIT_BASIC_STACK_VERSION     10                                    /**< Version of the implementation of the Zigbee stack (1 byte). */
#define BULB_INIT_BASIC_HW_VERSION        11                                    /**< Version of the hardware of the device (1 byte). */
#define BULB_INIT_BASIC_MANUF_NAME        "Stefan Tobiasiewicz"                              /**< Manufacturer name (32 bytes). */
#define BULB_INIT_BASIC_MODEL_ID          "PowerPlug"                  /**< Model number assigned by manufacturer (32-bytes long string). */
#define BULB_INIT_BASIC_DATE_CODE         "20220106"                            /**< First 8 bytes specify the date of manufacturer of the device in ISO 8601 format (YYYYMMDD). The rest (8 bytes) are manufacturer specific. */
#define BULB_INIT_BASIC_POWER_SOURCE      ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE   /**< Type of power sources available for the device. For possible values see section 3.2.2.2.8 of ZCL specification. */
#define BULB_INIT_BASIC_LOCATION_DESC     "Office desk"                         /**< Describes the physical location of the device (16 bytes). May be modified during commisioning process. */
#define BULB_INIT_BASIC_PH_ENV            ZB_ZCL_BASIC_ENV_UNSPECIFIED          /**< Describes the type of physical environment. For possible values see section 3.2.2.2.10 of ZCL specification. */




#define RELAY_PIN NRF_GPIO_PIN_MAP(0,02)   
#define NETWORK_LED_PIN NRF_GPIO_PIN_MAP(0,12)   
#define BOARD_BUTTON_PIN NRF_GPIO_PIN_MAP(1,06)   

#define ZB_ROUTER_ROLE

#if !defined ZB_ROUTER_ROLE
#error Define ZB_ROUTER_ROLE to compile light bulb (Router) source code.
#endif

/* Main application customizable context. Stores all settings and static values. */
typedef struct
{
    zb_zcl_basic_attrs_ext_t         basic_attr;
    zb_zcl_identify_attrs_t          identify_attr;
    zb_zcl_scenes_attrs_t            scenes_attr;
    zb_zcl_groups_attrs_t            groups_attr;
    zb_zcl_on_off_attrs_ext_t        on_off_attr;
} bulb_device_ctx_t;

static bulb_device_ctx_t m_dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &m_dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(groups_attr_list, &m_dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(scenes_attr_list,
                                  &m_dev_ctx.scenes_attr.scene_count,
                                  &m_dev_ctx.scenes_attr.current_scene,
                                  &m_dev_ctx.scenes_attr.current_group,
                                  &m_dev_ctx.scenes_attr.scene_valid,
                                  &m_dev_ctx.scenes_attr.name_support);

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

ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST_EXT(on_off_attr_list,
                                      &m_dev_ctx.on_off_attr.on_off,
                                      &m_dev_ctx.on_off_attr.global_scene_ctrl,
                                      &m_dev_ctx.on_off_attr.on_time,
                                      &m_dev_ctx.on_off_attr.off_wait_time);

ZB_HA_DECLARE_ON_OFF_OUTPUT_CLUSTER_LIST(power_plug_clusters,
                                          on_off_attr_list,
                                          basic_attr_list,
                                          identify_attr_list,
                                          groups_attr_list,
                                          scenes_attr_list);

ZB_HA_DECLARE_ON_OFF_OUTPUT_EP(power_plug_ed,
                                HA_ON_OFF_ENDPOINT,
                                power_plug_clusters);

ZB_HA_DECLARE_ON_OFF_OUTPUT_CTX(power_plug_ctx,
                                 power_plug_ed);

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

/**@brief Function for turning ON/OFF the light bulb.
 *
 * @param[in]   on   Boolean light bulb state.
 */
static void on_off_set_value(zb_bool_t on)
{
    NRF_LOG_INFO("Set ON/OFF value: %i", on);

    ZB_ZCL_SET_ATTRIBUTE(HA_ON_OFF_ENDPOINT, 
                         ZB_ZCL_CLUSTER_ID_ON_OFF,    
                         ZB_ZCL_CLUSTER_SERVER_ROLE,  
                         ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                         (zb_uint8_t *)&on,                        
                         ZB_FALSE);

    m_dev_ctx.on_off_attr.on_off = on;

    if (on)
    {
        nrf_drv_gpiote_out_set(RELAY_PIN);
    }
    else
    {
        nrf_drv_gpiote_out_clear(RELAY_PIN);
    }
}

static void on_off_set_on_start_value(){
    if (m_dev_ctx.on_off_attr.on_off)
    {
        nrf_drv_gpiote_out_set(RELAY_PIN);
    }
    else
    {
        nrf_drv_gpiote_out_clear(RELAY_PIN);
    }
}

/**@brief Function for initializing all clusters attributes.
 */
static void bulb_clusters_attr_init(void)
{
    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = BULB_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = BULB_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          BULB_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          BULB_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          BULB_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = BULB_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          BULB_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

    // /* On/Off cluster attributes data */
    // m_dev_ctx.on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;

    ZB_ZCL_SET_ATTRIBUTE(HA_ON_OFF_ENDPOINT, 
                         ZB_ZCL_CLUSTER_ID_ON_OFF,    
                         ZB_ZCL_CLUSTER_SERVER_ROLE,  
                         ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                         (zb_uint8_t *)&m_dev_ctx.on_off_attr.on_off,                        
                         ZB_FALSE);                   
                               
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
    zb_uint8_t                       cluster_id;
    zb_uint8_t                       attr_id;
    zb_zcl_device_callback_param_t * p_device_cb_param = ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

    NRF_LOG_INFO("zcl_device_cb id %hd", p_device_cb_param->device_cb_id);

    /* Set default response value. */
    p_device_cb_param->status = RET_OK;

    switch (p_device_cb_param->device_cb_id)
    {

        case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
            cluster_id = p_device_cb_param->cb_param.set_attr_value_param.cluster_id;
            attr_id    = p_device_cb_param->cb_param.set_attr_value_param.attr_id;

            if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
            {
                uint8_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data8;

                NRF_LOG_INFO("on/off attribute setting to %hd", value);
                if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
                {
                    on_off_set_value((zb_bool_t) value);
                }
            }
            break;

        default:
            p_device_cb_param->status = RET_ERROR;
            break;
    }

    NRF_LOG_INFO("zcl_device_cb status: %hd", p_device_cb_param->status);
}

void zigbee_status_led_update(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t  * p_sg_p = NULL;
    zb_zdo_app_signal_type_t   sig    = zb_get_app_signal(bufid, &p_sg_p);
    zb_ret_t                   status = ZB_GET_APP_SIGNAL_STATUS(bufid);

    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            /* fall-through */
        case ZB_BDB_SIGNAL_STEERING:
            if (status == RET_OK)
            {
                nrf_drv_gpiote_out_clear(NETWORK_LED_PIN);
            }
            else
            {
                nrf_drv_gpiote_out_set(NETWORK_LED_PIN);
            }
            break;

        case ZB_ZDO_SIGNAL_LEAVE:
            /* Update network status LED */
            nrf_drv_gpiote_out_set(NETWORK_LED_PIN);
            break;

        default:
            break;
    }
    NRF_LOG_INFO("Network led state update");
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    NRF_LOG_INFO("zboss_signal_handler id %hd", bufid);

    /* Update network status LED */
    zigbee_status_led_update(bufid);

    /* No application-specific behavior is required. Call default signal handler. */
    ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

    if (bufid)
    {
        zb_buf_free(bufid);
    }
}

static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){

    switch (pin)
    {
    case BOARD_BUTTON_PIN:
        if(action == NRF_GPIOTE_POLARITY_HITOLO){
            NRF_LOG_INFO("Button press");
            zb_bdb_reset_via_local_action(0);
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

    // struktura konfiguracyjna wyjściue - przekaźnik
    nrf_drv_gpiote_out_config_t relay_out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    // inicjalizacja wyjscia
    err_code = nrf_drv_gpiote_out_init(RELAY_PIN, &relay_out_config);
    APP_ERROR_CHECK(err_code);

    // struktura konfiguracyjna wyjściue - dioda led
    nrf_drv_gpiote_out_config_t led_out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    // inicjalizacja wyjscia
    err_code = nrf_drv_gpiote_out_init(NETWORK_LED_PIN, &led_out_config);
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
    ZB_INIT("led_bulb");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set static long IEEE address. */
    zb_set_network_router_role(IEEE_CHANNEL_MASK);
    zb_set_max_children(MAX_CHILDREN);


    uint32_t pin_state = nrf_gpio_pin_read(BOARD_BUTTON_PIN);
    if (pin_state == 0)
    {
        zb_bool_t erase = ZB_TRUE;
        NRF_LOG_INFO("Forcing flash erasure due to pin state");
        zb_set_nvram_erase_at_start(erase);
    }

    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register callback for handling ZCL commands. */
    ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&power_plug_ctx);

    bulb_clusters_attr_init();

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start_no_autostart();
    ZB_ERROR_CHECK(zb_err_code);

    on_off_set_on_start_value();

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}


/**
 * @}
 */
