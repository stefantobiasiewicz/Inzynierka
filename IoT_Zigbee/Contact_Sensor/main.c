

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

#include "nrf_drv_gpiote.h"

#define MAX_CHILDREN                      10                                    /**< The maximum amount of connected devices. Setting this value to 0 disables association to this device.  */
#define IEEE_CHANNEL_MASK                 (1l << 11)                            /**< Scan only one, predefined channel to find the coordinator. value for all chanel -> 0x07fff800U*/
#define HA_DIMMABLE_LIGHT_ENDPOINT        10                                    /**< Device endpoint, used to receive light controlling commands. */
#define ERASE_PERSISTENT_CONFIG           ZB_FALSE                              /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define ZB_ED_ROLE

#ifdef  BOARD_PCA10059                                                          /**< If it is Dongle */
#define IDENTIFY_MODE_BSP_EVT             BSP_EVENT_KEY_0                       /**< Button event used to enter the Bulb into the Identify mode. */
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_0                       /**< LED indicating that light switch successfully joind Zigbee network. */
#else
#define IDENTIFY_MODE_BSP_EVT             BSP_EVENT_KEY_3                       /**< Button event used to enter the Bulb into the Identify mode. */
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_2                       /**< LED indicating that light switch successfully joind Zigbee network. */
#endif
#define BULB_LED                          BSP_BOARD_LED_3                       /**< LED immitaing dimmable light bulb. */



#define BOARD_LED_PIN NRF_GPIO_PIN_MAP(0,13)  
#define BOARD_BUTTON_PIN  NRF_GPIO_PIN_MAP(0,02)
#define KONTACTOR_PIN  NRF_GPIO_PIN_MAP(0,24)


// TODO ZB_BDB_SIGNAL_DEVICE_FIRST_START -> DEFAULT_ZBOSS_HANDLER ZOBACZ TAM !!! JEST TAM IMPLEMENTACJA REJOINU
// SPRAWDZIC TA FUNKCJE Z LOW POWER.


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




/**@brief Callback for button events.
 *
 * @param[in]   evt      Incoming event from the BSP subsystem.
 */
// static void buttons_handler(bsp_event_t evt)
// {
//     zb_ret_t zb_err_code;
    

//     switch(evt)
//     {
//         case IDENTIFY_MODE_BSP_EVT:
//             /* Check if endpoint is in identifying mode, if not put desired endpoint in identifying mode. */
//             if (m_dev_ctx.identify_attr.identify_time == ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE)
//             {
//                 NRF_LOG_INFO("Bulb put in identifying mode");
//                 zb_err_code = zb_bdb_finding_binding_target(HA_DIMMABLE_LIGHT_ENDPOINT);
//                 ZB_ERROR_CHECK(zb_err_code);
//             }
//             else
//             {
//                 NRF_LOG_INFO("Cancel F&B target procedure");
//                 zb_bdb_finding_binding_target_cancel();
//             }
//             break;

//         // case BSP_EVENT_KEY_0:
//         //     NRF_LOG_INFO("BSP_EVENT_KEY_0 -> contact_send_close_open");
//         //     zb_err_code = zb_buf_get_out_delayed_ext(contact_send_notification_req,(zb_bool_t)true, 0);
//         //     ZB_ERROR_CHECK(zb_err_code);
//         // break;
//         // case BSP_EVENT_KEY_1:
//         //     NRF_LOG_INFO("BSP_EVENT_KEY_1 -> contact_send_close_open");
//         //     zb_err_code = zb_buf_get_out_delayed_ext(contact_send_notification_req,(zb_bool_t)false, 0);
//         //     ZB_ERROR_CHECK(zb_err_code);
//         // break;

//         // case BSP_EVENT_KEY_2:
//         //     NRF_LOG_INFO("BSP_EVENT_KEY_2 -> bdb_start_top_level_commissioning");
//         //     zb_bdb_reset_via_local_action(0);
//         // break;

//         default:
//             NRF_LOG_INFO("Unhandled BSP Event received: %d", evt);
//             break;
//     }
// }

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


    m_dev_ctx.ias_zone_attr.zone_state = ZB_ZCL_IAS_ZONE_ZONESTATE_NOT_ENROLLED;

    //ZB_ZCL_SET_DIRECTLY_ATTR_VAL8(m_dev_ctx.ias_zone_attr.zone_state, ZB_ZCL_IAS_ZONE_ZONESTATE_ENROLLED);

    // ZB_ZCL_SET_ATTRIBUTE(HA_CONTACT_ENDPOINT,
    //                     ZB_ZCL_CLUSTER_ID_IAS_ZONE,
    //                     ZB_ZCL_CLUSTER_SERVER_ROLE,
    //                     ZB_ZCL_ATTR_IAS_ZONE_ZONESTATE_ID,
    //                     &m_dev_ctx.ias_zone_attr.zone_state,
    //                     ZB_FALSE);
                           
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
    ZB_ZCL_IAS_ZONE_SEND_STATUS_CHANGE_NOTIFICATION_REQ(bufid,
                                                        m_dev_ctx.ias_zone_attr.ias_cie_address,
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

static void check_contact(){
    zb_ret_t zb_err_code;
    if(nrf_drv_gpiote_in_is_set(KONTACTOR_PIN)){
        // kontaktron otwarty
        NRF_LOG_INFO("Contact open");
        // zb_buf_get_out_delayed(izs_send_enroll_req);
        zb_err_code = zb_buf_get_out_delayed_ext(contact_send_notification_req,(zb_bool_t)true, 0);
        ZB_ERROR_CHECK(zb_err_code);
    } else{
        // kontaktron złączony
        NRF_LOG_INFO("Contact close");
        zb_err_code = zb_buf_get_out_delayed_ext(contact_send_notification_req,(zb_bool_t)false, 0);
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
        ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH,
        0);

}

static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){

    switch (pin)
    {
    case BOARD_BUTTON_PIN:
        if(action == NRF_GPIOTE_POLARITY_HITOLO){
            NRF_LOG_INFO("Button press");
            zb_bdb_reset_via_local_action(0);
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

    // inicjalizacja modułu GPIOTE
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // struktura konfiguracyjna wyjściue - dioda LED
    nrf_drv_gpiote_out_config_t led_out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    // inicjalizacja wyjscia
    err_code = nrf_drv_gpiote_out_init(BOARD_LED_PIN, &led_out_config);
    APP_ERROR_CHECK(err_code);

    // struktura konfiguracyjna wejście - przycisk
    nrf_drv_gpiote_in_config_t button_in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    button_in_config.pull = NRF_GPIO_PIN_PULLUP;
    // inicjalizacja wyścia
    err_code = nrf_drv_gpiote_in_init(BOARD_BUTTON_PIN, &button_in_config, gpio_handler);
    APP_ERROR_CHECK(err_code);
    // właczenie mechanizmu eventow przychodzących od pinu
    nrf_drv_gpiote_in_event_enable(BOARD_BUTTON_PIN, true);

    // struktura konfiguracyjna wejście - kontraktron
    nrf_drv_gpiote_in_config_t contact_in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    contact_in_config.pull = NRF_GPIO_PIN_PULLUP;
    // inicjalizacja wyścia
    err_code = nrf_drv_gpiote_in_init(KONTACTOR_PIN, &contact_in_config, gpio_handler);
    APP_ERROR_CHECK(err_code);
    // właczenie mechanizmu eventow przychodzących od pinu
    nrf_drv_gpiote_in_event_enable(KONTACTOR_PIN, true);
}

void write_response_request(zb_bufid_t bufid){
     NRF_LOG_INFO("ZB_ZCL_GENERAL_SEND_WRITE_ATTR_RESP");
    zb_uint8_t * p_cmd_buf;
    zb_uint8_t  seq_number = ZCL_CTX().seq_number;
     
    ZB_ZCL_GENERAL_INIT_WRITE_ATTR_RESP_EXT(bufid,
                                            p_cmd_buf,
                                            ZB_ZCL_FRAME_DIRECTION_TO_CLI,
                                            seq_number,
                                            0,
                                            0
    );
    ZB_ZCL_GENERAL_SUCCESS_WRITE_ATTR_RESP(p_cmd_buf);
    ZB_ZCL_GENERAL_SEND_WRITE_ATTR_RESP(bufid,
                                        p_cmd_buf,
                                        m_dev_ctx.ias_zone_attr.ias_cie_address,
                                        ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                        m_dev_ctx.ias_zone_attr.cie_ep,
                                        HA_CONTACT_ENDPOINT,
                                        ZB_AF_HA_PROFILE_ID,
                                        ZB_ZCL_CLUSTER_ID_IAS_ZONE,
                                        NULL
    );
}

zb_uint8_t izs_zcl_cmd_handler(zb_uint8_t param)
{
  zb_bufid_t zcl_cmd_buf = param;
  zb_zcl_parsed_hdr_t *cmd_info = ZB_BUF_GET_PARAM(zcl_cmd_buf, zb_zcl_parsed_hdr_t);
  zb_uint8_t cmd_processed = ZB_FALSE;
  TRACE_MSG(TRACE_APP1, ">> izs_zcl_cmd_handler %i", (FMT__H, param));
  ZB_ZCL_DEBUG_DUMP_HEADER(cmd_info);
  TRACE_MSG(TRACE_APP1, "payload size: %i", (FMT__D, zb_buf_len(zcl_cmd_buf)));
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
               /* Check that we receive the Write Attributes cmd for the CIE address
                  If so, start fast polling */
               write_attr_req = (zb_zcl_write_attr_req_t*)zb_buf_begin(zcl_cmd_buf);
               if (ZB_ZCL_ATTR_IAS_ZONE_IAS_CIE_ADDRESS_ID == write_attr_req->attr_id)
               {
                 m_dev_ctx.ias_zone_attr.cie_short_addr = cmd_info->addr_data.common_data.source.u.short_addr;
                 m_dev_ctx.ias_zone_attr.cie_ep = cmd_info->addr_data.common_data.src_endpoint;
                 NRF_LOG_INFO("CIE address is updated. New cie_short_addr = 0x%x, cie_ep = 0x%x ",
                          m_dev_ctx.ias_zone_attr.cie_short_addr, m_dev_ctx.ias_zone_attr.cie_ep);

                NRF_LOG_INFO("auto enroll request mode - send EnrollRequest");
                
                zb_buf_get_out_delayed(izs_send_enroll_req);
                zb_buf_get_out_delayed(write_response_request);

                cmd_processed = ZB_TRUE;
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
            NRF_LOG_INFO("ZB_ZCL_IAS_ZONE_ZONE_ENROLL_RESPONSE - STATUS: 0x%x, ZONE_ID: 0x%x", response_param->enroll_response, response_param->zone_id);
            if(response_param->enroll_response == ZB_ZCL_STATUS_SUCCESS){
                m_dev_ctx.ias_zone_attr.zone_state = ZB_ZCL_IAS_ZONE_ZONESTATE_ENROLLED;
                m_dev_ctx.ias_zone_attr.zone_id = response_param->zone_id;
                zb_nvram_write_dataset(ZB_NVRAM_HA_DATA);
                check_contact();
            }
           
        //   TRACE_MSG(TRACE_APP1, "ZB_ZCL_CMD_IAS_ZONE_ZONE_ENROLL_RESPONSE_ID", (FMT__0));
        //   src_ep = ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).src_endpoint;
        //   src_addr = ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).source.u.short_addr;
        //   /* Note: after a call to this function buffer becomes invalid */
        //   cmd_processed = zb_zcl_process_ias_zone_specific_commands(param);
        //   TRACE_MSG(TRACE_APP1, "cmd_processed %hd", (FMT__H, cmd_processed));
        //   if (IZS_DEVICE_IS_ENROLLED())
        //   {
            // TRACE_MSG(TRACE_APP1, "device is enrolled, save data", (FMT__0));
            // /* set cie short addr and ep values; correct values maybe
            //  * already set in izs_zcl_cmd_handler() */
            // g_device_ctx.zone_attr.cie_ep = src_ep;
            // g_device_ctx.zone_attr.cie_short_addr = src_addr;
            // zb_zcl_poll_control_set_client_addr(IZS_DEVICE_ENDPOINT, src_addr, src_ep);

            // zb_nvram_write_dataset(ZB_NVRAM_HA_DATA);
 
//             zb_buf_get_out_delayed(izs_go_on_guard);
// #ifdef IZS_OTA
//             izs_check_and_get_ota_server(0);
// #endif
// #ifdef FAST_POLLING_DURING_COMMISSIONING
//             izs_start_fast_polling_for_commissioning(IZS_DEVICE_TURBO_POLL_AFTER_ENROLL_DURATION * 1000l);
// #endif /* FAST_POLLING_DURING_COMMISSIONING */
//             /* force a ZoneStatusChange in case of silent tamper or motion alarms */
//             ZB_SCHEDULE_APP_ALARM(izs_read_sensor_status, 0, (ZB_TIME_ONE_SECOND>>1));
        //   }
          
          cmd_processed = ZB_TRUE;
          break;
        }
        default:
          NRF_LOG_WARNING("skip command %hd", cmd_info->cmd_id);
          break;
      }
    }
  }
  
  NRF_LOG_INFO("<< izs_zcl_cmd_handler processed %hd", cmd_processed);
  return cmd_processed;
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
        case ZB_ZCL_CMD_WRITE_ATTRIB:
            NRF_LOG_INFO("ZB_ZCL_CMD_WRITE_ATTRIB");
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

    ZB_AF_SET_ENDPOINT_HANDLER(HA_CONTACT_ENDPOINT, izs_zcl_cmd_handler);

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
