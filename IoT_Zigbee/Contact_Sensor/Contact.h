#ifndef CONTACT_H
#define CONTACT_H

#include "zb_zcl_ias_zone.h"
#include "zb_ha_ias_zone.h"


#define HA_CONTACT_ENDPOINT        1

/** Input clusters number. */
#define ZB_HA_CONTACT_IN_CLUSTER_NUM  4

/** Output clusters number. */
#define ZB_HA_CONTACT_OUT_CLUSTER_NUM 0

/*! @internal Number of attribute for reporting on Contact */
#define ZB_HA_CONTACT_REPORT_ATTR_COUNT \
  (ZB_ZCL_IAS_ZONE_REPORT_ATTR_COUNT + ZB_ZCL_POWER_CONFIG_REPORT_ATTR_COUNT)

/* Declare endpoint for Contact */
#define ZB_HA_CONTACT_EP(ep_name, ep_id, cluster_list)          \
      ZB_HA_DECLARE_CONTACT_SIMPLE_DESC(                       \
          ep_name,                                              \
          ep_id,                                                \
          ZB_HA_CONTACT_IN_CLUSTER_NUM,                        \
          ZB_HA_CONTACT_OUT_CLUSTER_NUM);                      \
  ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info## device_ctx_name,  \
                                     ZB_HA_CONTACT_REPORT_ATTR_COUNT); \
  ZB_AF_DECLARE_ENDPOINT_DESC(                                          \
    ep_name,                                                            \
            ep_id,                                              \
            ZB_AF_HA_PROFILE_ID,                                \
            0,                                                  \
            NULL,                                               \
            ZB_ZCL_ARRAY_SIZE(                                  \
                cluster_list,                                   \
                zb_zcl_cluster_desc_t),                         \
            cluster_list,                                       \
    (zb_af_simple_desc_1_1_t*)&simple_desc_##ep_name,                   \
    ZB_HA_CONTACT_REPORT_ATTR_COUNT, reporting_info## device_ctx_name, \
    0, NULL)


/**
 *  @brief Declare simple descriptor for IAS Zone
 *  @param ep_name - endpoint variable name.
 *  @param ep_id [IN] - endpoint ID.
 *  @param in_clust_num [IN] - number of supported input clusters.
 *  @param out_clust_num [IN] - number of supported output clusters.
 *  @note in_clust_num, out_clust_num should be defined by numeric constants, not variables or any
 *  definitions, because these values are used to form simple descriptor type name.
 */
#define ZB_HA_DECLARE_CONTACT_SIMPLE_DESC(                     \
  ep_name, ep_id, in_clust_num, out_clust_num)                  \
      ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);      \
      ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num)       \
            simple_desc_##ep_name =                             \
      {                                                         \
        ep_id,                                                  \
        ZB_AF_HA_PROFILE_ID,                                    \
        ZB_HA_IAS_ZONE_ID,                                      \
        ZB_HA_IAS_ZONE_VER,                                     \
        0,                                                      \
        in_clust_num,                                           \
        out_clust_num,                                          \
        {                                                       \
          ZB_ZCL_CLUSTER_ID_BASIC,                              \
          ZB_ZCL_CLUSTER_ID_IDENTIFY,                           \
          ZB_ZCL_CLUSTER_ID_IAS_ZONE,                           \
          ZB_ZCL_CLUSTER_ID_POWER_CONFIG,                       \
        }                                                       \
      }



/**
 *  @brief Declare cluster list for Contact
 *  @param cluster_list_name [IN] - cluster list variable name.
 *  @param basic_attr_list [IN] - attribute list for Basic cluster.
 *  @param identify_attr_list [IN] - attribute list for Identify cluster.
 *  @param ias_zone_attr_list [IN] - attribute list for IAS Zone cluster.
 *  @param power_config_attr_list [IN] - attribute list for Power configuration cluster.
 */
#define ZB_HA_CONTACT_CLUSTER_LIST(                     \
  cluster_list_name,                                                           \
  basic_attr_list,                                                             \
  identify_attr_list,                                                          \
  ias_zone_attr_list,                                                          \
  power_config_attr_list)                                                      \
      zb_zcl_cluster_desc_t cluster_list_name[] =                              \
      {                                                                        \
        ZB_ZCL_CLUSTER_DESC(                                                   \
          ZB_ZCL_CLUSTER_ID_BASIC,                                             \
          ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),                   \
          (basic_attr_list),                                                   \
          ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
          ZB_ZCL_MANUF_CODE_INVALID                                            \
        ),                                                                     \
        ZB_ZCL_CLUSTER_DESC(                                                   \
          ZB_ZCL_CLUSTER_ID_IDENTIFY,                                          \
          ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t),                \
          (identify_attr_list),                                                \
          ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
          ZB_ZCL_MANUF_CODE_INVALID                                            \
        ),                                                                     \
        ZB_ZCL_CLUSTER_DESC(                                                   \
          ZB_ZCL_CLUSTER_ID_IAS_ZONE,                                          \
          ZB_ZCL_ARRAY_SIZE(ias_zone_attr_list, zb_zcl_attr_t),                \
          (ias_zone_attr_list),                                                \
          ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
          ZB_ZCL_MANUF_CODE_INVALID                                            \
        ),                                                                     \
        ZB_ZCL_CLUSTER_DESC(                                                   \
          ZB_ZCL_CLUSTER_ID_POWER_CONFIG,                                      \
          ZB_ZCL_ARRAY_SIZE(power_config_attr_list, zb_zcl_attr_t),            \
          (power_config_attr_list),                                            \
          ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
          ZB_ZCL_MANUF_CODE_INVALID                                            \
        )                                                                      \
      }

/**
  @brief Declare application's device context for Contact
  @param device_ctx - device context variable
  @param ep_name - endpoint variable name
*/
#define ZB_HA_DECLARE_CONTACT_CTX(device_ctx, ep_name)                                                    \
  ZBOSS_DECLARE_DEVICE_CTX_1_EP(device_ctx, ep_name)


typedef struct {
    zb_uint8_t zone_state; 
    zb_uint16_t zone_type;  
    zb_uint16_t zone_status; 
    zb_uint64_t ias_cie_address; 
    zb_uint8_t  zone_id;
    zb_uint8_t  number_of_zone_sens_levels_supported;
    zb_uint8_t  current_zone_sens_level;
    zb_uint16_t cie_short_addr; 
    zb_uint8_t  cie_ep;
} ias_zone_attr_t;


//ias_zone_attr_t ias_zone_attr;


/** @brief Declare attribute list for IAS Zone cluster - server side (extended attribute set)
    @param attr_list - attribute list name
    @param zone_state - pointer to variable to store ZoneState attribute
    @param zone_type - pointer to variable to store ZoneType attribute
    @param zone_status - pointer to variable to store ZoneStatus attribute
    @param ias_cie_address - pointer to variable to store IAS-CIE address attribute
    @param zone_id - pointer to variable to store Zone ID attribute
    @param number_of_zone_sens_levels_supported - pointer to variable to store
    NumberOfZoneSensitivityLevelsSupported attribute
    @param current_zone_sens_level - pointer to variable to store CurrentZoneSensitivityLevel attribute
    @param cie_short_addr - custom attribute to store CIE short address
    @param cie_ep - custom attribute to store CIE Endpoint number
*/
/*
ZB_ZCL_DECLARE_IAS_ZONE_ATTRIB_LIST_EXT(ias_zone_attr_list,
                                    &ias_zone_attr.zone_state,
                                    &ias_zone_attr.zone_type,
                                    &ias_zone_attr.zone_status,
                                    &ias_zone_attr.number_of_zone_sens_levels_supported,
                                    &ias_zone_attr.current_zone_sens_level,
                                    &ias_zone_attr.ias_cie_address,
                                    &ias_zone_attr.zone_id,
                                    &ias_zone_attr.cie_short_addr,
                                    &ias_zone_attr.cie_ep
                                    );
*/

typedef struct
{
    zb_uint8_t voltage;  
    zb_uint8_t size;
    zb_uint8_t quantity;
    zb_uint8_t rated_voltage;
    zb_uint8_t alarm_mask;
    zb_uint8_t voltage_min_threshold;
    zb_uint8_t battery_percentage_remaining;
} power_config_attr_t;

//power_config_attr_t power_config_attr;

/** @brief Declare attribute list for Power Configuration cluster - server side
    @param attr_list - attribute list name
    @param voltage - pointer to variable to store BatteryVoltage attribute
    @param size - pointer to variable to store BatterySize attribute
    @param quantity - pointer to variable to store BatteryQuantity attribute
    @param rated_voltage - pointer to variable to store BatteryRatedVoltage attribute
    @param alarm_mask - pointer to variable to store BatteryAlarmMask attribute
    @param voltage_min_threshold - pointer to variable to store BatteryVoltageMinThreshold attribute
    @param battery_percentage_remaining - pointer to variable to store BatteryPercentageRemaining attribute
*/
#define ZB_ZCL_DECLARE_POWER_CONFIG_ATTRIB_LIST_EXTEND_WITH_PERCENTAGE(attr_list,                                                      \
    voltage, size, quantity, rated_voltage, alarm_mask, voltage_min_threshold, battery_percentage_remaining)                                  \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST(attr_list)                                                                   \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID((voltage),),                               \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID((size),),                                     \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID((quantity),),                             \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID((rated_voltage),),                   \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID((alarm_mask ),),                        \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID((voltage_min_threshold),),   \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID((battery_percentage_remaining),),    \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

#endif

