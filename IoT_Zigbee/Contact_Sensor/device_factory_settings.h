#ifndef FACTORY_SET_H
#define FACTORY_SET_H

/* Basic cluster attributes initial values. */
#define CONTACT_INIT_BASIC_APP_VERSION       02                                    /**< Version of the application software (1 byte). */
#define CONTACT_INIT_BASIC_STACK_VERSION     10                                    /**< Version of the implementation of the Zigbee stack (1 byte). */
#define CONTACT_INIT_BASIC_HW_VERSION        11                                    /**< Version of the hardware of the device (1 byte). */
#define CONTACT_INIT_BASIC_MANUF_NAME        "Stefan Tobiasiewicz"                              /**< Manufacturer name (32 bytes). */
#define CONTACT_INIT_BASIC_MODEL_ID          "Contact_v1"                  /**< Model number assigned by manufacturer (32-bytes long string). */
#define CONTACT_INIT_BASIC_DATE_CODE         "20211101"                            /**< First 8 bytes specify the date of manufacturer of the device in ISO 8601 format (YYYYMMDD). The rest (8 bytes) are manufacturer specific. */
#define CONTACT_INIT_BASIC_POWER_SOURCE      ZB_ZCL_BASIC_POWER_SOURCE_BATTERY   /**< Type of power sources available for the device. For possible values see section 3.2.2.2.8 of ZCL specification. */
#define CONTACT_INIT_BASIC_LOCATION_DESC     "Office desk"                         /**< Describes the physical location of the device (16 bytes). May be modified during commisioning process. */
#define CONTACT_INIT_BASIC_PH_ENV            ZB_ZCL_BASIC_ENV_UNSPECIFIED          /**< Describes the type of physical environment. For possible values see section 3.2.2.2.10 of ZCL specification. */

#endif
