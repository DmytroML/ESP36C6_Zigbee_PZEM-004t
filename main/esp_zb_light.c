/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_color_dimmable_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "pzem004tv3.h"
#include "temp_sensor_driver.h"
#include "switch_driver.h"

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

static const char *TAG = "ESP_ZB_ZB_ZCZR";
#define NEW_ADDRESS     0x03            // <= New Device Address (change is not permanent)

/* @brief Set ESP32  Serial Configuration */
pzem_setup_t pzConf =
{
    .pzem_uart   = UART_NUM_1,              /*  <== Specify the UART you want to use, UART_NUM_0, UART_NUM_1, UART_NUM_2 (ESP32 specific) */
    .pzem_rx_pin = GPIO_NUM_4,             /*  <== GPIO for RX */
    .pzem_tx_pin = GPIO_NUM_5,             /*  <== GPIO for TX */
    .pzem_addr   = 0xF8,      /*  If your module has a different address, specify here or update the variable in pzem004tv3.h */
};
//#define TXD_PIN (GPIO_NUM_5)
//#define RXD_PIN (GPIO_NUM_4)
_current_values_t pzValues;            /* Measured values */
TaskHandle_t PMonTHandle = NULL;
void PMonTask( void * pz );

bool  connected = false;
uint16_t RMSVOLTAGE= 0; //!< Represents the most recent RMS voltage reading in @e Volts (V). 
uint16_t RMSCURRENT= 0; //!< Represents the most recent RMS current reading in @e Amps (A). 
int16_t ACTIVE_POWER= 0;  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
uint16_t POWER_FACTOR= 0;  //!< Contains the single phase or PhaseA, Power Factor ratio in 1/100th. 
  /* AC Measurement (Non Phase) */
uint16_t AC_FREQUENCY= 0;  //!< The ACFrequency attribute represents the most recent AC Frequency reading in Hertz (Hz). 
uint64_t Summation_delivered= 0;  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 




static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static switch_func_pair_t button_func_pair[] = {    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}};

static void esp_app_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        /* Send report attributes command */
        esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
        report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
        esp_zb_lock_release();
        ESP_EARLY_LOGI(TAG, "Send 'report attributes' command");

    }
}

static void esp_app_temp_sensor_handler(float temperature)
{

    int16_t measured_value = zb_temperature_to_s16(temperature);




    /* Update temperature sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured_value, false);


    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &RMSVOLTAGE, false);  //!< Represents the most recent RMS voltage reading in @e Volts (V). 
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSCURRENT_ID, &RMSCURRENT, false); //!< Represents the most recent RMS current reading in @e Amps (A). 
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &ACTIVE_POWER, false);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_POWER_FACTOR_ID, &POWER_FACTOR, false);  //!< Contains the single phase or PhaseA, Power Factor ratio in 1/100th. 
   esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_ID, &AC_FREQUENCY, false);  //!< The ACFrequency attribute represents the most recent AC Frequency reading in Hertz (Hz). 
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &Summation_delivered, false);




    esp_zb_lock_release();

}
/********************* Define functions **************************/
static esp_err_t deferred_driver_init(void)
{

    temperature_sensor_config_t temp_sensor_config =
                                TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);


    ESP_RETURN_ON_ERROR(temp_sensor_driver_init(&temp_sensor_config, ESP_TEMP_SENSOR_UPDATE_INTERVAL, esp_app_temp_sensor_handler), TAG,
                        "Failed to initialize temperature sensor");
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_app_buttons_handler), ESP_FAIL, TAG,
                        "Failed to initialize switch driver");

    return ESP_OK;
}
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}
/*
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;
    uint8_t light_level = 0;
    uint16_t light_color_x = 0;
    uint16_t light_color_y = 0;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_COLOR_DIMMABLE_LIGHT_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                light_driver_set_power(light_state);
            } else {
                ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                light_color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : light_color_x;
                light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                                                                      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
                                     ->data_p;
                ESP_LOGI(TAG, "Light color x changes to 0x%x", light_color_x);
            } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
                       message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                light_color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : light_color_y;
                light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                                                                      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
                                     ->data_p;
                ESP_LOGI(TAG, "Light color y changes to 0x%x", light_color_y);
            } else {
                ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            light_driver_set_color_xy(light_color_x, light_color_y);
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : light_level;
                light_driver_set_level((uint8_t)light_level);
                ESP_LOGI(TAG, "Light level changes to %d", light_level);
            } else {
                ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}
*/
static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);


    //esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    esp_zb_configuration_tool_cfg_t sensor_cfg =ESP_ZB_DEFAULT_CONFIGURATION_TOOL_CONFIG();
    
    //esp_zb_ep_list_t *esp_zb_color_dimmable_light_ep = esp_zb_color_dimmable_light_ep_create(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, &sensor_cfg);

    esp_zb_ep_list_t *esp_zb_sensor_ep = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        //.app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_id = ESP_ZB_HA_COMBINED_INTERFACE_DEVICE_ID,
        .app_device_version = 0
        
    };
    esp_zb_endpoint_config_t endpoint_config_9 = {
        .endpoint = 9,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        //.app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_id = ESP_ZB_HA_COMBINED_INTERFACE_DEVICE_ID,
        .app_device_version = 0
        
    };
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(sensor_cfg.basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, FIRMWARE_VERSION));
    //char DATE_CODE[] = "\n""20240420"; 
    //ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, &DATE_CODE));


    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(sensor_cfg.identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    //ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(sensor_cfg.temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    uint16_t undefined_value;
    undefined_value = 0x0000;
     /* Temperature cluster */
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    //ESP_ZB_ZCL_CLUSTER_ID_METERING

    /* PZEM_004t cluster */

    esp_zb_attribute_list_t *esp_zb_PZEM_004t_1_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT);
    //MeasurementType
    uint32_t e_meas_type = (uint32_t)0x00000008;    
    //esp_zb_zcl_electrical_measurement_measurement_type_t e_meas_type =ESP_ZB_ZCL_ELECTRICAL_MEASUREMENT_PHASE_A_MEASUREMENT;
    //uint8_t tttt = 0x03;
    //esp_zb_electrical_meas_profilr_a
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_ID,&e_meas_type);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_PHB_ID, &undefined_value);
    /* AC Measurement (Phase A)*/
    //TOLERANCE
    //esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_toler, &undefined_value);  
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &undefined_value);         /*!< Represents the most recent RMS voltage reading in @e Volts (V). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MIN_ID, &undefined_value);   /*!< Represents the lowest RMS voltage value measured in Volts (V). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MAX_ID , &undefined_value);   /*!< Represents the highest RMS voltage value measured in Volts (V). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSCURRENT_ID, &undefined_value);         /*!< Represents the most recent RMS current reading in @e Amps (A). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_MIN_ID , &undefined_value);   /*!< Represents the lowest RMS current value measured in Amps (A). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_MAX_ID , &undefined_value);   /*!< Represents the highest RMS current value measured in Amps (A). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID  , &undefined_value);     /*!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MIN_ID , &undefined_value);  /*!< Represents the lowest AC power value measured in Watts (W). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MAX_ID  , &undefined_value); /*!< Represents the highest AC power value measured in Watts (W). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_POWER_FACTOR_ID, &undefined_value);       /*!< Contains the single phase or PhaseA, Power Factor ratio in 1/100th. */
    /* AC Measurement (Non Phase) */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_ID, &undefined_value);     /*!< The ACFrequency attribute represents the most recent AC Frequency reading in Hertz (Hz). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_MIN_ID,&undefined_value);     /*!< The ACFrequencyMin attribute represents the lowest AC Frequency value measured in Hertz (Hz). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_MAX_ID, &undefined_value);     /*!< The ACFrequencyMax attribute represents the highest AC Frequency value measured in Hertz (Hz). */
    //вилучити esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_APPARENT_POWER_ID, &undefined_value);     /*!< Reactive power represents the current demand of reactive power delivered or received at the premises, in kVAr. */
 
  /* AC Formatting */
    uint16_t u_value =  (uint16_t)(1);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_MULTIPLIER_ID, &u_value);     //!< Provides a value to be multiplied against the @e InstantaneousCurrent and @e RMSCurrent attributes 
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_MULTIPLIER_ID, &u_value);
    uint16_t u_value_1000 = (uint16_t)(1000); 
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_DIVISOR_ID,&u_value_1000);    //!< Provides a value to be divided against the @e ACCurrent, @e InstantaneousCurrent and @e RMSCurrent attributes. 
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_DIVISOR_ID,&u_value_1000);
    
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_electrical_meas_cluster(cluster_list, esp_zb_PZEM_004t_1_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));  
     

    //задаю лічильник саожитої енергії
    esp_zb_attribute_list_t *esp_zb_PZEM_004t_2_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_METERING);
    uint64_t undefined_value1 = (uint64_t)(10);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_U48
                            ,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY
                            ,&undefined_value1);  
    uint8_t u_value10 =  (uint8_t)(0);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_8BITMAP
                            ,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY
                            ,&u_value10);  
    uint16_t u_value11 =  (uint16_t)(1);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_MULTIPLIER_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_U24
                            ,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY
                            ,&u_value11);  

    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_8BITMAP
                            ,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY
                            ,&u_value10); 
    uint8_t u_value12 =  (uint8_t)(0);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM
                            ,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY
                            ,&u_value12); 
    uint16_t u_value13 =  (uint16_t)(1000);                        
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_DIVISOR_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_U24
                            ,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY
                            ,&u_value13);

                       
    //ESP_ZB_ZCL_ATTR_ACCESS_REPORTING
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_metering_cluster(cluster_list, esp_zb_PZEM_004t_2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    //---------------------------------------------------------------------------------------------------------------------------------------




    esp_zb_ep_list_add_ep(esp_zb_sensor_ep, cluster_list, endpoint_config);
    //esp_zb_ep_list_add_ep(esp_zb_sensor_ep, cluster_list, endpoint_config_9);






    esp_zb_device_register(esp_zb_sensor_ep);
    //можливо треба повернути, може відповідати за режим роутера
    //esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}
/*зчитуємо дані з PZEM-004t*/
void PMonTask( void * pz )
{
        while (1)
    {
    //for( ;; )
    //{

        // Update config struct with new address
        pzConf.pzem_addr = 0x03;
        PzemGetValues( &pzConf, &pzValues );
        //printf( "Vrms: %.1fV - Irms: %.3fA - P: %.1fW - E: %.2fWh - Freq: %.1fHz - PF: %.2f\n", pzValues.voltage, pzValues.current, pzValues.power, pzValues.energy, pzValues.frequency, pzValues.pf );
        ESP_LOGI("PZEM-004t 0x03:","Vrms: %.3fV - Irms: %.3fA - P: %.5fW - E: %.5fWh - Freq: %.1fHz - PF: %.2f", pzValues.voltage, pzValues.current, pzValues.power, pzValues.energy, pzValues.frequency, pzValues.pf );
        //printf( "Freq: %.1fHz - PF: %.2f\n", pzValues.frequency, pzValues.pf );
        //ESP_LOGI( TAG, "Stack High Water Mark: %ld Bytes free", ( unsigned long int ) uxTaskGetStackHighWaterMark( NULL ) );     /* Show's what's left of the specified stacksize */

        // Update config struct with new address
        pzConf.pzem_addr = 0x02;
        PzemGetValues( &pzConf, &pzValues );
        //printf( "Vrms: %.1fV - Irms: %.3fA - P: %.1fW - E: %.2fWh - Freq: %.1fHz - PF: %.2f\n", pzValues.voltage, pzValues.current, pzValues.power, pzValues.energy, pzValues.frequency, pzValues.pf );
        ESP_LOGI("PZEM-004t 0x02:","Vrms: %.3fV - Irms: %.3fA - P: %.5fW - E: %.5fWh - Freq: %.1fHz - PF: %.2f", pzValues.voltage, pzValues.current, pzValues.power, pzValues.energy, pzValues.frequency, pzValues.pf );
        //printf( "Freq: %.1fHz - PF: %.2f\n", pzValues.frequency, pzValues.pf );


        // Update config struct with new address
        pzConf.pzem_addr = 0x01;
        PzemGetValues( &pzConf, &pzValues );
        //printf( "Vrms: %.1fV - Irms: %.3fA - P: %.1fW - E: %.2fWh - Freq: %.1fHz - PF: %.2f\n", pzValues.voltage, pzValues.current, pzValues.power, pzValues.energy, pzValues.frequency, pzValues.pf );
        ESP_LOGI("PZEM-004t 0x01:","Vrms: %.3fV - Irms: %.3fA - P: %.5fW - E: %.5fWh - Freq: %.1fHz - PF: %.2f", pzValues.voltage, pzValues.current, pzValues.power, pzValues.energy, pzValues.frequency, pzValues.pf );
        //printf( "Freq: %.1fHz - PF: %.2f\n", pzValues.frequency, pzValues.pf );






        RMSVOLTAGE= (uint16_t)(pzValues.voltage); //!< Represents the most recent RMS voltage reading in @e Volts (V). 
        //RMSVOLTAGE = (uint16_t)(215);
        RMSCURRENT= (uint16_t)(pzValues.current*1000); //!< Represents the most recent RMS current reading in @e Amps (A). 
        //RMSCURRENT = (uint16_t)(1101);
        ACTIVE_POWER= (int16_t)(pzValues.power*1000);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
        //ACTIVE_POWER = (uint16_t)(1);
        POWER_FACTOR= (int16_t)(pzValues.pf*100);  //!< Contains the single phase or PhaseA, Power Factor ratio in 1/100th. 
        //POWER_FACTOR=(uint16_t)1;
        /* AC Measurement (Non Phase) */
        AC_FREQUENCY= (int16_t)(pzValues.frequency);  //!< The ACFrequency attribute represents the most recent AC Frequency reading in Hertz (Hz). 
        //AC_FREQUENCY=(uint16_t)60;     
        Summation_delivered= (uint64_t)(pzValues.energy*1000);  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 
        //Summation_delivered= (uint64_t)(16);  
            
        vTaskDelay( pdMS_TO_TICKS( 15000 ) );   




    }

    //vTaskDelete( NULL );
}


void app_main(void)
{
    PzemInit( &pzConf );

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    xTaskCreate(PMonTask, "Zigbee_main", 4096, NULL, 1, NULL);   
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
