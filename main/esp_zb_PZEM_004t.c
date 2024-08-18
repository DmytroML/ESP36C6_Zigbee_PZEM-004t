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

#include "esp_zb_PZEM_004t.h"
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

/* @brief Set ESP32  Serial Configuration */
pzem_setup_t pzConf =
{
    .pzem_uart   = UART_NUM_1,              /*  <== Specify the UART you want to use, UART_NUM_0, UART_NUM_1, UART_NUM_2 (ESP32 specific) */
    .pzem_rx_pin = GPIO_NUM_4,             /*  <== GPIO for RX */
    .pzem_tx_pin = GPIO_NUM_5,             /*  <== GPIO for TX */
    //.pzem_addr   = 0xF8,      /*  If your module has a different address, specify here or update the variable in pzem004tv3.h */
    .pzem_addr   = 0x01, 
};
//#define TXD_PIN (GPIO_NUM_5)
//#define RXD_PIN (GPIO_NUM_4)
_current_values_t pzValues;            /* Measured values */
TaskHandle_t PMonTHandle = NULL;
void PMonTask( void * pz );

bool  connected = false;
uint16_t RMSVOLTAGE= 0; //!< Represents the most recent RMS voltage reading in @e Volts (V). 
int16_t ACTIVE_POWER= 0;  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
uint64_t Summation_delivered= 0;  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 

uint16_t RMSVOLTAGE_1= 0; //!< Represents the most recent RMS voltage reading in @e Volts (V). 
int16_t ACTIVE_POWER_1= 0;  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
uint64_t Summation_delivered_1= 0;  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 

uint16_t RMSVOLTAGE_2= 0; //!< Represents the most recent RMS voltage reading in @e Volts (V). 
int16_t ACTIVE_POWER_2= 0;  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
uint64_t Summation_delivered_2= 0;  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 

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
    //esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //                            ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &RMSVOLTAGE, false);  //!< Represents the most recent RMS voltage reading in @e Volts (V). 
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &ACTIVE_POWER, false);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W).      esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &Summation_delivered, false);


    //esp_zb_zcl_set_attribute_val(9, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //                            ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &RMSVOLTAGE_1, false);  //!< Represents the most recent RMS voltage reading in @e Volts (V). 
    esp_zb_zcl_set_attribute_val(9, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &ACTIVE_POWER_1, false);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
    esp_zb_zcl_set_attribute_val(9, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &Summation_delivered_1, false);


    //esp_zb_zcl_set_attribute_val(8, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //                            ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &RMSVOLTAGE_2, false);  //!< Represents the most recent RMS voltage reading in @e Volts (V). 
    esp_zb_zcl_set_attribute_val(8, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &ACTIVE_POWER_2, false);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
    esp_zb_zcl_set_attribute_val(8, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &Summation_delivered_2, false);



    esp_zb_lock_release();
}
/********************* Define functions **************************/
static esp_err_t deferred_driver_init(void)
{
    temperature_sensor_config_t temp_sensor_config =TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);
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

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_configuration_tool_cfg_t sensor_cfg =ESP_ZB_DEFAULT_CONFIGURATION_TOOL_CONFIG();
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
        esp_zb_endpoint_config_t endpoint_config_8 = {
        .endpoint = 8,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        //.app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_id = ESP_ZB_HA_COMBINED_INTERFACE_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(sensor_cfg.basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    //ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, FIRMWARE_VERSION));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(sensor_cfg.identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    uint16_t undefined_value;
    undefined_value = 0x0000;
     /* Temperature cluster */
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    //ESP_ZB_ZCL_CLUSTER_ID_METERING
    esp_zb_attribute_list_t *esp_zb_PZEM_004t_1_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT);
    //MeasurementType
    uint16_t e_meas_type = (uint16_t)0x0008;    
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_ID,&e_meas_type);

    //esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_PHB_ID, &undefined_value);
    /* AC Measurement (Phase A)*/
    //TOLERANCE
    //esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &undefined_value);         /*!< Represents the most recent RMS voltage reading in @e Volts (V). */
    //esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MIN_ID, &undefined_value);   /*!< Represents the lowest RMS voltage value measured in Volts (V). */
    //esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MAX_ID , &undefined_value);   /*!< Represents the highest RMS voltage value measured in Volts (V). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID  , &undefined_value);     /*!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MIN_ID , &undefined_value);  /*!< Represents the lowest AC power value measured in Watts (W). */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_PZEM_004t_1_cluster,ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MAX_ID  , &undefined_value); /*!< Represents the highest AC power value measured in Watts (W). */
 
    /* AC Formatting */
    uint16_t u_value =  (uint16_t)(1);
    uint16_t u_value_1000 = (uint16_t)(10); 

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_electrical_meas_cluster(cluster_list, esp_zb_PZEM_004t_1_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));  
     
    //задаю лічильник саожитої енергії
    esp_zb_attribute_list_t *esp_zb_PZEM_004t_2_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_METERING);
    uint64_t undefined_value1 = (uint64_t)(10);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_U48,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,&undefined_value1);  
    uint8_t u_value10 =  (uint8_t)(0);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_8BITMAP,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,&u_value10);  
    uint16_t u_value11 =  (uint16_t)(1);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_MULTIPLIER_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_U24,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,&u_value11);  

    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_8BITMAP,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,&u_value10); 
    uint8_t u_value12 =  (uint8_t)(0);
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,&u_value12); 
    uint16_t u_value13 =  (uint16_t)(1000);      //!!!!!!                  
    esp_zb_cluster_add_attr(esp_zb_PZEM_004t_2_cluster,ESP_ZB_ZCL_CLUSTER_ID_METERING,ESP_ZB_ZCL_ATTR_METERING_DIVISOR_ID
                            ,ESP_ZB_ZCL_ATTR_TYPE_U24,ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,&u_value13);
               
    //ESP_ZB_ZCL_ATTR_ACCESS_REPORTING
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_metering_cluster(cluster_list, esp_zb_PZEM_004t_2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    //---------------------------------------------------------------------------------------------------------------------------------------

    esp_zb_ep_list_add_ep(esp_zb_sensor_ep, cluster_list, endpoint_config);
    esp_zb_ep_list_add_ep(esp_zb_sensor_ep, cluster_list, endpoint_config_9);
    esp_zb_ep_list_add_ep(esp_zb_sensor_ep, cluster_list, endpoint_config_8);

    esp_zb_device_register(esp_zb_sensor_ep);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}
/*зчитуємо дані з PZEM-004t*/
void PMonTask( void * pz )
{
    while (1)
    {
        vTaskDelay( pdMS_TO_TICKS( 100 ) ); 
        // Update config struct with new address
        pzConf.pzem_addr = 0x01;
        PzemGetValues( &pzConf, &pzValues );
        ACTIVE_POWER= (int16_t)(pzValues.power);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
        Summation_delivered= (uint64_t)(pzValues.energy*1000);  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 
        vTaskDelay( pdMS_TO_TICKS( 500 ) ); 

        // Update config struct with new address
        pzConf.pzem_addr = 0x02;
        PzemGetValues( &pzConf, &pzValues );
        ACTIVE_POWER_1= (int16_t)(pzValues.power);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
        Summation_delivered_1= (uint64_t)(pzValues.energy*1000);  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 

        vTaskDelay( pdMS_TO_TICKS( 500 ) ); 
        // Update config struct with new address
        pzConf.pzem_addr = 0x03;
        PzemGetValues( &pzConf, &pzValues );
        ACTIVE_POWER_2= (int16_t)(pzValues.power);  //!< Represents the single phase or Phase A, current demand of active power delivered or received at the premises, in @e Watts (W). 
        Summation_delivered_2= (uint64_t)(pzValues.energy*1000);  //!< Active power represents the current demand of active power delivered or received at the premises, in @e kW 
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
