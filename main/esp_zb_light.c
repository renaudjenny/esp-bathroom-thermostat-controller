#include "esp_zb_light.h"
#include "driver/gpio.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_endpoint.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"
#include "freertos/task.h"
#include "soc/gpio_num.h"
#include "switch_driver.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_binary_input.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "BATHROOM_THERMOSTAT_CONTROLLER";
/********************* Define functions **************************/

static void switch_isr_handler(void *data) {
    esp_zb_zcl_attr_t *binary_input_present_value = esp_zb_zcl_get_attribute(BATHROOM_BINARY_INPUT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID);
    bool binary_input_current_value = *(bool *)binary_input_present_value->data_p;
    bool binary_input_new_value = !binary_input_current_value;
    *((bool *)binary_input_present_value->data_p) = !binary_input_current_value;
    esp_zb_lock_acquire(portMAX_DELAY);
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(BATHROOM_BINARY_INPUT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, binary_input_present_value->data_p, false));
    esp_zb_lock_release();

    esp_zb_zcl_report_attr_cmd_t cmd_req = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .attributeID = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .zcl_basic_cmd.src_endpoint = BATHROOM_BINARY_INPUT_ENDPOINT, 
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&cmd_req));
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send binary input attribute report to %s", binary_input_new_value ? "true" : "false");
}

static void reset_isr_handler(void *data) {
    ESP_EARLY_LOGI(TAG, "Factory reset...");
    esp_zb_factory_reset();
}

static void switch_init(void) {
    gpio_config_t switch_config = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT(22),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&switch_config));

    gpio_config_t reset_config = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT(23),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&reset_config));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));

    ESP_ERROR_CHECK(gpio_isr_handler_add(22, switch_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(23, reset_isr_handler, NULL));
}

static esp_err_t deferred_driver_init(void)
{
    light_driver_init(LIGHT_DEFAULT_OFF);
    switch_init();
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
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
            /* commissioning failed */
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
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == BATHROOM_LIGHT_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                light_driver_set_power(light_state);
            }
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

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    // Light
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_cluster_list_t *light_cluster_list = esp_zb_on_off_light_clusters_create(&light_cfg);
    esp_zb_endpoint_config_t light_ep_config = {
        .endpoint = BATHROOM_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, light_cluster_list, light_ep_config);
    zcl_basic_manufacturer_info_t light_info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, BATHROOM_LIGHT_ENDPOINT, &light_info);

    // Binary input
    esp_zb_binary_input_cluster_cfg_t binary_input_cfg = {
        .out_of_service = ESP_ZB_ZCL_BINARY_INPUT_OUT_OF_SERVICE_DEFAULT_VALUE,
        .status_flags = ESP_ZB_ZCL_BINARY_INPUT_STATUS_FLAG_DEFAULT_VALUE
    };
    esp_zb_basic_cluster_cfg_t binary_input_basic_cfg = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    };
    esp_zb_identify_cluster_cfg_t binary_input_identify_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
    };
    esp_zb_cluster_list_t *binary_input_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *binary_input_basic_cluster = esp_zb_basic_cluster_create(&binary_input_basic_cfg);
    esp_zb_basic_cluster_add_attr(binary_input_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(binary_input_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(binary_input_cluster_list, binary_input_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(binary_input_cluster_list, esp_zb_identify_cluster_create(&binary_input_identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(binary_input_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_attribute_list_t *binary_input_attr_list = esp_zb_binary_input_cluster_create(&binary_input_cfg);
    esp_zb_binary_input_cluster_add_attr(binary_input_attr_list, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, "\x0C""Switch state");
    esp_zb_binary_input_cluster_add_attr(binary_input_attr_list, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &(bool *){ false });
    esp_zb_cluster_list_add_binary_input_cluster(binary_input_cluster_list, binary_input_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t binary_input_endpoint_config = {
        .endpoint = BATHROOM_BINARY_INPUT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, binary_input_cluster_list, binary_input_endpoint_config);

    if (esp_zb_device_register(ep_list) != ESP_OK) {
        ESP_LOGW(TAG,  "Can't register bathroom device");
    }

    esp_zb_core_action_handler_register(zb_action_handler);
    
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
