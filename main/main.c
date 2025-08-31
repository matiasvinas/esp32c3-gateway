/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "mesh/utils.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "esp_ble_mesh_local_data_operation_api.h"

#include "ble_mesh_example_init.h"

#include <stdint.h>
#include <stddef.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"

#include "esp_ota_ops.h"
#include <sys/param.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>

#include "driver/gpio.h"
//static const char *TAG = "mqtt_example";
///MQTT Includes///

#define TAG 				"BLE Mesh"
#define TAG_BLE_CLIENT		"BLE Mesh Clie"
#define TAG_PROV			"BLE Mesh Prov"
#define TAG_BLE_CONFIG		"BLE Mesh Conf"
#define TAG_BLE_MSG			"BLE Mesh Mesg"
#define TAG_MQTT 			"MQTT"
#define TAG_POOL			"Pooling"
#define TAG_PUBLISH			"Publish"

#define CID_ESP             0x02E5

#define PROV_OWN_ADDR       0x0001

#define MSG_SEND_TTL        3
#define MSG_TIMEOUT         4000
#define MSG_ROLE            ROLE_PROVISIONER

#define COMP_DATA_PAGE_0    0x00

#define APP_KEY_IDX         0x0000
#define APP_KEY_OCTET       0x12

#define COMP_DATA_1_OCTET(msg, offset)      (msg[offset])
#define COMP_DATA_2_OCTET(msg, offset)      (msg[offset + 1] << 8 | msg[offset])

#define ID_OFFSET_IN_UUID	2

/* ID de cada nodo sensor */
#define SENSOR_ID_NODE_1         0x01
#define SENSOR_ID_NODE_2         0x02
#define SENSOR_ID_NODE_3         0x03
/* ID de la malla */
#define SENSOR_ID_MESH_0                    0x32    
#define SENSOR_ID_MESH_1                    0x10

//MQTT CERT
extern const uint8_t or_fiuba_tpp_pem_start[]   asm("_binary_or_fiuba_tpp_pem_start");
extern const uint8_t or_fiuba_tpp_pem_end[]   asm("_binary_or_fiuba_tpp_pem_end");

bool is_timeout = false;

//Frequency of polling data from sensors
uint16_t frequency = 1;

//Irrigation system
bool irrigation_activated = false;

//MQTT Topics handled by Gateway
const char topic_frequency_str[] = "master/client12345/attributevalue/frecuencia/7OmPma6DzamIanc1g2RU2j";
const char topic_irrigation_str[] = "master/client12345/attributevalue/riego_activado/7OmPma6DzamIanc1g2RU2j";


//MQTT Topics handled by Sensors
typedef struct {
	uint8_t  id;
	char* topic_temp_val;
	char* topic_mois_val;
	char* topic_battery_val;
	char* topic_connection;
	char* topic_log;
} openremote_thing_t;

static openremote_thing_t or_things[3] = {
	[0] = {
		.id = SENSOR_ID_NODE_1,
		.topic_temp_val = "master/client12345/writeattributevalue/temperatura/35upINKAfdfg5uyZ6ztuzE",
		.topic_mois_val = "master/client12345/writeattributevalue/humedad_suelo/35upINKAfdfg5uyZ6ztuzE",
		.topic_battery_val = "master/client12345/writeattributevalue/bateria/35upINKAfdfg5uyZ6ztuzE",
		.topic_connection = "master/client12345/writeattributevalue/nodo_sensor_1_conectado/7OmPma6DzamIanc1g2RU2j",
	},
	[1] = {
		.id = SENSOR_ID_NODE_2,
		.topic_temp_val = "master/client12345/writeattributevalue/temperatura/4dSlnHNmdI75WdichaKMoJ",
		.topic_mois_val = "master/client12345/writeattributevalue/humedad_suelo/4dSlnHNmdI75WdichaKMoJ",
		.topic_battery_val = "master/client12345/writeattributevalue/bateria/4dSlnHNmdI75WdichaKMoJ",
		.topic_connection = "master/client12345/writeattributevalue/nodo_sensor_2_conectado/7OmPma6DzamIanc1g2RU2j",
	},
	[2] = {
		.id = SENSOR_ID_NODE_3,
		.topic_temp_val = "master/client12345/writeattributevalue/temperatura/2gMjAmepzfcx70yc9nY5KS",
		.topic_mois_val = "master/client12345/writeattributevalue/humedad_suelo/2gMjAmepzfcx70yc9nY5KS",
		.topic_battery_val = "master/client12345/writeattributevalue/bateria/2gMjAmepzfcx70yc9nY5KS",
		.topic_connection = "master/client12345/writeattributevalue/nodo_sensor_3_conectado/7OmPma6DzamIanc1g2RU2j",	
	}	
};

//BLE MESH operational codes
static uint32_t send_opcode[] = {
    [0] = ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET,
    [1] = ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET,
    [2] = ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET,
    [3] = ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
    [4] = ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET,
};

static uint8_t  dev_uuid[ESP_BLE_MESH_OCTET16_LEN];
static uint16_t sensor_prop_id;

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  app_key[ESP_BLE_MESH_OCTET16_LEN];
} prov_key;

//Data Structure to storage Sensors data
typedef struct {
    uint8_t  uuid[16];
    uint16_t unicast_addr;
    uint8_t  elem_num;
    float temp_state;
    float moisture_state;
    uint16_t battery_state;
    
} esp_ble_mesh_node_info_t;

esp_err_t ble_mesh_send_sensor_message(uint32_t opcode, uint8_t id);

static esp_ble_mesh_node_info_t nodes[3] = {0};

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_DISABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_client_t sensor_client;

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_SENSOR_CLI(NULL, &sensor_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid           = dev_uuid,
    .prov_unicast_addr   = PROV_OWN_ADDR,
    .prov_start_address  = 0x0005,
    .prov_attention      = 0x00,
    .prov_algorithm      = 0x00,
    .prov_pub_key_oob    = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags               = 0x00,
    .iv_index            = 0x00,
};

static esp_err_t example_ble_mesh_store_node_info(const uint8_t uuid[16], uint16_t unicast,
                                                  uint8_t elem_num)
{
    int i;

    if (!uuid || !ESP_BLE_MESH_ADDR_IS_UNICAST(unicast)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Judge if the device has been provisioned before */
    for (i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (!memcmp(nodes[i].uuid, uuid, 16)) {
            ESP_LOGW(TAG, "%s: reprovisioned device 0x%04x", __func__, unicast);
            nodes[i].unicast_addr = unicast;
            nodes[i].elem_num = elem_num;
            nodes[i].temp_state = 1.1;
            nodes[i].moisture_state = 0.0;
            nodes[i].battery_state = 0;
            return ESP_OK;
        }
    }

    for (i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (nodes[i].unicast_addr == ESP_BLE_MESH_ADDR_UNASSIGNED) {
            memcpy(nodes[i].uuid, uuid, 16);
            nodes[i].unicast_addr = unicast;
            nodes[i].elem_num = elem_num;
            nodes[i].temp_state = 1.1;
            nodes[i].moisture_state = 0.0;
            nodes[i].battery_state = 0;
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

static esp_ble_mesh_node_info_t *example_ble_mesh_get_node_info_by_id(uint8_t id)
{
    for (int i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (nodes[i].uuid[ID_OFFSET_IN_UUID] == id)
        {
            return &nodes[i];
        }
    }

    return NULL;
}

static esp_ble_mesh_node_info_t *example_ble_mesh_get_node_info(uint16_t unicast)
{
    int i;

    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast)) {
        return NULL;
    }

    for (i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (nodes[i].unicast_addr <= unicast &&
                nodes[i].unicast_addr + nodes[i].elem_num > unicast) {
            return &nodes[i];
        }
    }

    return NULL;
}

static void example_ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common,
                                            esp_ble_mesh_node_info_t *node,
                                            esp_ble_mesh_model_t *model, uint32_t opcode)
{
    common->opcode = opcode;
    common->model = model;
    common->ctx.net_idx = prov_key.net_idx;
    common->ctx.app_idx = prov_key.app_idx;
    common->ctx.addr = node->unicast_addr;
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->msg_timeout = MSG_TIMEOUT;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common->msg_role = MSG_ROLE;
#endif
}

static esp_err_t prov_complete(uint16_t node_index, const esp_ble_mesh_octet16_t uuid,
                               uint16_t primary_addr, uint8_t element_num, uint16_t net_idx)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get = {0};
    esp_ble_mesh_node_info_t *node = NULL;

    char name[11] = {'\0'};
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG_PROV, "node_index %u, primary_addr 0x%04x, element_num %u, net_idx 0x%03x",
        node_index, primary_addr, element_num, net_idx);
    ESP_LOG_BUFFER_HEX("uuid", uuid, ESP_BLE_MESH_OCTET16_LEN);

    uint16_t unicast = primary_addr;

    sprintf(name, "%s%02x", "NODE-", node_index);
    err = esp_ble_mesh_provisioner_set_node_name(node_index, name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PROV, "Failed to set node name");
        return ESP_FAIL;
    }

    err = example_ble_mesh_store_node_info(uuid, unicast, element_num);
    if (err) {
        ESP_LOGE(TAG_PROV, "%s: Store node info failed", __func__);
        return ESP_FAIL;
    }

    node = example_ble_mesh_get_node_info(unicast);
    if (!node) {
        ESP_LOGE(TAG_PROV, "%s: Get node info failed", __func__);
        return ESP_FAIL;
    }

    example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PROV, "Failed to send Config Composition Data Get in prov complete");
        return ESP_FAIL;
    }

	return ESP_OK;
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    esp_err_t err = ESP_OK;

    /* Due to the API esp_ble_mesh_provisioner_set_dev_uuid_match, Provisioner will only
     * use this callback to report the devices, whose device UUID starts with 0xdd & 0xdd,
     * to the application layer.
     */

    ESP_LOG_BUFFER_HEX("Device address", addr, BD_ADDR_LEN);
    ESP_LOGI(TAG_PROV, "Address type 0x%02x, adv type 0x%02x", addr_type, adv_type);
    ESP_LOG_BUFFER_HEX("Device UUID", dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    ESP_LOGI(TAG_PROV, "oob info 0x%04x, bearer %s", oob_info, (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = (esp_ble_mesh_addr_type_t)addr_type;
    memcpy(add_dev.uuid, dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    add_dev.oob_info = oob_info;
    add_dev.bearer = (esp_ble_mesh_prov_bearer_t)bearer;
    /* Note: If unprovisioned device adv packets have not been received, we should not add
             device with ADD_DEV_START_PROV_NOW_FLAG set. */
    err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
            ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PROV, "Failed to start provisioning device");
    }
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d", param->provisioner_prov_disable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT, bearer %s",
            param->provisioner_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT, bearer %s, reason 0x%02x",
            param->provisioner_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        prov_complete(param->provisioner_prov_complete.node_idx, param->provisioner_prov_complete.device_uuid,
                      param->provisioner_prov_complete.unicast_addr, param->provisioner_prov_complete.element_num,
                      param->provisioner_prov_complete.netkey_idx);
        uint8_t idxx = param->provisioner_prov_complete.device_uuid[2];
        //ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROV_COMPLETE_EVT SENDING GET MESS ID: %d", idxx);
        //ble_mesh_send_sensor_message(ESP_BLE_MESH_MODEL_OP_SENSOR_GET, idxx);
        
        ESP_LOGI(TAG_PROV, "ADDING TO SUB LIST");
/*        
		esp_ble_mesh_client_common_param_t common = {0};
		esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    
		common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD;
		common.model = config_client.model;
		common.ctx.net_idx = prov_key.net_idx;
		common.ctx.app_idx = prov_key.app_key;
		common.ctx.addr = elements[0].element_addr;

		set_state.model_sub_add.element_addr = elements[0].element_addr;
		set_state.model_sub_add.sub_addr = param->provisioner_prov_complete.unicast_addr;
		set_state.model_sub_add.model_id = config_client.model->vnd.model_id;
		set_state.model_sub_add.company_id = config_client.model->vnd.company_id;

		esp_ble_mesh_config_client_set_state(&common, &set_state);
*/
//		esp_ble_mesh_model_subscribe_group_addr(elements[0].element_addr, config_client.model->vnd.company_id, config_client.model->vnd.model_id, param->provisioner_prov_complete.unicast_addr);
        ESP_LOGI(TAG_PROV, "ADDING TO SUB LIST - FINISHED");
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d", param->provisioner_add_unprov_dev_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, err_code %d", param->provisioner_set_dev_uuid_match_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code %d", param->provisioner_set_node_name_comp.err_code);
        if (param->provisioner_set_node_name_comp.err_code == 0) {
            const char *name = esp_ble_mesh_provisioner_get_node_name(param->provisioner_set_node_name_comp.node_index);
            if (name) {
                ESP_LOGI(TAG_PROV, "Node %d name %s", param->provisioner_set_node_name_comp.node_index, name);
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == 0) {
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            esp_err_t err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                                ESP_BLE_MESH_MODEL_ID_SENSOR_CLI, ESP_BLE_MESH_CID_NVAL);
            if (err != ESP_OK) {
                ESP_LOGE(TAG_PROV, "Failed to bind AppKey to sensor client");
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, err_code %d", param->provisioner_bind_app_key_to_model_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT:
        ESP_LOGI(TAG_PROV, "ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT, err_code %d", param->provisioner_store_node_comp_data_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_parse_node_comp_data(const uint8_t *data, uint16_t length)
{
    uint16_t cid, pid, vid, crpl, feat;
    uint16_t loc, model_id, company_id;
    uint8_t nums, numv;
    uint16_t offset;
    int i;

    cid = COMP_DATA_2_OCTET(data, 0);
    pid = COMP_DATA_2_OCTET(data, 2);
    vid = COMP_DATA_2_OCTET(data, 4);
    crpl = COMP_DATA_2_OCTET(data, 6);
    feat = COMP_DATA_2_OCTET(data, 8);
    offset = 10;

    ESP_LOGI(TAG, "********************** Composition Data Start **********************");
    ESP_LOGI(TAG, "* CID 0x%04x, PID 0x%04x, VID 0x%04x, CRPL 0x%04x, Features 0x%04x *", cid, pid, vid, crpl, feat);
    for (; offset < length; ) {
        loc = COMP_DATA_2_OCTET(data, offset);
        nums = COMP_DATA_1_OCTET(data, offset + 2);
        numv = COMP_DATA_1_OCTET(data, offset + 3);
        offset += 4;
        ESP_LOGI(TAG, "* Loc 0x%04x, NumS 0x%02x, NumV 0x%02x *", loc, nums, numv);
        for (i = 0; i < nums; i++) {
            model_id = COMP_DATA_2_OCTET(data, offset);
            ESP_LOGI(TAG, "* SIG Model ID 0x%04x *", model_id);
            offset += 2;
        }
        for (i = 0; i < numv; i++) {
            company_id = COMP_DATA_2_OCTET(data, offset);
            model_id = COMP_DATA_2_OCTET(data, offset + 2);
            ESP_LOGI(TAG, "* Vendor Model ID 0x%04x, Company ID 0x%04x *", model_id, company_id);
            offset += 4;
        }
    }
    ESP_LOGI(TAG, "*********************** Composition Data End ***********************");
}

static void example_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                              esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set = {0};
    static uint16_t wait_model_id, wait_cid;
    esp_ble_mesh_node_info_t *node = NULL;
    esp_err_t err = ESP_OK;
    uint16_t addr;
    
    addr = param->params->ctx.addr;
	
    ESP_LOGI(TAG_BLE_CONFIG, "Config client, event %u, addr 0x%04x, opcode 0x%04" PRIx32,
        event, param->params->ctx.addr, param->params->opcode);

    if (param->error_code) {
        ESP_LOGE(TAG_BLE_CONFIG, "Send config client message failed (err %d)", param->error_code);
        return;
    }

    node = example_ble_mesh_get_node_info(addr);
    if (!node) {
        ESP_LOGE(TAG_BLE_CONFIG, "%s: Get node info failed", __func__);
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET) {
            ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            example_ble_mesh_parse_node_comp_data(param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            err = esp_ble_mesh_provisioner_store_node_comp_data(param->params->ctx.addr,
                param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to store node composition data");
                break;
            }

            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send Config AppKey Add");
            }
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = ESP_BLE_MESH_MODEL_ID_SENSOR_SRV;
            set.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send Config Model App Bind");
                return;
            }
            wait_model_id = ESP_BLE_MESH_MODEL_ID_SENSOR_SRV;
            wait_cid = ESP_BLE_MESH_CID_NVAL;
        } else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
            if (param->status_cb.model_app_status.model_id == ESP_BLE_MESH_MODEL_ID_SENSOR_SRV &&
                param->status_cb.model_app_status.company_id == ESP_BLE_MESH_CID_NVAL) {
                example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
                set.model_app_bind.element_addr = node->unicast_addr;
                set.model_app_bind.model_app_idx = prov_key.app_idx;
                set.model_app_bind.model_id = ESP_BLE_MESH_MODEL_ID_SENSOR_SETUP_SRV;
                set.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
                err = esp_ble_mesh_config_client_set_state(&common, &set);
                if (err) {
                    ESP_LOGE(TAG_BLE_CONFIG, "Failed to send Config Model App Bind");
                    return;
                }
                wait_model_id = ESP_BLE_MESH_MODEL_ID_SENSOR_SETUP_SRV;
                wait_cid = ESP_BLE_MESH_CID_NVAL;
            } else if (param->status_cb.model_app_status.model_id == ESP_BLE_MESH_MODEL_ID_SENSOR_SETUP_SRV &&
                param->status_cb.model_app_status.company_id == ESP_BLE_MESH_CID_NVAL) {
                ESP_LOGW(TAG_BLE_CONFIG, "Provision and config successfully");
            }
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS) {
            ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
            esp_ble_mesh_cfg_client_get_state_t get = {0};
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
            get.comp_data_get.page = COMP_DATA_PAGE_0;
            err = esp_ble_mesh_config_client_get_state(&common, &get);
            if (err != ESP_OK) {
                ESP_LOGE(TAG_BLE_CONFIG, "Failed to send Config Composition Data Get in config client cb");
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(TAG_BLE_CONFIG, "Failed to send Config AppKey Add");
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = wait_model_id;
            set.model_app_bind.company_id = wait_cid;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(TAG_BLE_CONFIG, "Failed to send Config Model App Bind");
            }
            break;
        default:
            break;
        }
        break;
    default:
        ESP_LOGE(TAG_BLE_CONFIG, "Invalid config client event %u", event);
        break;
    }
}

esp_err_t ble_mesh_send_sensor_message(uint32_t opcode, uint8_t id)
{
    esp_ble_mesh_sensor_client_get_state_t get = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

	esp_ble_mesh_node_info_t *node = NULL;
	
    node = example_ble_mesh_get_node_info_by_id(id);
    
    if (!node) {
        ESP_LOGE(TAG_BLE_MSG, "%s: Get node info failed", __func__);
        return ESP_FAIL;
    }
    
    example_ble_mesh_set_msg_common(&common, node, sensor_client.model, opcode);
    switch (opcode) {
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
        get.cadence_get.property_id = sensor_prop_id;
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
        get.settings_get.sensor_property_id = sensor_prop_id;
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
        get.series_get.property_id = sensor_prop_id;
        break;
    default:
        break;
    }

    err = esp_ble_mesh_sensor_client_get_state(&common, &get);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BLE_MSG, "Failed to send sensor message 0x%04" PRIx32, opcode);
    }
    
    ESP_LOGI(TAG_BLE_MSG, "Sent get state message");
	return err;
}

void ble_mesh_send_sensor_set_message(uint32_t opcode, uint8_t id)
{
	ESP_LOGI(TAG_BLE_MSG, "Calling function");
    esp_ble_mesh_sensor_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

	esp_ble_mesh_node_info_t *node = NULL;
	
    node = example_ble_mesh_get_node_info_by_id(id);
    
    if (!node) {
        ESP_LOGE(TAG_BLE_MSG, "%s: Get node info failed", __func__);
        return;
    }
    
    ESP_LOGI(TAG_BLE_MSG, "set msg common");
    example_ble_mesh_set_msg_common(&common, node, sensor_client.model, opcode);
    switch (opcode) {
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
        set.cadence_set.property_id = sensor_prop_id;
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
        set.setting_set.sensor_property_id = sensor_prop_id;
        break;
    default:
        break;
    }
	ESP_LOGI(TAG_BLE_MSG, "Sending set state");
    err = esp_ble_mesh_sensor_client_set_state(&common, &set);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BLE_MSG, "Failed to send sensor message 0x%04" PRIx32, opcode);
    }
    
    ESP_LOGI(TAG_BLE_MSG, "Ending function");

}

void example_ble_mesh_send_sensor_message(uint32_t opcode)
{
    esp_ble_mesh_sensor_client_get_state_t get = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

	for(uint8_t i=0; i < 1; i++ )
	{
		esp_ble_mesh_node_info_t *node = NULL;
		uint32_t addr = nodes[i].unicast_addr;
		ESP_LOGI(TAG_BLE_MSG, "iterator: %d",i);
		ESP_LOGI(TAG_BLE_MSG, "unicast addr: %d",(int) addr);
		ESP_LOGI(TAG_BLE_MSG, "addr: %d",ARRAY_SIZE(nodes));
	    node = example_ble_mesh_get_node_info(addr);
	    if (!node) {
	        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
	        return;
	    }
	    
	    example_ble_mesh_set_msg_common(&common, node, sensor_client.model, opcode);
	    switch (opcode) {
	    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
	        get.cadence_get.property_id = sensor_prop_id;
	        break;
	    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
	        get.settings_get.sensor_property_id = sensor_prop_id;
	        break;
	    case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
	        get.series_get.property_id = sensor_prop_id;
	        break;
	    default:
	        break;
	    }
	
	    err = esp_ble_mesh_sensor_client_get_state(&common, &get);
	    if (err != ESP_OK) {
	        ESP_LOGE(TAG_BLE_MSG, "Failed to send sensor message 0x%04" PRIx32, opcode);
	    }
	    
	    ESP_LOGI(TAG_BLE_MSG, "temp value stored: %f",node->temp_state);
	}
}

static void example_ble_mesh_sensor_timeout(uint32_t opcode)
{
    switch (opcode) {
    case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
        ESP_LOGW(TAG, "Sensor Descriptor Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
        ESP_LOGW(TAG, "Sensor Cadence Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
        ESP_LOGW(TAG, "Sensor Cadence Set timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
        ESP_LOGW(TAG, "Sensor Settings Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
        ESP_LOGW(TAG, "Sensor Setting Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
        ESP_LOGW(TAG, "Sensor Setting Set timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
        ESP_LOGW(TAG, "Sensor Get timeout, 0x%04" PRIx32, opcode);
        
		is_timeout = true;
		ESP_LOGI(TAG, "timeout true");
		
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
        ESP_LOGW(TAG, "Sensor Column Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
        ESP_LOGW(TAG, "Sensor Series Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Get/Set opcode 0x%04" PRIx32, opcode);
        return;
    }

//	ble_mesh_send_sensor_message(opcode, 2);
}

static void example_ble_mesh_sensor_client_cb(esp_ble_mesh_sensor_client_cb_event_t event,
                                              esp_ble_mesh_sensor_client_cb_param_t *param)
{
    esp_ble_mesh_node_info_t *node = NULL;

    uint16_t addr;
    addr = param->params->ctx.addr;
    
    ESP_LOGI(TAG_BLE_CLIENT, "Sensor client, event %u, addr 0x%04x", event, param->params->ctx.addr);

    if (param->error_code) {
        ESP_LOGE(TAG_BLE_CLIENT, "Send sensor client message failed (err %d)", param->error_code);
        return;
    }

    node = example_ble_mesh_get_node_info(addr);
    if (!node) {
        ESP_LOGE(TAG_BLE_CLIENT, "%s: Get node info failed", __func__);
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_SENSOR_CLIENT_GET_STATE_EVT:
        switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Descriptor Status, opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            if (param->status_cb.descriptor_status.descriptor->len != ESP_BLE_MESH_SENSOR_SETTING_PROPERTY_ID_LEN &&
                param->status_cb.descriptor_status.descriptor->len % ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN) {
                ESP_LOGE(TAG_BLE_CLIENT, "Invalid Sensor Descriptor Status length %d", param->status_cb.descriptor_status.descriptor->len);
                return;
            }
            if (param->status_cb.descriptor_status.descriptor->len) {
                ESP_LOG_BUFFER_HEX("Sensor Descriptor", param->status_cb.descriptor_status.descriptor->data,
                    param->status_cb.descriptor_status.descriptor->len);
                /* If running with sensor server example, sensor client can get two Sensor Property IDs.
                 * Currently we use the first Sensor Property ID for the following demonstration.
                 */
                sensor_prop_id = param->status_cb.descriptor_status.descriptor->data[1] << 8 |
                                 param->status_cb.descriptor_status.descriptor->data[0];
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Cadence Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.cadence_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Cadence", param->status_cb.cadence_status.sensor_cadence_value->data,
                param->status_cb.cadence_status.sensor_cadence_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Settings Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.settings_status.sensor_property_id);
            ESP_LOG_BUFFER_HEX("Sensor Settings", param->status_cb.settings_status.sensor_setting_property_ids->data,
                param->status_cb.settings_status.sensor_setting_property_ids->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Setting Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x, Sensor Setting Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.setting_status.sensor_property_id,
                param->status_cb.setting_status.sensor_setting_property_id);
            if (param->status_cb.setting_status.op_en) {
                ESP_LOGI(TAG, "Sensor Setting Access 0x%02x", param->status_cb.setting_status.sensor_setting_access);
                ESP_LOG_BUFFER_HEX("Sensor Setting Raw", param->status_cb.setting_status.sensor_setting_raw->data,
                    param->status_cb.setting_status.sensor_setting_raw->len);
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Status, opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            if (param->status_cb.sensor_status.marshalled_sensor_data->len) {
                ESP_LOG_BUFFER_HEX("Sensor Data", param->status_cb.sensor_status.marshalled_sensor_data->data,
                    param->status_cb.sensor_status.marshalled_sensor_data->len);
                uint8_t *data = param->status_cb.sensor_status.marshalled_sensor_data->data;
                
                /*PROCESS TEMPERATURE DATA */
                uint8_t MSB = data[0x03];
                uint8_t LSB = data[0x02];
                uint16_t combined = MSB << 8 | LSB;
                float s_temp = ( (float) combined ) * 0.01;
                
                /*check if is_temp_below_zero flag*/
                if( data[0x04] == true)
                {
					s_temp = s_temp * (-1);
				}
                
                /*PROCESS MOISTURE DATA*/
                uint8_t moisture_msb = data[0x08];
                uint8_t moisture_lsb = data[0x07];
                uint16_t moisture_combined = moisture_msb << 8 | moisture_lsb;
                float s_mois = ( (float) moisture_combined ) * 0.01;

 				/*PROCESS BATTERY LEVEL*/
                uint8_t battery_msb = data[0x0C];
                uint8_t battery_lsb = data[0x0B];
                uint16_t s_batt = battery_msb << 8 | battery_lsb;

                                
                node->temp_state = s_temp;
                node->moisture_state = s_mois;
                node->battery_state = s_batt;
                

            }

            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Column Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.column_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Column", param->status_cb.column_status.sensor_column_value->data,
                param->status_cb.column_status.sensor_column_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Series Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.series_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Series", param->status_cb.series_status.sensor_series_value->data,
                param->status_cb.series_status.sensor_series_value->len);
            break;
        default:
            ESP_LOGE(TAG_BLE_CLIENT, "Unknown Sensor Get opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            break;
        }
        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_SET_STATE_EVT:
        switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG, "Sensor Cadence Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.cadence_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Cadence", param->status_cb.cadence_status.sensor_cadence_value->data,
                param->status_cb.cadence_status.sensor_cadence_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG_BLE_CLIENT, "Sensor Setting Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x, Sensor Setting Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.setting_status.sensor_property_id,
                param->status_cb.setting_status.sensor_setting_property_id);
            if (param->status_cb.setting_status.op_en) {
                ESP_LOGI(TAG_BLE_CLIENT, "Sensor Setting Access 0x%02x", param->status_cb.setting_status.sensor_setting_access);
                ESP_LOG_BUFFER_HEX("Sensor Setting Raw", param->status_cb.setting_status.sensor_setting_raw->data,
                    param->status_cb.setting_status.sensor_setting_raw->len);
            }
            break;
        default:
            ESP_LOGE(TAG_BLE_CLIENT, "Unknown Sensor Set opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            break;
        }
        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT:
         ESP_LOGI(TAG_BLE_CLIENT, "CLIENT PUBLISH EVENT");
         ESP_LOG_BUFFER_HEX("Sensor Data", param->status_cb.sensor_status.marshalled_sensor_data->data,
         param->status_cb.sensor_status.marshalled_sensor_data->len);

        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_TIMEOUT_EVT:
        example_ble_mesh_sensor_timeout(param->params->opcode);
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    uint8_t match[2] = { SENSOR_ID_MESH_0, SENSOR_ID_MESH_1 };
    esp_err_t err = ESP_OK;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_sensor_client_callback(example_ble_mesh_sensor_client_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set matching device uuid");
        return err;
    }

    err = esp_ble_mesh_provisioner_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh provisioner");
        return err;
    }

    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add local AppKey");
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh sensor client initialized");

    return ESP_OK;
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/* Event handler registered to receive MQTT events
 * This function is called by the MQTT client event loop. */
 
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
		esp_mqtt_client_subscribe(client, "master/client12345/attributevalue/frecuencia/7OmPma6DzamIanc1g2RU2j", 0);
		esp_mqtt_client_subscribe(client, "master/client12345/attributevalue/riego_activado/7OmPma6DzamIanc1g2RU2j", 0);

        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_PUBLISHED:
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        
        char topic_event_str[80];
        sprintf(topic_event_str, "%s", event->topic);
        ESP_LOGI(TAG_MQTT, "Topic string Length %d", event->topic_len);
		topic_event_str[event->topic_len] = '\0';
        
        if (strcmp(topic_event_str, topic_frequency_str) == 0){
			frequency  = atoi(event->data);
			ESP_LOGI(TAG_MQTT, "Frequency received: %d", frequency);
		}

        if (strcmp(topic_event_str, topic_irrigation_str) == 0){
			char data_str[6];
			char true_str[] = "true";
			int irrigation_activated = 2;
			sprintf(data_str, "%s", event->data);
			data_str[event->data_len] = '\0';
			strcmp(data_str, true_str) == 0 ? (irrigation_activated = 1) : (irrigation_activated = 0);			
			ESP_LOGI(TAG, "Irrigation System: %d ", irrigation_activated);
			gpio_set_level(GPIO_NUM_4,irrigation_activated);
		}		
        
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
        
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

const esp_mqtt_client_config_t mqtts_cfg = {
	    .broker.address.transport = MQTT_TRANSPORT_OVER_SSL,
	    .broker.address.port = 8883,
	    .broker.address.path = "dev.openremote-fiuba-tpp.com",
	    .broker.address.hostname = "dev.openremote-fiuba-tpp.com",
	    .broker.verification.skip_cert_common_name_check = false,
	    .broker.verification.certificate = (const char *)or_fiuba_tpp_pem_start,
	    .credentials.client_id = "client12345",
	    .credentials.authentication.password = "QZxFVgZmQzdh0Nh8kann3TjIZfQ5CqfC",
	    .credentials.username = "master:matias"
	};
	
static void irrigator_init(void)
{
    ESP_LOGI(TAG, "Configuring irrigator GPIO");
    gpio_reset_pin(GPIO_NUM_4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    
    gpio_set_level(GPIO_NUM_4, 0);
}


void app_main(void)
{
    esp_err_t err = ESP_OK;
	float voltage;
	
    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

	irrigator_init();

    err = bluetooth_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* WIFI - Configuration */
    ESP_ERROR_CHECK(example_connect());

	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtts_cfg);
	/* The last argument may be used to pass data to the event handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    	
	while(1)
	{
	 	for(int or_thing_idx = 0; or_thing_idx < ARRAY_SIZE(or_things); or_thing_idx++)
	 	{
			//check if device is connected to the mesh()
			uint8_t id = or_things[or_thing_idx].id;
			ESP_LOGI("Pooling", "Connecting to device ID: %d", id);
			
			int node_idx = 0;
			bool is_connected = false;
			for(int i = 0; i < ARRAY_SIZE(nodes); i++)
			{
				if( id == nodes[i].uuid[ID_OFFSET_IN_UUID] && example_ble_mesh_get_node_info_by_id(id) != NULL )
				{					
					ESP_LOGI(TAG_POOL, "ID %d Connected", id);
					ESP_LOGI(TAG_POOL, "node index: %d", i);
					node_idx = i;
					is_connected = true;
					break;
				}	
			}
			
			//send_sensor_message()	
			ESP_LOGI(TAG_POOL, "Get data from device ID %d", id);
			if( ESP_OK != ble_mesh_send_sensor_message(ESP_BLE_MESH_MODEL_OP_SENSOR_GET, id))
			{
				is_connected = false;
			}		
			vTaskDelay(5000 / portTICK_PERIOD_MS);
						
			if(is_connected && (is_timeout != true) )
			{
				//publish connected state TRUE
				ESP_LOGI(TAG_PUBLISH, "ID %d. Connected: %d", id, is_connected);
				esp_mqtt_client_publish(client, or_things[or_thing_idx].topic_connection, "true", 0, 1, 0);														
				
				//Publish Temperature
				ESP_LOGI(TAG_PUBLISH, "ID %d. Temperature: %.2f C", id, nodes[node_idx].temp_state);
		        char str_temperature[10];
		        sprintf(str_temperature, "%f", nodes[node_idx].temp_state);
		        esp_mqtt_client_publish(client, or_things[or_thing_idx].topic_temp_val, str_temperature, 0, 1, 0);
				
				
				//Publish Moisture
				ESP_LOGI(TAG_PUBLISH, "ID %d. Moisture: %.2f [perc]", id, nodes[node_idx].moisture_state);
		        char str_moisture[10];
		        sprintf(str_moisture, "%f", nodes[node_idx].moisture_state);
		        esp_mqtt_client_publish(client, or_things[or_thing_idx].topic_mois_val, str_moisture, 0, 1, 0);				
				
				//Publish Battery Level
				ESP_LOGI(TAG_PUBLISH, "ID %d. Battery: %d mV", id, (int) nodes[node_idx].battery_state);
				voltage = (float) nodes[node_idx].battery_state;
				voltage = voltage / 1000;
				voltage = voltage * 3.2;
				ESP_LOGI(TAG_PUBLISH, "ID %d. Battery: %.3f V", id, voltage);
		        char str_battery[10];
		        sprintf(str_battery, "%.3f", voltage);
		        esp_mqtt_client_publish(client, or_things[or_thing_idx].topic_battery_val, str_battery, 0, 1, 0);			
			} 
			else 
			{
				is_connected = false;
				ESP_LOGI(TAG_PUBLISH, "ID %d. Connected: %d", id, is_connected);
				esp_mqtt_client_publish(client, or_things[or_thing_idx].topic_connection, "false", 0, 1, 0);						
			}
			
			is_timeout = false;
			
		}
		
		//Delay attached to frequency topic
		ESP_LOGI(TAG_POOL, "Wait %d seconds", frequency);
		vTaskDelay((frequency * 1000) / portTICK_PERIOD_MS);	 	      
	}  
}
