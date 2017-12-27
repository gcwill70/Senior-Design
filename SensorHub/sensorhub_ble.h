#ifndef SENSORHUB_BLE_H
#define SENSORHUB_BLE_H

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "bta_api.h"
#include "nvs_flash.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "sensorhub_sensordata.h"
#include "sensorhub_sdcard.h"
#include "sensorhub_time.h"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define DEVICE_NAME "SensorHub"
#define DATA_LEN 17
#define SH_SVC_INST_ID 0

#define CHAR_VAL_LEN_MAX 0x40
#define SI_VAL_LEN_MAX 20
#define FH_VAL_LEN_MAX 20
#define FT_VAL_LEN_MAX 20

///Attributes State Machine
enum {
    IDX_SH_SVC,

		IDX_SI_CHAR,
		IDX_SI_VAL,

		IDX_FH_CHAR,
		IDX_FH_VAL,

		IDX_FT_CHAR,
		IDX_FT_VAL,
		IDX_FT_NTF_CFG,

    IDX_SH_NB,
};

uint8_t char1_str[] ={0x11,0x22,0x33};

uint16_t sensorhub_handle_table[IDX_SH_NB];

esp_attr_value_t sensorhub_char1_val =
{
	.attr_max_len = CHAR_VAL_LEN_MAX,
	.attr_len = sizeof(char1_str),
	.attr_value = char1_str,
};


static uint8_t sensorhub_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xaf, 0x24, 0x3a, 0x8e, 0x3b, 0x51, 0x46, 0xd6, 0xb0, 0x27, 0x62, 0x86, 0xbc, 0xfa, 0xeb, 0x47,
};

static esp_ble_adv_data_t sensorhub_adv_config = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sensorhub_service_uuid),
    .p_service_uuid = sensorhub_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t sensorhub_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst sensorhub_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

///SensorHub Service
static const uint16_t sensorhub_svc[8] =
{0xaf24, 0x3a8e, 0x3b51, 0x46d6, 0xb027, 0x6286, 0xbcfa, 0xeb47};

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;

/// SensorHub Service - System Info Characteristic, read & write
static const uint16_t system_info_uuid[8] =
 {0xaf25, 0x3a8e, 0x3b51, 0x46d6, 0xb027, 0x6286, 0xbcfa, 0xeb47};

/// SensorHub Service - File Header Characteristic, read
static const uint16_t file_header_uuid[8] =
{0xaf26, 0x3a8e, 0x3b51, 0x46d6, 0xb027, 0x6286, 0xbcfa, 0xeb47};

/// SensorHub Service - File Transfer Characteristic, read
static const uint16_t file_transfer_uuid[8] =
{0xaf27, 0x3a8e, 0x3b51, 0x46d6, 0xb027, 0x6286, 0xbcfa, 0xeb47};

/// Full SensorHub Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t sensorhub_gatt_db[IDX_SH_NB] = {
    // SensorHub Service Declaration
    [IDX_SH_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(sensorhub_svc), (uint8_t *)&sensorhub_svc}},

    // SensorHub System Info Characteristic - Declaration
    [IDX_SI_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    // SensorHub System Info Characteristic - Value
    [IDX_SI_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&system_info_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      SI_VAL_LEN_MAX, sizeof(system_info_value), (uint8_t*)&system_info_value}},

		// SensorHub File Header Characteristic - Declaration
    [IDX_FH_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    // SensorHub File Header Characteristic - Value
    [IDX_FH_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&file_header_uuid, ESP_GATT_PERM_READ,
      FH_VAL_LEN_MAX, sizeof(file_header_value), (uint8_t*)&file_header_value}},


		// SensorHub File Transfer Characteristic - Declaration
    [IDX_FT_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    // SensorHub File Transfer Characteristic - Value
    [IDX_FT_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&file_transfer_uuid, ESP_GATT_PERM_READ,
      FT_VAL_LEN_MAX, sizeof(file_transfer_value), (uint8_t*)&file_transfer_value}},
};

//Update system info characteristic in the attribute table
void updateSystemInfo() {
  esp_err_t esp_err;
  esp_err = esp_ble_gatts_set_attr_value(sensorhub_handle_table[IDX_SI_VAL], sizeof(system_info_value), (uint8_t*)&system_info_value);
  if (esp_err != ESP_OK) {
    ESP_LOGE("Error", "Unable to update system info characteristic value in the attribute table.");
  }
}

//Update file header characteristic in the attribute table
void updateFileHeader() {
  esp_err_t esp_err;
  //htonl(file_header_value.filesize);
  esp_err = esp_ble_gatts_set_attr_value(sensorhub_handle_table[IDX_FH_VAL], sizeof(file_header_value), (uint8_t*)&file_header_value);
  if (esp_err != ESP_OK) {
    ESP_LOGE("Error", "Unable to update file header characteristic value in the attribute table.");
  }
}

//Update file transfer characteristic in the attribute table
void updateFileTransfer() {
  esp_err_t esp_err;
  esp_err = esp_ble_gatts_set_attr_value(sensorhub_handle_table[IDX_FT_VAL], sizeof(file_transfer_value), (uint8_t*)&file_transfer_value);
  if (esp_err != ESP_OK) {
    ESP_LOGE("Error", "Unable to update file transfer characteristic value in the attribute table.");
  }
}

void getNextBlock() {
  char filename[24];
  memset(filename, '\0', sizeof(filename));
  strcat(filename, "/sdcard/data_");
  strcat(filename, file_header_value.sensor_prefix);
  strcat(filename, ".csv");
  FILE* f = fopen(filename, "r");
  if (f == NULL) {
    ESP_LOGE("Error", "Unable to open file %s", filename);
    return;
  }
  //reset the buffer
  memset(file_transfer_value.data, 0, sizeof(file_transfer_value.data));
  //block_num is incremented every 16 bytes
  fseek(f, file_transfer_value.block_num * sizeof(file_transfer_value.data), SEEK_SET);
  //check if we read in a full 16 bytes
  fread(file_transfer_value.data, 1, sizeof(file_transfer_value.data), f);
  fclose(f);
  char* temp = malloc(sizeof(file_transfer_value.data) + 1);
  memset(temp, '\0', sizeof(file_transfer_value.data) + 1);
  memcpy(temp, file_transfer_value.data, sizeof(file_transfer_value.data));
  free(temp);
  //set the value in the attribute table
  updateFileTransfer();
}

void getNextFile() {
  memset(&file_header_value, 0, sizeof(file_header_value));
  memset(&file_transfer_value, 0, sizeof(file_transfer_value));
  if (!next_file) {
    updateFileHeader();
    updateFileTransfer();
    return;
  }
  getSensorPrefix(file_header_value.sensor_prefix, next_file->addr);
  char filename[24];
  memset(filename, '\0', sizeof(filename));
  strcat(filename, "/sdcard/data_");
  strcat(filename, file_header_value.sensor_prefix);
  strcat(filename, ".csv");
  file_header_value.filesize = getFileSize(filename);
  //set the value in the attribute table
  updateFileHeader();
  getNextBlock();
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI("Info", "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&sensorhub_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE("Info", "Advertising start failed\n");
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    //ESP_LOGI("Info", "event = %d\n",event);
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    switch (event) {
    	case ESP_GATTS_REG_EVT:
      ESP_LOGI("Info", "ESP_GATTS_REG_EVT");
		ESP_LOGI("Info", "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_set_device_name(DEVICE_NAME);
        	ESP_LOGI("Info", "%s %d\n", __func__, __LINE__);
       	esp_ble_gap_config_adv_data(&sensorhub_adv_config);

        	ESP_LOGI("Info", "%s %d\n", __func__, __LINE__);
		esp_ble_gatts_create_attr_tab(sensorhub_gatt_db, gatts_if,
								IDX_SH_NB, SH_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
        ESP_LOGI("Info", "ESP_GATTS_READ_EVT");
        if(sensorhub_handle_table[IDX_SI_VAL] == param->read.handle) {
          //System Info
        } else if (sensorhub_handle_table[IDX_FH_VAL] == param->read.handle) {
          //File Header
          max_block_num = file_header_value.filesize / sizeof(file_transfer_value.data);
          ESP_LOGI("Info", "Max block number = %d", max_block_num);
        } else if (sensorhub_handle_table[IDX_FT_VAL] == param->read.handle) {
          //File Transfer
          ESP_LOGI("Info", "Block number = %d / %d", file_transfer_value.block_num, max_block_num);
          //ESP_LOGI("Sent Data", "%s", file_transfer_value.data);
          //check if at end of current file
          if (file_transfer_value.block_num == max_block_num) {
            if (next_file) {
              next_file = next_file->next;
              getNextFile();
            } else {
              next_file = head;
              memset(&file_header_value, 0, sizeof(file_header_value));
              updateFileHeader();
            }
          } else {
            file_transfer_value.block_num++;
            getNextBlock();
          }
        }
        break;
    	case ESP_GATTS_WRITE_EVT:
        ESP_LOGI("Info", "ESP_GATTS_WRITE_EVT");
        if(sensorhub_handle_table[IDX_SI_VAL] == param->write.handle) {
          // memcpy(&system_info_value, param->write.value, param->write.len);
          // updateSystemInfo();
          tz.tz_minuteswest = -500;
          tz.tz_dsttime = 0;
          tv.tv_sec = (time_t)ntohl(*((uint32_t*)param->write.value));
          tv.tv_usec = 0;
          setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
          tzset();
          settimeofday(&tv, &tz);
        }
        break;
    	case ESP_GATTS_EXEC_WRITE_EVT:
      ESP_LOGI("Info", "ESP_GATTS_EXEC_WRITE_EVT");
        break;
    	case ESP_GATTS_MTU_EVT:
      ESP_LOGI("Info", "ESP_GATTS_MTU_EVT");
    		break;
   	 case ESP_GATTS_CONF_EVT:
     ESP_LOGI("Info", "ESP_GATTS_CONF_EVT");
		    break;
    	case ESP_GATTS_UNREG_EVT:
      ESP_LOGI("Info", "ESP_GATTS_UNREG_EVT");
        break;
    	case ESP_GATTS_DELETE_EVT:
      ESP_LOGI("Info", "ESP_GATTS_DELETE_EVT");
        break;
    	case ESP_GATTS_START_EVT:
      ESP_LOGI("Info", "ESP_GATTS_START_EVT");
        break;
    	case ESP_GATTS_STOP_EVT:
      ESP_LOGI("Info", "ESP_GATTS_STOP_EVT");
        break;
    	case ESP_GATTS_CONNECT_EVT:
      ESP_LOGI("Info", "ESP_GATTS_CONNECT_EVT");
        isBLEconnected = true;
        //Initialize GATT characteristic values
        memset(&file_header_value, 0, sizeof(file_header_value));
        memset(&file_transfer_value, 0, sizeof(file_transfer_value));
        memset(&system_info_value, 0, sizeof(system_info_value));
        if (isSDconnected) {
          system_info_value.battery_SD |= (1 << 7); //set MSB
        }
        updateFileHeader();
        updateFileTransfer();
        updateSystemInfo();
        //1. Initialize File Header characteristic
        getNextFile();
        break;
    	case ESP_GATTS_DISCONNECT_EVT:
      ESP_LOGI("Info", "ESP_GATTS_DISCONNECT_EVT");
        isBLEconnected = false;
        esp_ble_gap_start_advertising(&sensorhub_adv_params);
        if (!next_file) {next_file = head;}
        break;
    	case ESP_GATTS_OPEN_EVT:
      ESP_LOGI("Info", "ESP_GATTS_OPEN_EVT");
        break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
      ESP_LOGI("Info", "ESP_GATTS_CANCEL_OPEN_EVT");
        break;
    	case ESP_GATTS_CLOSE_EVT:
      ESP_LOGI("Info", "ESP_GATTS_CLOSE_EVT");
        break;
    	case ESP_GATTS_LISTEN_EVT:
      ESP_LOGI("Info", "ESP_GATTS_LISTEN_EVT");
		    break;
    	case ESP_GATTS_CONGEST_EVT:
      ESP_LOGI("Info", "ESP_GATTS_CONGEST_EVT");
		    break;
      case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
        ESP_LOGI("Info", "ESP_GATTS_CREAT_ATTR_TAB_EVT");
        ESP_LOGI("Info", "The number handle =%x\n",param->add_attr_tab.num_handle);
        if (param->add_attr_tab.status != ESP_GATT_OK){
            ESP_LOGE("Info", "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != IDX_SH_NB){
            ESP_LOGE("Info", "Create attribute table abnormally, num_handle (%d) \
                    doesn't equal to IDX_NB(%d)", param->add_attr_tab.num_handle, IDX_SH_NB);
        }
        else {
            memcpy(sensorhub_handle_table, param->add_attr_tab.handles, sizeof(sensorhub_handle_table));
            esp_ble_gatts_start_service(sensorhub_handle_table[IDX_SH_SVC]);
        }
        break;
      }
      case ESP_GATTS_SET_ATTR_VAL_EVT:
      //ESP_LOGI("Info", "ESP_GATTS_SET_ATTR_VAL_EVT");
      break;
      default:
      ESP_LOGI("Info", "UNKNOWN GATTS EVENT");
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
									esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI("Info", "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            sensorhub_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI("Info", "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == sensorhub_profile_tab[idx].gatts_if) {
                if (sensorhub_profile_tab[idx].gatts_cb) {
                    sensorhub_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_init()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE("Info", "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE("Info", "%s enable controller failed\n", __func__);
        return;
    }

    ESP_LOGI("Info", "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE("Info", "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE("Info", "%s enable bluetooth failed\n", __func__);
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_APP_ID);

    //Update GATT characteristic values in the attribute table
    updateSystemInfo();
    updateFileHeader();
    updateFileTransfer();

    return;
}
#endif
