#ifndef SENSORHUB_SENSORDATA_H
#define SENSORHUB_SENSORDATA_H

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_int_wdt.h"

/*Sensor Addresses:
* 8 bits long
* First 3 most-significant bits specify the type of sensor.
  001 - Gas
  010 - Noise
  011 - Rel. Humidity & Temp.
  100 - Thermocouple
  101 - Vibration
  110 - Reserved for scanning
* Remaining 5 bits are the unique sensor ID.
* Sensor address specifies how many milliseconds the sensor
  will wait to respond to a scan.
  Example: Sensor ID = 0101 0111
  A Noise sensor module that will wait for 87 ms to respond to a scan
*/
#define GAS_ADDRESS_PREFIX      0x20
#define NOISE_ADDRESS_PREFIX    0x40
#define RHT_ADDRESS_PREFIX      0x60
#define THERM_ADDRESS_PREFIX    0x80
#define VIB_ADDRESS_PREFIX      0xA0
#define SCAN_ADDRESS_PREFIX     0xC0
#define INVALID_ADDRESS_PREFIX  0x00
#define ADDRESS_PREFIX_BITMASK  0xE0
#define SENSOR_ADDRESS_SIZE (1 * sizeof(uint8_t))

/*Sensor data constants*/
#define SENSOR_MAX_DATA_SIZE (24 * sizeof(uint8_t))
#define GAS_SENSOR_DATA_SIZE (sizeof(int))
#define NOISE_SENSOR_DATA_SIZE (sizeof(int))
#define RHT_SENSOR_DATA_SIZE (sizeof(int))//(sizeof(float) + sizeof(int))
#define THERM_SENSOR_DATA_SIZE (sizeof(int))//(sizeof(double))
#define VIB_SENSOR_DATA_SIZE (sizeof(int))//(sizeof(float))

/*Sensor timeouts*/
#define SCAN_TIMEOUT 256 //Timeout period in ms for a scan of all sensors
#define SENSOR_TIMEOUT 100 //Timeout period in ms when polling a specific sensor

//GATT Characteristic Value Definitions
struct  __attribute__((__packed__)) system_info_value_t {
  //HUB
  /*Number of seconds since Jan. 1, 1970. Set to 32 bits because 64 bits puts
  characteristic size over the 20-byte GATT limit*/
  uint32_t current_time;
  /*Battery percentage and SD card connectivity (MSB indicates if SD card is connected)*/
  uint8_t battery_SD;
  /*LCD screen timeout in seconds*/
  uint8_t hub_LCD_timeout;
  //MIC
  /*Number of seconds per noise sample*/
  uint8_t mic_samp_per;
  //ACCELEROMETER
  /*Mercalli scale threshold (category 5-10)*/
  uint8_t acc_threshG;
  //HUMIDITY AND TEMPERATURE
  /*Threshold temperature (if MSB is 1, deg C).*/
  uint32_t rht_threshTemp;
  //THERMOCOUPLE
  /*Number of minutes per sample (if MSB is 1, deg C)*/
  uint8_t thermo_samp_per;
};
struct  __attribute__((__packed__)) file_header_value_t {
  /*Sensor type + address. Example: Gas_0x27*/
  char sensor_prefix[11];
  /*Size of file in bytes*/
  uint32_t filesize;
};
struct __attribute__((__packed__)) file_transfer_value_t {
  /*Raw data from file*/
  uint8_t data[16];
  /*Sequential number for block of data*/
  uint32_t block_num;
};

uint32_t max_block_num = 0; //maximum block number for the current transfer
bool isSDconnected = false;

struct system_info_value_t system_info_value;
struct file_header_value_t file_header_value;
struct file_transfer_value_t file_transfer_value;

uint8_t scan_addr[SENSOR_ADDRESS_SIZE];
bool isBLEconnected = false;

typedef struct sensor_node {
  uint8_t addr[SENSOR_ADDRESS_SIZE];
  bool connected;
  struct sensor_node* prev;
  struct sensor_node* next;
} sensor_node;

//head of linked list to store connected sensors
sensor_node* head = NULL;
//Next file to send
sensor_node* next_file = NULL;

typedef struct data_sensor {
  uint8_t addr[SENSOR_ADDRESS_SIZE];
  uint8_t data[SENSOR_MAX_DATA_SIZE];
  size_t len;
  //struct tm time;
} data_sensor_t;

sensor_node* makeNode(uint8_t* addr) {
  sensor_node* ret = malloc(sizeof(sensor_node));
  memcpy(ret->addr, addr, SENSOR_ADDRESS_SIZE);
  ret->prev = NULL;
  ret->next = NULL;
  ret->connected = true;
  return ret;
}

/*Add sensor to the end of the list.*/
void enqueue(sensor_node** head, uint8_t* addr) {
  if(!(*head)) { //if queue is empty
    *head = makeNode(addr);
    next_file = *head; //initialize next_file
    return;
  }
  sensor_node* cur = *head;
  //go to the end of the queue
  while (cur->next) {
    if (addr[0] == cur->addr[0]) { //edge case
      ESP_LOGE("Queue error", "Sensor with address 0x%x responded to a scan twice", addr[0]);
      return;
    }
    cur = cur->next;
  }
  //add a node at the end
  cur->next = makeNode(addr);
  cur->next->prev = cur;
}

void removeNode(sensor_node** cur, sensor_node** head) {
  //Case: head is NULL
  if(!(*head)) {return;}
  //Case: only 1 node in the list
  if(!((*head)->next)) {
    free(*head);
    *head = NULL;
    *cur = *head;
    return;
  }
  //Case: removing from start
  if(*head == *cur) {
    *head = (*head)->next;
    (*head)->prev = NULL;
    free(*cur);
    *cur = *head;
    return;
  }
  //Case: removing from end
  if(!((*cur)->next)) {
    *cur = (*cur)->prev;
    free((*cur)->next);
    (*cur)->next = NULL;
    return;
  }
  //Case: removing in the middle
  (*cur)->prev->next = (*cur)->next;
  (*cur)->next->prev = (*cur)->prev;
  *cur = (*cur)->next;
  free((*cur)->prev);
  return;
}

void printNodes(sensor_node* head) {
  sensor_node* cur = head;
  if(!head) {
    ESP_LOGI("Sensor List", "Empty");
    return;
  }
  int i = 1;
  while (cur) {
    ESP_LOGI("Sensor List", "#%d, address = 0x%x.", i++, cur->addr[0]);
    cur = cur->next;
  }
}

void freeNodes(sensor_node* cur) {
  if(cur->next) {freeNodes(cur->next);}
  free(cur);
}

bool isSensorAddr(uint8_t* addr) {
  uint8_t addr_prefix = addr[0] & ADDRESS_PREFIX_BITMASK;
  switch (addr_prefix) {
    case GAS_ADDRESS_PREFIX: return true;
    case NOISE_ADDRESS_PREFIX: return true;
    case RHT_ADDRESS_PREFIX: return true;
    case THERM_ADDRESS_PREFIX: return true;
    case VIB_ADDRESS_PREFIX: return true;
    default: return false;
  }
}

/*String representation of sensor type based on its address.*/
size_t getSensorType(char* ret, uint8_t* addr) {
  char sensorType = addr[0] & ADDRESS_PREFIX_BITMASK;
  switch (sensorType) {
    case GAS_ADDRESS_PREFIX:
      if(ret) {sprintf(ret, "Gas");}
      return GAS_SENSOR_DATA_SIZE;
    case NOISE_ADDRESS_PREFIX:
      if(ret) {sprintf(ret, "Noise");}
      return NOISE_SENSOR_DATA_SIZE;
    case RHT_ADDRESS_PREFIX:
      if(ret) {sprintf(ret, "RHT");}
      return RHT_SENSOR_DATA_SIZE;
    case THERM_ADDRESS_PREFIX:
      if(ret) {sprintf(ret, "Therm");}
      return THERM_SENSOR_DATA_SIZE;
    case VIB_ADDRESS_PREFIX:
      if(ret) {sprintf(ret, "Vib");}
      return VIB_ADDRESS_PREFIX;
    default:
      ESP_LOGE("Error", "Unrecognized sensor type.");
  }
  return 0;
}

void getUnits(char* ret, data_sensor_t data_sensor) {
  char sensorType = data_sensor.addr[0] & ADDRESS_PREFIX_BITMASK;
  //char buf[24];
  //memset(buf, '\0', sizeof(buf));
  switch (sensorType) {
    case GAS_ADDRESS_PREFIX: //Gas
      sprintf(ret, "%% Deviation");
      break;
    case NOISE_ADDRESS_PREFIX: //Noise
      sprintf(ret, "dB");
      break;
    case RHT_ADDRESS_PREFIX: //RHT
      sprintf(ret, "degrees C");
      //getTempPref(buf);
      //strcat(ret, buf);
      strcat(ret, " & %% Humditiy");
      break;
    case THERM_ADDRESS_PREFIX: //Thermocouple
      sprintf(ret, "degrees C");
      //getTempPref(buf);
      //strcat(ret, buf);
      break;
    case VIB_ADDRESS_PREFIX: //Vibration
      sprintf(ret, "g-force");
      break;
    default:
      ESP_LOGE("Error", "Unrecognized sensor type.");
      return;
  }
  return;
}

void getValue(char* ret, data_sensor_t data_sensor) {
  char sensorType = data_sensor.addr[0] & ADDRESS_PREFIX_BITMASK;
  char buf[24];
  int value;
  float temp;
  memset(buf, '\0', sizeof(buf));
  switch (sensorType) {
    case GAS_ADDRESS_PREFIX:
      value = (int)(*data_sensor.data);
      itoa(value, ret, 10);
      break;
    case NOISE_ADDRESS_PREFIX:
      value = (int)(*data_sensor.data);
      itoa(value, ret, 10);
      break;
    case RHT_ADDRESS_PREFIX:
      temp = (float)(*data_sensor.data);
      value = (int)(*(data_sensor.data + sizeof(temp)));
      //getTempPref(buf);
      //if (buf[0] == 'F') {temp = (9.0/5.0)*temp + 32.0;}
      sprintf(ret, "%.1f %d", temp, value);
      break;
    case THERM_ADDRESS_PREFIX:
      temp = (float)(*data_sensor.data);
      //getTempPref(buf);
      //if (buf[0] == 'F') {temp = (9.0/5.0)*temp + 32.0;}
      sprintf(ret, "%.1f", temp);
      break;
    case VIB_ADDRESS_PREFIX:
      temp = (float)(*data_sensor.data);
      sprintf(ret, "%.3f", temp);
      break;
    default:
      ESP_LOGE("Error", "Unrecognized sensor type.");
      return;
  }
}

//Create file suffix for sensor (doesn't add file extension)
//Format: PREFIX_SENSORTYPE_0xSENSORADDRESS
//Example: data_Gas_0x27
void getSensorPrefix(char* string, uint8_t* addr) {
  char temp[12];
  memset(temp, '\0', sizeof(temp));
  getSensorType(temp, addr);
  strcat(string, temp);
  //make string of sensor address with \0 at the end
  uint8_t sensor_addr[SENSOR_ADDRESS_SIZE + 1];
  memset(sensor_addr, '\0', SENSOR_ADDRESS_SIZE + 1);
  memcpy (sensor_addr, (char*)addr, SENSOR_ADDRESS_SIZE);
  strcat(string, "0x");
  int i;
  for(i = 0; i < SENSOR_ADDRESS_SIZE; i++) { //print address
    sprintf(string + strlen(string), "%x", sensor_addr[i]);
  }
}

uint32_t getFileSize(char* filename) {
  FILE* f = fopen(filename, "r");
  if (f == NULL) {
    return 0;
  }
  fseek(f, 0L, SEEK_END);
  uint32_t ret = (uint32_t)ftell(f);
  ESP_LOGI("Info", "Size of file = %d bytes", ret);
  fclose(f);
  return ret;
}

#endif
