#ifndef SENSORHUB_UART_H
#define SENSORHUB_UART_H

#include "sensorhub_sensordata.h"

#define BAUD_RATE 115200
#define ECHO_TEST_TXD  (GPIO_NUM_10)
#define ECHO_TEST_RXD  (GPIO_NUM_9)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_DE   (GPIO_NUM_4)
#define GPIO_OUTPUT_PIN_SEL  ((1 << ECHO_TEST_DE))

//Queue to handle UART events
static QueueHandle_t uart1_queue;

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    //size_t buffered_size;
    uint8_t* dtmp = malloc(1024);
    while (1) {
        /* Waiting for UART event.
           If it happens then print out information what is it */
        if (xQueueReceive(uart1_queue, (void * )&event, portMAX_DELAY)) {
            //ESP_LOGI("UART", "uart[%d] event:", UART_NUM_1);
            switch (event.type) {
            case UART_DATA:
                /* Event of UART receiving data
                 * We'd better handler data event fast, there would be much more data events
                 * than other types of events.
                 * If we take too much time on data event, the queue might be full.
                 * In this example, we don't process data in event, but read data outside.
                 */
                //uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
                //ESP_LOGI("UART", "data, len: %d; buffered len: %d", event.size, buffered_size);
                break;
            case UART_FIFO_OVF:
                ESP_LOGE("UART", "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // We can read data out out the buffer, or directly flush the Rx buffer.
                uart_flush(UART_NUM_1);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGE("UART", "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // We can read data out out the buffer, or directly flush the Rx buffer.
                uart_flush(UART_NUM_1);
                break;
            case UART_BREAK:
                ESP_LOGI("UART", "uart rx break detected");
                break;
            case UART_PARITY_ERR:
                ESP_LOGE("UART", "uart parity error");
                break;
            case UART_FRAME_ERR:
                ESP_LOGE("UART", "uart frame error");
                break;
            case UART_PATTERN_DET:
                ESP_LOGI("UART", "uart pattern detected");
                break;
            default:
                ESP_LOGE("UART", "not serviced uart event type: %d\n", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

/*Initialize the UART peripheral*/
void uart_init(void) {
  uart_config_t uart_config = {
      .baud_rate = BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
  uart_driver_install(UART_NUM_1, 2048, 2048, 10, &uart1_queue, 0);

  //initialize scan address
  memset(scan_addr, 0, SENSOR_ADDRESS_SIZE);
  scan_addr[0] = SCAN_ADDRESS_PREFIX;

  //Create DE gpio pin
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
  gpio_set_level(ECHO_TEST_DE, 0);

  // Create a task to handle UART events and errors
  xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

int sendData(uint8_t* data, size_t len) {
  gpio_set_level(ECHO_TEST_DE, 1);
  int ret = 0;
  ret = uart_write_bytes(UART_NUM_1, (char*)data, len);
  esp_err_t esp_err = uart_wait_tx_done(UART_NUM_1, 100);
  if (esp_err != ESP_OK) {
    if (esp_err == ESP_ERR_TIMEOUT) {
      ESP_LOGE("UART", "Timed out waiting for UART transmission");
    } else {
      ESP_LOGE("UART", "Error waiting for UART transmission to finish");
    }
  }
  gpio_set_level(ECHO_TEST_DE, 0);
  return ret;
}

int recvData(uint8_t* data, size_t len, size_t timeout) {
  //ESP_LOGI("UART", "uart_read_bytes start");
  int ret = uart_read_bytes(UART_NUM_1, data, len, timeout / portTICK_PERIOD_MS);
  //ESP_LOGI("UART", "uart_read_bytes end");
  return ret;
}

int sendAddr(uint8_t* addr, size_t sizeof_addr, uint8_t* ret, size_t sizeof_ret, size_t timeout) {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  //1. Send address
  if (sendData(addr, sizeof_addr) != sizeof_addr) {
    return 0;
  }
  //gpio_set_level(ECHO_TEST_DE, 0);
  //2. Wait for response
  int len = recvData(ret, sizeof_ret, timeout);
  //check if 1 byte was received
  return len;
}

/*Find a sensor in the list based on its address*/
sensor_node* findInList(sensor_node* head, uint8_t* toFind) {
  sensor_node* cur = head;
  if (!head) {return NULL;}
  while (cur) {
    if (memcmp(cur->addr, toFind, SENSOR_ADDRESS_SIZE) == 0) {return cur;}
    cur = cur->next;
  }
  return NULL;
}

/*Scan for new sensors. Returns true if a new sensor is found.*/
bool scan_sensors(sensor_node** head) {
  //ESP_LOGI("Info", "scan_sensors");
  uint8_t new_sensor[SENSOR_ADDRESS_SIZE];
  int len = 0;
  memset(new_sensor, 0, SENSOR_ADDRESS_SIZE);
  //scan for any sensor
  len = sendAddr(scan_addr, 1, new_sensor, SENSOR_ADDRESS_SIZE, SCAN_TIMEOUT);
  //check if a new sensor responded
  //ESP_LOGI("Info", "len = %d", len);
  if (len == SENSOR_ADDRESS_SIZE) {
    if(!isSensorAddr(new_sensor)) {
      //ESP_LOGE("UART", "Detected sensor with invalid address (0x%x).", new_sensor[0]);
      return false;
    }
    char sensorType[24];
    getSensorType(sensorType, new_sensor);
    sensor_node* sensor = findInList(*head, new_sensor);
    if (sensor == NULL) { //hasn't been connected before
      ESP_LOGI("Info", "New %s sensor module detected (address = 0x%x).", sensorType, new_sensor[0]);
      enqueue(head, new_sensor);
    } else { //sensor is being reconnected
      ESP_LOGI("Info", "%s sensor has been reconnected (address = 0x%x).", sensorType, new_sensor[0]);
      sensor->connected = true;
    }
    return true;
  }
  return false;
}

/*Poll sensor and gather its data.*/
data_sensor_t poll_sensor(sensor_node* sensor) {
  //ESP_LOGI("Info", "Polling sensor: address = 0x%x.", sensor->addr[0]);
  data_sensor_t ret;
  memset(&ret, 0, sizeof(data_sensor_t));
  uint8_t* buffer = malloc(SENSOR_MAX_DATA_SIZE);
  memset(buffer, 0, SENSOR_MAX_DATA_SIZE);
  //1. Get data
  size_t len = sendAddr(sensor->addr, SENSOR_ADDRESS_SIZE, buffer, SENSOR_MAX_DATA_SIZE, SENSOR_TIMEOUT);
  //check if appropriate amount of data was received
  if (len == getSensorType(NULL, sensor->addr)) {
    memcpy(ret.addr, sensor->addr, SENSOR_ADDRESS_SIZE);
    memcpy(ret.data, buffer, SENSOR_ADDRESS_SIZE);
    gettimeofday(&tv, &tz);
    //ret.time = timeinfo;
    ret.len = len;
  } else if (len != 0) {
    ESP_LOGE("Sensor Error", "Received incorrect amount of data from sensor 0x%x. Expected %d bytes but received %d.", sensor->addr[0], getSensorType(NULL, sensor->addr), len);
  }
  free(buffer);
  return ret;
}

/*Sensor didn't respond to a poll so remove it. (deprecated)*/
void remove_sensor(sensor_node** cur, sensor_node** head) {
  char sensorType[24];
  getSensorType(sensorType, (*cur)->addr);
  ESP_LOGE("Error", "Did not receive valid data from %s sensor module (address is 0x%x).", sensorType, (*cur)->addr[0]);
  ESP_LOGE("Error", "Removing %s sensor module...", sensorType);
  removeNode(cur, head);
}

void uart_deinit() {
  uart_driver_delete(UART_NUM_1);
}

#endif
