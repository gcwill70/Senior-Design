#include "sensorhub_time.h"
#include "sensorhub_uart.h"
#include "sensorhub_sdcard.h"
#include "sensorhub_sensordata.h"
#include "sensorhub_ble.h"

#define CHECK_ERROR_CODE(returned, expected) ({                        \
            if(returned != expected){                                  \
                printf("TWDT ERROR\n");                                \
                abort();                                               \
            }                                                          \
})

void sensor_data_task() {
  //linked list of sensor addresses
  head = NULL;
  //1. Perform intial scan
  while(scan_sensors(&head));
  ESP_LOGI("Info", "Initial scan complete.");
  data_sensor_t data_sensor;
  sensor_node* cur;
  //2. Enter main loop
  while (1) {
    cur = head;
    ///a) Poll each sensor
    while (cur) { //for each sensor
      vTaskDelay(5 / portTICK_PERIOD_MS);
      if (isBLEconnected) {continue;}
      //gpio_set_level(GPIO_NUM_4, 1);
      if (cur->connected) { //if sensor is connected
        data_sensor = poll_sensor(cur);
      } else {
        cur = cur->next;
        continue;
      }
      if (data_sensor.len) { //if sensor responded
        //store data
        print_data(data_sensor);
      } else { //sensor didn't respond
        cur->connected = false;
        char sensorType[24];
        getSensorType(sensorType, cur->addr);
        ESP_LOGI("Info", "%s sensor has been disconnected (address = 0x%x).", sensorType, cur->addr[0]);
      }
      //go to next sensor
      cur = cur->next;
      //gpio_set_level(GPIO_NUM_4, 0);
    }
    ///b) Check if a sensor was connected
    scan_sensors(&head);
  }
}

void main_task() {
  //1. Initialize modules
  ///a) Initialize UART
  uart_init();
  ESP_LOGI("Info", "UART initialized.");
  ///b) Initialize LCD screen
  //lcd_init();
  //ESP_LOGI("LCD", "LCD initialized.");
  ///c) Mount the SD card
  if (sdcard_setup()) {
    //lcd_deinit();
    esp_restart();
  }
  ///d) Initialize BLE
  ble_init();
  ESP_LOGI("Info", "BLE initialized.");
  ///e) Initialize the time
  if (time_setup()) {
    //lcd_deinit();
    sdcard_deinit();
    esp_restart();
  }
  //3. Start subsystems
  xTaskCreate(sensor_data_task, "sensor_data_task", 2048, NULL, 12, NULL);
  //xTaskCreate(lcd_task, "lcd_task", 2048, NULL, 12, NULL);
  //sensor_data_task();
  vTaskDelete(NULL);
}

void app_main() {
  xTaskCreate(main_task, "main_task", 2048, NULL, 12, NULL);
}
