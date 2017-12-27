#ifndef SENSORHUB_TIME_H
#define SENSORHUB_TIME_H

#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "apps/sntp/sntp.h"

#include "sensorhub_sensordata.h"

struct timeval tv;
struct timezone tz;

int time_init() {
  gettimeofday(&tv, &tz);
  //ESP_LOGI("Info", "Current time: %lu", (long)tv.tv_sec);
  if (tv.tv_sec < 1511967321) { //current epoch time as of 11/29/17, 10:09AM ET
      return 1;
  }
  return 0;
}

void printTime(char* tmbuf, size_t size) {
  gettimeofday(&tv, &tz);
  struct tm* nowtm = localtime(&(tv.tv_sec));
  strftime(tmbuf, size, "%Y-%m-%d %H:%M:%S", nowtm);
}

int time_setup() {
  char tmbuf[64];
  struct tm* nowtm;
  bool displayedError = false;
  while (time_init()) {
    //if (isBLEconnected) {continue;}
    if (!displayedError) { //display error only once
      ESP_LOGE("Current Time", "Could not initialize the current time. Please sync via the app.");
      displayedError = true;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 1 second and check again
  }
  gettimeofday(&tv, &tz);
  nowtm = localtime(&(tv.tv_sec));
  strftime(tmbuf, sizeof(tmbuf), "%Y-%m-%d %H:%M:%S", nowtm);
  ESP_LOGI("Info", "The current date/time at Purdue is: %s", tmbuf);
  return 0;
}
#endif
