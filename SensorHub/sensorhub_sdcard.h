#ifndef SENSORHUB_SDCARD_H
#define SENSORHUB_SDCARD_H

#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include "sensorhub_time.h"
#include "sensorhub_sensordata.h"

// Pin mapping when using SPI mode.
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13

int sdcard_init(void) {
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  slot_config.gpio_miso = PIN_NUM_MISO;
  slot_config.gpio_mosi = PIN_NUM_MOSI;
  slot_config.gpio_sck  = PIN_NUM_CLK;
  slot_config.gpio_cs   = PIN_NUM_CS;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5
  };

  sdmmc_card_t* card;
  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
      if (ret == ESP_FAIL) {
          ESP_LOGE("SD Card", "Failed to mount filesystem. "
              "If you want the card to be formatted, set format_if_mount_failed = true.");
      } else {
          ESP_LOGE("SD Card", "Failed to initialize the card (%d).", ret);
      }
      return 1;
  }

  isSDconnected = true;
  return 0;
}

void sdcard_deinit(void) {
  esp_vfs_fat_sdmmc_unmount();
  ESP_LOGI("SD Card", "Card unmounted.");
}

int sdcard_setup() {
  if (sdcard_init()) {
    //lcd_error("SD Card failure!");
    ESP_LOGE("SD Card", "Card could not be mounted properly. Restarting...");
    esp_restart();
  }
  return 0;
}

void printHeader(char* filename, data_sensor_t data_sensor) {
  //ESP_LOGI("Info", "File to open %s", filename);
  while(isBLEconnected); //wait for BLE to be disconnected
  FILE* data_file = fopen(filename, "a");
  if (data_file == NULL) {
    ESP_LOGE("SD Card", "Failed to open data file. Restarting...");
    //lcd_deinit();
    sdcard_setup();
    return;
  }
  //ESP_LOGI("Info", "printHeader found data file");
  fseek(data_file, 0, SEEK_END);
  long len = ftell(data_file);
  if (len == 0) {
    char buf[24];
    memset(buf, '\0', sizeof(buf));
    getUnits(buf, data_sensor);
    fprintf(data_file, "Date/Time,Value,Units = %s\n", buf);
  }
  fclose(data_file);
}

void print_data(data_sensor_t data_sensor) {
  char filename[32];
  memset(filename, '\0', sizeof(filename));
  strcat(filename , "/sdcard/data_");
  getSensorPrefix(filename, data_sensor.addr);
  strcat(filename, ".csv");
  printHeader(filename, data_sensor); //print header if necessary
  while(isBLEconnected); //wait for BLE to be disconnected
  FILE* data_file = fopen(filename, "a");
  if (data_file == NULL) {
    perror("data file error");
    ESP_LOGE("SD Card", "Failed to open data.csv file. Restarting...");
    //lcd_deinit();
    sdcard_setup();
    return;
  }
  //ESP_LOGI("SD Card", "Storing data from address 0x%x with value = %d", data_sensor.addr[0], (int)(data_sensor.data))
  char buf[24];
  //1. Date/Time
  memset(buf, '\0', sizeof(buf));
  printTime(buf, sizeof(buf));
  fprintf(data_file, "%s,", buf);
  //2. Value
  memset(buf, '\0', sizeof(buf));
  getValue(buf, data_sensor);
  fprintf(data_file, "%s\n", buf);
  // //3. Units
  // memset(buf, '\0', sizeof(buf));
  // getUnits(buf, data_sensor);
  // fprintf(data_file, "%s\n", buf);
  fclose(data_file);
}

#endif
