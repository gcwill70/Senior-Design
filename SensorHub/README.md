# SensorHub
## Description
This is my EE senior design project at Purdue. The goal was to create a modular sensor system. A central hub can connect via UART/RS485 to five different types of sensors:
1. Harmful gas
2. Noise
3. Vibration
4. Relative Humidity and Temperature
5. Thermocouple (high temperature sensing)

The provided code was run on the central hub which was an ESP32 microcontroller (see its [datasheet](https://github.com/gcwill70/Senior-Design/blob/master/SensorHub/Datasheets/ESP32.pdf) for more info). It is responsible for determining what sensors are connected, gathering their data, storing it in a CSV file (using a connected SD card), and transmitting that data over Bluetooth Low Energy (BLE) to an Android device. 

Each sensor can be attached and removed as needed at any time. The hub will continuously scan via the UART/RS485 interface to determine which sensors are connected and poll them. After data is logged, the user can connect to the hub via BLE GATT and transfer the sensor data to their device. There is also an initial sync required with an Android device so the internal time on the ESP32 can be initialized.

Please see the block diagrams and timing diagrams for more info.

## References
[ESP32 provided examples](https://github.com/espressif/esp-idf/tree/master/examples)
