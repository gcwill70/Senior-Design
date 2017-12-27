# Description
These timing diagrams describe the communication protocol that is implemented within the central hub to detect what types of sensors are connected.

# Addressing
Because more than one type of sensor can be connected to the central hub, an addressing scheme was developed. Each sensor will be given an 8-bit address. The three most-significant bits (hereafter referred to as a 'prefix') determine what type of sensor it is (gas, noise, vibration, etc.) and the remaining five bits are a unique identifier (in case more than one of the same type of sensor are connected). The list below shows each sensor's prefix.

1. 001 - Gas
2. 010 - Noise
3. 011 - Rel. Humidity & Temp.
4. 100 - Thermocouple
5. 101 - Vibration
6. 110 - Reserved for scanning

# Scanning
As shown in the scanning diagram above, the hub will send out a designated scan byte (whose prefix must be '110'). If a sensor module hasn't responded to a scan yet, it will wait for a certain number of milliseconds to respond. If no other sensor has responded after that delay, it will respond with its unique address. The scan delay for each sensor is determined by its unique address.
## Example
If the sensor address is 0101 0111, it is a noise sensor that will wait 87 ms to respond to a scan.

# Polling
After the hub has scanned for all the connected sensors, it will start polling each one. As sensors respond to the scans, the hub will form a linked list of each sensor that is connected. After each sensor is polled, it will send out a single scan byte to see if any new sensors are connected. If a sensor fails to respond to a scan, it will be considered disconnected and the sensor must respond to a scan again to be reconnected. See the polling timing diagram for more details.
