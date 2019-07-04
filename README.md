# IoT vehicle tracking - node firmware
A simple project demonstrating an IoT approach to vehicle tracking. The project has 3 software
components: node devices, the edge device, and the web client. Node devices notify the edge device
of new data via BLE GATT, which is then fetched by the edge device and cached locally in a redis
database. Data is then pushed to the cloud (AWS) via MQTT. When AWS receives data via MQTT, it is
stored in a dynamodb for later retreival. Finally, this data is pulled by the web client and presented
as both a table of values and a path on google maps (for GPS data).

This repository is specifically for node device firmware. For other software components, please see
their corresponding repositories (in the [Other repositories](#other-repositories) section below).

# Node devices
There are currently 2 node device implementations. One using an ESP32 with a GPS module for geo
tracking, and a second using an Arduino with a BLE module (HM-10) and an IMU module
(Sparkfun MPU-9250). Code for each node be found in the [esp\_ble\_serial](esp_ble_serial) and
[arduino\_accel](arduino_accel) directories respectively.

## ESP32 GPS node
The ESP32 GPS node utilises a custom Global Top GPS receiver module, an ESP32 module, and its built
in Bluetooth BLE tranciever hardware. Communications with the GPS module are via a unidirectional
UART connection, with the ESP32 using a simple ring buffer for incoming data. NMEA data from the
GPS module is then communicated to the edge device using a UART like protocol implemented on top of
BLE GATT. NMEA data is broken into blocks of length `ESP_GATT_MAX_ATTR_LEN` and set as the next
_chunk_ of the message.

Most of the heavy lifting of the BLE GATT protocol is performed by the ESP BLE framework, with only
minor things, such as fetching from the ring buffer and detecting when a client connects or
disconnects, being performed within the _.ino_ file.

## Arduino IMU node
The Arduino IMU node sports a 9DOF Sparkfun IMU sensor (MPU-9250) and an HM-10 based Jaycar BLE
module, as well as an Arduino Nano (although any Arduino device could theoretically be used). The
MPU-9250 is capable of measureing acceleration (accelerometer), relative direction changes
(gyroscope) and 3D magnetic field strength (magnetometer).

Data is collected from the IMU over I2C, using the Sparkfun provided MPU-9250 library. If an edge
device is connected via BLE (as indicated by the BLE module state pin), then collected data is
formatted in plain text and transmitted to the BLE module every 200 milliseconds. The BLE module
receives the data and transmits it via BLE to the connected edge device, in a smiliar fashion to
the ESP32 node above.

# Other repositories
- [Edge application](https://github.com/Aloz1/iot-raspi)
- [Web client](https://github.com/Aloz1/iot-website)

# The report
For more details, take a look at the [corresponding report](https://github.com/Aloz1/iot-report)
