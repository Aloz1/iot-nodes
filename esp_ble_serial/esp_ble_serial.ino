#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *ble_server = NULL;
BLEService *uart_svc;
BLECharacteristic *tx_chrstc, *rx_chrstc;

bool dev_connected = false;
bool prev_connected = false;

const size_t MSG_BUFF_LEN = 8192;

char msg_buff[MSG_BUFF_LEN] = {0};
size_t msg_buff_start = 0;
size_t msg_buff_end = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks
{
  void onConnect(BLEServer* ble_server) {
    dev_connected = true;
  }

  void onDisconnect(BLEServer* ble_server) {
    dev_connected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *chrstc) {
    std::string rx_val = chrstc->getValue();

    // Write message sent from RPi to GPS module.
    if (rx_val.length() > 0) {
      Serial.print(rx_val.c_str());
    }
  }
};


void setup() {
  /* UART for PC/Debug */
  Serial.begin(115200);

  /* UART for GPS module*/
  // UART Baudrates on the GTop module are selected through 10K pulldown resistors
  // on BR0 and BR1 pins. Configurations are as follows.
  //
  // |        |    BR1    |    BR0
  // |  Rate  | (Pin  12) | (Pin  11)
  // |--------|-----------|-----------
  // |   9600 |    N/C    |    N/C
  // | 115200 |    N/C    |    10K
  // |   4800 |    10K    |    N/C
  // |  38400 |    10K    |    10K
  Serial2.begin(9600);

  // Turn on the GPS module
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // Create the BLE Device
  BLEDevice::init("ESP GPS Node");

  // Create the BLE Server
  ble_server = BLEDevice::createServer();
  ble_server->setCallbacks(new MyServerCallbacks());

  // Create the BLE UART Service
  uart_svc = ble_server->createService(SERVICE_UUID);

  // Create BLE characteristics
  tx_chrstc = uart_svc->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  tx_chrstc->addDescriptor(new BLE2902());

  rx_chrstc = uart_svc->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  rx_chrstc->setCallbacks(new MyCallbacks());

  // Start the service
  uart_svc->start();

  // Start advertising
  ble_server->getAdvertising()->start();
}

void loop() {

  // Check if there's serial data
  while (Serial2.available()) {
    int new_char = Serial2.read();

    // If byte is borked, just skip it.
    if (new_char < 0 || new_char > 0xff)
      continue;

    // Increment "start" if the end is getting close.
    if ((msg_buff_end + 1) % MSG_BUFF_LEN == msg_buff_start)
      msg_buff_start = (msg_buff_start + 1) % MSG_BUFF_LEN;

    // Add data to ring buffer
    msg_buff[msg_buff_end++] = (char)new_char;
    if (msg_buff_end > MSG_BUFF_LEN - 1)
      msg_buff_end = 0;
  }

  if (dev_connected) { // Connected

    // We have data to send.
    if (msg_buff_end != msg_buff_start) {
      char next_msg[ESP_GATT_MAX_ATTR_LEN] = {0};
      int next_msg_len;

      int ext_msg_buff_end = msg_buff_end;
      int buff_right_len   = msg_buff_end - msg_buff_start;

      if (msg_buff_end < msg_buff_start) {
        ext_msg_buff_end += MSG_BUFF_LEN;
        buff_right_len = MSG_BUFF_LEN - msg_buff_start;
      }

      int buff_fullness = ext_msg_buff_end - msg_buff_start;
      next_msg_len = (buff_fullness > ESP_GATT_MAX_ATTR_LEN ? ESP_GATT_MAX_ATTR_LEN : buff_fullness);

      int first_cpy_len = (next_msg_len > buff_right_len ? buff_right_len : next_msg_len);
      int second_cpy_len = (next_msg_len > buff_right_len ? next_msg_len - first_cpy_len : 0);
      
      memcpy(next_msg, msg_buff + msg_buff_start, first_cpy_len);

      if (second_cpy_len)
        memcpy(next_msg + first_cpy_len, msg_buff, second_cpy_len);
      
      msg_buff_start = (msg_buff_start + next_msg_len) % MSG_BUFF_LEN;

      Serial.print("Notify send length: ");
      Serial.println(next_msg_len);
      
      tx_chrstc->setValue((uint8_t *)next_msg, next_msg_len);
      tx_chrstc->notify();
    }

    // Bluetooth stack will go into congestion, if too many packets are sent
		delay(10); 
	} else { // Nothing connected
    
	}

  // Disconnecting
  if (!dev_connected && prev_connected) {
    Serial.println("Client disconnected");
    delay(500); // Give the bluetooth stack time to get ready, before enabling advertising.

    // Restart advertising
    ble_server->startAdvertising();
    prev_connected = dev_connected;
  }
  
  // Connecting
  if (dev_connected && !prev_connected)
    Serial.println("Client connected");
    prev_connected = dev_connected;
}
