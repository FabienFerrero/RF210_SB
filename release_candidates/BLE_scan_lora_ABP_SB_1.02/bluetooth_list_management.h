#pragma once

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <vector>

// Parameters for BLE counting
const int nullscan = 5;           // number of undetected BLE scan to remove a device from the list
const int id_max_count = 15;      // number of continuous detection to move in the white list
const int margin_WL = 50;         // margin added to the white list counter when a device is added to white list
const int inc_margin_WL = 5;      // margin added to the white list counter when a device in the white list is detected again
const int rssi_threshold = -100;  // threshold to consider a BLE detection (in dBm)


class deviceDetected {
public:
  int8_t cnt;    // Number of detections for this device
  int8_t undet;  // Number of succesive undetections for this device
  int8_t cntWL;  // If in the white list, number of remaining iterations to be removed from it

  // Data to send for this device
  int8_t rssi;      // Received Signal Strength Indicator (RSSI) for the detection
  BLEAddress data;  // the address (example - 00:11:22:33:FF:EE)

  // Constructors
  deviceDetected() = default;

  deviceDetected(int8_t rssi, BLEAddress data)
    : cnt(1), undet(0), cntWL(margin_WL), rssi(rssi), data(data) {}

  // Destructor
  ~deviceDetected() {}

  // Rôle : display the informations of this device
  void display() const {
    BLEAddress addr = data;
    Serial.print(addr.toString().c_str());
    Serial.print(" ");
    Serial.print(cnt);
    Serial.print(" : ");
    Serial.print(undet);
    Serial.print(" RSSI: ");
    Serial.println(rssi);
  }
};

// Global variables for lists management
struct BluetoothData {
  int8_t id_num;    // number of devices in the main list
  int8_t scan_num;  // number of detected devices
  int8_t scan_new;  // number of devices in the main list
  int8_t scan_del;  // number of deleted devices in the main list
  int8_t cnt_wl;    // number of devices in the white list
};

// Initalize variables and lists
struct BluetoothData bluetoothData = { 0, 0, 0, 0, 0 };
static std::vector<deviceDetected> list;
static std::vector<deviceDetected> whiteList;


// Define arrays to store new devices
static deviceDetected* newBLE_adr[200];  // BLE adress to append the next lora packet
static int8_t new_BLE_cnt = 0;           // counter actual array index of my_BLE_adr

int scanTime = 4;  //In seconds
BLEScan* pBLEScan;

bool bluetoothActivation = 1;


class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
  }
};


// Role : convert an array of bytes into an array of characters
void array_to_string(byte array[], unsigned int len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++) {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
  }
  buffer[len * 2] = '\0';
}


// Role : print informations of devices of the xhite list
void printWhiteList() {
  int m = whiteList.size();
  if (m > 10) m = 10;

  Serial.printf(" List of White listed device \n");
  for (int i = 0; i < m; i++) {
    Serial.printf(" %x : ", whiteList[i].data);
    Serial.println(whiteList[i].cntWL);
    delay(1);
  }
}
// print informations of devices of the main list
void printDetectedDevices() {
  Serial.printf(" Short List of detected device \n");
  int l = list.size();
  if (l > 10) l = 10;

  for (const auto& device : list) {
    device.display();
  }
}

// Role : add a device to the list of the new devices detected
void new_add_payload(deviceDetected* newDevice) {
  newBLE_adr[new_BLE_cnt] = newDevice;
  new_BLE_cnt++;
}

// Role : remove old addresses in list and white list
void process_list() {


/*
  // for every device in the main list
  auto it = list.begin();
  while (it != list.end()) {
    if (it->cnt > id_max_count || it->undet > nullscan) {
      if (it->cnt > id_max_count) {
        // move device in white list
        whiteList.push_back(*it);
        it->cntWL = margin_WL;
      }
      // Supprimer l'élément du vecteur
      it = list.erase(it);  // `erase` renvoie un itérateur vers le prochain élément
      bluetoothData.scan_del++;
    } else {
      ++it;  // Passer à l'élément suivant si aucune suppression
    */
    // Process main list: 
    // if a device has not been scanned for a long time, remove from the main list
    // if a device has been scanned too much times, add it to the white list

    list.erase( // remove all elements between the two following positions :
    // put all elements we have to remove in the end of the vector
    std::remove_if(  //returns the position of the first element to delete
      list.begin(),
      list.end(),
      [&](auto& device) { // if the condition is validated, move to the end of the list
          // if too much successive detections
          if (device.cnt > id_max_count) {
            whiteList.push_back(device);  // Add it to the white list
            device.cntWL = margin_WL;
            bluetoothData.scan_del++;  // Update the counter
            return true;               // Suppress the element by validating the condition
          }
          // if too much undetections
          if (device.undet > nullscan) {
            bluetoothData.scan_del++;  // Update the counter
            return true;               // Suppress the element by validating the condition
          }
          return false;  // Do not delete
            }
      ),
      list.end() // second position : the position of the last element of the list
    );

/*
  // process white list, if a device not scan for a long time, remove from the white list
  it = whiteList.begin();
while (it != whiteList.end()) {
  if (it->cntWL == 0) {
    // Supprimer l'élément du vecteur
    it = whiteList.erase(it);  // `erase` renvoie un itérateur vers le prochain élément
  } else {
    ++it;  // Passer à l'élément suivant si aucune suppression
  }
}
*/ 

// Process white list, if a device has not been scanned for a long time, remove from the white list
whiteList.erase(
  std::remove_if(
    whiteList.begin(),
    whiteList.end(),
    [](const auto& device) {
      return device.cntWL == 0;  // Supprime l'élément si cntWL est égal à 0
    }
  ),
  whiteList.end()
);

// Mettre à jour les données Bluetooth après la modification des listes
bluetoothData.id_num = list.size();
bluetoothData.cnt_wl = whiteList.size();
}


void counter() {
  for (auto& device : list) {
    device.undet++;  // increment undet counter before new scan
  }
  for (auto& device : whiteList) {
    device.cntWL--;  // decrease white list counter before new scan
  }
  // Reset state counters
  bluetoothData.scan_num = 0;  // number of BLE adress detected on the last scan
  bluetoothData.scan_new = 0;  // number of new BLE adress detected on the last scan
  bluetoothData.scan_del = 0;  // number of BLE adress deleted on the last scan
  new_BLE_cnt = 0;             // reset BLE address payload
}


void enableBluetooth() {
  Serial.println("Setup BLE");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();  //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
  pBLEScan->setInterval(200);
  pBLEScan->setWindow(200);  // less or equal setInterval value
  bluetoothActivation = 1;
}

void disableBluetooth() {
  Serial.println("Disabling BLE");
  BLEDevice::deinit(true);
  bluetoothActivation = 0;
}


deviceDetected* find(BLEAddress ble_id, std::vector<deviceDetected>& list) {
  for (auto& device : list) {
    if (device.data == ble_id)
      return &device;
  }
  return nullptr;
}
// Function check list
// Compare the new BLE scan with detected BLE lists
// Append BLE list if new device detected
void checklist(BLEScanResults foundDevices) {
  bluetoothData.scan_num = foundDevices.getCount();
  deviceDetected* ptr;
  for (int i = 0; i < bluetoothData.scan_num; i++) {
    int rssi_now = foundDevices.getDevice(i).getRSSI();
    BLEAddress ble_id = foundDevices.getDevice(i).getAddress();

    // if RSSI too weak, we don't consider it
    if (rssi_now > rssi_threshold) {

      if ((ptr = find(ble_id, whiteList)) != nullptr) {
        if (ptr->cntWL < id_max_count) ptr->cntWL += inc_margin_WL;  // increase the counter

      } else if ((ptr = find(ble_id, list)) != nullptr) {
        //Serial.printf("Device exists: %x \n", list[j].data);
        ptr->cnt++;
        ptr->undet = 0;
        ptr->rssi = rssi_now;
      } else {
        // The device do not exist in the list or the white list, enter in the list
        deviceDetected newDevice = deviceDetected(foundDevices.getDevice(i).getRSSI(), foundDevices.getDevice(i).getAddress());
        list.push_back(newDevice);
        new_add_payload(&list.back());  // récup newDevice
        bluetoothData.scan_new++;
        //Serial.println("Ajout d'un device");
      }
    }
  }
}
