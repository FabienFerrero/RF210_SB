/*
   RRRR  FFFF  22   11   000
   R   R F    2  2 111  0  00
   RRRR  FFF    2   11  0 0 0
   R R   F     2    11  00  0
   R  RR F    2222 11l1  000


   @file BLE_scan_lora_ABP_SB.ino
   @author FabienFerrero

   @brief This sketch scan BLE adress, classify and report it through LoRaWan protocol

   @version 1.0.1
   @date 2024-05-23

   @board : RF210 using https://github.com/FabienFerrero/SUniCA/blob/main/Examples/atcommand.md

   @copyright Copyright (c) 2023
*/
#define LED 1
#define PERIOD 60       // Period of scanning
#define ID_array 1000   // Define current BLE ID array max size limit
#define WL_array 2000   // Define White list array max size of the array
#define LORA 1          // Define if LoRa packet are sent to network
#define SLEEP 1         // Define if LoRa packet are sent to network
#define SEND_BLE_ADD 1  // Define if LoRa packet are sent to network

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "lorawan_credential.h"
#include <cmath>
#include <Watchdog.h>

#if CONFIG_PM_ENABLE
esp_pm_config_esp32_t pm_config = {
  .max_freq_mhz = 80,
  .min_freq_mhz = 40,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
  .light_sleep_enable = true
#endif
};
ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

// Watchdog
Watchdog wdt(120);



// ESP32 C3 SERIAL1 (second UART)
HardwareSerial mySerial1(1);

// Define UART Pin for RA3172
static const int rxPin = 20;
static const int txPin = 21;

// Parameters for BLE counting
const int nullscan = 5;           // number of undetected BLE scan to remove a device from the list
const int id_max_count = 15;      //number of continuous detection to move in the white list
const int margin_WL = 50;         // margin added to the counter when a device is added to white list
const int inc_margin_WL = 5;      // margin added to the counter when a device is added to white list
const int rssi_threshold = -80;  // threshold to consider a BLE detection

int scanTime = 4;  //In seconds
BLEScan* pBLEScan;

// Define arrays to store BLE ID and counter informations
static BLEAddress* id = (BLEAddress*)malloc(ID_array * sizeof(BLEAddress));         // array of BLEAddress detected
static BLEAddress* whitelist = (BLEAddress*)malloc(WL_array * sizeof(BLEAddress));  // array of BLEAddress detected
static unsigned char myBLE_adr[100];                                                // BLE adress to append the next lora packet
static BLEAddress* newBLE_adr[100];                                                 // BLE adress to append the next lora packet 
//static BLEAddress* Ancient_newBLE_adr[100];
static int8_t ancient_new_BLE_cnt = 0; 
static BLEAddress* list = (BLEAddress*)malloc(ID_array * sizeof(BLEAddress));  
static BLEAddress* temp_list = (BLEAddress*)malloc(ID_array * sizeof(BLEAddress)); 
int temp_list_cnt=0;


static int8_t flagtest = 0;


static int8_t my_BLE_cnt = 0;                                                       // counter actual array index of my_BLE_adr
static int8_t new_BLE_cnt = 0;                                                      // counter actual array index of my_BLE_adr


static int8_t cnt[ID_array];    // counter of presence for the BLEAddress detected
static int8_t undet[ID_array];  // counter of presence for the BLEAddress detected
static int8_t cntWL[WL_array];  // counter of presence for the BLEAddress detected
static int rssi[ID_array];      // last RSSI for the BLEAddress detected

static int id_num = 0;    // number of BLE adress currently detected and not removed
static int scan_num = 0;  // number of BLE adress detected on the last scan
static int scan_new = 0;  // number of new BLE adress detected on the last scan
static int scan_del = 0;  // number of BLE address deleted on the last scan
static int cnt_wl = 0;    // number of BLE address in the white list

static float pourcentage_RAM = 0; // pourcentage of occupied RAM

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
  }
};

void setup() {


  // Starting the watchdog
  wdt.begin();
 
  Serial.begin(115200);
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(200);               // wait for a second
  digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW

  #ifdef LORA
    pinMode(10, OUTPUT);     //Rak enable
    digitalWrite(10, HIGH);  // Switch on RAK

    delay(4000);

    mySerial1.begin(115200, SERIAL_8N1, rxPin, txPin);

    Serial.println("Setup at command");
    mySerial1.println("ATE");       // Start AT command, renvoie le résultat de la commande qu'on a envoyé "ok" par exemple
    flush_serial_AT();              // nettoyage du tampon
    mySerial1.println("AT+NWM=1");  // Set LoRaWan mode
    delay(300);
    mySerial1.println("AT+NJM=0");  // Set ABP (Activation By Personalization) : clés de sessions définies par nous même, aucune négotiation entre l'antenne et le capteur
    delay(200);
    mySerial1.println("AT+BAND=4");  // 4: EU868  9 :  AS923-2 c'est la bande correspondant à l'europe
    delay(200);
    mySerial1.println("AT+DR=5");  // Set SF7  -  DR5 : data rate, 5 représente un facteur d'étalement fort et une transmission plus faible
    delay(200);
    mySerial1.printf("AT+DEVADDR=");  // définit l'adresse du capteur
    mySerial1.println(devAddr);       // avec cette valeur
    delay(200);
    mySerial1.printf("AT+NWKSKEY=");  // Network Session Key : définit la clé de sécurité entre le capteur et l'antenne
    mySerial1.println(nwkkey);
    delay(200);
    mySerial1.printf("AT+APPSKEY=");  // Application session key : définit la clé de sécurité entre l'antenne et le serveur
    mySerial1.println(appskey);
    delay(200);


    while (mySerial1.available()) {
      Serial.write(mySerial1.read());  // read it and send it out Serial (USB)
    }

  #endif

    delay(2000);

    Serial.println("Setup BLE");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();  //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
    pBLEScan->setInterval(200);
    pBLEScan->setWindow(200);  // less or equal setInterval value
}

void loop() {
/*
  Serial.println("Start Scanning BLE");
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);

  checklist(foundDevices, id, cnt,undet, whitelist);  // update the counters

  process_list(id, cnt, undet, whitelist);  // clean the list with undetected device

  foundDevices.dump();

  Serial.printf(" Short List of detected device \n");
  int k = id_num;
  if (id_num > 10) {
    k = 10;
  } else {
    k = id_num;
  }



  //  for (int i=0;i<id_num;i++){
  for (int i = 0; i < k; i++) {
    //Serial.printf(" %x : ", *(id + i).toString().c_str());
    BLEAddress addr = *(id + i);
    Serial.print(addr.toString().c_str());
    Serial.print(" ");
    Serial.print(cnt[i]);
    Serial.print(" : ");
    Serial.print(undet[i]);
    Serial.print(" RSSI: ");
    Serial.println(rssi[i]);
    delay(1);
  }

  int m = cnt_wl;
  if (cnt_wl > 10) {
    m = 10;
  } else {
    m = cnt_wl;
  }


  Serial.printf(" List of White listed device \n");
  for (int i = 0; i < m; i++) {
    Serial.printf(" %x : ", *(whitelist + i));
    Serial.println(cntWL[i]);
    delay(1);
  }

  pBLEScan->clearResults();  // delete results fromBLEScan buffer to release memory

  Serial.print("Total counter: ");
  Serial.print(id_num);
  Serial.print(" Devices scan: ");
  Serial.print(scan_num);
  Serial.print(" Devices new: ");
  Serial.print(scan_new);
  Serial.print(" Devices del: ");
  Serial.print(scan_del);
  Serial.print(" Devices wl: ");
  Serial.println(cnt_wl);
  Serial.println("");
  delay(1);



  #ifdef SEND_BLE_ADD
    sendLoraAdd();
    digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(50);                // wait for a second
    digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
    delay(25);


  #endif


  #ifdef SLEEP
    esp_sleep_enable_timer_wakeup(PERIOD * 333000);  // 10 sec
    delay(100);
    esp_light_sleep_start();
  #else
    delay(PERIOD * 333);
  #endif



    //  Serial.println("Move to stop");
    //   // Sleep during period
    //   delay(20*1000);

  #ifdef LORA
    sendLora();
    digitalWrite(4, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);              // wait for a second
    digitalWrite(4, LOW);   // turn the LED off by making the voltage LOW
    delay(25);
   #endif

    counter();
*/
  #ifdef SLEEP

    Serial.println("Move to sleep");

    esp_sleep_enable_timer_wakeup(PERIOD * 333000);  // 10 sec
    delay(100);
    esp_light_sleep_start();

    Serial.println("Send environment data");
    flush_serial_AT();

    mySerial1.println("AT");
    mySerial1.println("ATC+SENSOR");  // Send lora sensor
    wdt.handle(); // Reset the watchdog
    delay(10000);

     if (mySerial1.available() > 0) {
              delay(100);
              String receivedString = mySerial1.readStringUntil('\n');  
              receivedString = mySerial1.readStringUntil('\n');              
              receivedString = mySerial1.readStringUntil('\n');              
              receivedString = mySerial1.readStringUntil('\n');   
              mySerial1.println("AT");
              mySerial1.println("AT+SEND=5:" + receivedString);
            }


    esp_sleep_enable_timer_wakeup(PERIOD * 333000);  // 10 sec
    delay(100);
    esp_light_sleep_start();

  #else

    Serial.println("Move to stop");
    // Sleep during period
    delay(PERIOD * 333);

    mySerial1.println("AT");
    mySerial1.println("ATC+SENSOR");  // Send lora sensor
    Serial.println("Send LoRa");
    flush_serial_AT();  // flush AT Serial reading buffer
    delay(2000);
    delay(PERIOD * 333);

  #endif
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

// Function check list
// Compare the new BLE scan with detected BLE list
// Increment counter and append BLE list if new device detected
void checklist(BLEScanResults foundDevices, BLEAddress* id, int8_t* cnt, int8_t* undet, BLEAddress* whitelist) {
  scan_num = foundDevices.getCount();
  for (int i = 0; i < scan_num; i++) {
    boolean flag = false;
    int rssi_now = foundDevices.getDevice(i).getRSSI();
    BLEAddress ble_id = foundDevices.getDevice(i).getAddress();
    //if(sizeof(ble_id) != 6){
    //  Serial.printf("Device with %d byte: %x \n",sizeof(ble_id), ble_id);
    //}

    if (checkWL(ble_id, whitelist) || rssi_now < rssi_threshold) {  // if device exists in white list
      flag = true;
      // Serial.printf("Device in WL: %x \n", *(id + id_num));
    } else {

      for (int j = 0; j < id_num; j++) {  // check if the device already exists
        if (*(id + j) == ble_id) {
          //    Serial.printf("Device exist: %x \n", *(id + id_num));
          flag = true;          // The device exist
          cnt[j] = cnt[j] + 1;  // increment the counter
          undet[j] = 0;         //reset undetect counter
          rssi[j] = rssi_now;
          if (cnt[j] > id_max_count) {
            undet[j] = nullscan + 1;  //set to undetect to remove from list on the next check pass
          }
        }

      }  //end for

      if (flag == false) {  // The device do not exist, enter in the list
        *(id + id_num) = foundDevices.getDevice(i).getAddress();
        cnt[id_num] = 1;  // set the counter
        rssi[id_num] = foundDevices.getDevice(i).getRSSI();
        undet[id_num] = 0;  // set undetect counter to 0
        //   Serial.printf("New Device detected: %x RSSI:", *(id + id_num));
        //   Serial.println(rssi[id_num]);
        scan_new++;
       id_num++;  //iterate the number of device detected
       //new_add_payload((id + id_num)); // to check, why after the id_num++ ?
      }

    
    for(int i = 0; i<ancient_new_BLE_cnt;i++){
      if(ble_id==*(list+i)){
        flagtest=1;
        *(temp_list+temp_list_cnt)=ble_id;
        temp_list_cnt++;
        break;
      }
    }
    if(flagtest==0){
      *(list+new_BLE_cnt)=ble_id;
      new_add_payload(list+new_BLE_cnt);
    }
    flagtest = 0;

    }

  }  // end for
  ancient_new_BLE_cnt=new_BLE_cnt;

  // adding the new BLE addresses to the list
  for(int i=0;i<new_BLE_cnt;i++){
    *(list+i)=(*newBLE_adr[i]);
  }

  // adding the addresses that we detect and were in the list before to the list
  for(int i =0;i<temp_list_cnt;i++){
    *(list+i+new_BLE_cnt)=*(temp_list+i);
  }
  temp_list_cnt=0;

}

// Function counter
// Update counter and BLE address list

void counter() {
  for (int i = 0; i < id_num; i++) {
    undet[i]++;  // increment undet counter before new scan
  }
  for (int i = 0; i < cnt_wl; i++) {
    cntWL[i] = cntWL[i] - 1;  // decrease white list counter before new scan
  }
  // Reset state counters
  scan_num = 0;     // number of BLE adress detected on the last scan
  scan_new = 0;     // number of new BLE adress detected on the last scan
  scan_del = 0;     // number of BLE adress deleted on the last scan
  new_BLE_cnt = 0;  // reset BLE address payload
}

// Function process_list
// Update counter and BLE adress list

void process_list(BLEAddress* id, int8_t* cnt, int8_t* undet, BLEAddress* whitelist) {
  int j = 0;
  int i = 0;
    while (j < id_num) {          // process ID list
      *(id + j) = *(id + j + i);  // copy next device ID in the list to current block in the array
      cnt[j] = cnt[j + i];        // copy next device counter in the list to current unit in the array
      undet[j] = undet[j + i];    // copy next device undetect counter in the list to current block in the array

    if (undet[j] > nullscan) {  // if the device has not been detected for a nullscan time
  #ifdef APPEND_BLE_ADD
      append_payload(*(id + j), cnt[j]);
  #endif
      appendWL(j, id, whitelist);  // move device in white list
                                   // Serial.printf("Remove Device : %x \n", *(id + j));
      id_num = id_num - 1;         // decrease array length by 1 unit
      i++;
      scan_del++;  // increase delete counter

    } else {  // move to next unit
      j++;
    }
  }

  // process white list, if a device not scan for a long time, remove from the white list

  j = 0;
  i = 0;
  while (j < cnt_wl) {                        // process White list, if a device not scan for a long time, remove from the white list
    *(whitelist + j) = *(whitelist + j + i);  // copy next device ID in the list to current block in the array
    cntWL[j] = cntWL[j + i];                  // copy next device counter in the list to current unit in the array
    if (cntWL[j] == 0) {
      cnt_wl = cnt_wl - 1;
      i++;
      //  Serial.printf("Remove from white list : %x \n", *(whitelist+j));
    } else {
      j++;
    }
  }
}

// Function append_payload
// Update payload with BLE address and duration

void append_payload(BLEAddress BLE, int8_t cnt) {

  unsigned char(*data)[6];
  data = (BLE.getNative());


  myBLE_adr[my_BLE_cnt] = *data[0];      //BLE;
  myBLE_adr[my_BLE_cnt + 1] = *data[1];  //BLE;
  myBLE_adr[my_BLE_cnt + 2] = *data[2];  //BLE;
  myBLE_adr[my_BLE_cnt + 3] = *data[3];  //BLE;
  myBLE_adr[my_BLE_cnt + 4] = *data[2];  //BLE;
  myBLE_adr[my_BLE_cnt + 5] = *data[3];  //BLE;
  myBLE_adr[my_BLE_cnt + 6] = cnt;
  my_BLE_cnt = my_BLE_cnt + 7;
}

/////////////////////////////////////////////////////////

// Function newAdd_payload
// Update payload with BLE address

void new_add_payload(BLEAddress* BLE) {

  newBLE_adr[new_BLE_cnt] = BLE;
  new_BLE_cnt++;
}

/////////////////////////////////////////////////////////

void array_to_string(byte array[], unsigned int len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++) {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
  }
  buffer[len * 2] = '\0';
}

/////////////////////////////////////

void sendLora() {

  //int batlevel=measure_bat(); // measure bat level

  unsigned char mydata[7];
  mydata[0] = (char)id_num;    // actual counter
  mydata[1] = (char)scan_num;  // last scan counter
  mydata[2] = (char)scan_new;  // new device from last scan
  mydata[3] = (char)scan_del;  // deleted device from last scan
  mydata[4] = (char)cnt_wl;    // white list counter
  pourcentage_RAM=round(pourcentage_RAM);
  mydata[5] = (char)pourcentage_RAM; // occupied RAM
  // mydata[5] = batlevel >> 8;
  // mydata[6] = batlevel & 0xFF;


  char str[56] = "";
  array_to_string(mydata, 6, str);
  Serial.println(str);

  mySerial1.println("AT");
  mySerial1.println("AT+SEND=3:" + str);

  delay(5000);
  if (mySerial1.available()) {  // If anything comes in Serial1 (pins 4 & 5)
    while (mySerial1.available())
      Serial.write(mySerial1.read());  // read it and send it out Serial (USB)
  }
  delay(100);
  Serial.println("AT set complete with downlink");
}

/////////////////////////////////////

void sendLoraAdd() {
  // unsigned char mydata[new_BLE_cnt];
  String temp;
  String str = "";

  for (int i = 0; i < new_BLE_cnt; i++) {
    BLEAddress addr = *newBLE_adr[i];
    temp = addr.toString().c_str();
    temp.remove(14, 1);
    temp.remove(11, 1);
    temp.remove(8, 1);
    temp.remove(5, 1);
    temp.remove(2, 1); 

    str = str + temp;
  }
  
  if(str.length()>0){
    mySerial1.println("AT");
    mySerial1.println("AT+SEND=4:" + str);
  }


  delay(5000);
  if (mySerial1.available()) {
    // If anything comes in Serial1 (pins 4 & 5)
    while (mySerial1.available())
      Serial.write(mySerial1.read());  // read it and send it out Serial (USB)
  }
  delay(100);
  Serial.println("New BLE Add : AT set complete with downlink");
}

//////////////////////////////////////

void appendWL(int j, BLEAddress* id, BLEAddress* whitelist) {
  BLEAddress ble_id = *(id + j);
  if (checkWL(ble_id, whitelist) == false) {
    *(whitelist + cnt_wl) = ble_id;
    cntWL[cnt_wl] = margin_WL;  // define a high number to get some margin
    cnt_wl++;
    // Serial.printf("New device in White list : %x \n", *(id + j));
  }
}

//////////////////////////////

boolean checkWL(BLEAddress BLE, BLEAddress* whitelist) {
  int i = 0;
  boolean flag = false;
  // Pour tous les éléments de la white list
  while (i < cnt_wl) {
    if (*(whitelist + i) == BLE) {    // Si l'élément correspond à mon adresse
      if (cntWL[i] < id_max_count) {  // Si le nombre d'occurrences est inférieur au nombre maximal d'occurrences
        cntWL[i] = cntWL[i] + inc_margin_WL;
      }  // incrémenter le nombre d'occurences
      // Serial.printf("Device already in White list : %x \n", *(whitelist + i));
      flag = true;
    }
    i++;
  }
  return flag;
}

// Return bat level in mv
int measure_bat() {
  flush_serial_AT();  // flush AT Serial reading buffer

  mySerial1.println("ATC+BAT=?");  // Request bat value
  String battery;
  delay(300);

  if (mySerial1.available()) {
    battery = mySerial1.readStringUntil('=');
    battery = mySerial1.readStringUntil('=');
    battery = mySerial1.readStringUntil('\n');
    //Serial.print("bat level:");
    //Serial.println(battery);
  }

  return battery.toInt();
}

void flush_serial_AT() {

  if (mySerial1.available()) {  // If anything comes in Serial1 (pins 4 & 5)
    while (mySerial1.available())
      Serial.write(mySerial1.read());  // read it and send it out Serial (USB)
  }
  delay(100);
}
