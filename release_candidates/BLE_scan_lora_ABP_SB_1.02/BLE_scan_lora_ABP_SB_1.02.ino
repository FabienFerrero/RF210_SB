/*
   RRRR  FFFF  22   11   000
   R   R F    2  2 111  0  00
   RRRR  FFF    2   11  0 0 0
   R R   F     2    11  00  0
   R  RR F    2222 11l1  000


   @file BLE_scan_lora_ABP_SB.ino
   @author FabienFerrero

   @brief This sketch scan BLE adress, classify and report it through LoRaWan protocol. In controls energy consumption and messages sending

   @version 1.0.4
   @date 2024-09-11

   @board : RF210 using https://github.com/FabienFerrero/SUniCA/blob/main/Examples/atcommand.md

   @copyright Copyright (c) 2023
*/

#include "lorawan_credential.h"
#include "bluetooth_list_management.h"
#include <Watchdog.h>

                  /* Pins of the ESP32, do not change it */
#define RAK3172_RESET_PIN 10
#define LED_R 8
#define LED_G 9
#define LED_B 2

                  /* Parameters that you can change */

#define PERIOD_IN_CHARGE 60    // Period between each message sending  if it is alimented by the USB port (seconds) 
#define PERIOD_ON_BATTERY 600  // Period between each message sending if it is alimented by the battery (seconds)

#define BATTERY_LIMIT 3.5   // lowest limit of battery before an extreme energy saving mode (volts)
#define EMERGENCY_SLEEP 3600  // 3600 seconds = 1 hour:  Duration of the emergency sleep before a new check 

          /* Parameters that can deactivate some parts of the code : comment it to deactivate */

#define CHECK_RAK 1  // Security to check if the RAK3172 sends environmental data or not, and resets if not
#define BLUETOOTH 1  // Define if bluetooth detection is activated
#define LORA 1       // Define if LoRa packet are sent to network
//#define SLEEP 1      // Define if the ESP32 uses light sleep mode between lora packets sending or not. Does not work for me, but forums talk about it...
#define BLUETOOTH_SWITCH 1 // Define if the Bluetooth is deactivated during the energy saving mode

// These ones will only work for the new version of the card, please deactivate them if you are using the old version
#define BATTERY_PRESENCE 1  // Activate it if there is a battery : if so, the card will save as much energy as possible
#define LED_INDICATOR 1 // Activate it you want to debug using leds. To add some debug signals, use the function blinkLeds

            /* Debuging colors code */

// Blue : blinks 2 seconds for the setup function

// Green : blinks one time for BLE detections sending
// Green : blinks two times for BLE data sending (id_num, ...)
// Green : blinks three times for environment data sending

// Red :  blinks one time if energy saving mode activation
// Red :  blinks two times if energy saving mode deactivation
// Red : blinks five times if entering deep sleep when energy saving mode activated
// Red : blinks ten times if entering deep sleep for emergency saving mode, if the battery is empty

// Yellow : blinks one time if the RAK3172 resets because of the following problem : no environment data sending confirmation








// Watchdog
Watchdog wdt(120);


// ESP32 C3 SERIAL1 (second UART)
HardwareSerial mySerial1(1);

// Define LoRaWan ABP credential
//RF210-SB0
// String devAddr = "260B3500";
// String nwkkey = "BD1C68CF9ED71883A1BB2E53CCEA07A6";
// String appskey = "525DDB7FDDF6BCFB07791A3C330640D5";

// Define UART Pin for RA3172
static const int rxPin = 20;
static const int txPin = 21;

// Battery parameters to control the activation of Bluetooth
String previousPowerStatus = "1";
String new_powerStatus = "1";
float batteryLevel;
bool isDeepSleep = 0;

#ifdef CHECK_RAK
// Check RAK3172 working
bool flag_environmentDataSent = false;
String current_data = "";
#endif

// Role : blinks numBlinks times the led(s) we want
void blinkLed(int ledPin1, int numBlinks = 1, int ledPin2 = -1) {
#ifdef LED_INDICATOR
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(ledPin1, LOW);  // Allumer la première LED
    if (ledPin2 != -1) {
      digitalWrite(ledPin2, LOW);  // Allumer la deuxième LED si spécifiée
    }
    delay(500);                   // Attendre 500 ms
    digitalWrite(ledPin1, HIGH);  // Éteindre la première LED
    if (ledPin2 != -1) {
      digitalWrite(ledPin2, HIGH);  // Éteindre la deuxième LED si spécifiée
    }
    delay(500);  // Attendre 500 ms
  }
#endif
}


void setup() {
#ifdef LED_INDICATOR
  pinMode(LED_R, OUTPUT);  // LED Red
  pinMode(LED_G, OUTPUT);  // LED Green
  pinMode(LED_B, OUTPUT);  // LED Blue

  digitalWrite(LED_R, HIGH);  // turn the LED off (HIGH is the voltage level)
  digitalWrite(LED_G, HIGH);  // turn the LED off (HIGH is the voltage level)
  digitalWrite(LED_B, HIGH);  // turn the LED off (HIGH is the voltage level)
  delay(1000);
#endif
  /*
  blinkLed(LED_B,5);
    blinkLed(LED_R,5);
  blinkLed(LED_G,5);
*/
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);

  blinkLed(LED_B);

  Serial.begin(115200);

  // Starting the watchdog
  wdt.begin();


#ifdef LORA
  pinMode(RAK3172_RESET_PIN, OUTPUT);     //Rak enable
  digitalWrite(RAK3172_RESET_PIN, HIGH);  // Switch on RAK

  mySerial1.begin(115200, SERIAL_8N1, rxPin, txPin);
  while (!mySerial1) {
  }
  delay(3000);

  Serial.println("Setup at command");
  mySerial1.println("AT");  // Start AT command
  delay(300);
  mySerial1.println("ATE");
  delay(300);
  mySerial1.println("AT+NWM=1");  // Set LoRaWan
  delay(300);
  mySerial1.println("AT+NJM=0");  // Set ABP
  delay(200);
  mySerial1.println("AT+BAND=4");  // 4: EU868  9 :  AS923-2
  delay(200);
  mySerial1.println("AT+DR=5");  // Set SF7  -  DR5
  delay(200);
  mySerial1.printf("AT+DEVADDR=");
  mySerial1.println(devAddr);
  delay(200);
  mySerial1.printf("AT+NWKSKEY=");
  mySerial1.println(nwkkey);
  delay(200);
  mySerial1.printf("AT+APPSKEY=");
  mySerial1.println(appskey);
  delay(200);

  delay(3000);
  flush_serial_AT();

#endif
#ifdef BLUETOOTH
  enableBluetooth();
#endif
}




void EnterModeSleep() {
#ifdef SLEEP
  //Serial.flush();
  //if(bluetoothActivation==0){
  esp_sleep_enable_timer_wakeup(PERIOD_IN_CHARGE * 333000);  // 20 sec
  delay(100);
  esp_light_sleep_start();
  delay(2000);
  //mySerial1.begin(115200, SERIAL_8N1, rxPin, txPin);
  //while (!mySerial1) {}
  //Serial.begin(115200);
  //} else {
  //  delay(PERIOD_IN_CHARGE * 333);
  //}
#else
  delay(PERIOD_IN_CHARGE * 333);
#endif
}

// Manage the activation / deactivation of the Bluetooth according to the previousPowerStatus activation
void manageBluetooth() {

  // Get the power status : 0 if working on the battery, 1 if powered by the USB port
  mySerial1.println("AT");
  delay(300);
  mySerial1.println("ATC+POWER");
  delay(300);
  if (mySerial1.available()) {
    while (mySerial1.available()) {
      new_powerStatus = mySerial1.readStringUntil('\n');
      new_powerStatus.trim();
      if (new_powerStatus == "1" || new_powerStatus == "0") {
        Serial.print("Power is :");
        Serial.println(new_powerStatus);

        if (previousPowerStatus != new_powerStatus) {
          if (new_powerStatus == "1") {
            #ifdef BLUETOOTH_SWITCH
              enableBluetooth();
            #endif
            isDeepSleep = 0;  //
            // Color red :  blinks one time if energy saving mode deactivation
            blinkLed(LED_R);
          } else if (new_powerStatus == "0") {
            #ifdef BLUETOOTH_SWITCH
              disableBluetooth();
            #endif

            isDeepSleep = 1;
            // Color red :  blinks two times if energy saving mode activation
            blinkLed(LED_R, 2);
              batteryLevel = measure_bat();
              Serial.print("Battery is :");
              Serial.println(batteryLevel);
              if ((batteryLevel / 1000) < BATTERY_LIMIT) {
                // Color Red : blinks 10 times if supply voltage of the battery is too weak, entering in deep sleep
                blinkLed(LED_R, 10); 
                Serial.println("Tension d'alimentation incorrecte, passage en mode deep sleep pendant 1 heure");
                delay(1000);
                // One hour of sleep, ESP32 can not sleep one hour, we put two cycles of 30 minutes
                // First sleep cycle (30 minutes)
                esp_sleep_enable_timer_wakeup(EMERGENCY_SLEEP/2 * 1000000);  // Temps en microsecondes
                esp_deep_sleep_start();
                // Second sleep cycle (30 minutes)
                esp_sleep_enable_timer_wakeup(EMERGENCY_SLEEP/2 * 1000000);  // Temps en microsecondes
                esp_deep_sleep_start();
              } else {
                Serial.println("Tension d'alimentation correcte, pas de deep sleep");
              }
          
          }
          previousPowerStatus = new_powerStatus;  // Mettre à jour l'état du previousPowerStatus
          break;
        }
      }
    }
  }
  
}

  void checklist();
  void process_list();
  void loop() {


    /*_______________Scanning BLE and processing the lists_________________*/
#ifdef BLUETOOTH
#ifdef BATTERY_PRESENCE
    manageBluetooth();  // Control bluetoothActivation, otherwise it is always activated
     // if energy saving mode activated, deep sleep before next sending
    if (isDeepSleep) {
      blinkLed(LED_R, 5);
      esp_sleep_enable_timer_wakeup(PERIOD_ON_BATTERY * 1000000);  // 10 minutes
      delay(100);
      esp_deep_sleep_start();  // deep sleep : the ESP32 will be reset when waking up
      delay(2000);
    }
#endif
    if (bluetoothActivation == 1) {
      Serial.println("Start Scanning BLE");
      BLEScanResults foundDevices = *pBLEScan->start(scanTime, false);
      checklist(foundDevices);  // update the counters
      process_list();           // clean the list with undetected device
      foundDevices.dump();
      printDetectedDevices();
      printWhiteList();
      pBLEScan->clearResults();  // delete results fromBLEScan buffer to release memory
    }
#endif

    /*___________________Sending data_______________________________________*/

#ifdef CHECK_RAK
    flag_environmentDataSent = false;
    Serial.println("Flag of the environment data becomes false");
#endif

#if defined(BLUETOOTH) && defined(LORA)
    if (bluetoothActivation == 1) {
      blinkLed(LED_G);
      sendLoraAddresses();
    }
#endif

    Serial.println("Sleep one");
    EnterModeSleep();
    Serial.println("Sleep one over");


#ifdef BLUETOOTH
    if (bluetoothActivation == 1) {
#ifdef LORA
      blinkLed(LED_G, 2);
      sendLoraBleData();
#endif
      counter();
    }
#endif

    Serial.println("Sleep two");
    EnterModeSleep();
    Serial.println("Sleep two over");

    flush_serial_AT();


#ifdef LORA
    Serial.println("Send environment data");
    mySerial1.println("AT");
    mySerial1.println("ATC+SEND");  // Send lora sensor
    delay(10000);
    blinkLed(LED_G, 3);
    if (flush_serial_AT()) {
      flag_environmentDataSent = true;  // Important : we check this specific message sending because it is the one causing problems
      Serial.println("Flag of the environment data becomes true");
    }
#endif

    wdt.handle();

    Serial.println("Sleep three");
    EnterModeSleep();
    Serial.println("Sleep three over");


    /*___________________Resets if no environmental data sending___________________________________*/

    if (flush_serial_AT()) {  // Very important : the confirmation could have arrived during the sleep so we need to check again before the security process
      flag_environmentDataSent = true;
      Serial.println("Flag of the environment data becomes true");
    }
#if defined(CHECK_RAK) && defined(LORA)
    // If no confirmation received for environement data, reset everything
    if (flag_environmentDataSent == false) {
      // Yellow : blinks one time if a reset caused by a RAK3172 problem : no environement data sending confirmation
      blinkLed(LED_R, 1, LED_G);

      // Reset RAK3172
      digitalWrite(RAK3172_RESET_PIN, LOW);
      delay(100);                             // Attendre un moment pour s'assurer que le RAK3172 est réinitialisé
      digitalWrite(RAK3172_RESET_PIN, HIGH);  // Remettre la broche à HIGH
      // Reset ESP32
      esp_restart();
    }
#endif

   
  }


  // Return bat level in mv
  float measure_bat() {

    flush_serial_AT();  // flush AT Serial reading buffer

    mySerial1.println("ATC+BAT=?");  // Request bat value
    String battery;
    delay(300);

     String result = "";
    while (mySerial1.available()) {
        char c = mySerial1.read();  // Lire un caractère
        result += c;                // Ajouter le caractère à la chaîne

        if (result.endsWith("Battery is:")) {
            // Lire la ligne suivante qui contient le niveau de la batterie
            String temp = mySerial1.readStringUntil('\n');
            temp = mySerial1.readStringUntil('\n');
            temp.trim();
            if (temp.length() > 0) {
                battery = temp;
            }
            break;
        }
    }

    // Débogage pour voir ce qui est reçu
    Serial.println("Réponse complète : " + result);

    Serial.print("Battery is : ");
    Serial.println(battery);

  /*
    if (mySerial1.available()) {
      Serial.println(battery = mySerial1.readStringUntil('='));
      Serial.println(battery = mySerial1.readStringUntil('='));
      Serial.println(battery = mySerial1.readStringUntil('\n'));
      //Serial.print("bat level:");
      //Serial.println(battery);
    }
*/
    return battery.toFloat();
  }


  bool flush_serial_AT() {
    /*
    if (mySerial1.available()) {  // If anything comes in Serial1 (pins 4 & 5)
      while (mySerial1.available())
        Serial.write(mySerial1.read());  // read it and send it out Serial (USB)
    }
    delay(100);
  */

#ifdef CHECK_RAK
    while (mySerial1.available()) {
      current_data = mySerial1.readStringUntil('\n');
      current_data.trim();
      if (current_data == "+EVT:TX_DONE") {
        Serial.print(current_data);
        Serial.println();
        return true;
      }
      Serial.print(current_data);
      Serial.println();
    }
#else
    if (mySerial1.available()) {  // If anything comes in Serial1 (pins 4 & 5)
      while (mySerial1.available())
        Serial.write(mySerial1.read());  // read it and send it out Serial (USB)
    }
#endif

    delay(100);
    return false;
  }


  void sendLoraBleData() {

    //int batlevel=measure_bat(); // measure bat level

    unsigned char mydata[7];
    mydata[0] = (char)bluetoothData.id_num;    // actual counter
    mydata[1] = (char)bluetoothData.scan_num;  // last scan counter
    mydata[2] = (char)bluetoothData.scan_new;  // new device from last scan
    mydata[3] = (char)bluetoothData.scan_del;  // deleted device from last scan
    mydata[4] = (char)bluetoothData.cnt_wl;    // white list counter
    // mydata[5] = batlevel >> 8;
    // mydata[6] = batlevel & 0xFF;


    char str[56] = "";
    array_to_string(mydata, 5, str);
    flush_serial_AT();
    mySerial1.println("AT");
    mySerial1.printf("AT+SEND=3:");
    mySerial1.println(str);
    delay(5000);
    flush_serial_AT();
    Serial.println("BLE data : AT set complete with downlink");
  }

  void sendLoraAddresses() {

    // unsigned char mydata[new_BLE_cnt];
    String temp;
    String str = "";
    String rssi;

    for (int i = 0; i < new_BLE_cnt; i++) {
      BLEAddress addr = newBLE_adr[i]->data;
      rssi = String(abs(newBLE_adr[i]->rssi));
      temp = addr.toString().c_str();
      temp.remove(14, 1);  //Remove 1 character starting from position 3
      temp.remove(11, 1);  //Remove 1 character starting from position 3
      temp.remove(8, 1);   //Remove 1 character starting from position 3
      temp.remove(5, 1);   //Remove 1 character starting from position 3
      temp.remove(2, 1);   //Remove 1 character starting from position 3

      str = str + temp + rssi;
    }
    if (str.length() > 0) {
      flush_serial_AT();
      mySerial1.println("AT");
      mySerial1.printf("AT+SEND=4:");
      mySerial1.println(str);
    } else {
      Serial.println("AT+SEND=4: no data to send ");
    }

    delay(5000);
    flush_serial_AT();

    Serial.println("New BLE Add : AT set complete with downlink");
  }
