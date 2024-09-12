/**
 * @file Watchdog.cpp
 * @author meirarc
 * @brief Watchdog library
 * @version 0.1
 * @date 2021-10-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Watchdog.h>
#include "esp32-hal-gpio.h"

#include <esp_task_wdt.h>
#include "esp32-hal-gpio.h"

#define RAK3172_RESET_PIN 10


/**
 * @brief Construct a new Watchdog:: Watchdog object
 * 
 * @param alarmPeriod in seconds
 */
Watchdog::Watchdog(int alarmPeriod){
  _alarmPeriod = alarmPeriod;
}

/**
 * @brief begin(). Initiate the service
 * 
 */
void Watchdog::begin(){
  // Initialiser la broche de réinitialisation du RAK3172
  pinMode(RAK3172_RESET_PIN, OUTPUT);
  digitalWrite(RAK3172_RESET_PIN, HIGH); // Assurez-vous que la broche est HIGH par défaut
  esp_task_wdt_deinit();

  // Initialiser le watchdog
 esp_task_wdt_config_t config = {
    .timeout_ms = _alarmPeriod*1000,
    .trigger_panic = true
  };
  esp_task_wdt_init(&config);
  esp_task_wdt_add(NULL);
}

/**
 * @brief handle(). To trigger the watchdog on the loop()
 * 
 */
void Watchdog::handle(){
  // Réinitialiser le watchdog
  esp_task_wdt_reset();
  delay(1);
}

/**
 * @brief triggerReset(). Trigger a reset for both ESP32 and RAK3172
 * 
 */
void Watchdog::triggerReset(){
  // Mettre la broche de réinitialisation du RAK3172 à LOW pour le réinitialiser
  digitalWrite(RAK3172_RESET_PIN, LOW);
  delay(100); // Attendre un moment pour s'assurer que le RAK3172 est réinitialisé
  digitalWrite(RAK3172_RESET_PIN, HIGH); // Remettre la broche à HIGH
  // Réinitialiser l'ESP32
  esp_restart();
}

// Définir le gestionnaire de watchdog
void IRAM_ATTR watchdogHandler() {
  Watchdog::triggerReset();
}

