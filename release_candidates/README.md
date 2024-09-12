# Code for Smart Building application

This repository contains the codes used for a Smart Building sensor. You can program the RAK3172 and the ESP32 which are on the device.

## ESP32 code instructions

You can find below the different parameters in the code of the ESP32, with the values I used for my deployment. Please chose the ID of your device in Lorawan_Credential.h, and set the parameters you want.

### Numeric parameters

- **PERIOD_IN_CHARGE**: `60` seconds  
  Period between each message sent when powered by USB.

- **PERIOD_ON_BATTERY**: `600` seconds  
  Period between each message sent when powered by battery.

- **BATTERY_LIMIT**: `3.5` volts  
  Minimum battery voltage before entering extreme energy saving mode.

- **EMERGENCY_SLEEP**: `3600` seconds  
  Duration of emergency sleep before rechecking battery voltage. For longer durations, ensure it works as expected or consider using two shorter sleep cycles. The ESP32 does not accept very high values of sleep durations (if so, it wakes up without waiting)

### Feature Toggles

You can enable or disable specific functionalities by commenting or uncommenting the following parameters:

- **CHECK_RAK**: `1`  
  Enables security check for RAK3172 data transmission. Resets if no data is sent.

- **BLUETOOTH**: `1`  
  Activates Bluetooth detection. If disabled, only environmental measurements are sent.

- **LORA**: `1`  
  Enables LoRa packet transmission to the network.

- **BLUETOOTH_SWITCH**: `1`  
  Controls Bluetooth deactivation during energy saving mode.
  
- The parameter **SLEEP** may cause issues with Serial Port communication and measurement sending. It activates light sleep mode between LoRa packet transmissions. It was not activated for my deployment.

### Parameters for New Card Version

If you are using an old version of the card, please deactivate the following parameters:

- **BATTERY_PRESENCE**: `1`  
  Activate if a battery is present to optimize energy savings.

- **LED_INDICATOR**: `1`  
  Enables LED indicators for debugging. Use the `blinkLeds` function for additional debug signals.
 
### BLE Parameters

Configure the following parameters for BLE device counting and management:

- **`nullscan`**: `5`  
  Number of undetected BLE scans before removing a device from the list.

- **`id_max_count`**: `15`  
  Number of continuous detections required to add a device to the whitelist.

- **`margin_WL`**: `50`  
  Margin added to the whitelist counter when a device is added to the whitelist.

- **`inc_margin_WL`**: `5`  
  Margin added to the whitelist counter when a device already in the whitelist is detected again.

- **`rssi_threshold`**: `-100` dBm  
  RSSI threshold to consider a BLE detection valid. Detections with an RSSI below this threshold are ignored.

## Possible improvements

- The most important : Temperature and humidity sensors calibrations. Note : there are differences between SCD4X and BME680 measurements.
- Optimizing energy saving mode. For example : remove Bluetooth detection while on battery 
- Process sound for the new card version. The mean done here gives no information, maybe calculate the sound in dBs.
- Optimize environment datas sending for the new code of the RAK3172

## Possible problems

The RAK3172 is sometimes hard to flash. If it is impossible, please try:
- resetting the RAK3172
- unplug and plug the RAK3172
- erase all flash memory and reinstall the firmware with STM32 Cube Programmer

  
## Date & Author

- **Date**: 09/12/2024
- **Author**: Augustin De La Bourdonnaye
- **Email**: [augustindelabourdonnaye@yahoo.fr](mailto:augustindelabourdonnaye@yahoo.fr)

