/**
 SSS  M   M  AA  RRRR  TTTTTT     BBBB  U   U III L    DDD  III N   N  GGG 
S     MM MM A  A R   R   TT       B   B U   U  I  L    D  D  I  NN  N G    
 SSS  M M M AAAA RRRR    TT       BBBB  U   U  I  L    D  D  I  N N N G  GG
    S M   M A  A R R     TT       B   B U   U  I  L    D  D  I  N  NN G   G
SSSS  M   M A  A R  RR   TT       BBBB   UUU  III LLLL DDD  III N   N  GGG 

   @file ATC_Command_SB_1_2.ino
   @author Duynghk

   @brief This sketch add custom ATC command to RFThings RF210SB board ver 3.0. These commands help controlling on-board sensors,
   For detail description, please visit: https://github.com/RFThings/SMART-BUILDING.git

   @version 0.1.2
   @date 2024-06-27

   @copyright Copyright (c) 2024

*/

#define SND_INTERVAL 600 //ms
#define SAMPLES 30
#define INT1_PIN PB4 // Interrupt for accelero
//#define PIR_LED 1

//BME680 variable
float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score
int   humidity_score, gas_score;
float gas_reference = 2500;
float hum_reference = 40;
int   getgasreference_count = 0;
int   gas_lower_limit = 10000;  // Bad air quality limit
int   gas_upper_limit = 300000; // Good air quality limit
//bool valid;
//bool ltr_status;
unsigned int visible, infrared2;
//double lux;

#define DATA_INTERVAL 5000 //ms

#include <LTR303.h>
#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include "Zanshin_BME680.h"
#include <Wire.h>
#include "kxtj3-1057.h"
#include "Adafruit_LTR329_LTR303.h"
#include "Adafruit_SGP30.h"
#include "ATC_Command_SB_1_2.h"
  


#define DATA_INTERVAL 500 // ms


/*************************************
   LoRaWAN band setting:
     RAK_REGION_EU433
     RAK_REGION_CN470
     RAK_REGION_RU864
     RAK_REGION_IN865
     RAK_REGION_EU868
     RAK_REGION_US915
     RAK_REGION_AU915
     RAK_REGION_KR920
     RAK_REGION_AS923

 *************************************/
#define ABP_BAND     (9)
#define ABP_DEVADDR  {0x26, 0x0B, 0x24, 0xF4}
#define ABP_APPSKEY  {0x39, 0x23, 0x1A, 0xFE, 0x93, 0x20, 0x93, 0x99, 0xB4, 0x24, 0x9E, 0x0A, 0x95, 0xE9, 0xE3, 0x92}
#define ABP_NWKSKEY  {0x9E, 0xAA, 0xED, 0xAD, 0x01, 0x9A, 0x29, 0x05, 0x94, 0x9F, 0x27, 0x5A, 0x06, 0xE7, 0x2C, 0xE0}

KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F
volatile bool flag;
Adafruit_LTR303 ltr = Adafruit_LTR303();
LTR303 light;

SensirionI2CScd4x scd4x;
BME680_Class BME680;  ///< Create an instance of the BME680 class
Adafruit_SGP30 sgp;


int it=0; // iteration for PIR
bool pir=0;
int PIR_sum=0;
int PIR_counter=0;


uint16_t error;
unsigned char gain = 0;   // Gain setting, values = 0-7
unsigned char integrationTime = 0; // Integration ("shutter") time in milliseconds
uint16_t voltage_adc;
uint16_t voltage;
//sensors_event_t hum, temp;
bool valid;
bool ltr_status;
uint16_t visible_plus_ir, infrared;
double lux;
unsigned long currentMillis = 0, getSensorDataPrevMillis = 0;
//bool sht_status;

uint16_t co2 = 0;
float temperature = 0.0f;
float humidity = 0.0f;
uint8_t Sound_array[12];

float   sampleRate = 6.25;  // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange = 2;     // Accelerometer range = 2, 4, 8, 16g
bool KXTJ3_status;

static char     buf[16];                        // sprintf text buffer
static float    alt;                            // Temporary variable
static uint16_t loopCounter = 0;                // Display iterations
int counter = 0;

int ATC_Ver(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0) {
      Serial.println("0.1.2");
      // Serial1.println("0.1.2");
    }

  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

void recvCallback(SERVICE_LORA_RECEIVE_T * data)
{
    if (data->BufferSize > 0) {
        Serial.println("Something received!");
        for (int i = 0; i < data->BufferSize; i++) {
            Serial.printf("%x", data->Buffer[i]);
        }
        Serial.print("\r\n");
    }
}

void sendCallback(int32_t status)
{
    if (status == RAK_LORAMAC_STATUS_OK) {
        Serial.println("Successfully sent");
    } else {
        Serial.println("Sending failed");
    }
}

void setup_LoRaWan() 
{
  if(api.lorawan.nwm.get() != 1)
  {
    Serial.printf("Set Node device work mode %s\r\n",
    api.lorawan.nwm.set() ? "Success" : "Fail");
    api.system.reboot();
  }

    // ABP Device Address MSB first
  uint8_t node_dev_addr[4] = ABP_DEVADDR;
    // ABP Application Session Key
  uint8_t node_app_skey[16] = ABP_APPSKEY;
    // ABP Network Session Key
  uint8_t node_nwk_skey[16] = ABP_NWKSKEY;
  
  if (!api.lorawan.njm.set(RAK_LORA_ABP))	// Set the network join mode to ABP
  {
    Serial.printf("LoRaWan ABP - set network join mode is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.daddr.set(node_dev_addr, 4)) {
    Serial.printf("LoRaWan ABP - set device addr is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.appskey.set(node_app_skey, 16)) {
    Serial.printf("LoRaWan ABP - set application session key is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.nwkskey.set(node_nwk_skey, 16)) {
    Serial.printf("LoRaWan ABP - set network session key is incorrect! \r\n");
    return;
    }
  if (!api.lorawan.band.set(ABP_BAND)) {
    Serial.printf("LoRaWan ABP - set band is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_A)) {
    Serial.printf("LoRaWan ABP - set device class is incorrect! \r\n");
    return;
  }
  
  if (!api.lorawan.adr.set(true)) {
    Serial.printf("LoRaWan ABP - set adaptive data rate is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.rety.set(1)) {
    Serial.printf("LoRaWan ABP - set retry times is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.cfm.set(1)) {
    Serial.printf("LoRaWan ABP - set confirm mode is incorrect! \r\n");
    return;
  }
  
    /** Check LoRaWan Status*/
  Serial.printf("Duty cycle is %s\r\n", api.lorawan.dcs.get()? "ON" : "OFF");	// Check Duty Cycle status
  Serial.printf("Packet is %s\r\n", api.lorawan.cfm.get()? "CONFIRMED" : "UNCONFIRMED");	// Check Confirm status
  uint8_t assigned_dev_addr[4] = { 0 };
  api.lorawan.daddr.get(assigned_dev_addr, 4);
  Serial.printf("Device Address is %02X%02X%02X%02X\r\n", assigned_dev_addr[0], assigned_dev_addr[1], assigned_dev_addr[2], assigned_dev_addr[3]);	// Check Device Address
  Serial.println("");
  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerSendCallback(sendCallback);
}

void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

// // void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
//   Serial.print("Serial: 0x");
//   printUint16Hex(serial0);
//   // printUint16Hex(serial1);
//   printUint16Hex(serial2);
//   Serial.println();
// }

void SCD4x_init()
{
  Wire.begin();

  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);


  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    // Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    // Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    //Serial.println(errorMessage);
  } 

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.println("SCD4x error!");
    // errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
  }
  else 
  {
    Serial.println("Successfully init SCD4x");
  }

  // Serial.println("Waiting for first measurement... (5 sec)");
}

SCD4xData read_SCD4x() {
  SCD4xData sensorData;
  sensorData.valid = false;

  char errorMessage[256];

  bool isDataReady = false;
  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
    errorToString(error, errorMessage, 256);
    return sensorData;
  }
  if (!isDataReady) {
    return sensorData;
  }
  scd4x.startPeriodicMeasurement();
  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    errorToString(error, errorMessage, 256);
  } else if (co2 != 0) {
    sensorData.co2 = co2;
    sensorData.temperature = temperature;
    sensorData.humidity = humidity;
    sensorData.valid = true;
  }
  return sensorData;
}

int SCD4x_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) 
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    SCD4xData data;
    data = read_SCD4x();
    Serial.print("\nCO2: ");
    Serial.println(data.co2);
    // Serial1.println(data.co2);
    Serial.print("Temperature: ");
    Serial.println(data.temperature);
    // Serial1.println(data.temperature);
    Serial.print("Humidity: ");
    Serial.println(data.humidity);
    // Serial1.println(data.humidity);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}


int SCD_CO2(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    SCD4xData data;
    data = read_SCD4x();
    Serial.println(data.co2);
    // Serial1.println(data.co2);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

int SCD_temp(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    SCD4xData data;
    data = read_SCD4x();
    Serial.println(data.temperature);
    // Serial1.println(data.temperature);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

int SCD_humid(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    SCD4xData data;
    data = read_SCD4x();
    Serial.println(data.humidity);
    // Serial1.println(data.humidity);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
    @brief     This converts a pressure measurement into a height in meters
    @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
    @param[in] press    Pressure reading from BME680
    @param[in] seaLevel Sea-Level pressure in millibars
    @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
    44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()

void BME680_init() 
{
  if(!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.println("BME680 error!");
  }  // of loop until device is located
  else 
  {
    BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
    BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
    BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
    BME680.setIIRFilter(IIR4);  // Use enumerated type values
    BME680.setGas(320, 50);  // 320�c for 150 milliseconds
    Serial.println("Successfully init BME680");
  }
}

BME680Data read_BME680() {
  BME680Data sensorData;
  sensorData.valid = false;

  static int32_t  temp, humidity, pressure, gas;  // BME readings
  BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings

  // Lưu dữ liệu vào struct
  sensorData.temperature = temp;
  sensorData.humidity = humidity;
  sensorData.pressure = pressure / 100.0; // hPa
  sensorData.gas_resistance = gas / 1000.0; // kOhms
  sensorData.valid = true;

  return sensorData;
}

int BME680_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) {

  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0))) {   
    BME680Data data;
    data = read_BME680(); 
    Serial.print("\nTemperature:");
    Serial.println(data.temperature);
    // Serial1.println(data.temperature);
    Serial.print("Humidity:");
    Serial.println(data.humidity);
    // Serial1.println(data.humidity);
    Serial.print("Pressure:");
    Serial.println(data.pressure);
    // Serial1.println(data.pressure);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}
  
void SGP30_init() 
{

  if (!sgp.begin()){
    Serial.println("SGP30 error!");
  }
  else 
  {
    Serial.println("Successfully init SGP30");
  }
}

SGP30Data read_SGP30() {
  SGP30Data sensorData;

  if (! sgp.IAQmeasure()) {
    Serial.println("SGP30 Measurement failed");
    // Trả về giá trị mặc định nếu đo thất bại
    sensorData.TVOC = 0;
    sensorData.eCO2 = 0;
    return sensorData;
  }
  
  // Lưu dữ liệu vào struct
  sensorData.TVOC = sgp.TVOC;
  sensorData.eCO2 = sgp.eCO2;
  if (! sgp.IAQmeasureRaw()) {
    sensorData.rawH2 = 0;
    sensorData.rawEthanol = 0;
    return sensorData;
  }
  sensorData.rawH2 = sgp.rawH2;
  sensorData.rawEthanol = sgp.rawEthanol;
  
  delay(1000);

  counter++;
  if (counter == 30) {
    counter = 0;

    uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.println("SGP30 Failed to get baseline readings");
      return sensorData;
    }
  }

  return sensorData;
}

int SGP30_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) {

  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0))) { 
    SGP30Data data;
    data = read_SGP30();
    Serial.print("\nTVOC:");
    Serial.println(data.TVOC);
    // Serial1.println(data.TVOC);
    Serial.print("eCO2:");
    Serial.println(data.eCO2);
    // Serial1.println(data.eCO2);
    Serial.print("H2:");
    Serial.println(data.rawH2);
    // Serial1.println(data.rawH2);
    Serial.print("Ethanol:");
    Serial.println(data.rawEthanol);
    // Serial1.println(data.rawEthanol);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

void KXTJ3_init()
{
  if( myIMU.begin(sampleRate, accelRange) != 0 )
  {
    Serial.println("KXTJ3 error!");
  }
  else
  {
    Serial.println("Successfully init KXTJ3");
  }
  
  // Detection threshold, movement duration and polarity
  myIMU.intConf(123, 1, 10, HIGH);

  uint8_t readData = 0;

  // Get the ID:
  myIMU.readRegister(&readData, KXTJ3_WHO_AM_I);
}

KXTJ3Data read_KXTJ3() {
  KXTJ3Data sensorData;
  sensorData.valid = false;

  // Đọc giá trị gia tốc từ các trục X, Y, Z
  sensorData.x = myIMU.axisAccel(X);
  sensorData.y = myIMU.axisAccel(Y);
  sensorData.z = myIMU.axisAccel(Z);

  // Giả sử các giá trị được đọc hợp lệ nếu không phải là giá trị mặc định (thay đổi điều kiện nếu cần)
  if (sensorData.x != 0 || sensorData.y != 0 || sensorData.z != 0) {
    sensorData.valid = true;
  }

  return sensorData;
}

int KXTJ3_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) {
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0))) {
    KXTJ3Data data;
    data = read_KXTJ3();
    Serial.print("\nX coordinate:");
    Serial.println(data.x);
    // Serial1.println(data.x);
    Serial.print("Y coordinate:");
    Serial.println(data.y);
    // Serial1.println(data.y);
    Serial.print("Z coordinate:");
    Serial.println(data.z);
    // Serial1.println(data.z);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

void LTR_init()
{
  if ( ! ltr.begin() ) {
    Serial.println("LTR error!");
  }
  else 
  {
    // Set gain of 1 (see advanced demo for all options!
    ltr.setGain(LTR3XX_GAIN_1);
    // Set integration time of 50ms (see advanced demo for all options!
    ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
    // Set measurement rate of 50ms (see advanced demo for all options!
    ltr.setMeasurementRate(LTR3XX_MEASRATE_50);
    Serial.println("Successfully init LTR303");
  }
}

LTRData read_LTR() {
  LTRData sensorData;
  sensorData.valid = ltr.readBothChannels(sensorData.visible_plus_ir, sensorData.infrared);
  return sensorData;
}

int LTR_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) {
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0))) { 
    LTRData data;
    data = read_LTR();
    if (data.valid) {
      Serial.print("\nVisible IR:");
      Serial.println(data.visible_plus_ir);
      // Serial1.println(data.visible_plus_ir);
      Serial.print("Infrared:");
      Serial.println(data.infrared);
      // Serial1.println(data.infrared);
    } 
    else 
    {
      Serial.println("LTR invalid data");
      // Serial1.println("LTR invalid data");
    }
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

int read_Battery() 
{
  int voltage_adc = (uint16_t)analogRead(BATVOLT_PIN);
  voltage = (uint16_t)((ADC_AREF / 1.024) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)voltage_adc);
  return voltage;
}

int Battery_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    int voltage = read_Battery();
    Serial.print("Battery is:");
    Serial.println(voltage);
    // Serial1.println(voltage);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

bool check_power_status()
{
  int adc_value = analogRead(POWER_STATUS);
  if (adc_value < POWER_THRESHOLD) return false;
  return true;
}

int POWER_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    bool power_status = check_power_status(); 
    Serial.println(power_status ? 1:0);
    // Serial1.println(power_status ? 1:0);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

void read_sound(SoundData* sensorData) {
  sensorData->soundArray = new int[sensorData->numSamples];

  unsigned long startTime;
  unsigned long elapsedTime;
  unsigned long intervalMicros = 1000000 / sensorData->sampleRate; // Khoảng thời gian giữa các mẫu (micro giây)
  unsigned long mean;

  pinMode(LED, OUTPUT); // Thiết lập chân LED là output

  for (int i = 0; i < sensorData->numSamples; i++) {
    startTime = micros();
    int soundValue = analogRead(MIC);
    sensorData->soundArray[i] = soundValue; // Lưu giá trị vào mảng
    mean += soundValue;
    /*
    // Điều khiển LED dựa trên giá trị âm thanh
    if (soundValue > 380 && soundValue < 1000) {
      digitalWrite(LED, LOW); // Nếu âm thanh nằm trong ngưỡng, bật LED
    } else {
      digitalWrite(LED, HIGH); // Nếu âm thanh không nằm trong ngưỡng, tắt LED
    }
    */
    // Đợi đủ thời gian giữa các lần lấy mẫu
    do {
      elapsedTime = micros() - startTime;
    } while (elapsedTime < intervalMicros);
  }
  // Calcul de la moyenne
  sensorData->mean = mean/sensorData->numSamples;
  mean = 0;
  // Giả sử dữ liệu hợp lệ nếu có bất kỳ mẫu nào trong dải giá trị hợp lệ
  sensorData->valid = true;

  //digitalWrite(LED, HIGH); // Tắt LED khi kết thúc lấy mẫu
}

// Định nghĩa hàm xử lý lệnh AT
int Sound_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) {
  SoundData data;

  if (param->argc == 1) {
    char *token;
    token = strtok(param->argv[0], " ,"); // Phân tách chuỗi theo dấu cách và dấu phẩy

    // Lấy sampleRate từ token đầu tiên
    if (token != NULL) {
      data.sampleRate = strtoul(token, NULL, 10);
    }

    // Lấy sampleDuration từ token tiếp theo
    token = strtok(NULL, " ,");
    if (token != NULL) {
      data.sampleDuration = strtoul(token, NULL, 10);
    }

    // Kiểm tra tính hợp lệ của các tham số
    if (data.sampleRate <= 0 || data.sampleDuration <= 0) {
      return AT_PARAM_ERROR;
    }

    // Tạo đối tượng SoundData và gọi hàm read_sound
    read_sound(&data);
    /*
    if (data.valid) {
      Serial.println("Sound data: ");
      for (int i = 0; i < data.numSamples; i++) {
        Serial.print(data.soundArray[i]);
        Serial.println(" ");
      }
    } else {
      Serial.println("No significant sound detected.");
    }
    */
  } 
  else if (param->argc == 0 || (param->argc == 1 && strcmp(param->argv[0], "?") == 0)) {
    // Tạo đối tượng SoundData và gọi hàm read_sound với giá trị mặc định
    SoundData data;
    read_sound(&data);

    if (data.valid) {
      Serial.println("Sound data:");
      for (int i = 0; i < data.numSamples; i++) {
        Serial.print(data.soundArray[i]);
        Serial.println(" ");
      }
    } else {
      Serial.println("No significant sound detected.");
    }
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

uint8_t read_PIR(){
  uint8_t result = static_cast<uint8_t>((PIR_sum/PIR_counter)*100);
  PIR_counter = 0;
  PIR_sum=0;
  //uint16_t result = static_cast<uint16_t>(12);
  return result;
}

int PIR_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) {
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    int pir_value = read_PIR();
    Serial.println(pir_value);
    // Serial1.println(pir_value);
    return AT_OK;
  }
  return AT_PARAM_ERROR;
}

int get_fan_status() 
{
  return digitalRead(FAN);
}

void turn_on_fan() 
{
  digitalWrite(FAN, HIGH);
}

void turn_off_fan() 
{
  digitalWrite(FAN, LOW);
}

int FAN_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param) { 
  // Kiểm tra nếu không có đối số hoặc đối số là "?"
  if (param->argc == 0 || (param->argc == 1 && strcmp(param->argv[0], "?") == 0)) {
    if (param->argc == 1 && strcmp(param->argv[0], "?") == 0) {
      // Trả về trạng thái hiện tại của quạt (giả sử có hàm get_fan_status)
      int current_status = get_fan_status();
      // Serial1.print("Fan status: ");
      // Serial1.println(current_status);
      Serial.print("Fan status: ");
      Serial.println(current_status);
      return AT_OK;
    }
    return AT_PARAM_ERROR;
  }
  
  // Kiểm tra nếu có đối số khác "?"
  if (param->argc == 1) {
    for (int i = 0; i < strlen(param->argv[0]); i++) {
      if (!isdigit(*(param->argv[0] + i))) {
        return AT_PARAM_ERROR;
      }
    }
    int fan_status = strtoul(param->argv[0], NULL, 10);
    
    if (fan_status != 0 && fan_status != 1) {
      return AT_PARAM_ERROR;
    }
    if (fan_status == 1) {
      turn_on_fan();
    } else {
      turn_off_fan();
    }
    return AT_OK;
  }
  // Nếu số lượng đối số không phù hợp
  return AT_PARAM_ERROR;
}

void read_Sensors(SensorsData *sensorsData) 
{
  sensorsData->ltr = read_LTR();
  sensorsData->scd4x = read_SCD4x();
  sensorsData->bme680 = read_BME680();
  //sensorsData->sgp30 = read_SGP30();
  sensorsData->kxtj3 = read_KXTJ3();
  read_sound(&(sensorsData->sound));
  sensorsData->pir = read_PIR();
  sensorsData->battery_voltage = read_Battery();
  sensorsData->power_status = check_power_status();
}

void serializeSensorData(const SensorsData& sensorsData, unsigned char*& buffer, size_t& size) {
  // Calculate total size needed for the buffer
  size = sizeof(sensorsData.scd4x.co2) + sizeof(uint16_t) + sizeof(uint16_t) +          // scd4x : co2, temperature, humidity
          sizeof(uint16_t) + sizeof(uint16_t) + sizeof(int) + sizeof(uint16_t) +             // bme : temperature, humidity, pressure, gas_resistance
          sizeof(sensorsData.sgp30.eCO2) + sizeof(sensorsData.sgp30.TVOC) +   // sgp30 : eCO2 and TVOC
          sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t) +                           // kxtj : x , y, z
          sizeof(sensorsData.ltr.visible_plus_ir) +                           // ltr : visible plus ir 
          sizeof(sensorsData.sound.mean) +                                     // sound : mean
          //sizeof(sensorsData.sound.sampleRate) +
         //sizeof(sensorsData.sound.sampleDuration) + sizeof(sensorsData.sound.numSamples) +
         //sizeof(sensorsData.sound.valid) + sensorsData.sound.numSamples * sizeof(int) +
         sizeof(sensorsData.pir) + 
         sizeof(uint16_t) + // battery_voltage
         sizeof(sensorsData.power_status); 

  // Allocate memory for the buffer
  buffer = new unsigned char[size];
  unsigned char* ptr = buffer;

  // Formatted data 
  uint16_t formatted_uint16;
  int formatted_int;

  // Copy data to buffer
  std::memcpy(ptr, &sensorsData.scd4x.co2, sizeof(sensorsData.scd4x.co2));
  ptr += sizeof(sensorsData.scd4x.co2);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.scd4x.temperature*100);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.scd4x.humidity*100);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.bme680.temperature);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.bme680.humidity);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  formatted_int = static_cast<int>(sensorsData.bme680.pressure*100);
  std::memcpy(ptr, &formatted_int, sizeof(formatted_int));
  ptr += sizeof(formatted_int);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.bme680.gas_resistance*100);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.kxtj3.x*100);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.kxtj3.y*100);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.kxtj3.z*100);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  std::memcpy(ptr, &sensorsData.ltr.visible_plus_ir, sizeof(sensorsData.ltr.visible_plus_ir));
  ptr += sizeof(sensorsData.ltr.visible_plus_ir);

  std::memcpy(ptr, &sensorsData.sgp30.eCO2, sizeof(sensorsData.sgp30.eCO2));
  ptr += sizeof(sensorsData.sgp30.eCO2);

  std::memcpy(ptr, &sensorsData.sgp30.TVOC, sizeof(sensorsData.sgp30.TVOC));
  ptr += sizeof(sensorsData.sgp30.TVOC);

  std::memcpy(ptr, &sensorsData.sound.mean, sizeof(sensorsData.sound.mean));
  ptr += sizeof(sensorsData.sound.mean);
/*
  std::memcpy(ptr, &sensorsData.sound.sampleRate, sizeof(sensorsData.sound.sampleRate));
  ptr += sizeof(sensorsData.sound.sampleRate);

  std::memcpy(ptr, &sensorsData.sound.sampleDuration, sizeof(sensorsData.sound.sampleDuration));
  ptr += sizeof(sensorsData.sound.sampleDuration);

  std::memcpy(ptr, &sensorsData.sound.numSamples, sizeof(sensorsData.sound.numSamples));
  ptr += sizeof(sensorsData.sound.numSamples);

  std::memcpy(ptr, &sensorsData.sound.valid, sizeof(sensorsData.sound.valid));
  ptr += sizeof(sensorsData.sound.valid);

  std::memcpy(ptr, sensorsData.sound.soundArray, sensorsData.sound.numSamples * sizeof(int));
  ptr += sensorsData.sound.numSamples * sizeof(int);
*/
  std::memcpy(ptr, &sensorsData.pir, sizeof(sensorsData.pir));
  ptr += sizeof(sensorsData.pir);

  formatted_uint16 = static_cast<uint16_t>(sensorsData.battery_voltage);
  std::memcpy(ptr, &formatted_uint16, sizeof(formatted_uint16));
  ptr += sizeof(formatted_uint16);

  std::memcpy(ptr, &sensorsData.power_status, sizeof(sensorsData.power_status));
  Serial.println("Arrays");



  Serial.println("Sizes of arrays");
  Serial.println(sizeof(sensorsData.scd4x));
  Serial.println(sizeof(sensorsData.bme680));
  Serial.println(sizeof(sensorsData.sgp30));
  Serial.println(sizeof(sensorsData.kxtj3));
  Serial.println(sizeof(sensorsData.ltr));
  Serial.println(sizeof(sensorsData.sound.sampleRate));
  Serial.println(sizeof(sensorsData.sound.sampleDuration));
  Serial.println(sizeof(sensorsData.sound.numSamples));
  Serial.println(sizeof(sensorsData.sound.valid));
  Serial.println(sensorsData.sound.numSamples * sizeof(int));
  Serial.println(sizeof(sensorsData.pir));
  Serial.println(sizeof(sensorsData.battery_voltage));
  Serial.println(sizeof(sensorsData.power_status));
  
}

int Send_Data_AT_Handler(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
    SensorsData sensorsData;
    read_Sensors(&sensorsData);

    // Send to ESP32
    // Serial1.println("LTR Data:");
    // // Serial1.print("  Visible + IR: "); Serial1.println(sensorsData.ltr.visible_plus_ir);
    // // Serial1.print("  Infrared: "); Serial1.println(sensorsData.ltr.infrared);
    // // Serial1.print("  Valid: "); Serial1.println(sensorsData.ltr.valid);
    
    // Serial1.println("BME680 Data:");
    // // Serial1.print("  Temperature: "); Serial1.println(sensorsData.bme680.temperature);
    // // Serial1.print("  Humidity: "); Serial1.println(sensorsData.bme680.humidity);
    // // Serial1.print("  Pressure: "); Serial1.println(sensorsData.bme680.pressure);
    // // Serial1.print("  Gas Resistance: "); Serial1.println(sensorsData.bme680.gas_resistance);
    // // Serial1.print("  Valid: "); Serial1.println(sensorsData.bme680.valid);

    // Serial1.println("SGP30 Data:");
    // // Serial1.print("  TVOC: "); Serial1.println(sensorsData.sgp30.TVOC);
    // // Serial1.print("  eCO2: "); Serial1.println(sensorsData.sgp30.eCO2);
    // // Serial1.print("  Raw H2: "); Serial1.println(sensorsData.sgp30.rawH2);
    // // Serial1.print("  Raw Ethanol: "); Serial1.println(sensorsData.sgp30.rawEthanol);

    // Serial1.println("KXTJ3 Data:");
    // // Serial1.print("  X: "); Serial1.println(sensorsData.kxtj3.x);
    // // Serial1.print("  Y: "); Serial1.println(sensorsData.kxtj3.y);
    // // Serial1.print("  Z: "); Serial1.println(sensorsData.kxtj3.z);
    // // Serial1.print("  Valid: "); Serial1.println(sensorsData.kxtj3.valid);

    // Serial1.println("Sound Data:");
    // Serial1.print("  Sample Rate: "); Serial1.println(sensorsData.sound.sampleRate);
    // Serial1.print("  Sample Duration: "); Serial1.println(sensorsData.sound.sampleDuration);
    // Serial1.print("  Number of Samples: "); Serial1.println(sensorsData.sound.numSamples);
    // Serial1.print("  Valid: "); Serial1.println(sensorsData.sound.valid);
    // Serial1.print("  Sound Array: ");
    
    for (int i = 0; i < sensorsData.sound.numSamples; i++) {
      // Serial1.print(sensorsData.sound.soundArray[i]);
      // Serial1.print(" ");
    }
    // Serial1.println();

    // Serial1.println("SCD4x Data:");
    // // Serial1.print("  CO2: "); Serial1.println(sensorsData.scd4x.co2);
    // // Serial1.print("  Temperature: "); Serial1.println(sensorsData.scd4x.temperature);
    // // Serial1.print("  Humidity: "); Serial1.println(sensorsData.scd4x.humidity);
    // // Serial1.print("  Valid: "); Serial1.println(sensorsData.scd4x.valid);

    // Serial1.println("Additional Data:");
    // // Serial1.print("  PIR: "); Serial1.println(sensorsData.pir);
    // // Serial1.print("  Battery Voltage: "); Serial1.println(sensorsData.battery_voltage);
    // // Serial1.print("  Power Status: "); Serial1.println(sensorsData.power_status);

    // Print to Serial
    Serial.println("LTR Data:");
    Serial.print("  Visible + IR: "); Serial.println(sensorsData.ltr.visible_plus_ir);
    Serial.print("  Infrared: "); Serial.println(sensorsData.ltr.infrared);
    Serial.print("  Valid: "); Serial.println(sensorsData.ltr.valid);
    
    Serial.println("BME680 Data:");
    Serial.print("  Temperature: "); Serial.println(sensorsData.bme680.temperature);
    Serial.print("  Humidity: "); Serial.println(sensorsData.bme680.humidity);
    Serial.print("  Pressure: "); Serial.println(sensorsData.bme680.pressure);
    Serial.print("  Gas Resistance: "); Serial.println(sensorsData.bme680.gas_resistance);
    Serial.print("  Valid: "); Serial.println(sensorsData.bme680.valid);

    Serial.println("SGP30 Data:");
    Serial.print("  TVOC: "); Serial.println(sensorsData.sgp30.TVOC);
    Serial.print("  eCO2: "); Serial.println(sensorsData.sgp30.eCO2);
    Serial.print("  Raw H2: "); Serial.println(sensorsData.sgp30.rawH2);
    Serial.print("  Raw Ethanol: "); Serial.println(sensorsData.sgp30.rawEthanol);

    Serial.println("KXTJ3 Data:");
    Serial.print("  X: "); Serial.println(sensorsData.kxtj3.x);
    Serial.print("  Y: "); Serial.println(sensorsData.kxtj3.y);
    Serial.print("  Z: "); Serial.println(sensorsData.kxtj3.z);
    Serial.print("  Valid: "); Serial.println(sensorsData.kxtj3.valid);

    Serial.println("Sound Data:");
    Serial.print("  Sample Rate: "); Serial.println(sensorsData.sound.sampleRate);
    Serial.print("  Sample Duration: "); Serial.println(sensorsData.sound.sampleDuration);
    Serial.print("  Number of Samples: "); Serial.println(sensorsData.sound.numSamples);
    Serial.print("  Valid: "); Serial.println(sensorsData.sound.valid);
    Serial.print("  Mean: "); Serial.println(sensorsData.sound.mean);
    Serial.print("  Sound Array: ");
    
    for (int i = 0; i < sensorsData.sound.numSamples; i++) {
      Serial.print(sensorsData.sound.soundArray[i]);
      Serial.print(" ");
      delay(1);
    }
    
    Serial.println();

    Serial.println("SCD4x Data:");
    Serial.print("  CO2: "); Serial.println(sensorsData.scd4x.co2);
    Serial.print("  Temperature: "); Serial.println(sensorsData.scd4x.temperature);
    Serial.print("  Humidity: "); Serial.println(sensorsData.scd4x.humidity);
    Serial.print("  Valid: "); Serial.println(sensorsData.scd4x.valid);

    Serial.println("Additional Data:");
    Serial.print("  PIR: "); Serial.println(sensorsData.pir);
    Serial.print("  Battery Voltage: "); Serial.println(sensorsData.battery_voltage);
    Serial.print("  Power Status: "); Serial.println(sensorsData.power_status);

    if (param->argc == 0){
     Serial.println(cmd);
     }
    
    // Serialize sensor data
    unsigned char* buffer = nullptr;
    size_t bufferSize = 0;
    serializeSensorData(sensorsData, buffer, bufferSize);

    // Print the buffer
     Serial.println("Serialized Sensor Data:");
     for (size_t i = 0; i < bufferSize; ++i) {
       Serial.print(buffer[i], HEX);
       Serial.print(" ");
     }
    Serial.println();

    api.lorawan.send(bufferSize, buffer, 5);
    delete[] buffer;
    
  }
  
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

void setup()
{

  Serial1.begin(115200, RAK_AT_MODE);
  delay(2000);
  Serial.begin(115200);  
  pinMode(POWER_STATUS, INPUT);
  pinMode(EN_SENSOR, OUTPUT);
  digitalWrite(EN_SENSOR, HIGH);
  pinMode(PIR, INPUT);
  pinMode(MIC, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW);
  
  //setup_LoRaWan();
  SCD4x_init();
  BME680_init();
  SGP30_init();
  LTR_init();
  KXTJ3_init();

  api.system.atMode.add("VER", "Return the firmware version", "VER", ATC_Ver, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SCD4x", "Return all values from the SCD41 sensor including temperature, humidity, and CO2", "SCD4x", SCD4x_AT_Handler, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SCDCO2", "Return the CO2 value from the SCD41 sensor", "SCDCO2", SCD_CO2, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SCDTEMP", "Return the temperature value with 0.01°C resolution from the SCD41 sensor", "SCDTEMP", SCD_temp, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SCDHUM", "Return the humidity value with 1% resolution from the SCD41 sensor", "SCDHUM", SCD_humid, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("BME680", "Return temperature, humidity, pressure, and gas resistance values from the BME680 sensor", "BME680", BME680_AT_Handler, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SGP30", "Return TVOC, eCO2, rawH2, and rawEthanol values from the SGP30 sensor", "SGP30", SGP30_AT_Handler, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("KXTJ3", "Return the status of the KXTJ3 sensor including x, y, and z acceleration values", "KXTJ3", KXTJ3_AT_Handler, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LTR", "Return the status of the LTR-303 sensor including visible_plus_ir and infrared values", "LTR", LTR_AT_Handler, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SOUND", "Return a sound recording", "SOUND", Sound_AT_Handler, RAK_ATCMD_PERM_WRITE);
  api.system.atMode.add("PIR", "Return 0 or 1 indicating if motion is detected (0: No motion, 1: Motion detected)", "PIR", PIR_AT_Handler, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SEND", "Send a LoRaWAN packet with sensor data", "SEND", Send_Data_AT_Handler, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("BAT", "Return battery voltage in mV. Return 0 if not available", "BAT", Battery_AT_Handler, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("POWER", "Return power status", "POWER", POWER_AT_Handler, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("FAN", "Turn on/off the Fan", "FAN", FAN_AT_Handler, RAK_ATCMD_PERM_WRITE);

  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);

}

void loop()
{
  //delay(10000);
  
  currentMillis = millis();       
  if (currentMillis - getSensorDataPrevMillis > DATA_INTERVAL){
        getSensorDataPrevMillis = currentMillis;
        PIR_sum += digitalRead(PIR) && pir; // PIR value is considered positive only if it was already positive on the last slot
        PIR_counter++;
        pir = digitalRead(PIR);    
  }
  
  //api.system.sleep.all(500);
}
