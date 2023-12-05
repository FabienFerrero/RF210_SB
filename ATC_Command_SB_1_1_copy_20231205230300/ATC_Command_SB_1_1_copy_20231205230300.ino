/**

   RRRR  FFFF  22   11   000
   R   R F    2  2 111  0  00
   RRRR  FFF    2   11  0 0 0
   R R   F     2    11  00  0
   R  RR F    2222 11l1  000


   @file ATC_Command_RF210.ino
   @author XuanMinh201, FabienFerrero

   @brief This sketch add custom ATC command to RFThings RF210 board. These commands help controlling on-board sensors,
   GNSS & measuring battery level. For detail description, please visit: https://github.com/XuanMinh201/RF210

   @version 0.1.1
   @date 2023-10-26

   @copyright Copyright (c) 2023

*/

#define DATA_INTERVAL 5000 //ms

#include <Arduino.h>
#include <SensirionI2CScd4x.h> // Sensirion SCD40
#include "Zanshin_BME680.h" // https://github.com/Zanduino/BME680/tree/master
#include <Wire.h>

#include <Kionix_KX023.h>           // TO-DO: Add this original
//#include "Adafruit_LTR329_LTR303.h" // http://librarymanager/All#Adafruit_LTR329_LTR303
#include <LTR303.h> // https://github.com/automote/LTR303     // update lib with : value = (high << 8) + low;

#define boot_button PH3
#define LED PA0
#define DATA_INTERVAL 5000 // ms
#define LS_ADC_AREF 3.0f
#define LS_BATVOLT_R1 1.0f
#define LS_BATVOLT_R2 2.0f
#define LS_BATVOLT_PIN PB4

//Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
KX023 myIMU;
LTR303 light;
SensirionI2CScd4x scd4x;
BME680_Class BME680;  ///< Create an instance of the BME680 class



unsigned char gain = 0;   // Gain setting, values = 0-7
unsigned char integrationTime = 0; // Integration ("shutter") time in milliseconds
uint16_t voltage_adc;
uint16_t voltage;
float kx_x, kx_y, kx_z;
//int humi, temper;
//sensors_event_t humidity, temp;
bool valid;
bool ltr_status;
unsigned int visible, infrared;
double lux;

uint16_t co2 = 0;
float temperature = 0.0f;
float humidity = 0.0f;


void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}
void scd41()
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
  } else {
    printSerialNumber(serial0, serial1, serial2);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    // Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    //Serial.println(errorMessage);
  }

  // Serial.println("Waiting for first measurement... (5 sec)");
}
void scd41_get() {
  uint16_t error;
  char errorMessage[256];

  //  delay(100);

  // Read Measurement

  bool isDataReady = false;
  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
    //Serial.print("Error trying to execute getDataReadyFlag(): ");
    errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
    return;
  }
  if (!isDataReady) {
    return;
  }
  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    //Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
  } else if (co2 == 0) {
    // Serial.println("Invalid sample detected, skipping.");
  } else {
    //        Serial.print("Co2:");
    //        Serial.print(co2);
    //        Serial.print("\t");
    //        Serial.print("Temperature:");
    //        Serial.print(temperature);
    //        Serial.print("\t");
    //        Serial.print("Humidity:");
    //        Serial.println(humidity);
  }
}


int ATC_Ver(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");

      Serial1.print(cmd);
      Serial1.print("=");
    }

    Serial.println("0.1.0");
    Serial1.println("0.1.0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}


int scd41_co2(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

   
      if (param->argc == 0) {
        Serial1.print(cmd);
      Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      scd41_get();
      Serial.println(co2);
      Serial1.println(co2);


  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int SHTC3_temp(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

   
      if (param->argc == 0) {
        Serial1.print(cmd);
      Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      scd41_get();
      Serial.println(temperature);


  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int SHTC3_humi(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    scd41_get();
    Serial.println(int(humidity));

  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_temp(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

   
      if (param->argc == 0) {
        Serial1.print(cmd);
      Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      
      static int32_t  temp, humidity, pressure, gas;  // BME readings
      BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
      Serial.println(temp/100);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_hum(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

   
      if (param->argc == 0) {
        Serial1.print(cmd);
      Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      
      static int32_t  temp, humidity, pressure, gas;  // BME readings
      BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
      Serial.println(humidity/1000);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_bar(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

   
      if (param->argc == 0) {
        Serial1.print(cmd);
      Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      
      static int32_t  temp, humidity, pressure, gas;  // BME readings
      BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
      Serial.println(pressure);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_gas(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

   
      if (param->argc == 0) {
        Serial1.print(cmd);
      Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      
      static int32_t  temp, humidity, pressure, gas;  // BME readings
      BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
      Serial.println(gas/10);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}



bool kx023_status;

bool getKX023status()
{
  if (myIMU.begin())
  {
    return 0;
  }
  else
  {
    myIMU.configContinuousReading(LOWPOWER, RANGE_8G, DATARATE_100HZ);
    return 1;
  }
}

int KX023_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0) {
      Serial1.print(cmd);
      Serial1.print("=");
      Serial.print(cmd);
      Serial.print("=");
    }
    Serial1.println(kx023_status ? "1" : "0");
    Serial.println(kx023_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int KX023_AX(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0) {
        Serial1.print(cmd);
        Serial1.print("=");
        Serial.print(cmd);
      Serial.print("=");
      }
      Serial1.println(kx_x);
      Serial.println(kx_x);
    }
    else
    {
      if (param->argc == 0) {
        Serial1.print(cmd);
        Serial1.print("=");
        Serial.print(cmd);
      Serial.print("=");
      }
      Serial1.println("0");
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int KX023_AY(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0) {
        Serial1.print(cmd);
        Serial1.print("=");
        Serial.print(cmd);
      Serial.print("=");
      }
      Serial1.println(kx_y);
      Serial.println(kx_y);
    }
    else
    {
      if (param->argc == 0) {
        Serial1.print(cmd);
        Serial1.print("=");
        Serial.print(cmd);
      Serial.print("=");
      }
      Serial1.println("0");
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int KX023_AZ(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0) {
        Serial1.print(cmd);
        Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      Serial1.println(kx_z);
      Serial.println(kx_z);
    }
    else
    {
      if (param->argc == 0) {
        Serial1.print(cmd);
        Serial1.print("=");
        Serial.print(cmd);
        Serial.print("=");
      }
      Serial1.println("0");
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}


bool getLTRstatus()
{
  unsigned char ID;

  if (light.getPartID(ID)) {
    light.setControl(gain, false, false);
    light.setMeasurementRate(1, 3);
    return 1;
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else {
    return 0;
  }
}

unsigned char getLTRID()
{
  unsigned char ID;

  if (light.getPartID(ID)) {
    return ID;
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else {
    return 0;
  }
}

int LTR_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //light.setPowerUp();
    ltr_status = getLTRstatus();
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    Serial.println(ltr_status ? "1" : "0");
    Serial.print("LTR Sensor Part ID: 0X");
    Serial.println(getLTRID(), HEX);

    Serial1.println(ltr_status ? "1" : "0");
    Serial1.print("LTR Sensor Part ID: 0X");
    Serial1.println(getLTRID(), HEX);
    
    //light.setPowerDown();
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int LTR_ch0(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //ltr_status = getLTRstatus();
    //light.setPowerUp();
    //delay(5000);
    if (light.getData(visible, infrared)) {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println(visible);
      Serial1.println(visible);
      // light.setPowerDown();
    }
    else
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println("0");
      Serial1.println("0");
    }
  }


  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int LTR_ch1(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

    //ltr_status = getLTRstatus();
    //light.setPowerUp();
    //delay(200);
    if (light.getData(visible, infrared))
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println(infrared);
      Serial1.println(infrared);
      // light.setPowerDown();
    }
    else
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println("0");
      Serial1.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int LTR_lux(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //ltr_status = getLTRstatus();
    // light.setPowerUp();
    //delay(5000);
    if (light.getData(visible, infrared))
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }

      valid = light.getLux(gain, integrationTime, visible, infrared, lux);
      Serial.println(lux);
      Serial1.println(lux);
      // light.setPowerDown();
    }

    else
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println("0");
      Serial1.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}



int battery(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

    voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
    voltage = (uint16_t)((LS_ADC_AREF / 1.024) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    Serial.println(voltage);
    Serial1.println(voltage);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int ldo_read(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    Serial.println(api.system.bat.get() * 1000);
    Serial1.println(api.system.bat.get() * 1000);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
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

void setup()
{
  Serial.begin(115200, RAK_AT_MODE);
  Serial1.begin(115200, RAK_AT_MODE);

  pinMode(boot_button, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  scd41();
  kx023_status = getKX023status();
  light.begin();
  light.setPowerUp();

  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds


  while (!Serial)
  {
    delay(10); // wait for serial port to open
  }
  while (!Serial1)
  {
    delay(10); // wait for serial port to open
  }

  api.system.atMode.add("VER", "Return firmware version", "VER", ATC_Ver, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SCDCO2", "Return the status of the SHTC3 sensor. 1 if available.", "SCDCO2", scd41_co2, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SCDTEMP", "Return the temperature value with 0.01° resolution", "SCDTEMP", SHTC3_temp, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SCDHUM", "Return the humidity value with 1% resolution", "SCDHUM", SHTC3_humi, RAK_ATCMD_PERM_READ);

    api.system.atMode.add("BMEGAS", "Return the gaz resistivity in ohmable.", "BMEGAS", bme_gas, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("BMETEMP", "Return the temperature value with 0.01° resolution", "BMETEMP", bme_temp, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("BMEHUM", "Return the humidity value with 1% resolution", "BMEHUM", bme_hum, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("BMEBAR", "Return the pressure value in mbar ", "BMEHUM", bme_bar, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("TEMP", "Return the temperature value with 0.01° resolution", "TEMP", SHTC3_temp, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("HUM", "Return the humidity value with 1% resolution", "HUM", SHTC3_humi, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("KX023", "Return the status of the KX023 sensor. 1 if available.", "KX023", KX023_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AX", "Return the value of X acceleration with 0.01G resolution", "AX", KX023_AX, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AY", "Return the value of Y acceleration with 0.01G resolution", "AY", KX023_AY, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AZ", "Return the value of Z acceleration with 0.01G resolution", "AZ", KX023_AZ, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("LTR", "Return the status of the LTR-303 sensor. 1 if available.", "LTR", LTR_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH0", "Return the CHANNEL0 value of the LTR-303 sensor", "LUMCH0", LTR_ch0, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH1", "Return the CHANNEL1 value of the LTR-303 sensor", "LUMCH1", LTR_ch1, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUM", "Return the CHANNEL1 value of the LTR-303 sensor", "LUM", LTR_lux, RAK_ATCMD_PERM_READ);


  api.system.atMode.add("BAT", "Return battery voltage in mV | Return 0 if not available", "BAT", battery, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LDO", "Return LDO voltage in mV | Return 0 if not available", "LDO", ldo_read, RAK_ATCMD_PERM_READ);
}

void loop()
{


}



//void print_line(void)
//{
//  Serial.print(it);
//  Serial.print("***********************");
//  Serial.print(it);
//  Serial.print("*************************");
//  Serial.println(it);
//  it++;
//
//}
