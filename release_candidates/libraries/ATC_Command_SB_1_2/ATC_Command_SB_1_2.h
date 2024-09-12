#include<stdio.h>

#define LED PA9
#define PIR PA0
#define MIC PB3
#define POWER_STATUS PB2
#define FAN PA4
#define EN_SENSOR PA8
#define POWER_THRESHOLD 600
#define ADC_AREF 3.3f
#define BATVOLT_R1 1.0f
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN PB4
#define DEFAULT_SAMPLE_RATE 1000
#define DEFAULT_DURATION 3


struct SCD4xData {
  uint16_t co2;
  float temperature;
  float humidity;
  bool valid;

  SCD4xData() : co2(0), temperature(0), humidity(0), valid(false) {}
};

struct BME680Data {
  float temperature;
  float humidity;
  float pressure;
  float gas_resistance;
  bool valid;

  BME680Data() : temperature(0), humidity(0), pressure(0), gas_resistance(0), valid(false) {}
};

struct SGP30Data {
  uint16_t TVOC;
  uint16_t eCO2;
  uint16_t rawH2;
  uint16_t rawEthanol;

  SGP30Data() : TVOC(0), eCO2(0), rawH2(0), rawEthanol(0) {}
};

struct KXTJ3Data {
  float x;
  float y;
  float z;
  bool valid;

  KXTJ3Data() : x(0), y(0), z(0), valid(false) {}
};

struct LTRData {
  uint16_t visible_plus_ir;
  uint16_t infrared;
  bool valid;

  LTRData() : visible_plus_ir(0), infrared(0), valid(false) {}
};

struct SoundData {
  int sampleRate;
  int sampleDuration;
  int* soundArray;
  int numSamples;
  bool valid;

  SoundData() : sampleRate(DEFAULT_SAMPLE_RATE), sampleDuration(DEFAULT_DURATION), soundArray(nullptr), numSamples(DEFAULT_SAMPLE_RATE*DEFAULT_DURATION), valid(false) {}

  // Destructor để giải phóng bộ nhớ cho soundArray
  ~SoundData() {
    delete[] soundArray;
  }

  // Copy constructor
  SoundData(const SoundData& other) {
    sampleRate = other.sampleRate;
    sampleDuration = other.sampleDuration;
    numSamples = other.numSamples;
    valid = other.valid;

    if (other.soundArray) {
      soundArray = new int[numSamples];
      std::memcpy(soundArray, other.soundArray, numSamples * sizeof(int));
    } else {
      soundArray = nullptr;
    }
  }

  // Copy assignment operator
  SoundData& operator=(const SoundData& other) {
    if (this == &other) return *this;

    delete[] soundArray;

    sampleRate = other.sampleRate;
    sampleDuration = other.sampleDuration;
    numSamples = other.numSamples;
    valid = other.valid;

    if (other.soundArray) {
      soundArray = new int[numSamples];
      std::memcpy(soundArray, other.soundArray, numSamples * sizeof(int));
    } else {
      soundArray = nullptr;
    }

    return *this;
  }
};

struct SensorsData {
  SCD4xData scd4x;
  BME680Data bme680;
  SGP30Data sgp30;
  KXTJ3Data kxtj3;
  LTRData ltr;
  SoundData sound;
  int pir;
  int battery_voltage;
  bool power_status;

  SensorsData() : pir(0), battery_voltage(0), power_status(false) {}
};
