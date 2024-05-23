#include <Wire.h>
#include <Kionix_KX023.h>

#define INT1_PIN PB4 // Update your pin number here
#define INT2_PIN -1 // Update your pin number here

KX023 myIMU;
uint8_t buffer[252];
volatile bool flag;

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("KX023-1025 Buffer Full Interrupt");

  KX023_Status_t status = myIMU.begin();
  if (status == KX023_STATUS_OK)
  {
    Serial.println("KX023-1025: OK");
  }
  else
  {
    Serial.println(kx023_get_status_string(status));
  }

  myIMU.configBufferFullInterrupt(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);

  PhysicalInterruptParameter_t params;
  params.polarity = KX023_INTERRUPT_POLARITY_ACTIVE_HIGH;
  params.signal_type = KX023_INTERRUPT_TYPE_PULSE;
  params.events.buffer_full_interrupt = true;
  myIMU.configPhysicalInterruptPin(1, params);

  flag = false;
  pinMode(INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), data_available_isr, FALLING);

  myIMU.setOperatingMode();
  delay(50);
}

void loop(void)
{
  if (flag)
  {
    memset(buffer, 0, 252);
    myIMU.readBuffer(buffer);

    uint8_t i = 0;
    while (i < 252)
    {
      int8_t raw_x = buffer[i];
      int8_t raw_y = buffer[i + 1];
      int8_t raw_z = buffer[i + 2];

      Serial.print("RAW ");
      Serial.print(i / 3);
      Serial.print(" | x = ");
      Serial.print(raw_x);
      Serial.print(" | y = ");
      Serial.print(raw_y);
      Serial.print(" | z = ");
      Serial.println(raw_z);

      Serial.print("Decoded ");
      Serial.print(i / 3);
      Serial.print(" | x = ");
      Serial.print(raw_x * 2.0 / 127.0);
      Serial.print(" | y = ");
      Serial.print(raw_y * 2.0 / 127.0);
      Serial.print(" | z = ");
      Serial.println(raw_z * 2.0 / 127.0);

      i += 3;
    }

    flag = false;
  }

  // Do other works here
}

void data_available_isr(void)
{
  flag = true;
}