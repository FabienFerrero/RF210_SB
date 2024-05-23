#include <Wire.h>
#include <Kionix_KX023.h>

#define INT1_PIN PB4 // Update your pin number here
#define INT2_PIN -1 // Update your pin number here

KX023 myIMU;
float kx_x, kx_y, kx_z;
volatile bool flag;

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("KX023-1025 Synchronous Hardware Interrupt Read Back Acceleration Data");

  KX023_Status_t status = myIMU.begin();
  if (status == KX023_STATUS_OK)
  {
    Serial.println("KX023-1025: OK");
  }
  else
  {
    Serial.println(kx023_get_status_string(status));
  }

  myIMU.configSynchronousHardwareInterruptReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);

  PhysicalInterruptParameter_t params;
  params.polarity = KX023_INTERRUPT_POLARITY_ACTIVE_HIGH;
  params.signal_type = KX023_INTERRUPT_TYPE_PULSE;
  params.events.data_ready_interrupt = true;
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
    myIMU.readAsynchronousReadBackAccelerationData(&kx_x, &kx_y, &kx_z);
    Serial.print(kx_x);
    Serial.print(", ");
    Serial.print(kx_y);
    Serial.print(", ");
    Serial.print(kx_z);
    Serial.println("");

    flag = false;
  }

  // Do other works here
}

void data_available_isr(void)
{
  flag = true;
}