#include <Wire.h>
#include <Kionix_KX023.h>

#define INT1_PIN -1 // Update your pin number here
#define INT2_PIN -1 // Update your pin number here

KX023 myIMU;
volatile bool flag;

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("KX023-1025 Activate Tilt Position");

  KX023_Status_t status = myIMU.begin();
  if (status == KX023_STATUS_OK)
  {
    Serial.println("KX023-1025: OK");
  }
  else
  {
    Serial.println(kx023_get_status_string(status));
  }

  DirectionInfoParams_t direction;
  direction.x_negative = false;
  direction.x_positive = false;
  direction.y_negative = false;
  direction.y_positive = false;
  direction.z_negative = true;
  direction.z_positive = true;
  myIMU.configActivateTiltPosition(KX023_ACCLERATION_RANGE_2G, KX023_TPODR_12_5HZ, 80, direction);

  PhysicalInterruptParameter_t params;
  params.polarity = KX023_INTERRUPT_POLARITY_ACTIVE_HIGH;
  params.signal_type = KX023_INTERRUPT_TYPE_PULSE;
  params.events.tilt_position_function_interrupt = true;
  myIMU.configPhysicalInterruptPin(1, params);

  flag = false;
  pinMode(INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), tilt_detect_isr, FALLING);

  myIMU.setOperatingMode();
  delay(50);
}

void loop(void)
{
  if (flag)
  {
    Serial.println("Tilt detected!");

    myIMU.clearInterrupt();
    flag = false;
  }

  // Do other works here
}

void tilt_detect_isr(void)
{
  flag = true;
}