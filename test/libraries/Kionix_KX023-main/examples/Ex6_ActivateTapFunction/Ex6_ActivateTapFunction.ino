#include <Wire.h>
#include <Kionix_KX023.h>

#define INT1_PIN PB4 // Update your pin number here
#define INT2_PIN -1 // Update your pin number here

KX023 myIMU;
volatile bool flag;

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("KX023-1025 Activate Tap Function");

  KX023_Status_t status = myIMU.begin();
  if (status == KX023_STATUS_OK)
  {
    Serial.println("KX023-1025: OK");
  }
  else
  {
    Serial.println(kx023_get_status_string(status));
  }

  DirectionInfoParams_t tap_direction;
  tap_direction.x_negative = false;
  tap_direction.x_positive = false;
  tap_direction.y_negative = false;
  tap_direction.y_positive = false;
  tap_direction.z_negative = true;
  tap_direction.z_positive = true;
  myIMU.configActivateTapFunction(KX023_ACCLERATION_RANGE_2G, KX023_DTODR_400HZ, tap_direction, KX023_TAP_MODE_BOTH);

  PhysicalInterruptParameter_t params;
  params.polarity = KX023_INTERRUPT_POLARITY_ACTIVE_HIGH;
  params.signal_type = KX023_INTERRUPT_TYPE_PULSE;
  params.events.tap_function_interrupt = true;
  myIMU.configPhysicalInterruptPin(1, params);

  flag = false;
  pinMode(INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), tap_detect_isr, FALLING);

  myIMU.setOperatingMode();
  delay(50);
}

void loop(void)
{
  if (flag)
  {
    Serial.println("Tap detected!");

    switch (myIMU.getInterruptType())
    {
    case KX023_INTERRUPT_SINGLE_TAP:
      Serial.println("Single Tap detected!");
      break;
    case KX023_INTERRUPT_DOUBLE_TAP:
      Serial.println("Double Tap detected!");
      break;
    default:
      Serial.println("Error");
      break;
    }

    myIMU.clearInterrupt();
    flag = false;
  }

  // Do other works here
}

void tap_detect_isr(void)
{
  flag = true;
}