#include <Wire.h>
#include <Kionix_KX023.h>

KX023 myIMU;
float kx_x, kx_y, kx_z;

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("KX023-1025 Asynchronous Read Back Acceleration Data");

  KX023_Status_t status = myIMU.begin();
  if (status == KX023_STATUS_OK)
  {
    Serial.println("KX023-1025: OK");
  }
  else
  {
    Serial.println(kx023_get_status_string(status));
  }

  myIMU.configAsynchronousReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);

  myIMU.setOperatingMode();
  delay(50);
}

void loop(void)
{
  myIMU.readAsynchronousReadBackAccelerationData(&kx_x, &kx_y, &kx_z);
  Serial.print(kx_x);
  Serial.print(", ");
  Serial.print(kx_y);
  Serial.print(", ");
  Serial.print(kx_z);
  Serial.println("");
}
