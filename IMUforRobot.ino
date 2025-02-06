#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 100

#define radius_to_degrees 180.00 /3.14159


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  bno.begin();

  delay(1000);

  //int8_t temp = bno.getTemp();

}


void quaternion_Delta () {
  // Quaternion data 1
  imu::Quaternion quat = bno.getQuat();
  // Euler 1
  imu::Vector<3> euler_one = quat.toEuler();
  //delay between samples
  delay(BNO055_SAMPLERATE_DELAY_MS);
  // Quaternion data 2
  imu::Quaternion quat_two = bno.getQuat();
  // Euler 2
  imu::Vector<3> euler_two = quat_two.toEuler();
  // quaternion comparison
  imu::Quaternion quatDelta = quat_two * quat.conjugate ();
  imu::Vector<3> euler = quatDelta.toEuler();
  Serial.print(" X: ");
  Serial.print(euler.x()*radius_to_degrees, 2);
  Serial.print(" Y: ");
  Serial.print(euler.y()*radius_to_degrees, 2);
  Serial.print(" Z: ");
  Serial.print(euler.z()*radius_to_degrees, 2);
  Serial.print("\n");
  delay(1000);
  // Euler comparison
  // imu::Vector<3> euler_comparison = euler_two - euler_one;
  // Serial.print(" x: ");
  // Serial.print(euler_comparison.x()*radius_to_degrees, 2);
  // Serial.print(" y: ");
  // Serial.print(euler_comparison.y()*radius_to_degrees, 2);
  // Serial.print(" z: ");
  // Serial.print(euler_comparison.z()*radius_to_degrees, 2);
  // Serial.print("\n");
}

void loop() {
  // put your main code here, to run repeatedly:

  quaternion_Delta();

}
