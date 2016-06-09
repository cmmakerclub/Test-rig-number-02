#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>
#include <SPI.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define VR_1 A0
#define Bt_1 3
#define Bt_2 7

#define Gry_offset 0.00
#define Gyr_Gain 0.00763358
#define Angle_offset -2.0f
#define offset_motor1  1275
#define offset_motor2  1275
#define pi 3.14159

float limit(float input, int min_limit, int max_limit);
float smooth(float alfa, float new_data, float old_data);

float P_CompCoeff = 0.98;

long data;
int gyro_offset1;

float r_angle, i_angle, f_angle, gyro_offset;

uint32_t time_now, time_prev, time_prev2;


Servo myservo1;
Servo myservo2;

float vr1, vr2, vr3, vr4, vr5, vr6;


void setup()
{
  pinMode(VR_1, INPUT);
  pinMode(Bt_1, INPUT_PULLUP);
  pinMode(Bt_2, INPUT_PULLUP);
  Wire.begin();
  delay(300);
  accelgyro.initialize();
  Serial.begin(115200);
}

void loop()
{
  time_now = millis();

  if (time_now - time_prev >= 10)
  {
    time_prev = time_now;
    ///////////////////////////////////////////////////
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //  ay  az gx

    gyro_offset = smooth(0.02f, gx, gyro_offset);

    f_angle += (((gx - gyro_offset1) / 32.8f) * (0.01f));
    float pitchAcc = atan2(ay, az) * RAD_TO_DEG;
    pitchAcc += 3.7;
    
    f_angle = P_CompCoeff * f_angle + (1.0f - P_CompCoeff) * pitchAcc;

    i_angle += (((gx - gyro_offset1) / 32.8f) * (0.01f));

    r_angle = (vr1 - 341) / 3.1f;

    if (!digitalRead(Bt_2))
    {
      i_angle = r_angle;
      f_angle = r_angle;
    }
    if (!digitalRead(Bt_1))
    {
      gyro_offset1 = gyro_offset;
    }



    Serial.println(f_angle-r_angle);
    //Serial.println(gyro_offset);
  }

  if (time_now - time_prev2 >= 20)
  {
    time_prev2 = time_now;
    int8_t  i_angle_m = (int8_t)i_angle;
    int8_t  f_angle_m = (int8_t)f_angle;
    int8_t  r_anglem = (int8_t)r_angle;


    //
    //    Serial.print("&");
    //    Serial.print(",");
    //    Serial.print((int8_t)(motor_a_m)); //ความแรงเป็น%มอเตอร์ซ้าย
    //    Serial.print(",");
    //    Serial.print((int8_t)(motor_b_m)); //ความแรงเป็น%มอเตอร์ขวา
    //    Serial.print(",");
    //    Serial.print(angle_m);  // มุมจากใจโร
    //    Serial.print(",");
    //    Serial.print(error_m); //มุมที่ต้องการ
    //    Serial.print(",");
    //    Serial.print(kp_m); //  kp
    //    Serial.print(",");
    //    Serial.print(ki_m); //  ki
    //    Serial.print(",");
    //    Serial.print(kd_m); //  kd
    //    Serial.println(",");
  }

  vr1 = smooth(0.05f, (analogRead(VR_1)), vr1);

}

float limit(float input, int min_limit, int max_limit)
{
  if (input > max_limit)input = max_limit;
  if (input < min_limit)input = min_limit;
  return input;
}

float smooth(float alfa, float new_data, float old_data)
{
  return (old_data + alfa * (new_data - old_data));
}
