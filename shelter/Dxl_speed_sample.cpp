#include <Arduino.h>
#include<Dxl.h>

// IDは，モータ個別にDynamixel Wizard2で割り振ってください．
int id=1;
Dxl Dxl(1,&Serial);

void setup(){
  Serial.begin(115200);
  // 制御設定
  //1   Velocity Control Mode
  //3   Position Control Mode
  //4   Extended Position Control Mode
  int ctrl=1;
  Dxl.servo_control(ctrl); //角度制御設定
  Dxl.servo_torque(1);
  delay(2000);//2秒待機

}

void loop(){
  Dxl.servo_speed(0);
  delay(1000);
  Dxl.servo_speed(120);//初期設定では最大速度が±128
  delay(1000);
  Dxl.servo_speed(0);
  delay(1000);
  Dxl.servo_speed(-120);
  delay(1000);
}