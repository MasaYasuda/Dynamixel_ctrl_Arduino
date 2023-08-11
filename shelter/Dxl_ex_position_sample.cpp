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
  int ctrl=4;
  Dxl.servo_control(ctrl); //角度制御設定

  Dxl.servo_pwm(885); //角度制御のモータ回転速度に相当（0～885）
  // PWMかPDゲインで角度制御時における速度調整が可能
  Dxl.servo_pgain(640); //角度制御のモータ回転速度に相当（初期設定：640）
  Dxl.servo_dgain(4000); //角度制御のモータ回転速度に相当（初期設定：4000）
  Dxl.servo_torque(1);
  delay(2000);//2秒待機

}

void loop(){
  Dxl.servo_position(0);
  delay(1000);
  Dxl.servo_position(2000);
  delay(1000);
  Dxl.servo_position(4000);
  delay(1000);
  Dxl.servo_position(6000);
  delay(1000);
  Dxl.servo_position(4000);
  delay(1000);
  Dxl.servo_position(2000);
  delay(1000);
}