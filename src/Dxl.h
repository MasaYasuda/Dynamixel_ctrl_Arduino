#ifndef DXL_H
#define DXL_H
#include <Arduino.h>
#include <HardwareSerial.h>

class Dxl{
  public:
    /**
     * 初期化、
     * モード設定、
     * PDParam書き込み、
     * トルクON・OFF、
     * 位置書き込み
     * （位置読みとり）
    */
    Dxl(int id,HardwareSerial *ser);
    void servo_control(int8_t CTRL);
    void servo_torque(int8_t ON_OFF);
    void servo_position(int32_t POS);
    void servo_speed(int32_t SPE2);//速度制御時速度定義
    void servo_pwm(int16_t PWM);//角度制御時速度定義
    void servo_pgain(int16_t PGA);
    void servo_dgain(int16_t DGA);

  private:
    /**
     * id、
     * モード
     * （チェックサム計算）
    */
    int ID;
    int CTRL;//Mode
    HardwareSerial *serial;
    unsigned char cbuf[100];

    uint16_t sumcheck(int length,unsigned char *cbuf);

};


#endif