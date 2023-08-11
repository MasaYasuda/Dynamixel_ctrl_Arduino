#include <Dxl.h>

Dxl::Dxl(int id,HardwareSerial *ser){
    ID=id;
    serial=ser;
    cbuf[0]  = (unsigned char)0xFF;     // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;     // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;     // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;     // ヘッダー4
    cbuf[4]  = (unsigned char)ID;       // サーボID
}

uint16_t Dxl::sumcheck(int length,unsigned char *cbuf){
    uint16_t checksum = 0x0000;
    for(int i=0;i<length;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(int j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    return checksum;
}


void Dxl::servo_control(int8_t CTRL){
    
  //1   Velocity Control Mode
  //3   Position Control Mode
  //4   Extended Position Control Mode
  //16   Current-Base Position Control 

    cbuf[5]  = (unsigned char)0x06;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)11;       // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)CTRL;     // 書込データ（1バイト）
    uint16_t checksum = sumcheck(11,cbuf);
    cbuf[11] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[12] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    serial->write(cbuf,13);
    delay(20);//時間待機が必須
}


void Dxl::servo_torque(int8_t ON_OFF){
// 1でトルクON，0でトルクオフ
    cbuf[5]  = (unsigned char)0x06;    // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;    // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;    // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)0x40;    // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;    // メモリ位置（2バイト中，上位バイト）
    cbuf[10]  = (unsigned char)ON_OFF; // 書込データ（１バイト）
    uint16_t checksum = sumcheck(11,cbuf);
    cbuf[11] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[12] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    serial->write(cbuf,13);
    delay(20);//時間待機が必須
}


void Dxl::servo_position(int32_t POS){
// 角度範囲：0～4095（0度～360度）
// 拡張位置制御モード：-1048575~1048575（-256~+256回転）
    cbuf[5]  = (unsigned char)0x09;    // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;    // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;    // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)0x74;    // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;    // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&POS);      // 書込データ（4バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(POS>>8)); // 書込データ（4バイト中，上から2バイト目）
    cbuf[12] = (unsigned char)(0x000000ff&(POS>>16));// 書込データ（4バイト中，上から3バイト目）
    cbuf[13] = (unsigned char)(0x000000ff&(POS>>24));// 書込データ（4バイト中，上から4バイト目）
    uint16_t checksum = sumcheck(14,cbuf);
    cbuf[14] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[15] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    serial->write(cbuf,16);
    delay(20);//時間待機が必須
}

void Dxl::servo_speed(int32_t SPE2){
// 速度範囲：-1023 ～ +1023  
// 速度制御時の速度値
// なお，角度制御時の回転速度はPWMで調整
    cbuf[5]  = (unsigned char)0x09;    // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;    // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;    // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)104;     // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;    // メモリ位置（2バイト中，上位バイト）   
    cbuf[10] = (unsigned char)(0x000000ff&SPE2);        // 書込データ（4バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(SPE2>>8));   // 書込データ（4バイト中，上から2バイト目）
    cbuf[12] = (unsigned char)(0x000000ff&(SPE2>>16));  // 書込データ（4バイト中，上から3バイト目）
    cbuf[13] = (unsigned char)(0x000000ff&(SPE2>>24));  // 書込データ（4バイト中，上から4バイト目）
    uint16_t checksum = sumcheck(14,cbuf);
    cbuf[14] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[15] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    serial->write(cbuf,16);
    delay(20);//時間待機が必須
}


void Dxl::servo_pwm(int16_t PWM){
// PWM範囲:0 ～ +885
// 角度制御時の速度と関係有り
    cbuf[5]  = (unsigned char)0x07;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)100;      // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&PWM);       // 書込データ（2バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(PWM>>8));  // 書込データ（2バイト中，上から2バイト目）
    uint16_t checksum = sumcheck(12,cbuf);
    cbuf[12] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[13] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    serial->write(cbuf,14);
    delay(20);//時間待機が必須
}


void Dxl::servo_pgain(int16_t PGA){
// Pgain範囲:0 ～ +16383 (基準：640)
// 角度制御時の速度と関係有り
    cbuf[5]  = (unsigned char)0x07;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)84;       // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&PGA);       // 書込データ（2バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(PGA>>8));  // 書込データ（2バイト中，上から2バイト目）
    uint16_t checksum = sumcheck(12,cbuf);
    cbuf[12] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[13] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    serial->write(cbuf,14);
    delay(20);//時間待機が必須
}


void Dxl::servo_dgain(int16_t DGA){
// Dgain範囲:0 ～ +16383 (基準：400)
// 角度制御時の速度と関係有り
    cbuf[5]  = (unsigned char)0x07;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)80;       // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&DGA);       // 書込データ（2バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(DGA>>8));  // 書込データ（2バイト中，上から2バイト目）
    uint16_t checksum = sumcheck(12,cbuf);
    cbuf[12] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[13] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    serial->write(cbuf,14);
    delay(20);//時間待機が必須
}

