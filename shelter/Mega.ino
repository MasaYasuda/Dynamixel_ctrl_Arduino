// Dyanmixelの情報
//https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#present-position132

int i=0,j=0;

//関数の中のダイナミクセル・シリアル通信のための暗号化バッファ
uint16_t checksum;

//関数の中のダイナミクセル・シリアル通信バッファ
unsigned char cbuf[100];
unsigned char ch;
unsigned char rx[15];

// モータ角度読み取り変数
int32_t theta;

void setup() {
  Serial.begin(115200);

  // シリアル通信速度（ダイナミクセルをマイコンとシリアル通信する場合は115200で！57600はNG）
  // この部分は，あらかじめDynamixel Wizard2でDynamixelモータの方を「115200」に設定してください．
  Serial2.begin(115200);
  //Serial2.setTimeout(2000);
}

void loop() {

  // IDは，モータ個別にDynamixel Wizard2で割り振ってください．
  int id=1;

  // 制御設定
  //1   Velocity Control Mode
  //3   Position Control Mode
  //4   Extended Position Control Mode
  int ctrl=1;
  servo_control(id,ctrl); //角度制御設定

  servo_pwm,(id,885); //角度制御のモータ回転速度に相当（0～885）

  // PWMかPDゲインで角度制御時における速度調整が可能
  servo_pgain(1,640); //角度制御のモータ回転速度に相当（初期設定：640）
  servo_dgain(1,4000); //角度制御のモータ回転速度に相当（初期設定：4000）

  servo_torque(id,1); // トルクON
  delay(2000);//2秒待機

  if(ctrl==3){
    servo_position(id,1000);//角度指定：0～4095（0度～360度）
    delay(2000);
    servo_position(id,3000);//角度指定：0～4095（0度～360度）
    delay(2000);
    servo_position(id,1000);//角度指定：0～4095（0度～360度）
    delay(2000);  
  }
  else if(ctrl==1){
    theta=servo_read(id);
    if(theta<8000){
      servo_speed(id,+100);//角度制御（－1023～+1023）   
      while(1){
        theta=servo_read(id);
        Serial.println(theta);
        delay(100);
        if(theta>=8000){
          servo_speed(id,0);//角度制御（－1023～+1023）
          break;
        }
      }
    }
    else {
      servo_speed(id,-100);//角度制御（－1023～+1023）   
      while(1){
        theta=servo_read(id);
        Serial.println(theta);
        delay(100);
        if(theta<=0){
          servo_speed(id,0);//角度制御（－1023～+1023）
          break;
        }
      }
    }
  }

  servo_torque(id,0);//トルクOFF    
  delay(2000); 
}

//----------------------------------------------
//----------------------------------------------
//----------------------------------------------
void servo_control(int id, int8_t CTRL){

  //1   Velocity Control Mode
  //3   Position Control Mode
  //4   Extended Position Control Mode
  //16   Current-Base Position Control 
  
    cbuf[0]  = (unsigned char)0xFF;     // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;     // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;     // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;     // ヘッダー4
    cbuf[4]  = (unsigned char)id;       // サーボID
    cbuf[5]  = (unsigned char)0x06;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)11;       // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)CTRL;     // 書込データ（1バイト）
      
    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<11;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[11] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[12] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    Serial2.write(cbuf,13);
    delay(20);//時間待機が必須
}


// 関数：トルクON/OFF
void servo_torque(int id, int8_t ON_OFF){
// 1でトルクON，0でトルクオフ

    cbuf[0]  = (unsigned char)0xFF;    // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;    // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;    // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;    // ヘッダー4
    cbuf[4]  = (unsigned char)id;      // サーボID
    cbuf[5]  = (unsigned char)0x06;    // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;    // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;    // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)0x40;    // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;    // メモリ位置（2バイト中，上位バイト）
    cbuf[10]  = (unsigned char)ON_OFF; // 書込データ（１バイト）

    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<11;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[11] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[12] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    Serial2.write(cbuf,13);
    delay(20);//時間待機が必須
}

void servo_position(int id, int32_t POS){
// 角度範囲：0～4095（0度～360度）

    cbuf[0]  = (unsigned char)0xFF;    // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;    // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;    // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;    // ヘッダー4
    cbuf[4]  = (unsigned char)id;      // サーボID
    cbuf[5]  = (unsigned char)0x09;    // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;    // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;    // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)0x74;    // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;    // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&POS);      // 書込データ（4バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(POS>>8)); // 書込データ（4バイト中，上から2バイト目）
    cbuf[12] = (unsigned char)(0x000000ff&(POS>>16));// 書込データ（4バイト中，上から3バイト目）
    cbuf[13] = (unsigned char)(0x000000ff&(POS>>24));// 書込データ（4バイト中，上から4バイト目）
    
    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<14;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[14] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[15] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    Serial2.write(cbuf,16);
    delay(20);//時間待機が必須
}

void servo_speed(int id, int32_t SPE2){
// 速度範囲：-1023 ～ +1023  
// 速度制御時の速度値
// なお，角度制御時の回転速度はPWMで調整
    cbuf[0]  = (unsigned char)0xFF;    // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;    // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;    // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;    // ヘッダー4
    cbuf[4]  = (unsigned char)id;      // サーボID
    cbuf[5]  = (unsigned char)0x09;    // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;    // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;    // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)104;     // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;    // メモリ位置（2バイト中，上位バイト）   
    cbuf[10] = (unsigned char)(0x000000ff&SPE2);        // 書込データ（4バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(SPE2>>8));   // 書込データ（4バイト中，上から2バイト目）
    cbuf[12] = (unsigned char)(0x000000ff&(SPE2>>16));  // 書込データ（4バイト中，上から3バイト目）
    cbuf[13] = (unsigned char)(0x000000ff&(SPE2>>24));  // 書込データ（4バイト中，上から4バイト目）

    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<14;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[14] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[15] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    Serial2.write(cbuf,16);
    delay(20);//時間待機が必須
}

void servo_pwm(int id, int16_t PWM){
// PWM範囲:0 ～ +885
// 角度制御時の速度と関係有り
 
    cbuf[0]  = (unsigned char)0xFF;     // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;     // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;     // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;     // ヘッダー4
    cbuf[4]  = (unsigned char)id;       // サーボID
    cbuf[5]  = (unsigned char)0x07;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)100;      // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&PWM);       // 書込データ（2バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(PWM>>8));  // 書込データ（2バイト中，上から2バイト目）

    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<12;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[12] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[13] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    Serial2.write(cbuf,14);
    delay(20);//時間待機が必須
}

void servo_pgain(int id, int16_t PGA){
// Pgain範囲:0 ～ +16383 (基準：640)
// 角度制御時の速度と関係有り
 
    cbuf[0]  = (unsigned char)0xFF;     // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;     // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;     // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;     // ヘッダー4
    cbuf[4]  = (unsigned char)id;       // サーボID
    cbuf[5]  = (unsigned char)0x07;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)84;       // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&PGA);       // 書込データ（2バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(PGA>>8));  // 書込データ（2バイト中，上から2バイト目）

    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<12;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[12] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[13] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    Serial2.write(cbuf,14);
    delay(20);//時間待機が必須
}

void servo_dgain(int id, int16_t DGA){
// Dgain範囲:0 ～ +16383 (基準：400)
// 角度制御時の速度と関係有り
 
    cbuf[0]  = (unsigned char)0xFF;     // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;     // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;     // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;     // ヘッダー4
    cbuf[4]  = (unsigned char)id;       // サーボID
    cbuf[5]  = (unsigned char)0x07;     // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;     // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x03;     // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)80;       // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;     // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)(0x000000ff&DGA);       // 書込データ（2バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)(0x000000ff&(DGA>>8));  // 書込データ（2バイト中，上から2バイト目）
    
    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<12;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[12] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[13] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    // シリアルデータ送信
    Serial2.write(cbuf,14);
    delay(20);//時間待機が必須
}

int32_t servo_read(int id){
    int flag=0;
    int32_t theta=0;

    cbuf[0]  = (unsigned char)0xFF;    // ヘッダー1
    cbuf[1]  = (unsigned char)0xFF;    // ヘッダー2
    cbuf[2]  = (unsigned char)0xFD;    // ヘッダー3
    cbuf[3]  = (unsigned char)0x00;    // ヘッダー4
    cbuf[4]  = (unsigned char)id;      // サーボID
    cbuf[5]  = (unsigned char)0x07;    // 長さ（2バイト中，下位バイト）
    cbuf[6]  = (unsigned char)0x00;    // 長さ（2バイト中，上位バイト）
    cbuf[7]  = (unsigned char)0x02;    // コマンドの種類を示すバイト
    cbuf[8]  = (unsigned char)132;     // メモリ位置（2バイト中，下位バイト）
    cbuf[9]  = (unsigned char)0x00;    // メモリ位置（2バイト中，上位バイト）
    cbuf[10] = (unsigned char)0x04;    // 書込データ（4バイト中，上から1バイト目）
    cbuf[11] = (unsigned char)0x00;    // 書込データ（4バイト中，上から1バイト目）
    
    // チェックサム（誤り検出符号≒暗号化）
    checksum = 0x0000;
    for(i=0;i<12;i++){
      checksum ^= ( ((uint16_t)cbuf[i]) << 8);
      for(j=0;j<8;j++){
        if(checksum & 0x8000){
          checksum = (checksum << 1) ^ 0x8005; // 生成多項式
        }
        else{
          checksum <<= 1;
        }
      }
    }
    cbuf[12] = (unsigned char)(checksum & 0xFF); //　チェックサム（2バイト中，下位バイト）
    cbuf[13] = (unsigned char)(checksum >> 8);   //　チェックサム（2バイト中，上位バイト）

    //受信バッファクリア
    while (Serial2.available() > 0) {
      ch = Serial2.read();
    }   
    delay(20);//時間待機が必須

    // シリアルデータ送信
    Serial2.write(cbuf,14);

    flag=0;
    while(1){
        if(Serial2.available()>= 15){
            for(i=0;i<15;i++){
              rx[i]=(unsigned char)Serial2.read();   
            }
            theta=(int32_t)((((uint32_t)rx[12])<<24)+(((uint32_t)rx[11])<<16)+(((uint32_t)rx[10])<<8)+((uint32_t)rx[9]<<0));
            flag=1;
        }
        if(flag==1){
          break;
        }
    }
    return theta;
}
