#include <Wire.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）
PCA9685 pwm2 = PCA9685(0x41);   //PCA9685のアドレス指定（A0接続時）

#define SERVOMIN 150            //最小パルス幅 (標準的なサーボパルスに設定)
#define SERVOMAX 600            //最大パルス幅 (標準的なサーボパルスに設定)

void setup() {
 pwm.begin();                   //初期設定 (アドレス0x40用)
 pwm.setPWMFreq(60);            //PWM周期を60Hzに設定 (アドレス0x40用)
 pwm2.begin();                   //初期設定 (アドレス0x41用)
 pwm2.setPWMFreq(60);            //PWM周期を60Hzに設定 (アドレス0x41用)
}

int n=0;

void loop() {
//サンプルソース　１６ｃｈすべてのチャンネルより0度～180度の移動を繰り返します。
  for(int i=0; i<16; i++)
      servo_write(i,n);
  n=n+10;
  if(n>=170)n=0;
  delay(100);  
}
void servo_write(int ch, int ang){ //動かすサーボチャンネルと角度を指定
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～600）に変換
  pwm.setPWM(ch, 0, ang);
  pwm2.setPWM(ch, 0, ang);
  //delay(1);
}
