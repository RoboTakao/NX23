#include "M5CoreS3.h"
#include <Wire.h>          // I2C setting
#include <PCA9685.h>       //for PCA9685
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "1010"
#define CHRX_UUID "1012"
#define CHTX_UUID "1011"

M5Canvas canvas(&CoreS3.Display);

byte joyLX=100, joyLY=100, joyRX=100, joyRY=100, joyLSW, joyRSW, joyLDistance, joyRDistance;

BLEServer* pServer = NULL;
BLECharacteristic* pCharTx = NULL;
BLECharacteristic* pCharRx = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3, srv_CH4 = 4, srv_CH5 = 5, srv_CH6 = 6, srv_CH7 = 7, srv_CH8 = 8; //PCA9685チャンネル 0-8
const uint8_t srv_CH9 = 9, srv_CH10 = 10, srv_CH11 = 11; //PCA9685チャンネル 9-11

const float Pi = 3.141593;

double gyro_angle_x = 0, gyro_angle_z = 0;
float angleX, angleZ;
float acc_angle_x, acc_angle_z;

float interval, preInterval;
float interval_pid, preInterval_pid, T;

int angZero[] = {90,98,98,103,93,95,94,88,106,90,97,95};
int angHome[] = {0,23,23,0,-23,-23,0,-23,-23,-0,23,23};
int ang0[12];
int ang1[12];
int ang_b[12];
char ang_c[12];
float ts=80;  //100msごとに次のステップに移る
float td=5;   //10回で分割

float L1 = 50;
float L2 = 70;
float L0 = 13;

float X0[4] = {-31.8, -31.8, 31.8, 31.8};
float Z0[4] = {31.8, -31.8, 31.8, -31.8};

float H0 = 50; //Height
float Zc = 31.5; //Half Width
float Nee = 3.69;

float Fs = 37; //FWDstep
float Ss = 20; //SideStep
float Dis = 70.7; //Distance
float Adj = 0; //Adjust
float Up = 30; //FootUp
float Tn = 30; //Turn
float Wd = 100; //Width
float Wd_c = Wd*cos(Pi/4);
float TnX = 0.0; //TurnX
float TnY = 0.0; //TurnY
float TnZ = 0.0; //TurnZ
float CenX = 0.0; //CenterX
float CenZ = 0.0; //CenterZ

int eye_x = 0;
int eye_y = 0;

int walk_mode = -1;

// Base Step
int bs_s[11][12]={
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}};
// Base_angle
float Theta[3]={0.0,0.0,0.0};
// Base Head Step
float bh_s[12]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

//PCA9685のアドレス指定
PCA9685 pwm = PCA9685(0x40);

#define SERVOMIN 104            //Min pulse width (12bit 500μs) 
#define SERVOMAX 512            //Max pulse width (12bit 2500μs) 

void servo_write(int ch, int ang){
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //angle（0～180）-> pulse width（150～500）
  pwm.setPWM(ch, 0, ang);
}

void servo_set()
{
  int a[12],b[12];

  for (int j=0; j <=11 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }
  
  for (int k=0; k <= td; k++){

    // PCA9685
      servo_write(srv_CH0,a[0]*float(k)/td+b[0]);
      servo_write(srv_CH1,a[1]*float(k)/td+b[1]);
      servo_write(srv_CH2,a[2]*float(k)/td+b[2]);
      servo_write(srv_CH3,a[3]*float(k)/td+b[3]);
      servo_write(srv_CH4,a[4]*float(k)/td+b[4]);
      servo_write(srv_CH5,a[5]*float(k)/td+b[5]);
      servo_write(srv_CH6,a[6]*float(k)/td+b[6]);
      servo_write(srv_CH7,a[7]*float(k)/td+b[7]);
      servo_write(srv_CH8,a[8]*float(k)/td+b[8]);
      servo_write(srv_CH9,a[9]*float(k)/td+b[9]);
      servo_write(srv_CH10,a[10]*float(k)/td+b[10]);
      servo_write(srv_CH11,a[11]*float(k)/td+b[11]);

      delay(ts/td);
  }
}

void ik(float *X3,float *Y3,float *Z3,float *Xx0,float *Zz0,float *Theta1_deg,float *Theta2_deg,float *Theta3_deg)
{
  float L = sqrt(pow(*X3 - *Xx0,2) + pow(*Z3 - *Zz0,2));
  float Xd2 = L - L0;

  float Theta1_rad = -acos((pow(Xd2,2) + pow(*Y3,2) + pow(L1,2) - pow(L2,2))/(2*L1*sqrt(pow(Xd2,2) + pow(*Y3,2)))) + atan2(*Y3,Xd2);
  float Theta2_rad = atan2(*Y3 - L1 * sin(Theta1_rad),Xd2 - L1 * cos(Theta1_rad)) - Theta1_rad;
  float Theta3_rad = atan2(*Z3 - *Zz0, *X3 - *Xx0);

  *Theta1_deg = Theta1_rad / Pi * 180;
  *Theta2_deg = Theta2_rad / Pi * 180;
  *Theta3_deg = Theta3_rad / Pi * 180;
}

void rot(float *X3, float *Y3, float *Z3, float *XX3, float *YY3, float *ZZ3, float *Tr_X, float *Tr_Y, float *Tr_Z)
{
  float TrX_c = *Tr_X/180*Pi;
  float TrY_c = *Tr_Y/180*Pi;
  float TrZ_c = *Tr_Z/180*Pi;
  
  *X3 = *XX3 * cos(TrZ_c)*cos(TrY_c) + *YY3*(cos(TrZ_c)*sin(TrY_c)*sin(TrX_c) - sin(TrZ_c)*cos(TrX_c)) + *ZZ3*(cos(TrZ_c)*sin(TrY_c)*cos(TrX_c)+sin(TrZ_c)*sin(TrX_c));
  *Y3 = *XX3 * sin(TrZ_c)*cos(TrY_c) + *YY3*(sin(TrZ_c)*sin(TrY_c)*sin(TrX_c) + cos(TrZ_c)*cos(TrX_c)) + *ZZ3*(sin(TrZ_c)*sin(TrY_c)*cos(TrX_c)-cos(TrZ_c)*sin(TrX_c));
  *Z3 = - *XX3 * sin(TrY_c) + *YY3 * cos(TrY_c)*sin(TrX_c) + *ZZ3*cos(TrY_c)*cos(TrX_c);
}

void angle_cul_RF(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  if(*Theta_3 < 0){
    *bss_0 = 180 + int(*Theta_3);
  }
  else{
    *bss_0 = 135 - int(*Theta_3);
  }
  *bss_1 = -int(*Theta_1);
  *bss_2 = int(*Theta_2 - Nee) - 90;
}

void angle_cul_RB(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  if(*Theta_3 > 0){
    *bss_0 = 180 - int(*Theta_3);
  }
  else{
    *bss_0 = -135 - int(*Theta_3);
  }
  *bss_1 = int(*Theta_1);
  *bss_2 = -int(*Theta_2 - Nee) + 90;
}

void angle_cul_LF(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  *bss_0 = 45 - int(*Theta_3);
  *bss_1 = int(*Theta_1);
  *bss_2 = -int(*Theta_2 - Nee) + 90;
}

void angle_cul_LB(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  *bss_0 = -45 - int(*Theta_3);
  *bss_1 = -int(*Theta_1);
  *bss_2 = int(*Theta_2 - Nee) - 90;
}

void center_step()
{
  float ch_s[12] = {-Wd_c-Adj+CenX,H0,Dis+CenZ,-Wd_c-Adj+CenX,H0,-Dis+CenZ,Wd_c+Adj+CenX,H0,Dis+CenZ,Wd_c+Adj+CenX,H0,-Dis+CenZ};

  ik(&ch_s[0],&ch_s[1],&ch_s[2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
  angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][0], &bs_s[0][1], &bs_s[0][2]);

  ik(&ch_s[3],&ch_s[4],&ch_s[5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Back
  angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][3], &bs_s[0][4], &bs_s[0][5]);

  ik(&ch_s[6],&ch_s[7],&ch_s[8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Left Front
  angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][6], &bs_s[0][7], &bs_s[0][8]);

  ik(&ch_s[9],&ch_s[10],&ch_s[11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Back
  angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][9], &bs_s[0][10], &bs_s[0][11]);

  for (int j=0; j <=11 ; j++){
      ang1[j] = angZero[j] + bs_s[0][j];
  }
  servo_set();
}

void forward_step()
{
  float f_s[8][12] = {
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ-Fs  ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ+Fs  ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c-Ss-Adj+CenX,H0-Up,Dis+CenZ-Fs/2,-Wd_c-Ss-Adj+CenX,H0   ,-Dis+CenZ+Fs/2,Wd_c-Ss+Adj+CenX,H0   ,Dis+CenZ-Fs/2,Wd_c-Ss+Adj+CenX,H0   ,-Dis+CenZ-Fs/2},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ     ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ-Fs  ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ-Fs},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ     ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ-Fs  ,Wd_c   +Adj+CenX,H0-Up,-Dis+CenZ+Fs/2},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ     ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ+Fs},
    {-Wd_c+Ss-Adj+CenX,H0   ,Dis+CenZ-Fs/2,-Wd_c+Ss-Adj+CenX,H0   ,-Dis+CenZ-Fs/2,Wd_c+Ss+Adj+CenX,H0-Up,Dis+CenZ-Fs/2,Wd_c+Ss+Adj+CenX,H0   ,-Dis+CenZ+Fs/2},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ-Fs  ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ-Fs  ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ-Fs  ,-Wd_c   -Adj+CenX,H0-Up,-Dis+CenZ+Fs/2,Wd_c   +Adj+CenX,H0   ,Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ}};

  for (int i=0; i <=7 ; i++){
    ik(&f_s[i][0],&f_s[i][1],&f_s[i][2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
    angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][0], &bs_s[i][1], &bs_s[i][2]);

    ik(&f_s[i][3],&f_s[i][4],&f_s[i][5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Back
    angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5]);

    ik(&f_s[i][6],&f_s[i][7],&f_s[i][8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Left Front
    angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][6], &bs_s[i][7], &bs_s[i][8]);

    ik(&f_s[i][9],&f_s[i][10],&f_s[i][11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Back
    angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }

  for (int i=0; i <=7 ; i++){
    for (int j=0; j <=11 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
    //Serial.printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n", bs_s[i][0], bs_s[i][1], bs_s[i][2], bs_s[i][3], bs_s[i][4], bs_s[i][5], bs_s[i][6], bs_s[i][7], bs_s[i][8], bs_s[i][9], bs_s[i][10], bs_s[i][11]);
  servo_set();
  }
}

void left_step()
{
  float Tnp = -Tn/180*Pi;
  float l_s[8][12] = {
    {-Wd_c-Ss-Adj+CenX,H0   ,Dis+CenZ,-Wd_c-Ss-Adj+CenX,H0   ,-Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c-Ss-Adj+CenX,H0-Up,Dis+CenZ,-Wd_c-Ss-Adj+CenX,H0   ,-Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c-Ss-Adj+CenX,H0   ,Dis+CenZ,-Wd_c-Ss-Adj+CenX,H0   ,-Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ,Wd_c   +Adj+CenX,H0-Up,Dis+CenZ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ,Wd_c   +Adj+CenX,H0-Up,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c-Ss-Adj+CenX,H0   ,Dis+CenZ,-Wd_c-Ss-Adj+CenX,H0-Up,-Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,Dis+CenZ,Wd_c-Ss+Adj+CenX,H0   ,-Dis+CenZ}};

  float l_s1[8][12] = {
    {l_s[0][0],l_s[0][1],l_s[0][2],l_s[0][3],l_s[0][4],l_s[0][5],l_s[0][6],l_s[0][7],l_s[0][8],l_s[0][9],l_s[0][10],l_s[0][11]},
    {l_s[1][0]*cos(Tnp*3/4)-l_s[1][2]*sin(Tnp*3/4),l_s[1][1],l_s[1][0]*sin(Tnp*3/4)+l_s[1][2]*cos(Tnp*3/4),l_s[1][3]*cos(-Tnp  /4)-l_s[1][5]*sin(-Tnp  /4),l_s[1][4],l_s[1][3]*sin(-Tnp  /4)+l_s[1][5]*cos(-Tnp  /4),l_s[1][6]*cos(-Tnp  /4)-l_s[1][8]*sin(-Tnp/4),l_s[1][7],l_s[1][6]*sin(-Tnp/4)+l_s[1][8]*cos(-Tnp/4),l_s[1][9]*cos(-Tnp/4)-l_s[1][11]*sin(-Tnp/4),l_s[1][10],l_s[1][9]*sin(-Tnp/4)+l_s[1][11]*cos(-Tnp/4)},
    {l_s[2][0]*cos(Tnp*3/4)-l_s[2][2]*sin(Tnp*3/4),l_s[2][1],l_s[2][0]*sin(Tnp*3/4)+l_s[2][2]*cos(Tnp*3/4),l_s[2][3]*cos(-Tnp  /4)-l_s[2][5]*sin(-Tnp  /4),l_s[2][4],l_s[2][3]*sin(-Tnp  /4)+l_s[2][5]*cos(-Tnp  /4),l_s[2][6]*cos(-Tnp  /4)-l_s[2][8]*sin(-Tnp/4),l_s[2][7],l_s[2][6]*sin(-Tnp/4)+l_s[2][8]*cos(-Tnp/4),l_s[2][9]*cos(-Tnp/4)-l_s[2][11]*sin(-Tnp/4),l_s[2][10],l_s[2][9]*sin(-Tnp/4)+l_s[2][11]*cos(-Tnp/4)},
    {l_s[3][0]*cos(Tnp  /2)-l_s[3][2]*sin(Tnp  /2),l_s[3][1],l_s[3][0]*sin(Tnp  /2)+l_s[3][2]*cos(Tnp  /2),l_s[3][3]*cos(-Tnp  /2)-l_s[3][5]*sin(-Tnp  /2),l_s[3][4],l_s[3][3]*sin(-Tnp  /2)+l_s[3][5]*cos(-Tnp  /2),l_s[3][6]*cos( Tnp  /2)-l_s[3][8]*sin( Tnp/2),l_s[3][7],l_s[3][6]*sin( Tnp/2)+l_s[3][8]*cos( Tnp/2),l_s[3][9]*cos(-Tnp/2)-l_s[3][11]*sin(-Tnp/2),l_s[3][10],l_s[3][9]*sin(-Tnp/2)+l_s[3][11]*cos(-Tnp/2)},
    {l_s[4][0]*cos(Tnp  /2)-l_s[4][2]*sin(Tnp  /2),l_s[4][1],l_s[4][0]*sin(Tnp  /2)+l_s[4][2]*cos(Tnp  /2),l_s[4][3]*cos(-Tnp  /2)-l_s[4][5]*sin(-Tnp  /2),l_s[4][4],l_s[4][3]*sin(-Tnp  /2)+l_s[4][5]*cos(-Tnp  /2),l_s[4][6]*cos( Tnp  /2)-l_s[4][8]*sin( Tnp/2),l_s[4][7],l_s[4][6]*sin( Tnp/2)+l_s[4][8]*cos( Tnp/2),l_s[4][9]*cos(-Tnp/2)-l_s[4][11]*sin(-Tnp/2),l_s[4][10],l_s[4][9]*sin(-Tnp/2)+l_s[4][11]*cos(-Tnp/2)},
    {l_s[5][0]*cos(Tnp  /4)-l_s[5][2]*sin(Tnp  /4),l_s[5][1],l_s[5][0]*sin(Tnp  /4)+l_s[5][2]*cos(Tnp  /4),l_s[5][3]*cos(-Tnp*3/4)-l_s[5][5]*sin(-Tnp*3/4),l_s[5][4],l_s[5][3]*sin(-Tnp*3/4)+l_s[5][5]*cos(-Tnp*3/4),l_s[5][6]*cos( Tnp  /4)-l_s[5][8]*sin( Tnp/4),l_s[5][7],l_s[5][6]*sin( Tnp/4)+l_s[5][8]*cos( Tnp/4),l_s[5][9]*cos( Tnp/4)-l_s[5][11]*sin( Tnp/4),l_s[5][10],l_s[5][9]*sin( Tnp/4)+l_s[5][11]*cos( Tnp/4)},
    {l_s[6][0]*cos(Tnp  /4)-l_s[6][2]*sin(Tnp  /4),l_s[6][1],l_s[6][0]*sin(Tnp  /4)+l_s[6][2]*cos(Tnp  /4),l_s[6][3]*cos(-Tnp*3/4)-l_s[6][5]*sin(-Tnp*3/4),l_s[6][4],l_s[6][3]*sin(-Tnp*3/4)+l_s[6][5]*cos(-Tnp*3/4),l_s[6][6]*cos( Tnp  /4)-l_s[6][8]*sin( Tnp/4),l_s[6][7],l_s[6][6]*sin( Tnp/4)+l_s[6][8]*cos( Tnp/4),l_s[6][9]*cos( Tnp/4)-l_s[6][11]*sin( Tnp/4),l_s[6][10],l_s[6][9]*sin( Tnp/4)+l_s[6][11]*cos( Tnp/4)},
    {l_s[7][0],l_s[7][1],l_s[7][2],l_s[7][3],l_s[7][4],l_s[7][5],l_s[7][6],l_s[7][7],l_s[7][8],l_s[7][9],l_s[7][10],l_s[7][11]}};

  for (int i=0; i <=7 ; i++){
    ik(&l_s1[i][0],&l_s1[i][1],&l_s1[i][2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
    angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][0], &bs_s[i][1], &bs_s[i][2]);

    ik(&l_s1[i][3],&l_s1[i][4],&l_s1[i][5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Back
    angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5]);

    ik(&l_s1[i][6],&l_s1[i][7],&l_s1[i][8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Left Front
    angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][6], &bs_s[i][7], &bs_s[i][8]);

    ik(&l_s1[i][9],&l_s1[i][10],&l_s1[i][11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Back
    angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }

  for (int i=0; i <=7 ; i++){
    for (int j=0; j <=11 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
    //Serial.printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n", bs_s[i][0], bs_s[i][1], bs_s[i][2], bs_s[i][3], bs_s[i][4], bs_s[i][5], bs_s[i][6], bs_s[i][7], bs_s[i][8], bs_s[i][9], bs_s[i][10], bs_s[i][11]);
  servo_set();
  }
}

void right_step()
{
  float Tnp = Tn/180*Pi;
  float r_s[8][12] = {
    {-Wd_c+Ss-Adj+CenX,H0   ,Dis+CenZ,-Wd_c+Ss-Adj+CenX,H0   ,-Dis+CenZ,Wd_c+Ss+Adj+CenX,H0   ,Dis+CenZ,Wd_c+Ss+Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c+Ss-Adj+CenX,H0   ,Dis+CenZ,-Wd_c+Ss-Adj+CenX,H0   ,-Dis+CenZ,Wd_c+Ss+Adj+CenX,H0-Up,Dis+CenZ,Wd_c+Ss+Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c+Ss-Adj+CenX,H0   ,Dis+CenZ,-Wd_c+Ss-Adj+CenX,H0   ,-Dis+CenZ,Wd_c+Ss+Adj+CenX,H0   ,Dis+CenZ,Wd_c+Ss+Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0-Up,Dis+CenZ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ,-Wd_c   -Adj+CenX,H0-Up,-Dis+CenZ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c+Ss-Adj+CenX,H0   ,Dis+CenZ,-Wd_c+Ss-Adj+CenX,H0   ,-Dis+CenZ,Wd_c+Ss+Adj+CenX,H0   ,Dis+CenZ,Wd_c+Ss+Adj+CenX,H0-Up,-Dis+CenZ}};

  float r_s1[8][12] = {
    {r_s[0][0],r_s[0][1],r_s[0][2],r_s[0][3],r_s[0][4],r_s[0][5],r_s[0][6],r_s[0][7],r_s[0][8],r_s[0][9],r_s[0][10],r_s[0][11]},
    {r_s[1][0]*cos(-Tnp/4)-r_s[1][2]*sin(-Tnp/4),r_s[1][1],r_s[1][0]*sin(-Tnp/4)+r_s[1][2]*cos(-Tnp/4),r_s[1][3]*cos(-Tnp/4)-r_s[1][5]*sin(-Tnp/4),r_s[1][4],r_s[1][3]*sin(-Tnp/4)+r_s[1][5]*cos(-Tnp/4),r_s[1][6]*cos( Tnp*3/4)-r_s[1][8]*sin( Tnp*3/4),r_s[1][7],r_s[1][6]*sin( Tnp*3/4)+r_s[1][8]*cos( Tnp*3/4),r_s[1][9]*cos(-Tnp  /4)-r_s[1][11]*sin(-Tnp  /4),r_s[1][10],r_s[1][9]*sin(-Tnp  /4)+r_s[1][11]*cos(-Tnp  /4)},
    {r_s[2][0]*cos(-Tnp/4)-r_s[2][2]*sin(-Tnp/4),r_s[2][1],r_s[2][0]*sin(-Tnp/4)+r_s[2][2]*cos(-Tnp/4),r_s[2][3]*cos(-Tnp/4)-r_s[2][5]*sin(-Tnp/4),r_s[2][4],r_s[2][3]*sin(-Tnp/4)+r_s[2][5]*cos(-Tnp/4),r_s[2][6]*cos( Tnp*3/4)-r_s[2][8]*sin( Tnp*3/4),r_s[2][7],r_s[2][6]*sin( Tnp*3/4)+r_s[2][8]*cos( Tnp*3/4),r_s[2][9]*cos(-Tnp  /4)-r_s[2][11]*sin(-Tnp  /4),r_s[2][10],r_s[2][9]*sin(-Tnp  /4)+r_s[2][11]*cos(-Tnp  /4)},
    {r_s[3][0]*cos( Tnp/2)-r_s[3][2]*sin( Tnp/2),r_s[3][1],r_s[3][0]*sin( Tnp/2)+r_s[3][2]*cos( Tnp/2),r_s[3][3]*cos(-Tnp/2)-r_s[3][5]*sin(-Tnp/2),r_s[3][4],r_s[3][3]*sin(-Tnp/2)+r_s[3][5]*cos(-Tnp/2),r_s[3][6]*cos( Tnp  /2)-r_s[3][8]*sin( Tnp  /2),r_s[3][7],r_s[3][6]*sin( Tnp  /2)+r_s[3][8]*cos( Tnp  /2),r_s[3][9]*cos(-Tnp  /2)-r_s[3][11]*sin(-Tnp  /2),r_s[3][10],r_s[3][9]*sin(-Tnp  /2)+r_s[3][11]*cos(-Tnp  /2)},
    {r_s[4][0]*cos( Tnp/2)-r_s[4][2]*sin( Tnp/2),r_s[4][1],r_s[4][0]*sin( Tnp/2)+r_s[4][2]*cos( Tnp/2),r_s[4][3]*cos(-Tnp/2)-r_s[4][5]*sin(-Tnp/2),r_s[4][4],r_s[4][3]*sin(-Tnp/2)+r_s[4][5]*cos(-Tnp/2),r_s[4][6]*cos( Tnp  /2)-r_s[4][8]*sin( Tnp  /2),r_s[4][7],r_s[4][6]*sin( Tnp  /2)+r_s[4][8]*cos( Tnp  /2),r_s[4][9]*cos(-Tnp  /2)-r_s[4][11]*sin(-Tnp  /2),r_s[4][10],r_s[4][9]*sin(-Tnp  /2)+r_s[4][11]*cos(-Tnp  /2)},
    {r_s[5][0]*cos( Tnp/4)-r_s[5][2]*sin( Tnp/4),r_s[5][1],r_s[5][0]*sin( Tnp/4)+r_s[5][2]*cos( Tnp/4),r_s[5][3]*cos( Tnp/4)-r_s[5][5]*sin( Tnp/4),r_s[5][4],r_s[5][3]*sin( Tnp/4)+r_s[5][5]*cos( Tnp/4),r_s[5][6]*cos( Tnp  /4)-r_s[5][8]*sin( Tnp  /4),r_s[5][7],r_s[5][6]*sin( Tnp  /4)+r_s[5][8]*cos( Tnp  /4),r_s[5][9]*cos(-Tnp*3/4)-r_s[5][11]*sin(-Tnp*3/4),r_s[5][10],r_s[5][9]*sin(-Tnp*3/4)+r_s[5][11]*cos(-Tnp*3/4)},
    {r_s[6][0]*cos( Tnp/4)-r_s[6][2]*sin( Tnp/4),r_s[6][1],r_s[6][0]*sin( Tnp/4)+r_s[6][2]*cos( Tnp/4),r_s[6][3]*cos( Tnp/4)-r_s[6][5]*sin( Tnp/4),r_s[6][4],r_s[6][3]*sin( Tnp/4)+r_s[6][5]*cos( Tnp/4),r_s[6][6]*cos( Tnp  /4)-r_s[6][8]*sin( Tnp  /4),r_s[6][7],r_s[6][6]*sin( Tnp  /4)+r_s[6][8]*cos( Tnp  /4),r_s[6][9]*cos(-Tnp*3/4)-r_s[6][11]*sin(-Tnp*3/4),r_s[6][10],r_s[6][9]*sin(-Tnp*3/4)+r_s[6][11]*cos(-Tnp*3/4)},
    {r_s[7][0],r_s[7][1],r_s[7][2],r_s[7][3],r_s[7][4],r_s[7][5],r_s[7][6],r_s[7][7],r_s[7][8],r_s[7][9],r_s[7][10],r_s[7][11]}};

  for (int i=0; i <=7 ; i++){
    ik(&r_s1[i][0],&r_s1[i][1],&r_s1[i][2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
    angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][0], &bs_s[i][1], &bs_s[i][2]);

    ik(&r_s1[i][3],&r_s1[i][4],&r_s1[i][5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Back
    angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5]);

    ik(&r_s1[i][6],&r_s1[i][7],&r_s1[i][8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Left Front
    angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][6], &bs_s[i][7], &bs_s[i][8]);

    ik(&r_s1[i][9],&r_s1[i][10],&r_s1[i][11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Back
    angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }

  for (int i=0; i <=7 ; i++){
    for (int j=0; j <=11 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
    //Serial.printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n", bs_s[i][0], bs_s[i][1], bs_s[i][2], bs_s[i][3], bs_s[i][4], bs_s[i][5], bs_s[i][6], bs_s[i][7], bs_s[i][8], bs_s[i][9], bs_s[i][10], bs_s[i][11]);
  servo_set();
  }
}

void back_step()
{
  float b_s[8][12] = {
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ     ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ-Fs  ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ+Fs},
    {-Wd_c+Ss-Adj+CenX,H0   ,Dis+CenZ+Fs/2,-Wd_c+Ss-Adj+CenX,H0   ,-Dis+CenZ+Fs/2,Wd_c+Ss+Adj+CenX,H0   ,Dis+CenZ-Fs/2,Wd_c+Ss+Adj+CenX,H0-Up,-Dis+CenZ+Fs/2},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ+Fs  ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ+Fs  ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0-Up,Dis+CenZ-Fs/2,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ+Fs  ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ-Fs  ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ+Fs  ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ},
    {-Wd_c-Ss-Adj+CenX,H0   ,Dis+CenZ-Fs/2,-Wd_c-Ss-Adj+CenX,H0-Up,-Dis+CenZ+Fs/2,Wd_c-Ss+Adj+CenX,H0   ,Dis+CenZ+Fs/2,Wd_c-Ss+Adj+CenX,H0   ,-Dis+CenZ+Fs/2},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ     ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ     ,Wd_c   +Adj+CenX,H0   ,Dis+CenZ+Fs  ,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ+Fs},
    {-Wd_c   -Adj+CenX,H0   ,Dis+CenZ     ,-Wd_c   -Adj+CenX,H0   ,-Dis+CenZ     ,Wd_c   +Adj+CenX,H0-Up,Dis+CenZ-Fs/2,Wd_c   +Adj+CenX,H0   ,-Dis+CenZ+Fs}};

  for (int i=0; i <=7 ; i++){
    ik(&b_s[i][0],&b_s[i][1],&b_s[i][2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
    angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][0], &bs_s[i][1], &bs_s[i][2]);

    ik(&b_s[i][3],&b_s[i][4],&b_s[i][5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Back
    angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5]);

    ik(&b_s[i][6],&b_s[i][7],&b_s[i][8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Left Front
    angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][6], &bs_s[i][7], &bs_s[i][8]);

    ik(&b_s[i][9],&b_s[i][10],&b_s[i][11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Back
    angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }

  for (int i=0; i <=7 ; i++){
    for (int j=0; j <=11 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
    //Serial.printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n", bs_s[i][0], bs_s[i][1], bs_s[i][2], bs_s[i][3], bs_s[i][4], bs_s[i][5], bs_s[i][6], bs_s[i][7], bs_s[i][8], bs_s[i][9], bs_s[i][10], bs_s[i][11]);
  servo_set();
  }
}

void rot_head_step(float TnX, float TnY, float TnZ)
{
  float ch_s[12] = {-Wd_c-Adj+CenX,H0,Dis+CenZ,-Wd_c-Adj+CenX,H0,-Dis+CenZ,Wd_c+Adj+CenX,H0,Dis+CenZ,Wd_c+Adj+CenX,H0,-Dis+CenZ};
  
  rot(&bh_s[0], &bh_s[1], &bh_s[2], &ch_s[0], &ch_s[1], &ch_s[2], &TnX, &TnY, &TnZ);
  rot(&bh_s[3], &bh_s[4], &bh_s[5], &ch_s[3], &ch_s[4], &ch_s[5], &TnX, &TnY, &TnZ);
  rot(&bh_s[6], &bh_s[7], &bh_s[8], &ch_s[6], &ch_s[7], &ch_s[8], &TnX, &TnY, &TnZ);
  rot(&bh_s[9], &bh_s[10], &bh_s[11], &ch_s[9], &ch_s[10], &ch_s[11], &TnX, &TnY, &TnZ);
  
  ik(&bh_s[0],&bh_s[1],&bh_s[2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
  angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][0], &bs_s[0][1], &bs_s[0][2]);

  ik(&bh_s[3],&bh_s[4],&bh_s[5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Back
  angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][3], &bs_s[0][4], &bs_s[0][5]);

  ik(&bh_s[6],&bh_s[7],&bh_s[8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Left Front
  angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][6], &bs_s[0][7], &bs_s[0][8]);

  ik(&bh_s[9],&bh_s[10],&bh_s[11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Back
  angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][9], &bs_s[0][10], &bs_s[0][11]);
  
  for (int j=0; j <=11 ; j++){
    ang1[j] = angZero[j] + bs_s[0][j];
  }
  servo_set();
}

void eye_open()
{
  canvas.fillCircle(80+eye_x,100+eye_y,46,TFT_BLUE);
  canvas.fillCircle(80+eye_x,100+eye_y,30,TFT_CYAN);
  canvas.fillCircle(80+eye_x,100+eye_y,20,TFT_NAVY);
  canvas.fillCircle(100+eye_x,80+eye_y,12,TFT_WHITE);

  canvas.fillCircle(240+eye_x,100+eye_y,46,TFT_BLUE);
  canvas.fillCircle(240+eye_x,100+eye_y,30,TFT_CYAN);
  canvas.fillCircle(240+eye_x,100+eye_y,20,TFT_NAVY);
  canvas.fillCircle(260+eye_x,80+eye_y,12,TFT_WHITE);

  canvas.fillRect(34+eye_x,54+eye_y,92,20,TFT_BLACK);
  canvas.fillRect(194+eye_x,54+eye_y,92,20,TFT_BLACK);
}

void eye_clear()
{
  canvas.fillScreen(TFT_BLACK);
}

void eye_close()
{
  canvas.fillRoundRect(40+eye_x,90+eye_y,80,20,8,TFT_BLUE);
  canvas.fillRoundRect(200+eye_x,90+eye_y,80,20,8,TFT_BLUE);
}

void face_open_eye(void *pvParameters){
  int k = 0;
  while(1)
  {
    eye_clear();
    eye_open();
    CoreS3.Display.startWrite(); 
    canvas.pushSprite(0,0);
    CoreS3.Display.endWrite();

    if(k == 10){
      eye_clear();
      eye_close();
      CoreS3.Display.startWrite(); 
      canvas.pushSprite(0,0);
      CoreS3.Display.endWrite();
      delay(100);
      k = 0;
    }

    k += 1;

    delay(200);
  }
}

void imu_angle_cal() {
    auto imu_update = M5.Imu.update();
    if (imu_update) {
        auto data = M5.Imu.getImuData();

        //前回計算した時から今までの経過時間を算出
        interval = millis() - preInterval;
        preInterval = millis();

        acc_angle_x = atan2(data.accel.y, data.accel.z) * 180 / PI -90;
        acc_angle_z = atan2(data.accel.x, data.accel.y) * 180 / PI;

        //数値積分
        gyro_angle_x += data.gyro.x * (interval * 0.001);
        gyro_angle_z += data.gyro.z * (interval * 0.001);

        //相補フィルター
        angleX = (0.85 * gyro_angle_x) + (0.15 * acc_angle_x);
        angleZ = (0.85 * gyro_angle_z) + (0.15 * acc_angle_z);
        gyro_angle_x = angleX;
        gyro_angle_z = angleZ;

        //Serial.printf("angleX:%f angleZ:%f\r\n", angleX, angleZ);
    }
}

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    if (value.length()>0) {
      joyLX=value[0];
      joyLY=value[1];
      joyRX=value[2];
      joyRY=value[3];
      joyLSW=value[4];
      joyRSW=value[5];
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setupBLE() {
  BLEDevice::init("NX23_M5CoreS3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharTx = pService->createCharacteristic(CHTX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharRx = pService->createCharacteristic(CHRX_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
  pCharRx ->setCallbacks(new MyCallbacks());
  pCharTx->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void checkBLE() {
  // notify changed value
  if (deviceConnected) {
      pCharTx->setValue((uint8_t*)&value, 6);
      pCharTx->notify();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}

void setup() 
{ 
  auto cfg = M5.config();
  CoreS3.begin(cfg);

  canvas.createSprite(320, 240);

  setupBLE();
  
  Wire.begin(2, 1); //SDA-2, SCL-1

  pwm.begin();                   //initial setting (for 0x40) (PCA9685)
  pwm.setPWMFreq(50);            //PWM 50Hz (for 0x40) (PCA9685)

  //initial servo angle
  for (int j=0; j <=11 ; j++){
      ang0[j] = angZero[j] + angHome[j];
  }
  for (int j=0; j <=11 ; j++){
      ang1[j] = angZero[j] + angHome[j];
  }

  xTaskCreatePinnedToCore(face_open_eye, "face_open_eye", 4096, NULL, 5, NULL, 1);

  center_step();
} 
 
void loop()  
{   
    float eX, eZ, eX_pre = 0, eZ_pre = 0;
    float deX, deZ;
    float ieX = 0, ieZ = 0;
    float uX, uZ;

    float Kp = 0.7;
    float Kd = 0.01;
    float Ki = 1.5;
 
    interval_pid = millis() - preInterval_pid;
    preInterval_pid = millis();
    T = interval_pid * 0.001;

    checkBLE();

    imu_angle_cal();

    eye_y = map(joyLY, 200, 0, -50, 50);
    eye_x = map(joyLX, 200, 0, -50, 50);

    if (joyRSW == 1)
    {
      TnX = 0;
      TnZ = 0;
      walk_mode += 1;
      for (int i=0; i <=9 ; i++){
        imu_angle_cal();
        delay(100);
      }
      if(walk_mode == 2) walk_mode = -1;
    }

    if(walk_mode == -1)
    {
      if (joyRY > 150)
      {
        forward_step();
      }

      if (joyRY < 50)
      {
        back_step();
      }

      if (joyRX > 150)
      {
        eye_x = 50;
        left_step();
        eye_x = 0;
      }

      if (joyRX < 50)
      {
        eye_x = -50;
        right_step();
        eye_x = 0;
      }

      if (((joyRY <= 150) && (joyRY >= 50)) && ((joyRX <= 150) && (joyRX >= 50)))
      {
        center_step();
      }
    }

    if (walk_mode == 0)
    {
        TnX = map(joyLY, 200, 0, -20, 20);
        TnY = map(joyLX, 0, 200, -20, 20);
        TnZ = map(joyRX, 200, 0, -20, 20);
        CenZ = map(joyRY, 200, 0, -20, 20);
        rot_head_step(TnX, TnY, TnZ);
    } 

    if (walk_mode == 1)
    {
        float limit_angle = 30;

        eX = - angleX;
        eZ = - angleZ;
        deX = (eX - eX_pre)/T;
        deZ = (eZ - eZ_pre)/T;
        ieX = ieX + (eX + eX_pre)*T/2;
        ieX = ieZ + (eX + eZ_pre)*T/2;
        uX = Kp*eX + Ki*ieX + Kd*deX;
        uZ = Kp*eZ + Ki*ieZ + Kd*deZ;
        eX_pre = eX;
        eZ_pre = eZ;

        TnX = TnX + uX;
        TnZ = TnZ + uZ;
        if (TnX >= limit_angle) TnX = limit_angle; 
        if (TnX <= -limit_angle) TnX = -limit_angle;
        if (TnZ >= limit_angle) TnZ = limit_angle; 
        if (TnZ <= -limit_angle) TnZ = -limit_angle;
        rot_head_step(TnX, TnY, TnZ);
    } 
} 