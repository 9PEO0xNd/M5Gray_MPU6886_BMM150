//#include <SD.h> // SD-Updaterを使うとき
#include <M5Unified.h>
//#include <M5StackUpdater.h> // SD-Updaterを使うとき
#include <BMM150class.h>
#include "src/quaternionFilters.h"

#define MAHONY
//#define MADGWICK

//static M5GFX lcd;
auto &lcd = M5.Display; // for unified
static M5Canvas compass(&lcd);     // オフスクリーン描画用バッファ
static M5Canvas base(&compass);    // オフスクリーン描画用バッファ

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

int AVERAGENUM_INIT = 256;
float init_gyroX = 0.0F;
float init_gyroY = 0.0F;
float init_gyroZ = 0.0F;

float magnetX = 0.0F;
float magnetY = 0.0F;
float magnetZ = 0.0F;

//for hard iron correction
float magoffsetX = 0.0F;
float magoffsetY = 0.0F;
float magoffsetZ = 0.0F;

//for soft iron correction
float magscaleX = 0.0F;
float magscaleY = 0.0F;
float magscaleZ = 0.0F;

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

BMM150class bmm150;

uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;
static uint32_t fsec, psec;
static size_t fps = 0, frame_count = 0;

void initGyro() {
  lcd.clear();        // 黒で塗り潰し
  lcd.setCursor(0, 0);
  lcd.print("begin gyro calibration");

  for (int i = 0;i < AVERAGENUM_INIT;i++) {
    M5.Imu.getGyro(&gyroX,&gyroY,&gyroZ);
    init_gyroX += gyroX;
    init_gyroY += gyroY;
    init_gyroZ += gyroZ;
    delay(5);
  }
  init_gyroX /= AVERAGENUM_INIT;
  init_gyroY /= AVERAGENUM_INIT;
  init_gyroZ /= AVERAGENUM_INIT;
}

void setup()
{
  // put your setup code here, to run once:
  auto cfg = M5.config();
  cfg.internal_imu = true;

  M5.begin(cfg);
  //checkSDUpdater(SD); // SD-Updaterを使うとき
  
//  lcd.init();
// 回転方向を 0～3 の4方向から設定します。(4～7を使用すると上下反転になります。)
  lcd.setRotation(1);

  if (bmm150.Init() != BMM150_OK)
  {
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.print("BMM150 init failed");
    for (;;)
    {
        delay(100);
    }
  }

  initGyro();
  bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
  bmm150.getMagnetScale(&magscaleX, &magscaleY, &magscaleZ);
  bmm150.getMagnetData(&magnetX, &magnetY, &magnetZ);

// 必要に応じてカラーモードを設定します。（初期値は16）
// 16の方がSPI通信量が少なく高速に動作しますが、赤と青の諧調が5bitになります。
  lcd.setColorDepth(16);
// clearまたはfillScreenで画面全体を塗り潰します。
  lcd.clear();        // 黒で塗り潰し
// スプライト（オフスクリーン）への描画も同様の描画関数が使えます。
// 最初にスプライトの色深度をsetColorDepthで指定します。（省略した場合は16として扱われます。）
  compass.setColorDepth(16);
  base.setColorDepth(16);
// createSpriteで幅と高さを指定してメモリを確保します。
// 消費するメモリは色深度と面積に比例します。大きすぎるとメモリ確保に失敗しますので注意してください。
  compass.createSprite(180,180);
  base.createSprite(120,120);

// base airplane
  base.drawLine( 52, 36, 52, 14, TFT_WHITE); //front L
  base.drawLine(  2, 62, 52, 36, TFT_WHITE); //wing L
  base.drawLine(  4, 76,  2, 62, TFT_WHITE); //wing L
  base.drawLine( 52, 64,  4, 76, TFT_WHITE); //wing L
  base.drawLine( 52, 82, 52, 64, TFT_WHITE); //body L
  base.drawLine( 36,104, 54, 96, TFT_WHITE); //
  base.drawLine( 36,114, 36,104, TFT_WHITE);
  base.drawLine( 58,110, 36,114, TFT_WHITE);
  base.drawLine( 60,116, 58,110, TFT_WHITE);
  base.drawLine( 62,110, 60,116, TFT_WHITE);
  base.drawLine( 84,114, 62,110, TFT_WHITE);
  base.drawLine( 84,104, 84,114, TFT_WHITE);
  base.drawLine( 66, 96, 84,104, TFT_WHITE);
  base.drawLine( 68, 64, 68, 82, TFT_WHITE); //body R
  base.drawLine(116, 76, 68, 64, TFT_WHITE); //wing R
  base.drawLine(118, 62,116, 76, TFT_WHITE); //wing R
  base.drawLine( 68, 36,118, 62, TFT_WHITE); //wing R
  base.drawLine( 68, 14, 68, 36, TFT_WHITE);
  base.drawLine( 54, 96, 52, 82, TFT_WHITE);
  base.drawLine( 66, 96, 68, 82, TFT_WHITE);
  base.drawLine( 52, 14, 54,  4, TFT_WHITE);
  base.drawLine( 68, 14, 66,  4, TFT_WHITE);
  base.drawLine( 66,  4, 60,  0, TFT_WHITE); //head
  base.drawLine( 54,  4, 60,  0, TFT_WHITE); //head
  
// 作成したスプライトはpushSpriteで任意の座標に出力できます。

}

void compassplot(float a) {
  int ang;
  a = 360.0 - a;
  compass.setTextDatum(middle_center);
  compass.setFont(&fonts::Font2);
  compass.fillScreen(0); // fill black

  for (ang=0 ; ang<36 ; ang++) {
    compass.drawLine(90+80*sin((a+ang*10)/180.0*PI),90-80*cos((a+ang*10)/180.0*PI),
  90+90*sin((a+ang*10)/180.0*PI), 90-90*cos((a+ang*10)/180.0*PI), TFT_WHITE); //0
    compass.setTextSize(1.5);
    if (ang==0) {
      compass.drawString("N", 90+72*sin((a+  0)/180.0*PI), 90-72*cos((a+  0)/180.0*PI)); //0
    } else if (ang==9) {
      compass.drawString("E", 90+72*sin((a+ 90)/180.0*PI), 90-72*cos((a+ 90)/180.0*PI)); //90
    } else if (ang==18) {
      compass.drawString("S", 90+72*sin((a+180)/180.0*PI), 90-72*cos((a+180)/180.0*PI)); //180
    } else if (ang==27) {
      compass.drawString("W", 90+72*sin((a+270)/180.0*PI), 90-72*cos((a+270)/180.0*PI)); //270
    } else if ((ang%3)==0) {
      compass.setTextSize(1);
      compass.drawNumber(ang, 90+72*sin((a+ang*10)/180.0*PI), 90-72*cos((a+ang*10)/180.0*PI));
    }
  }
  compass.fillTriangle(224-70,  56-30, 228-70,  47-30, 233-70,  52-30, TFT_WHITE);// 45
  compass.fillTriangle(224-70, 184-30, 233-70, 188-30, 228-70, 193-30, TFT_WHITE);//135
  compass.fillTriangle( 96-70, 184-30,  92-70, 193-30,  87-70, 188-30, TFT_WHITE);//225
  compass.fillTriangle( 96-70,  56-30,  87-70,  52-30,  92-70,  47-30, TFT_WHITE);//315
  
// 作成したスプライトはpushSpriteで任意の座標に出力できます。
  base.pushSprite(30,30,0); // (x,y)=((180-120)/2,(180-120)/2)  compassに出力する。透過色あり
  compass.pushSprite(70,30); // (x,y)=((320-180)/2,(240-180)/2) lcdに出力する。透過色なし

}

void loop()
{
  // put your main code here, to run repeatedly:
  M5.update();
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);
  gyroX = gyroX - init_gyroX;
  gyroY = gyroY - init_gyroY;
  gyroZ = gyroZ - init_gyroZ;
  M5.Imu.getAccel(&accX, &accY, &accZ);
  
  bmm150.getMagnetData(&magnetX, &magnetY, &magnetZ);
  magnetX = (magnetX - magoffsetX) * magscaleX;
  magnetY = (magnetY - magoffsetY) * magscaleY;
  magnetZ = (magnetZ - magoffsetZ) * magscaleZ;

  float head_dir = atan2(magnetX, magnetY);
  if(head_dir < 0)
    head_dir += 2*PI;
  if(head_dir > 2*PI)
    head_dir -= 2*PI;

    head_dir *= RAD_TO_DEG;
    
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);//0.09
  lastUpdate = Now;

#ifdef MADGWICK
  MadgwickQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                           gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD,
                           -magnetX, magnetY, -magnetZ, deltat);
#endif

#ifdef MAHONY
  MahonyQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                           gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD,
                           -magnetX, magnetY, -magnetZ, deltat);
#endif

  yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                                          *(getQ() + 3)),
              *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
  pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                                            *(getQ() + 2)));
  roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
                                                     *(getQ() + 3)),
               *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
  yaw = -0.5*M_PI-yaw;
  if(yaw < 0)
    yaw += 2*M_PI;
  if(yaw > 2*M_PI)
    yaw -= 2*M_PI;
  pitch *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;

#ifdef MAG
  Serial.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", magnetX, magnetY, magnetZ);
#endif
#ifdef ANGLE
  Serial.printf(" %5.2f  \r\n", head_dir);
#endif

  compassplot(yaw);
  // for processing display
  Serial.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", roll, pitch, yaw); // to processing

  lcd.fillTriangle(160,  29, 157,  20, 163,  20, TFT_WHITE);//  0
  lcd.fillTriangle(251, 120, 260, 117, 260, 123, TFT_WHITE);// 90
  lcd.fillTriangle(160, 211, 163, 220, 157, 220, TFT_WHITE);//180
  lcd.fillTriangle( 69, 120,  60, 123,  60, 117, TFT_WHITE);//270

  char text_string[100];
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("MAG X-Y", 20, 35);
  sprintf(text_string, "%5.1f   ", head_dir);
  lcd.drawString(text_string, 30, 50);
  lcd.drawString("Heading", 20, 65);
  sprintf(text_string, "%5.1f   ", yaw);
  lcd.drawString(text_string, 30, 80);
  sprintf(text_string, "fps:%d   ", fps);
  lcd.drawString(text_string, 270, 215);

  ++frame_count;
  uint32_t fsec = m5gfx::millis() / 1000;
  if (psec != fsec)
  {
    psec = fsec;
    fps = frame_count;
    frame_count = 0;
  }

  lcd.setCursor(40, 230);
  lcd.printf("BTN_A:CAL ");

  if(M5.BtnA.wasPressed())
  {
    lcd.clear();        // 黒で塗り潰し
    lcd.setCursor(0, 0);
    lcd.print("begin calibration in 3 seconds");
    delay(3000);
    lcd.setCursor(0, 10);
    lcd.print("Flip + rotate core calibration");
    bmm150.bmm150_calibrate(10000);
    delay(100);

    bmm150.Init();
    bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
    bmm150.getMagnetScale(&magscaleX, &magscaleY, &magscaleZ);
    lcd.clear();        // 黒で塗り潰し
  }
}
