/* M5RCAモジュールで縞表示デモ
 *　　縞種類　矩形波、正弦波、三角波
 *　　縞本数はAmgle8のボリュームで行う、縞角度もボリュームで設定（縞が傾いたらどうなるかのデモ）
 *   ※縞本数が36本あたりから描画がおかしい（キャンバスの貼り付け回数が多いからかも。実際にはもう少しちゃんと作る必要ある）
 *     stripeNumをint、floatへのキャスト　など試したが変わらず。
 *
 *　　テレモロ実装 タイマーでON/OFFを切り替える
 *    テレモロ周期をボリュームで設定
 *    
 *   描画系を最優先で実行できるようCOre1の通常loop内で実施　(Core1はWDGタイマクリア用にdelay入れる必要あるので) 
 * 　それほど速度必要ではない、外部からの入力はCore0で動作
 * 　※タスク間の変数の変数やり取りは今回は気にせず作った。メモリの読み出しだけならいいのかも。queueで送ったほうが確実。
 *
*/

#include <Arduino.h>
#include <SD.h>
#include <SPIFFS.h>

// If you use Module RCA, write this.
#include <M5ModuleRCA.h>

// Include this to enable the M5 global instance.
#include <M5Unified.h>

#include "M5_ANGLE8.h"
#define telemoro_us 62500    //テレモロ周期

/* タイマー割込み用 */
hw_timer_t * timer = NULL;                                  //ハードウエアタイマ
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;       //ミューテックス  変数に同時アクセスしないようにする
volatile SemaphoreHandle_t semaphore;                       //セマフォ　割込みのタイミング同期に使う
volatile uint32_t timerCounter = 0;                         //割込み回数
uint32_t timerus = telemoro_us;                             //割込み周期

/* lcd描画用mutex */
portMUX_TYPE lcdMutex = portMUX_INITIALIZER_UNLOCKED;       //ボタンタスクとI2Cタスクから画面描画するので排他制御用にMutex使用

/* ブラウン管表示用のスプライト */
M5Canvas crtCanvasRect;      //矩形波
M5Canvas crtCanvasSin;       //正弦波
M5Canvas crtCanvasTri;       //三角波

/* Angle8用 */
M5_ANGLE8 angle8;
uint16_t adc_v[8];

int rotateAngle = 0;
int stripeNum = 1;
int mode = 0;
volatile bool telemolo = false;
int  preStripeNum;
int preRoteAngle; 

uint32_t fpsCount = 0;            //fps表示用
uint32_t fpsSec = 0;              //fps表示用

/* timer interrupt */
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);              //ミューテックスを利用して排他制御
  telemolo = !telemolo;
  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(semaphore, NULL);         //セマフォを開放  1フレーム周期で動作させるためメインルーチンでセマフォ解放待ちさせる
}

/* function prototype */
void checkButton(void* arg);
void checkI2C(void* arg);
void createWaveData();
void drawRectStripe(float sNum, float rAngle);
void drawSineStripe(float sNum, float rAngle);
void drawTriangleStripe(float sNum, float rAngle);
void drawText(String s, int x, int y, int fColor, int bColor);
void drawText(String s, int x, int y);

void setup(void)
{
  auto cfg = M5.config();
  cfg.external_display.module_rca     = true;  // default=true. use ModuleRCA VideoOutput

  #if defined ( __M5GFX_M5MODULERCA__ ) // setting for Module RCA.
  cfg.module_rca.logical_width  = 120;
  cfg.module_rca.logical_height = 480;
  cfg.module_rca.output_width   = 120;
  cfg.module_rca.output_height  = 480;
  cfg.module_rca.signal_type    = M5ModuleRCA::signal_type_t::NTSC;     //  NTSC / NTSC_J / PAL_M / PAL_N
  cfg.module_rca.use_psram      = M5ModuleRCA::use_psram_t::psram_no_use; // psram_no_use / psram_half_use
  cfg.module_rca.pin_dac        = GPIO_NUM_26;
  cfg.module_rca.output_level   = 128;
  #endif

  // begin M5Unified.
  M5.begin(cfg);
  angle8.begin(ANGLE8_I2C_ADDR);
  M5.Display.setRotation(3);

  M5.Displays(1).setColorDepth(16);
  // crtCanvas.setColorDepth(lgfx::color_depth_t::grayscale_8bit);
  // M5.Displays(1).setColorDepth(lgfx::color_depth_t::grayscale_8bit);

  crtCanvasRect.createSprite(1,480);
  crtCanvasRect.setColorDepth(16);
  crtCanvasSin.createSprite(1,480); 
  crtCanvasSin.setColorDepth(16); 
  crtCanvasTri.createSprite(1,480); 
  crtCanvasTri.setColorDepth(16); 
  createWaveData();                               //Canvasに1画面分の波形描画

  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextSize(2);

  /* timer setting */
  semaphore = xSemaphoreCreateBinary();           // バイナリセマフォ作成
  timer = timerBegin(0, 80, true);                // タイマ作成 80MHzを80分周
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timerus, true);          //タイマ周期設定
  //timerAlarmEnable(timer);                      //スイッチでタイマースタートする

  M5.Display.clear(TFT_BLACK);
  drawText("Rect Wave", 5, 10, TFT_WHITE, TFT_BLACK);
  
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setCursor(5, 130);
  M5.Display.printf("stripe:%02d\n",stripeNum);

  M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Display.setCursor(5, 180);
  M5.Display.printf("angle:%02d", -rotateAngle);
  
  if (angle8.getDigitalInput()){
    drawText("Telemolo!", 5, 80, TFT_RED, TFT_BLACK);
  }  
  /* check IO task start */
  xTaskCreatePinnedToCore(checkButton, "checkButton", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(checkI2C, "checkI2C", 2048, NULL, 1, NULL, 0);
}

void loop(void)
{
  if (telemolo) {
    M5.Displays(1).clear(TFT_BLACK);
  } else {
    switch (mode)
    {
      case 0:
        drawRectStripe((float)stripeNum, rotateAngle);
        break;
      case 1:
        drawSineStripe((float)stripeNum, rotateAngle);
        break;
      case 2:
        drawTriangleStripe((float)stripeNum, rotateAngle);
        break;
      default:
        break;
    }
  }

  fpsCount++;                              //Count Frame rate 
  if (fpsSec != millis() / 1000) {
    fpsSec = millis() / 1000;
    Serial.printf("fps:%d / %.2fmsec\n",fpsCount, 1000.0f/fpsCount);
    fpsCount = 0;    
  }
}

void checkButton(void* arg){
  while(1){
    M5.update(); 
    if (M5.BtnA.wasPressed()){
      mode = 0;
      drawText("Rect Wave", 5, 10, TFT_WHITE, TFT_BLACK);
    }
    else if (M5.BtnB.wasPressed()){
      mode = 1;
      drawText("Sin Wave", 5, 10, TFT_MAGENTA, TFT_BLACK);
    } 
    else if (M5.BtnC.wasPressed()){
      mode =2;
      drawText("Tri Wave", 5, 10,TFT_CYAN, TFT_BLACK);
    }
    vTaskDelay(10); 
  } 
}

void checkI2C(void* arg){
  while(1){
    if (angle8.getDigitalInput()){
      if (!timerAlarmEnabled(timer)){
        timerAlarmEnable(timer);
        drawText("Telemo!", 5, 80, TFT_RED, TFT_BLACK);
      }
        portENTER_CRITICAL(&lcdMutex);
        M5.Display.setTextColor(TFT_RED, TFT_BLACK);
        M5.Display.setCursor(220, 80);
        M5.Display.printf("%03d\n", 60*1000000/4/2/timerus);  //timer_us = (60/bpm/4)/2*1000000
        portEXIT_CRITICAL(&lcdMutex);
    } else {
      timerAlarmDisable(timer);
      drawText("", 5, 80, TFT_BLACK, TFT_BLACK);  
      telemolo = false;
    }

    for (int i=0; i<3; i++){
      adc_v[i] = angle8.getAnalogInput(i, _12bit);
    }
    stripeNum = adc_v[0]/120 +1;
    rotateAngle = -adc_v[1] * 45.0f / 4096.0f;
    timerus = adc_v[2] * 50 + 40000;
    timerAlarmWrite(timer, timerus, true);

    if (stripeNum != preStripeNum){
      portENTER_CRITICAL(&lcdMutex);
        M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
        M5.Display.setCursor(5, 130);
        M5.Display.printf("stripe:%02d\n",stripeNum);
      portEXIT_CRITICAL(&lcdMutex);
    }

    if (rotateAngle != preRoteAngle){
      portENTER_CRITICAL(&lcdMutex);
        M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
        M5.Display.setCursor(5, 180);
        M5.Display.printf("angle:%02d", -rotateAngle);
      portEXIT_CRITICAL(&lcdMutex);
    }

    preStripeNum = stripeNum;
    preRoteAngle = rotateAngle;
   
    vTaskDelay(10); 
  } 
}

/* 1画面分の波形を作っておく */
void createWaveData(){
  crtCanvasRect.fillRect(0, 0*crtCanvasRect.height()/4, M5.Displays(1).width(), crtCanvasRect.height()/4, TFT_BLACK);
  crtCanvasRect.fillRect(0, 1*crtCanvasRect.height()/4, M5.Displays(1).width(), 2*crtCanvasRect.height()/4, TFT_WHITE);
  crtCanvasRect.fillRect(0, 3*crtCanvasRect.height()/4, M5.Displays(1).width(), crtCanvasRect.height()/4, TFT_BLACK);
  crtCanvasRect.setPivot(0, 0);

  for (int i=0; i<crtCanvasSin.height(); i++){
    uint8_t c = 255.0f * (-cos(2*PI*(float)(i)/(float)crtCanvasSin.height()) +1.0)/2.0f;
    uint32_t color = M5.Displays(1).color888(c, 0, c); 
    crtCanvasSin.drawFastHLine ( 0, i, 1 , color);
  }
  crtCanvasSin.setPivot(0, 0);

  for (int i=0; i<crtCanvasTri.height()/2; i++){
    uint8_t c = 255.0f * ((float)(i*2)/(float)crtCanvasTri.height());
    uint32_t color = M5.Displays(1).color888(0, c, c); 
    crtCanvasTri.drawFastHLine ( 0, i, 1 , color);
    crtCanvasTri.drawFastHLine ( 0, crtCanvasTri.height()-1-i, 1 , color);
  }
  crtCanvasTri.setPivot(0, 0);
}

void drawRectStripe(float sNum, float rAngle){
  for (int i=0; i<sNum; i+=1){
    crtCanvasRect.pushRotateZoom(&M5.Displays(1), 60, i*M5.Displays(1).height()/sNum, rAngle, M5.Displays(1).width()*4, 1/sNum);
  }
}

void drawSineStripe(float sNum, float rAngle){
  for (int i=0; i<sNum; i++){
    crtCanvasSin.pushRotateZoom(&M5.Displays(1), 60, i*M5.Displays(1).height()/sNum, rAngle, M5.Displays(1).width()*4, 1/sNum);
  }
}

void drawTriangleStripe(float sNum, float rAngle){
  for (int i=0; i<sNum; i++){
    crtCanvasTri.pushRotateZoom(&M5.Displays(1), 60, i*M5.Displays(1).height()/sNum, rAngle, M5.Displays(1).width()*4, 1/sNum);
  }
}

void drawText(String s, int x, int y, int fColor, int bColor){
  portENTER_CRITICAL(&lcdMutex);
    M5.Display.setTextColor(fColor, bColor);
    M5.Display.setCursor(x, y);
    M5.Display.fillRect(x, y, 320, 52, TFT_BLACK);
    M5.Display.printf("%s\n",s.c_str());
  portEXIT_CRITICAL(&lcdMutex);
}

void drawText(String s, int x, int y){
  portENTER_CRITICAL(&lcdMutex);
    M5.Display.setCursor(x, y);
    M5.Display.fillRect(x, y, 320, 52, TFT_BLACK);
    M5.Display.printf("%s\n",s.c_str());
  portEXIT_CRITICAL(&lcdMutex);
}
