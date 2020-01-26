/*
  BTN_UP:
  Клик в режиме часов -> термометр
  Клик в режиме термометра -> вольтметр
  Клик в режиме вольтметра -> режим часов

  BTN_SET:
  Удержание -> установка времени
  Удержание режиме установки времени -> режим часов
  Клик режиме установки времени -> переключение настройки часов/минут
*/

#include <Arduino.h>
#include "GyverTimer.h"
#include "GyverTM1637.h"
#include "GyverButton.h"
#include <CyberLib.h>
#include "microDS18B20.h"

#include "EEPROM.h"
#include <Wire.h>
#include "RTClib.h"

// *************************** НАСТРОЙКИ ***************************

#define CLOCK_EFFECT 1    // эффект перелистывания часов: 0 - обычный, 1 - прокрутка, 2 - скрутка
#define MAX_BRIGHT 5      // яркость дисплея дневная (0 - 7)
#define MIN_BRIGHT 1      // яркость дисплея ночная (0 - 7)
#define  NIGHT_START 22   // час перехода на ночную подсветку (MIN_BRIGHT)
#define NIGHT_END 7       // час перехода на дневную подсветку (MAX_BRIGHT)

// ************ ПИНЫ ************

#define CLK 12        // дисплей
#define DIO 11        // дисплей
#define DOT 10        // дисплей


#define ILL 9         // габариты
#define VOLT A0       // вольтметр

#define DATA 8        // DS18B20

#define BTN_SET 3
#define BTN_UP 4      // сенсорная

// SDA A4   // RTC
// SCL A5   // RTC


// ***************** ОБЪЕКТЫ И ПЕРЕМЕННЫЕ *****************

GTimer_ms halfsTimer(500);
GTimer_ms blinkTimer(800);
GTimer_ms timeoutTimer(15000);
GTimer_ms tempTimer(3 * 1000);

GyverTM1637 disp(CLK, DIO);

MicroDS18B20 sensor(DATA);

GButton btnSet(BTN_SET);
GButton btnUp(BTN_UP, LOW_PULL, NORM_OPEN);

RTC_DS3231 rtc;

boolean dotFlag, minuteFlag, blinkFlag, newTimeFlag;
int8_t hrs = 21, mins = 55, secs;
int8_t mode = 0;
boolean changeFlag;
boolean illState, illFlag;

float filtered_vin = 13.0;
float k = 0.15;
float R1 = 4453000.0; // resistance of R1
float R2 = 1490000.0; // resistance of R2

void setup() {
  Serial.begin(9600);
  btnSet.setTimeout(400);
  btnSet.setDebounce(90);

  //pinMode(6, OUTPUT);

  pinMode(DOT, OUTPUT);
  pinMode(ILL, INPUT);

  disp.clear();

  rtc.begin();
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();
  secs = now.second();
  mins = now.minute();
  hrs = now.hour();

  disp.brightness(MAX_BRIGHT);
  disp.displayClock(hrs, mins);
}

void loop() {
  changeBright();
  buttonsTick();
  clockTick();    // считаем время
  //settings();     // настройки

  if (minuteFlag && mode == 0) {    // если новая минута и стоит режим часов
    minuteFlag = false;
    // выводим время
    printTime();
  }
}

void displayDot(boolean status) {
  if (status) digitalWrite(DOT, HIGH);
  else digitalWrite(DOT, LOW);
}

// установка яркости
void changeBright(){
  illState = digitalRead(ILL);   // читаем состояние кнопки с инверсией. 1 - нажата, 0 - нет
  if (illState && !illFlag) {    // если нажата и была отпущена (illFlag 0)
    illFlag = true;              // запомнили что нажата
    disp.brightness(MIN_BRIGHT);
  }
  if (!illState && illFlag) {    // если отпущена и была нажата (illFlag 1)
    illFlag = false;             // запомнили что отпущена
    disp.brightness(MAX_BRIGHT);
  }
  //if (digitalRead(ILL)) disp.brightness(MIN_BRIGHT);
  //else disp.brightness(MAX_BRIGHT);
}

void printTime(){
  if (CLOCK_EFFECT == 0) disp.displayClock(hrs, mins);
  else if (CLOCK_EFFECT == 1) disp.displayClockScroll(hrs, mins, 70);
  else disp.displayClockTwist(hrs, mins, 35);
}

void buttonsTick() {
  btnSet.tick();
  btnUp.tick();

  // 0 часы, 1 температура, 2 вольтметр, 3 настройка часов
  
  if (mode == 0 && btnUp.isClick()) {
    mode = 1;
  }

  if (mode == 1 && btnUp.isClick()) {
    mode = 2;
  }

  if (mode == 2 && btnUp.isClick()) {
    mode = 0;
    displayDot(false);
    disp.displayClock(hrs, mins);
  }

  if ((mode != 3) && btnSet.isHolded()) {
    mode = 3;
  }

  if (mode == 3) {
    //if (timeoutTimer.isReady()) mode = 0;   // если сработал таймаут, вернёмся в режим 0
    if (!newTimeFlag) newTimeFlag = true;   // флаг на изменение времени
    if (btnUp.isClick() || btnUp.isHold()) {
      if (!changeFlag) {
        mins++;
        if (mins > 59) mins = 0;
      } else {
        delay(50);
        hrs++;
        if (hrs > 23) hrs = 0;
      }
      disp.displayClock(hrs, mins);
      delay(50);
      //disp.displayClockScroll(hrs, mins,20);
      timeoutTimer.reset();           // сбросить таймаут
    }

    /*
    if (blinkTimer.isReady()) {
      if (blinkFlag) blinkTimer.setInterval(800);
      else blinkTimer.setInterval(200);
      blinkFlag = !blinkFlag;
    }
    */

    if (blinkTimer.isReady()) {
      // прикол с перенастройкой таймера, чтобы цифры дольше горели
      disp.point(1);
      if (blinkFlag) {
        blinkFlag = false;
        blinkTimer.setInterval(700);
        disp.displayClock(hrs, mins);
      } else {
        blinkFlag = true;
        blinkTimer.setInterval(300);
        //disp.clear();
        if (!changeFlag) {
          if(hrs/10 != 0) disp.display(0,(int8_t) (hrs/10));
          disp.display(1,(int8_t) (hrs%10));
          disp.displayByte(2,0X00);
          disp.displayByte(3,0x00);
        } else {
          disp.displayByte(0,0X00);
          disp.displayByte(1,0X00);
          disp.display(2,(int8_t) (mins/10));
          disp.display(3,(int8_t) (mins%10));
        }
      }
    }
  }

  if (mode == 3 && btnSet.isHolded()) {
    //sendTime();
    mode = 0;
    secs = 0;
    rtc.adjust(DateTime(2014, 1, 21, hrs, mins, 0));
    minuteFlag = true;
  }

  if (mode == 3 && btnSet.isClick()) {
    changeFlag = !changeFlag;
  }
}

void clockTick() {
  if (halfsTimer.isReady()) {
    if (newTimeFlag) {
      newTimeFlag = false;
      secs = 0;
      rtc.adjust(DateTime(2014, 1, 21, hrs, mins, 0)); // установка нового времени в RTC
    }

    dotFlag = !dotFlag;
    
    if (mode == 0) {
      disp.point(dotFlag);                 // выкл/выкл точки
      //disp.displayClock(hrs, mins);
    }

    if (mode == 1) {
      if (tempTimer.isReady()) {
        disp.point(false);
        sensor.requestTemp();
        delay(20);
        int temp = (sensor.getTemp() * 10.0);
        delay(20);
        disp.displayByte(0,0x58);
        disp.display(1,temp/100);
        disp.display(2,temp/10%10);
        displayDot(true);
        disp.display(3,temp%10);
      }
    }

    if (mode == 2) {
      // read the value at analog input
      filtered_vin += (((analogRead(VOLT) * 5.0) / 1024.0) / (R2/(R1+R2)) - filtered_vin) * k;
      int volt = filtered_vin*10;

      disp.displayByte(0,0x1c);
      disp.display(1,volt/100);
      disp.display(2,volt/10%10);
      displayDot(true);
      disp.display(3,volt%10);
    }

    if (dotFlag) {          // каждую секунду пересчёт времени
      secs++;
      if (secs > 59) {      // каждую минуту
        secs = 0;
        mins++;
        minuteFlag = true;
      }
      if (mins > 59) {      // каждый час
        DateTime now = rtc.now();
        secs = now.second();
        mins = now.minute();
        hrs = now.hour();
      }
    }
  }
}
