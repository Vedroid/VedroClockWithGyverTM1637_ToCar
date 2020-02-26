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
#include <Wire.h>
#include "RTClib.h"
#include "GyverTimer.h"
#include "GyverTM1637.h"
#include "GyverButton.h"
#include "microDS18B20.h"

// *************************** НАСТРОЙКИ ***************************

#define CLOCK_EFFECT 1    // эффект перелистывания часов: 0 - обычный, 1 - прокрутка, 2 - скрутка
#define MAX_BRIGHT 5      // яркость дисплея дневная (0 - 7)
#define MIN_BRIGHT 1      // яркость дисплея ночная (0 - 7)

// ************ ПИНЫ ************

#define DOT 12        // дисплей
#define CLK 11        // дисплей
#define DIO 10        // дисплей

#define ILL 9         // габариты

#define DATA 8        // DS18B20

#define BTN_UP 7      // сенсорная
#define BTN_SET 6

#define VOLT A0       // вольтметр

// SDA A4   // RTC
// SCL A5   // RTC


// ***************** ОБЪЕКТЫ И ПЕРЕМЕННЫЕ *****************

GTimer_ms halfsTimer(500);
GTimer_ms blinkTimer(800);
GTimer_ms timeoutTimer(10000);
GTimer_ms changeTimer(4000);
GTimer_ms tempTimer(3 * 1000);

GyverTM1637 disp(CLK, DIO);

MicroDS18B20 sensor(DATA);

GButton btnSet(BTN_SET);
GButton btnUp(BTN_UP, LOW_PULL, NORM_OPEN);

RTC_DS3231 rtc;

boolean dotFlag, minuteFlag, blinkFlag, newTimeFlag;
boolean changeFlag, illFlag, illState;
int8_t hrs, mins, secs;
int8_t mode = 2;  // 0 часы, 1 температура, 2 вольтметр, 3 настройка часов

float filtered_vin = 12.0;
float k = 0.3;       // Коэффициент сглаживания (0-1)
float R1 = 4453000.0; // resistance of R1
float R2 = 1480000.0; // resistance of R2

void setup() {
  btnSet.setTimeout(400);
  btnSet.setDebounce(90);

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
  
  while (true){
    clockTick();
    if(changeTimer.isReady()) mode--;
    if (mode <= 0) break;
  }
  displayDot(false);
  disp.displayClock(hrs, mins);
}

void loop() {
  changeBright();
  buttonsTick();
  clockTick();

  if (minuteFlag && mode == 0) {
    minuteFlag = false;
    printTime();
  }
}

// Верхняя точка между разрядами 2 и 3
void displayDot(boolean status) {
  if (status) digitalWrite(DOT, HIGH);
  else digitalWrite(DOT, LOW);
}

// Установка яркости
void changeBright(){
  illState = digitalRead(ILL);
  if (illState && !illFlag) {
    illFlag = true;
    disp.brightness(MIN_BRIGHT);
  }
  if (!illState && illFlag) {
    illFlag = false;
    disp.brightness(MAX_BRIGHT);
  }
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
    timeoutTimer.reset();
  }

  if (mode == 3) {
    if (timeoutTimer.isReady()){
      mode = 0;
      changeFlag = false;
      disp.displayClock(hrs, mins);
    }
    if (!newTimeFlag) newTimeFlag = true;
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
      timeoutTimer.reset();
    }

    if (blinkTimer.isReady()) {
      disp.point(true);
      if (blinkFlag) {
        blinkFlag = false;
        blinkTimer.setInterval(700);
        disp.displayClock(hrs, mins);
      } else {
        blinkFlag = true;
        blinkTimer.setInterval(300);
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
    mode = 0;
    secs = 0;
    rtc.adjust(DateTime(2020, 1, 1, hrs, mins, 0));
    minuteFlag = true;
  }

  if (mode == 3 && btnSet.isClick()) {
    changeFlag = !changeFlag;
    timeoutTimer.reset();
  }
}

void clockTick() {
  if (halfsTimer.isReady()) {
    if (newTimeFlag) {
      newTimeFlag = false;
      secs = 0;
      rtc.adjust(DateTime(2020, 1, 1, hrs, mins, 0));
    }

    dotFlag = !dotFlag;
    
    if (mode == 0) {
      disp.point(dotFlag);
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
      filtered_vin += (((analogRead(VOLT) * 5.05) / 1024.0) / (R2/(R1+R2)) - filtered_vin) * k;
      int volt = filtered_vin * 10;

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
