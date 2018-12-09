
/*
   Убираю энкодер (слишком усложненный интерфейс и нет возможности управления поворотным механизмом вручную

   выход в меню и ежесекундная проверка таймера без использования аппаратного прерывания (ибо нету   :((  )
   аппаратные прерывания займут кнопки +/- для оперативного реагирования на изменения

   регулировка температуры будет осуществлятся следующим образом
   имеется некое значение температуры "Т" при данном значении я предполагаю (с потолка взял)
   что значение (PWM) яркости лампы нагревателя можно принять за 100 (пределы 0 - 255)
   при изменении температуры меняем значение PWM на лампе
   | T - Заданная температура
   | t - реальная температура
   | k = (T - t) * 2000 (поправочный коеффициент выбран с учетом допустимого превышения температуры на 0.05 градуса)
   | PWM_L = 100 + k
*/

// переделка прграммы для работы с SHT22
// и использование MOSFET вместо реле для плавной регулировки нагрева
// в планах прикрутить часы реального времени DS3231

//========== Подключаемые библиотеки =============
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // для экрана
#include <EEPROM.h> // EE
#include <TimerOne.h> // прерывания по таймеру1
#include <Sodaq_SHT2x.h>    // библиотека датчика SHT21
#include "Sodaq_DS3231.h" //библиотека для работы с часами

//================== Константы ===================
#define ButMinusPin  2  //кнопка минус(вниз)
#define ButPlusPin 3 //кнопка плюс(вверх)
#define ButMenuPin  4  // кнопка меню
#define MOSFET 5  //пин управления нагревателем через MOSFET
#define intClockPin  7  // пин прерывания по часам (все таки ежесекундное)
#define PovorotPwmPin 8 //запуск поворотный механизм
#define PovorotPin  9 //направление поворотный механизм
#define humPin 10 //пин управления увлажнителем

// EEPROM addresses
#define TempSetEEaddr 0 // EE - адрес для сохранения температуры , 4 байта(float)!
#define HumSetEEaddr 4 // EE - адрес для сохранения влажности, 4 байта(float)!
#define timerHEEaddr 8 // EE - адрес для сохранения времени следующего поворота часы 2 байта(int)
#define timerMEEaddr 10 // EE - адрес для сохранения времени следующего поворота минуты 2 байта(int)
#define timerWEEaddr 12 // EE - адрес для сохранения дня недели следующего поворота 2 байта(int)
#define intHEEaddr 14 // EE - адрес для сохранения интервала поворота часы 2 байта(int)
#define intMEEaddr 16 // EE - адрес для сохранения интервала поворота минуты 2 байта(int)
#define BeepEnabledEEaddr 18 // EE - адрес для сохранения признака разрешения звука (boolean)
#define TimerEnabledEEaddr 19 // EE - адрес для сохранения признака включения таймера (boolean)
#define PovorotEEaddr 20 // EE - адрес для сохранения направления поворота (boolean)
//------------------------------------------------

//=================== Железо =====================

LiquidCrystal_I2C lcd(0x27, 16, 2);
//------------------------------------------------

//================== Переменные ==================
//термостатирование
float tempSet; //Заданная температура
float humSet; //Заданная влажность
float temperatura;        //Считанная температура
float humidity; //Считанная влажность
int C_PWM = 100;//значение ШИМ по умолчанию в рабочей точке
byte PWM;       //Текущее значение ШИМ
int k = 0;      //Коэффициент поправки для ШИМ

int encoderVal = 0;

//menuSet Заголовки меню установок
char MenuSet[][15] = {"TEMPERATURA", "HUMIDITY", "TIMER", "TIME", "DATA", "HAND_MODE", "DEFAULT", "EXIT"};
boolean menuOn = false; //флаг входа в меню настроек
boolean menuEdit = false; //флаг редактирования пункта меню

boolean lastButton = HIGH;
boolean currentButton = HIGH;

boolean handFlag = false; //флаг ручного управления если поднят управление поворотом только вручную

int intHour;  //интервал поворота часы
int intMin; //итервал поворота минуты
boolean timerEnable = false;  //флаг ON/OFF таймер
boolean timerInit = false;    //флаг инициализации таймера
int timerHour; //время следующего поворота часы
int timerMin;  //время следующего поворота минуты
int timerWd;   //время следующего поворота день недели

byte MenuTimeoutTimer = 10;

int rtWd, rtHour, rtMin, rtSec;

int itemDisp = 0;

boolean povorot;

void setup() {
  Serial.begin(9600);
  pinMode(MOSFET, OUTPUT);  //нагреватель
  pinMode(ButMenuPin, INPUT_PULLUP); // меню
  pinMode(ButPlusPin, INPUT_PULLUP); // вверх/плюс
  pinMode(ButMinusPin, INPUT_PULLUP); // вниз/минус

  pinMode(PovorotPin, OUTPUT);  //поворотный механизм направление
  pinMode(PovorotPwmPin, OUTPUT); //повортный механизм старт/стоп

  //считывание данных из памяти
  EEPROM.get(TempSetEEaddr, tempSet);
  EEPROM.get(HumSetEEaddr, humSet);
  EEPROM.get(intHEEaddr, intHour);
  EEPROM.get(intMEEaddr, intMin);
  EEPROM.get(TimerEnabledEEaddr, timerEnable);
  EEPROM.get(PovorotEEaddr, povorot);

  lcd.init(); //инициализация экрана
  lcd.backlight();

  Wire.begin(); //для работы с часами
  rtc.begin();  //для работы с часами

  timerInit = true;

  // interrupt at  EverySecond, EveryMinute, EveryHour
  // interrupt at (h,m,s)
  // rtc.enableInterrupts(EverySecond);  //Enable Interrupt

  attachInterrupt(0, doButDoun, FALLING); //Нажата кнопка вниз
  attachInterrupt(1, doButUp, FALLING);   //Нажата кнопка вверх

  Timer1.initialize(1000000); // Timer0 interrupt - set a timer of length 1000000 microseconds
  Timer1.attachInterrupt(doClock); // attach the service routine here
}

void loop() {
  humidity = SHT2x.GetHumidity();
  // термостатирование
  temperatura = SHT2x.GetTemperature();
  k = ((tempSet - temperatura) * 2000) / 1;
  PWM = constrain((C_PWM + k), 0, 255);
  analogWrite(MOSFET, PWM);

  //  по моему мнению этого достаточно что бы упрвлять увлажнителем
  if (humidity > humSet) {
    DigitalWrite(humPin, HIGH);
  }
  else {
    DigitalWrite(humPin, LOW);
  }
  //---------------------------------------------------------------

  if (MenuTimeoutTimer > 0) {
    lcd.backlight();
  }
  else  {
    lcd.backlight();
  }

  DateTime now = rtc.now(); //get the current date-time
  rtHour = int(now.hour());
  rtMin = int(now.minute());
  rtSec = int(now.second());
  rtWd = int(now.dayOfWeek());

  digitalWrite(PovorotPin, povorot);
  digitalWrite(PovorotPwmPin, timerEnable);

  rtc.clearINTStatus();

  if (timerInit)  TimerInit();  //если кто-то поднял флаг инициализации таймера то тут мы его и проинициализируем чуть язык не сломал :)))

  //  Обработка нажатий кнопок вверх/вниз на лету
  if (encoderVal != 0)  {
    itemDisp = itemDisp + encoderVal;
    encoderVal = 0;
    if (itemDisp > 2) itemDisp = 0;
    if (itemDisp < 0) itemDisp = 2;
  }
  //--------------------------------------------

  currentButton = digitalRead(ButMenuPin);
  if (lastButton == LOW && currentButton == HIGH) {
    lastButton = currentButton;
    menuOn = true;
    MenuTimeoutTimer = 10;
    menuSet();
  }
  else  {
    lastButton = currentButton;
    mineDisp(itemDisp);
  }
}
//======== Пользовательские функции ==============
void mineDisp(byte Disp) {
  DateTime now = rtc.now(); //get the current date-time
  lcd.clear();
  lcd.noBlink();
  switch (Disp) {
    case 0:
      lcd.setCursor(0, 0);
      PrintTimer(now.hour());
      lcd.print(now.hour());
      lcd.print(':');
      PrintTimer(now.minute());
      lcd.print(now.minute());
      lcd.print(' ');
      PrintTimer(now.date());
      lcd.print(now.date());
      lcd.print('.');
      PrintTimer(now.month());
      lcd.print(now.month());
      lcd.print('.');
      lcd.print(now.year());

      lcd.setCursor(0, 1);
      lcd.print(temperatura);
      lcd.print("C    ");
      lcd.print(humidity);
      lcd.print("%");
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("NEXT <> ");
      PrintTimer(timerHour);
      lcd.print(timerHour);
      lcd.print(":");
      PrintTimer(timerMin);
      lcd.print(timerMin);

      lcd.setCursor(0, 1);
      lcd.print(temperatura);
      lcd.print("C ");
      lcd.print(humidity);
      lcd.print("%");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("S ");
      lcd.print(tempSet);
      lcd.print("C ");
      lcd.print(humSet);
      lcd.print("%");

      lcd.setCursor(0, 1);
      lcd.print("  ");
      lcd.print(temperatura);
      lcd.print("C ");
      lcd.print(humidity);
      lcd.print("%");
      break;
  }
}
void menuSet()  {
  /*
     меню установки работы термостата
     навигация по меню осуществляется с помощью заголовочной переменной MenuSet
     и функции switch case
  */
  MenuTimeoutTimer = 10;
  lcd.backlight();
  lcd.clear();
  int menuitem = 0;
  while (menuOn && MenuTimeoutTimer > 0) {
    rtc.clearINTStatus();
    lcd.setCursor(0, 0);
    lcd.print("SETTING         ");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.print(MenuSet[menuitem]);
    if (encoderVal != 0)  {
      menuitem = menuitem + encoderVal;
      encoderVal = 0;
      if (menuitem > 7) menuitem = 0;
      if (menuitem < 0) menuitem = 7;
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.print(MenuSet[menuitem]);
    }
    currentButton = digitalRead(ButMenuPin);
    if (lastButton == LOW && currentButton == HIGH) {
      lastButton = currentButton;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(MenuSet[menuitem]);
      lcd.setCursor(0, 1);
      lcd.print("                ");
      switch (menuitem) {
        case 0: // настройки, температуры
          menuEdit = true;
          setTemp();
          break;
        case 1: // настройки, влажности
          menuEdit = true;
          setHum();
          break;
        case 2: // настройки, таймер
          menuEdit = true;
          setTimer();
          break;
        case 3: // настройки, время
          menuEdit = true;
          setTime();
          break;
        case 4: // настройки, дата
          menuEdit = true;
          setData();
          break;
        case 5: // ручной режим управления поворотной платформой
          menuEdit = true;
          setHandMode();
          break;
        case 6: // сброс
          menuEdit = true;
          setDef();
          break;
        case 7: // выход
          menuOn = false;
          break;
      }
    }
    else  {
      lastButton = currentButton;
    }
  }
  menuOn = false;
}

void setTemp()  { //установка температуры
  while (menuEdit && MenuTimeoutTimer > 0) {
    rtc.clearINTStatus();
    lcd.setCursor(0, 1);
    lcd.print(tempSet);
    lcd.print("C");
    lcd.setCursor(3, 1);
    lcd.blink();
    while (encoderVal == 0 && MenuTimeoutTimer > 0) {
      rtc.clearINTStatus();
      currentButton = digitalRead(ButMenuPin);
      if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
        lastButton = currentButton;
        EEPROM.put(TempSetEEaddr, tempSet); //записываем новое значение в память
        menuEdit = false;
        break;
      }
      else  {
        lastButton = currentButton;
      }
    }
    if (encoderVal != 0) tempSet = tempSet + (0.1 * encoderVal); //обработка поворота энкодера
    encoderVal = 0;
  }
  EEPROM.get(TempSetEEaddr, tempSet);
  menuSet();
}

void setHum() { //установка влажности
  while (menuEdit && MenuTimeoutTimer > 0) {
    rtc.clearINTStatus();
    lcd.setCursor(0, 1);
    lcd.print(humSet);
    lcd.print("%");
    lcd.setCursor(3, 1);
    lcd.blink();
    while (encoderVal == 0 && MenuTimeoutTimer > 0) {
      rtc.clearINTStatus();
      currentButton = digitalRead(ButMenuPin);
      if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
        lastButton = currentButton;
        EEPROM.put(HumSetEEaddr, humSet); //записываем новое значение в память
        menuEdit = false;
        break;
      }
      else  {
        lastButton = currentButton;
      }
    }
    if (encoderVal != 0) humSet = humSet + (0.1 * encoderVal); //обработка поворота энкодера
    encoderVal = 0;

  }
  EEPROM.get(HumSetEEaddr, humSet);
  menuSet();
}

void setTimer() { //установка таймера
  MenuTimeoutTimer = 10;
  int t = 0;  //
  while (menuEdit && MenuTimeoutTimer > 0) {
    rtc.clearINTStatus();
    lcd.setCursor(0, 1);
    PrintTimer(intHour);
    lcd.print(intHour);
    lcd.print(":");
    PrintTimer(intMin);
    lcd.print(intMin);
    lcd.print(" ");
    if (timerEnable) {
      lcd.print("ON ");
    }
    else  {
      lcd.print("OFF");
    }
    switch (t)  {
      case 0:
        lcd.setCursor(1, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          intHour = intHour + encoderVal;
          encoderVal = 0;
          if (intHour > 23) intHour = 0;
          if (intHour < 0) intHour = 23;
        }
        break;
      case 1:
        lcd.setCursor(4, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          intMin = intMin + encoderVal;
          encoderVal = 0;
          if (intMin > 59) intMin = 0;
          if (intMin < 0) intMin = 59;
        }
        break;
      case 2:
        lcd.setCursor(6, 1);
        if (timerEnable) {
          lcd.print("ON ");
        }
        else  {
          lcd.print("OFF");
        }
        lcd.setCursor(6, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  timerEnable = !timerEnable;
        encoderVal = 0;
        break;
      default:
        if (intHour != 0 || intMin != 0)  {
          EEPROM.put(intHEEaddr, intHour); //записываем новое значение в память
          EEPROM.put(intMEEaddr, intMin); //записываем новое значение в память
          EEPROM.put(TimerEnabledEEaddr, timerEnable); //записываем новое значение в память
        }
        menuEdit = false;
        break;
    }
  }
  EEPROM.get(intHEEaddr, intHour);
  EEPROM.get(intMEEaddr, intMin);
  EEPROM.get(TimerEnabledEEaddr, timerEnable);
  //  timerInit = true;
  TimerInit();
  menuSet();
}

void setTime()  { //установка времени
  MenuTimeoutTimer = 10;
  int t = 0;  //
  while (menuEdit && MenuTimeoutTimer > 0) {
    rtc.clearINTStatus();
    lcd.setCursor(0, 1);
    PrintTimer(rtHour);
    lcd.print(rtHour);
    lcd.print(":");
    PrintTimer(rtMin);
    lcd.print(rtMin);
    lcd.print(":00");

    switch (t)  {
      case 0:
        lcd.setCursor(1, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            rtc.setHour(rtHour);
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          rtHour = rtHour + encoderVal;
          encoderVal = 0;
          if (rtHour > 23)  rtHour = 0;
          if (rtHour < 0) rtHour = 23;
        }
        break;
      case 1:
        lcd.setCursor(4, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            rtc.setMinute(rtMin);
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          rtMin = rtMin + encoderVal;
          encoderVal = 0;
          if (rtMin > 59)  rtMin = 0;
          if (rtMin < 0) rtMin = 59;
        }
        break;
      case 2:
        lcd.setCursor(7, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            rtc.setSecond(0);
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          encoderVal = 0;
        }
        break;
      default:
        menuEdit = false;
        break;
    }
  }
  menuSet();
}

void setData()  { //установка даты
  DateTime now = rtc.now(); //get the current date-time
  int tD = int(now.date());
  int tM = int(now.month());
  int tY = int(now.year());
  int t = 0;
  while (menuEdit && MenuTimeoutTimer > 0) {
    rtc.clearINTStatus();
    lcd.setCursor(0, 1);
    PrintTimer(tD);
    lcd.print(tD);
    lcd.print(".");
    PrintTimer(tM);
    lcd.print(tM);
    lcd.print(".");
    lcd.print(tY);
    switch (t)  {
      case 0:
        lcd.setCursor(1, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            rtc.setDate(tD);
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          tD = tD + encoderVal;
          encoderVal = 0;
          if (tD > 31)  tD = 1;
          if (tD < 1) tD = 31;
        }
        break;
      case 1:
        lcd.setCursor(4, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            rtc.setMonth(tM);
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          tM = tM + encoderVal;
          encoderVal = 0;
          if (tM > 12)  tD = 1;
          if (tM < 1) tM = 12;
        }
        break;
      case 2:
        lcd.setCursor(9, 1);
        lcd.blink();
        while (encoderVal == 0 && MenuTimeoutTimer > 0) {
          rtc.clearINTStatus();
          currentButton = digitalRead(ButMenuPin);
          if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
            lastButton = currentButton;
            MenuTimeoutTimer = 10;
            rtc.setYear(tY);
            t++;
            break;
          }
          else  {
            lastButton = currentButton;
          }
        }
        if (encoderVal != 0)  {
          tY = tY + encoderVal;
          encoderVal = 0;
          tY = constrain(tY, 2000, 2100);
        }
        break;
      default:
        menuEdit = false;
        break;
    }
  }
  menuSet();
}

void setHandMode()  { //Ручное управление поворотным механизмом
  detachInterrupt(0);   // отключаю прерывания на время ручного управления поворотным механизмом
  detachInterrupt(1);
  while (menuEdit) {
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("HAND MODE");
    if (digitalRead(ButPlusPin) == HIGH)  {
      lcd.setCursor(0, 1);
      lcd.print("     RIGHT>>>>> ");
      digitalWrite(PovorotPin, HIGH);
      digitalWrite(PovorotPwmPin, HIGH);
    }
    else if (digitalRead(ButMinusPin) == HIGH) {
      lcd.setCursor(0, 1);
      lcd.print("<<<<<LEFT       ");
      digitalWrite(PovorotPin, LOW);
      digitalWrite(PovorotPwmPin, HIGH);
    }
    else  {
      lcd.setCursor(0, 1);
      lcd.print("     STOP       ");
      digitalWrite(PovorotPwmPin, LOW);
    }
    currentButton = digitalRead(ButMenuPin);
    if (lastButton == LOW && currentButton == HIGH) {
      lastButton = currentButton;
      menuEdit = false;
    }
    else {
      lastButton = currentButton;
    }
  }

  // Включаю обратно прерывания
  attachInterrupt(0, doButDoun, FALLING); //Нажата кнопка вниз
  attachInterrupt(1, doButUp, FALLING);   //Нажата кнопка вверх
  menuSet();
}

void setDef() { //сброс на установки по умолчанию
  boolean D = false;
  while (menuEdit && MenuTimeoutTimer > 0) {
    rtc.clearINTStatus();
    lcd.setCursor(0, 1);
    if (D == true)  {
      lcd.print(" ON");
    }
    else  {
      lcd.print("OFF");
    }
    lcd.setCursor(2, 1);
    lcd.blink();
    while (encoderVal == 0 && MenuTimeoutTimer > 0) {
      rtc.clearINTStatus();
      currentButton = digitalRead(ButMenuPin);
      if (lastButton == LOW && currentButton == HIGH) { //если нажата кнопка
        lastButton = currentButton;
        if (D)  {
          tempSet = 34.2;
          humSet = 65.0;
          intHour = 2;
          intMin = 0;
          timerEnable = false;
          povorot = false;
          EEPROM.put(TempSetEEaddr, tempSet);
          EEPROM.put(HumSetEEaddr, humSet);
          EEPROM.put(intHEEaddr, intHour);
          EEPROM.put(intMEEaddr, intMin);
          EEPROM.put(TimerEnabledEEaddr, timerEnable);
          EEPROM.put(PovorotEEaddr, povorot);
          TimerInit();
        }
        menuEdit = false;
        break;
      }
      else  {
        lastButton = currentButton;
      }
    }
    if (encoderVal != 0) D = !D;
    encoderVal = 0;
  }
  EEPROM.get(TempSetEEaddr, tempSet);
  EEPROM.get(HumSetEEaddr, humSet);
  EEPROM.get(intHEEaddr, intHour);
  EEPROM.get(intMEEaddr, intMin);
  EEPROM.get(TimerEnabledEEaddr, timerEnable);
  EEPROM.get(PovorotEEaddr, povorot);
  menuSet();
}

void doButDoun() {  //Обработчик прерываний кнопка вниз
  MenuTimeoutTimer = 10;
  encoderVal = -1;
}

void doButUp() {  //Обработчик прерываний кнопка вверх
  MenuTimeoutTimer = 10;
  encoderVal = 1;
}

void doClock()  { //обработчик прерываний от часов (ежесекундно)
  if (MenuTimeoutTimer != 0)  MenuTimeoutTimer --;  // ежесекундный декремент этого таймера

  if (timerEnable) {
    if (rtSec == 5) {
      if (timerHour < rtHour) {
        if (timerWd == rtWd)  {
          povorot = !povorot;
          timerInit = true;
        }
      }
      else if (timerHour == rtHour)  {
        if (timerWd == rtWd)  {
          povorot = !povorot;
          timerInit = true;
        }
      }
    }
    else if (timerHour > rtHour)  {
      if (timerWd != rtWd)  {
        povorot = !povorot;
        timerInit = true;
      }
    }
  }
}

//------------------------------------------------
void PrintTimer(byte timer) {
  if (timer < 10) {
    lcd.print("0");
  }
}

//====Инициализация таймера===
void TimerInit()  {
  DateTime now = rtc.now();
  timerWd = int(now.dayOfWeek());
  timerHour = int(now.hour()) + intHour;
  if (timerHour > 23) {
    timerHour = timerHour - 24;
    timerWd = int(now.dayOfWeek()) + 1;
    if (timerWd > 6)  timerWd = timerWd - 7;
  }
  timerMin = int(now.minute()) + intMin;
  if (timerMin > 59)  {
    timerMin = timerMin - 60;
    timerHour = timerHour + 1;
    if (timerHour > 23) {
      timerHour = timerHour - 24;
      timerWd = int(now.dayOfWeek()) + 1;
      if (timerWd > 6)  timerWd = timerWd - 7;
    }
  }
  EEPROM.put(timerHEEaddr, timerHour);
  EEPROM.put(timerMEEaddr, timerMin);
  EEPROM.put(timerWEEaddr, timerWd);
  timerInit = false;
  return;
}
//=============================================
