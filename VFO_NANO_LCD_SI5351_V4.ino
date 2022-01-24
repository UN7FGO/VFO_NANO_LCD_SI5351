// http://un7fgo.gengen.ru (C) 2021
// https://github.com/UN7FGO 
//
// VFO_NANO_LCD_SI5351 Ver.1.4
// С дополнениями для работы с трансиверами типа PIXIE или MICRO-80
// 
#include <avr/eeprom.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal.h>
// Библиотека для ситезатора
#include "si5351.h"
// Библиотека для обработки энкодера
#include <RotaryEncoder.h>

Si5351 si5351;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);
  
// Определяем контакты, к которым у нас подключен энкодер
#define ENC_CLK_PIN 4
#define ENC_DT_PIN  5
#define ENC_SW_PIN  6
// Создаем переменную-объект для работы с энкодером
RotaryEncoder encoder(ENC_DT_PIN, ENC_CLK_PIN);   

// определяем контакты для управляющих кнопок
// key1 - используем для переключения диапазонов "по кругу"
#define KEY1_PIN 3
// key2 - используем для переключения шага перестройки частоты
#define KEY2_PIN 2
// Аналоговый вход, для отслеживания перехода на передачу
#define TX_PIN A3

// Количество диапазонов и массивы с их параметрами 
int MAXBAND = 0;
// массив "текущих частот"  по диапазонам
unsigned long int cur_freq[9];
// массив "максимальных частот" по диапазонам
unsigned long int max_freq[9];
// массив "минимальных частот"  по диапазонам
unsigned long int min_freq[9];

// максимальное количество шагов перестройки частоты
#define MAXFREQ 4
// массив "шагов перестройки" в Герцах
int d_freq[MAXFREQ]  = {10000, 1000, 100, 10 };

// Прочие пераметры
// тип ПЧ false - до 10 МГц +, выше 10 МГц-, true - всегда +
bool IFTYPE = false;
// частота ПЧ в Гц
unsigned long int IFFREQ = 500000;
// умножитель частоты ГПД, для использования в схемах прямого преобразования или SDR
int VFOMULT = 1;
// коэффициент умножения для считываемых значений S-метра
int SMETER = 5;
// усреднение значений S-метра
int SLAZY = 5;
// подключен ли внешний переключатель выбора диапазона
bool EXTBAND = false;
// включен ли второй генератор, для ПЧ
bool BFO = false;
// частота второго генератора в Гц
unsigned long int BFOFREQ = 500000;
// Отслеживаем ли переход на передачу
bool PIXIE = false;
// Сдвиг частоты, при переходе на передачу
int TX_SHIFT = 700;
// Тип сдвига частоты при переходе на передачу
int SHIFT_TYPE = 3;
// Время "залипания" частоты на передачу, в миллисекундах
int SLIP_TIME = 200;


// S-meter
#define STIME 100
// символы дл S-метра
byte c_k[8] = {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};
byte c_p[8] = {B11111,B10001,B10101,B10101,B10101,B10101,B10001,B11111};

// переменные для работы
unsigned long int current_freq;
unsigned long int old_freq;
long int pressed, smeter, ssum;
int Pos, Band, c_Band, nfreq, dfreq, scount, slen;
float ll;

/* =================================================== */
void setup() {
  // инициализируем дисплей 16х2
  lcd.begin(16,2);    
  // формируем символы для шкалы S-метра                 
  lcd.createChar(1,c_k);
  lcd.createChar(2,c_p);
  // Информационное сообщение
  lcd.clear();
  lcd.noBlink();
  lcd.setCursor(0, 0);
  lcd.print("UN7FGO VFO V.1.4");
  lcd.setCursor(0, 1);
  lcd.print(" radiodiy.club  ");
  delay(1000);
 
  // все связанное с энкодером
  pinMode (ENC_CLK_PIN,INPUT_PULLUP);
  pinMode (ENC_DT_PIN,INPUT_PULLUP);
  pinMode (ENC_SW_PIN,INPUT_PULLUP);

  // задаем режим работы выводов для кнопок управления
  pinMode (KEY2_PIN,INPUT_PULLUP);
  pinMode (KEY1_PIN,INPUT_PULLUP);

  // обрабатываем нажатие 1-й кнопки - ENTER - для входа в меню
  if (digitalRead(KEY2_PIN) == 0) {
    ConfigMenu();
  }  
  
  // читаем текущие частоты по диапазонам из EEPROM
  while (MAXBAND == 0) {
    ReadConfig();
  }
  if (EXTBAND) {
    c_Band = ReadBand()-1;
    if (c_Band >= MAXBAND) {
      c_Band = MAXBAND-1;
    }
    Band = c_Band;
    eeprom_write_dword(95, Band); 
    current_freq = cur_freq[Band];
    Refresh_LCD();
  }
  
  // "старая" частота "по умолчанию"
  old_freq = 0;
  // Текущая частота
  current_freq = cur_freq[Band];
  // Текущий номер шага изменения частоты
  nfreq = 2;
  // Текущий шаг изменения частоты
  dfreq = d_freq[nfreq];

  bool i2c_found;
  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  { Serial.println("Not found I2C");}
  Wire.setClock(400000L);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);  

  if (BFO) {
    si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);  
    si5351.set_freq(BFOFREQ*100, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK1,1);
  }

  smeter = 0;
  ssum = 0;
  scount = 0;
  slen = 0;
}

void loop() {
  // Если частота у нас изменилась, 
  // то обновляем ее значение на индикаторе и на синтезаторе
  if ( current_freq != old_freq ) {
    // обновляем дисплей
    Refresh_LCD();
    setFreq();
  }

  // обрабатываем переход на передачу, если таковой режим включен
  if (PIXIE) {
    if (analogRead(TX_PIN) == 0) {
      // выводим на экран о перходе в режим передачи
      lcd.clear();
      lcd.noBlink();
      lcd.setCursor(6, 0);
      lcd.print("TX");
      // запоминаем текущую частоту
      old_freq = current_freq;
      // в зависимости от типа сдвига частоты, вычисляем частоту, на которую нужно перейти
      switch (SHIFT_TYPE) {
        case 1:
          current_freq = current_freq + TX_SHIFT;
          break;
        case 2:
          current_freq = current_freq - TX_SHIFT;
          break;
        case 3:
          if (current_freq > 10000000) {
            current_freq = current_freq - TX_SHIFT;
          } else {
            current_freq = current_freq + TX_SHIFT;
          }
          break;
        case 4:
          if (current_freq > 10000000) {
            current_freq = current_freq + TX_SHIFT;
          } else {
            current_freq = current_freq - TX_SHIFT;
          }
          break;
        default:
          current_freq = current_freq + TX_SHIFT;
          break;
      }
      // вычисляем "плановое" время выхода из режима передачи
      pressed = millis() + SLIP_TIME;
      // устанавливаем новую частоту
      setFreq();
      // ждем, пока перейдем на прием + время "залипания"
      while (millis() < pressed) {
        if (analogRead(TX_PIN) == 0) {
          pressed = millis() + SLIP_TIME;
        }
      }
      // восстанавливаем чатоту приема
      current_freq = old_freq;
      // устанавливаем частоту на сиентезаторе
      setFreq();
      // обновляем дисплей
      Refresh_LCD();
    }
  }

  // обрабатываем внешний переключатель диапазонов, если такой включен
  if (EXTBAND) {
    c_Band = ReadBand()-1;
    if (c_Band >= MAXBAND) {
      c_Band = MAXBAND-1;
    }
    if (c_Band != Band) {
      // запоминаем текущую частоту на текущем диапазоне
      cur_freq[Band] = current_freq;
      // записывавем частоту в EEPROM
      eeprom_write_dword(Band*4+4, current_freq); 
      // "переходим" на новый диапазон
      Band = c_Band;
      // запоминаем текущий диапазон
      eeprom_write_dword(95, Band); 
      // считываем текущую частоту выбранного диапазона
      current_freq = cur_freq[Band];
      Refresh_LCD();
    }
  }

  // обрабатываем нажатие кнопки - меняем шаг перестройки частоты
  if (digitalRead(KEY2_PIN) == 0) {
    while (digitalRead(KEY2_PIN) == 0) {
    }
    // меняем шаг перестройки, переходим на следующий шаг
    nfreq += 1;
    // если шаг больше возможного, переходим к первому значению
    if ( nfreq == MAXFREQ ) {
      nfreq = 0;
    }
    // запоминаем выбранный шаг перестройки
    dfreq = d_freq[nfreq];
    delay(500);
    Refresh_LCD();      
  }

  if (!EXTBAND) {
    // обрабатываем нажатие кнопки - выбор диапазона
    if (digitalRead(KEY1_PIN) == 0) {
      while (digitalRead(KEY1_PIN) == 0) {
      }
      // запоминаем текущую частоту на текущем диапазоне
      cur_freq[Band] = current_freq;
      // записывавем частоту в EEPROM
      eeprom_write_dword(Band*4+4, current_freq); 
      // увеличиваем номер диапазона
      Band +=1;
      // если номер больше максимального, возвращаемся в начало
      if ( Band == MAXBAND ) {
        Band = 0;
      }
      // запоминаем текущий диапазон
      eeprom_write_dword(95, Band); 
      // считываем текущую частоту выбранного диапазона
      current_freq = cur_freq[Band];
      Refresh_LCD();
      // задержка 0.5 секунды, на случай, если оператор зажал и не отпускает кнопку смены диапазона
      delay(500);
    }
  }
  // обрабатываем кнопку энкодера
  if (digitalRead(ENC_SW_PIN) == 0) {
    // запомнаем время нажатия кнопки
    pressed = millis();
    // ждем, пока кнопку отпустят
    while (digitalRead(ENC_SW_PIN) == 0) {
    }
    // считаем время, сколько была нажата кнопка
    pressed = millis() - pressed;
    // если время нажатия больше 1 секунды, то переключаем диапазон
    if ( pressed > 1000 ) {
      if (!EXTBAND) {
        // запоминаем текущую частоту на текущем диапазоне
        cur_freq[Band] = current_freq;
        // записывавем частоту в EEPROM
        eeprom_write_dword(Band*4+4, current_freq); 
        // увеличиваем номер диапазона
        Band +=1;
        // если номер больше максимального, возвращаемся в начало
        if ( Band == MAXBAND ) {
          Band = 0;
        }
        // считываем текущую частоту выбранного диапазона
        current_freq = cur_freq[Band];
        Refresh_LCD();      
      }
    } else {
      // если кнопка былв нажаты менее 1 секунды, меняем шаг перестройки
      // переходим на следующий шаг
      nfreq += 1;
      // если шаг больше возможного, переходим к первому значению
      if ( nfreq == MAXFREQ ) {
        nfreq = 0;
      }
      // запоминаем выбранный шаг перестройки
      dfreq = d_freq[nfreq];
      // выводим на индикатор информацию о выбранном шаге перестройки
      Refresh_LCD();      
    }
  }

  // обрабатываем энкодер
  encoder.tick();
  Pos = encoder.getPosition();
  // проверяем, был ли произведен поворот ручки энкодера
  if (Pos != 0){ 
    // определяем направление вращения энкодера
    if (Pos > 0) {
       // повернули энкодер "по часовой стрелке" (CW)
       current_freq += dfreq;
       // не даем частоте уйти за верхний предел диапазона
       if ( current_freq > max_freq[Band] ) {
         current_freq = max_freq[Band];
       }
     } else {
       // повернули энкодер "против часовой стрелки" (CCW)
       current_freq -= dfreq;
       // не даем частоте уйти за нижний предел диапазона
       if ( current_freq < min_freq[Band] ) {
         current_freq = min_freq[Band];
       }
     }
     encoder.setPosition(0);
  }

  // обрабатываем S-метер
  if (millis()-smeter > STIME) {
    scount++;
    ssum = ssum + analogRead(A0);
    if (scount > SLAZY){
      // вычисляем длину индикатора
      ll =  ssum / SLAZY / 73 * SMETER;
      slen = trunc(ll);
      if (slen > 14) { slen = 14; }
      Refresh_LCD();
      scount = 0;
      ssum = 0;
    }
    smeter = millis();
  }
}

// функция возведения числа в степень
long int intpow(int p) {
  long int k = 1;
  for (int j=1; j<p; j++) {
    k = k * 10;
  }
  return k;
}

// функция обновления информации на дисплее
void Refresh_LCD()
{
  int ost;
  long int fr, k;
  String Ss;
  lcd.clear();
  lcd.setCursor(0, 0);
  // далее переводим текущую частоту в текстовый формат ##.###.###
  Ss = ""; 
  fr = current_freq;
  for (int i=8; i>0; i--) {
    k = intpow(i);
    ost = fr / k;
    Ss += ost;
    fr = fr % k; 
    if (i == 7 || i == 4) {
      Ss += ".";    
    }
  }
  // выводим частоту в новом формате
  lcd.print(Ss);
  // выводим информацию текущем шаге перестройки частоты
  lcd.setCursor(15-log10(dfreq), 0);
  lcd.print(dfreq);
  // выводим информацию о выбранной частоте ПЧ в виде LSB/USB
  lcd.setCursor(0, 1);
  lcd.print("S ");
  for (int i=0; i<13; i++) {
    if (i>slen) {
      lcd.print(" ");
    } else {
      if (i<9) {
        lcd.write(1);
      } else {
        lcd.write(2);
      }
    }
  }
}


void(* resetFunc) (void) = 0;

// Меню
void ConfigMenu() {
  bool ToExit = false;
  int Pos;
  int Poss = 4;
  int Num = 0;
  int maxNum = 21;
 // массивы под пункты меню
  char*    Ms[22] = {"160M","80M","40M","30M","20M","17M","15M","12M","10M",
                     "6M","IFFREQ","IFTYPE","VFOMULT","SMETER","SLAZY","EXTSW","BFO","BFOFR",
                     "PIXIE","TXSH","SHTYPE","SLIP"};
  long int Mp[22] = {0,1,1,0,1,0,1,0,1,0, 5000000,0,1, 5, 5,0,0, 5000000,0, 700,3, 2};
  int      Mt[22] = {1,1,1,1,1,1,1,1,1,1,       3,1,2, 2, 2,1,1,       3,1,   2,2, 2};
  long int Mn[22] = {0,0,0,0,0,0,0,0,0,0,       0,0,1, 1, 1,0,0,  100000,0, 400,1, 1};
  long int Mx[22] = {1,1,1,1,1,1,1,1,1,1,50000000,1,4,20,20,1,1,50000000,1,1200,4,20};
  long int Par, Press, Dec;

  // Сообщаем о том, что мы в конфигурационном меню
  lcd.clear();
  lcd.noBlink();
  lcd.setCursor(0, 0);
  lcd.print("     V F O      ");
  lcd.setCursor(0, 1);
  lcd.print("  Setting Menu  ");
  delay(2000);
  
  // считываем данные из EEPROM
  for (int i=0; i<18; i++) {
    Mp[i] = eeprom_read_dword(100 + i*4); 
    if ( Mp[i] < Mn[i] || Mp[i] > Mx[i] ) {
      Mp[i] = Mn[i];
    }
  }
  // обновляем экран        
  PrintToLCD(Num+1,Ms[Num],Mt[Num],Mp[Num]);
      
  while (!ToExit) {
      // обрабатываем нажатие 2-й кнопки - ESC - выход из меню
     if (digitalRead(KEY1_PIN) == 0) {
       while (digitalRead(KEY1_PIN) == 0) {
       }
       ToExit = true;
     }
// ==================================================================================
// ==================================================================================
    // обрабатываем нажатие кнопки - ENTER - входим в режим редактирования параметра
    if (digitalRead(KEY2_PIN) == 0) {
      // ждем пока отпустят кнопку
      while (digitalRead(KEY2_PIN) == 0) {
      }
      // Запоминаем параметр для редактирования
      Par = Mp[Num];
      // обновляем дисплей
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Edit - ");
      lcd.print(Ms[Num]);
      if (Mt[Num] == 3) {
    // редактируем "длинные" параметры
        lcd.setCursor(0, 1);
        lcd.print("    ");
        lcd.setCursor(8-log10(Par), 1);
        lcd.print(Par);
        lcd.print("    ");
        lcd.blink();
        lcd.setCursor(8 - Poss, 1);
        ToExit = false;
        while (!ToExit) {
          // обрабатываем нажатие кнопки - ENTER - Выход с сохранением параметра
          if (digitalRead(KEY2_PIN) == 0) {
            Press = millis();
            while (digitalRead(KEY2_PIN) == 0) {
            }
            Press = millis() - Press;
            if (Press > 2000) {
              Mp[Num] = Par;
              ToExit = true;
            } else {
              Poss++;
              if (Poss>8) { Poss = 1; }
              lcd.setCursor(8 - Poss, 1);
              Dec = intpow(Poss);
              delay(200);
            }  
          }
          // обрабатываем нажатие кнопки - ESC - Выход без изменения параметра
          if (digitalRead(KEY1_PIN) == 0) {
            while (digitalRead(KEY1_PIN) == 0) {
            }
            ToExit = true;
          }
          // обрабатываем энкодер - перебираем пункты меню
          encoder.tick();
          Pos = encoder.getPosition();
          // проверяем, был ли произведен поворот ручки энкодера
          if (Pos != 0){ 
            // определяем направление вращения энкодера
            if (Pos > 0) {
              // повернули энкодер "по часовой стрелке" (CW) - переходим на следующий пункт меню
              Par += Dec;
            } else {
              // повернули энкодер "против часовой стрелки" (CCW) - переходим на предыдущий пункт меню
              Par -= Dec;
            }
            // отрабатываем граничные значения для параметра
            if (Par < Mn[Num]) { Par = Mn[Num]; }
            if (Par > Mx[Num]) { Par = Mx[Num]; }
            lcd.setCursor(0, 1);
            lcd.print("         ");
            lcd.setCursor(8-log10(Par)-0.001, 1);
            lcd.print(Par);
            lcd.print("    ");
            lcd.setCursor(8 - Poss, 1);
            encoder.setPosition(0);
          }
        }  
        lcd.noBlink();
      } else {
    // редактируем "простые" параметры
        lcd.setCursor(0, 1);
        // если тип параметра Вкл/Выкл, то отображаем Yes/No
        if (Mt[Num] == 1){
          if (Par == 1) {
            lcd.print("Yes ");
          } else {
            lcd.print("No  ");
          }
        } else {
          // для других типов параметров, отображаем их числовое значение
          lcd.print(Par);
          lcd.print("  ");
        }
        delay(200);
        // редактируем, пока не нажаты кнопки
        ToExit = false;
        while (!ToExit) {
          // обрабатываем нажатие кнопки - ENTER - Выход с сохранением параметра
          if (digitalRead(KEY2_PIN) == 0) {
            while (digitalRead(KEY2_PIN) == 0) {
            }
            Mp[Num] = Par;
            ToExit = true;
          }
          // обрабатываем нажатие кнопки - ESC - Выход без изменения параметра
          if (digitalRead(KEY1_PIN) == 0) {
            while (digitalRead(KEY1_PIN) == 0) {
            }
            ToExit = true;
          }
          // обрабатываем энкодер - перебираем пункты меню
          encoder.tick();
          Pos = encoder.getPosition();
          // проверяем, был ли произведен поворот ручки энкодера
          if (Pos != 0){ 
            // определяем направление вращения энкодера
            if (Pos > 0) {
              // повернули энкодер "по часовой стрелке" (CW) - переходим на следующий пункт меню
              Par++;
            } else {
              // повернули энкодер "против часовой стрелки" (CCW) - переходим на предыдущий пункт меню
              Par--;
            }
            // отрабатываем граничные значения для параметра
            if (Par < Mn[Num]) { Par = Mn[Num]; }
            if (Par > Mx[Num]) { Par = Mx[Num]; }
            lcd.setCursor(0, 1);
            // если тип параметра Вкл/Выкл, то отображаем Yes/No
            if (Mt[Num] == 1){
              if (Par == 1) {
                lcd.print("Yes ");
              } else {
                lcd.print("No  ");
              }
            } else {
              // для других типов параметров, отображаем их числовое значение
              lcd.print(Par);
              lcd.print("  ");
            }
            encoder.setPosition(0);
          }
        }
      }
      // обновляем дисплей
      PrintToLCD(Num+1,Ms[Num],Mt[Num],Mp[Num]);
      ToExit = false;
      delay(200);
    }
  // ==================================================================================
  // ==================================================================================
    // обрабатываем энкодер - перебираем пункты меню
    encoder.tick();
    Pos = encoder.getPosition();
    // проверяем, был ли произведен поворот ручки энкодера
    if (Pos != 0){ 
      // определяем направление вращения энкодера
      if (Pos > 0) {
        // повернули энкодер "по часовой стрелке" (CW) - переходим на следующий пункт меню
        Num++;
      } else {
        // повернули энкодер "против часовой стрелки" (CCW) - переходим на предыдущий пункт меню
        Num--;
      }
      // обрабатываем "граничные" условия движения по пунктам меню
      if (Num<0) { Num = maxNum; }
      if (Num>maxNum) { Num = 0; }
      
      // обновляем дисплей
      PrintToLCD(Num+1,Ms[Num],Mt[Num],Mp[Num]);
      delay(100);
      encoder.setPosition(0);
    }
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SAVED PARAMETERS");
  // сохраняем параметры в EEPROM
  for (int i=0; i<18; i++) {
    eeprom_write_dword(100 + i*4, Mp[i]); 
  }
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print("      OK.       ");
  delay(2000);
  resetFunc();
}

void PrintToLCD(long a, char* s, long b, long c) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(a);
  lcd.print(". ");
  lcd.print(s);
  lcd.setCursor(0, 1);
  // если тип параметра Вкл/Выкл, то отображаем Yes/No
  if (b == 1){
    if (c == 1) {
      lcd.print("Yes");
    } else {
      lcd.print("No");
    }
  } else {
    // для других типов параметров, отображаем их числовое значение
    lcd.print(c);
  } 
}


void ReadConfig() {
unsigned long int Fn[10] = {1500000, 3000000, 5500000,  9000000, 13500000, 17500000, 20500000, 24500000, 26500000, 49000000};
unsigned long int Fx[10] = {2500000, 4000000, 7500000, 12100000, 16000000, 19500000, 22000000, 26500000, 31000000, 54000000};
long int Re;
unsigned long int freq, param;

  MAXBAND = 0;
  // считываем данные общих настроек из EEPROM
  for (int i=0; i<9; i++) {
    Re = eeprom_read_dword(100 + i*4); 
    if ( Re == 1 ) {
      MAXBAND++;
      freq = eeprom_read_dword((MAXBAND-1)*4+4); 
      if (freq<Fn[i] || freq>Fx[i]) {
        freq = (Fn[i]+Fx[i])/2;
        eeprom_write_dword((MAXBAND-1)*4+4, freq); 
      } 
      cur_freq[MAXBAND-1] = freq;
      max_freq[MAXBAND-1] = Fx[i];
      min_freq[MAXBAND-1] = Fn[i];
    }
  } 

  if (MAXBAND == 0) {
    ConfigMenu();
  }
  // Текужий диапазон
  Band = eeprom_read_dword(95); 
  if (Band<0 || Band>MAXBAND) {
    Band = 0;
  }
  // читаем прочие параметры
  // тип ПЧ false - до 10 МГц +, выше 10 МГц-, true - всегда +
  param = eeprom_read_dword(144); 
  if (param == 1) {
    IFTYPE = true;
  }
  // частота ПЧ в Гц
  param = eeprom_read_dword(140); 
  if (param >= 0) {
    IFFREQ = param;
  }
  // умножитель частоты ГПД, для использования в схемах прямого преобразования или SDR
  param = eeprom_read_dword(148); 
  if (param > 0 && param < 5 ) {
    VFOMULT = param;
  }
  // коэффициент умножения для считываемых значений S-метра
  param = eeprom_read_dword(152); 
  if (param > 0 && param < 11 ) {
    SMETER = param;
  }
  // усреднение значений S-метра
  param = eeprom_read_dword(156); 
  if (param > 0 && param < 11 ) {
    SLAZY = param;
  }
  // подключен ли внешний переключатель выбора диапазона
  param = eeprom_read_dword(160); 
  if (param == 1) {
    EXTBAND = true;
  }
  // включен ли второй генератор, для ПЧ
  param = eeprom_read_dword(164); 
  if (param == 1) {
    BFO = true;
  }
  // частота второго генератора в Гц
  param = eeprom_read_dword(168); 
  if (param >= 0) {
    BFOFREQ = param;
  }
  // Отслеживаем ли переход на передачу
  param = eeprom_read_dword(172); 
  if (param == 1) {
    PIXIE = true;
  }
  // Сдвиг частоты, при переходе на передачу
  param = eeprom_read_dword(176); 
  if (param > 499 && param < 1501 ) {
    TX_SHIFT = param;
  }
  // Тип сдвига частоты при переходе на передачу
  param = eeprom_read_dword(180); 
  if (param > 0 && param < 5 ) {
    SHIFT_TYPE = param;
  }
  // Время "залипания" частоты на передачу, в миллисекундах
  param = eeprom_read_dword(184); 
  if (param > 0 && param < 21 ) {
    SLIP_TIME = param * 100;
  }
  
}

// Читаем аналоговый вход и расчитываем номер диапазона
// подробности тут - http://blog.gengen.ru/?p=2288
int ReadBand() {
  int a, b;
  float c;
  b = Band;
  a = analogRead(A1);
  if ( a < 520 ) {
    c = 0.00003*a*a + 0.0053*a + 1.3;
    b = trunc(c);
  }
  return b;
}

void setFreq() {
  unsigned long int freq;
  // Проверяем выбранный режим учета ПЧ 
  // true - ПЧ всегда прибавляется к текущей частоте
  // false - до 10 МГц прибавляем ПЧ, выше 10 МГц вычитаем ПЧ 
  if (IFTYPE) {
    freq = current_freq + IFFREQ;
  } else {
    if ( current_freq < 10000000 ) {
      freq = current_freq + IFFREQ;
    } else {
      freq = current_freq - IFFREQ;
    }
  }
  // режим умножения частоты ГПД, если выбран.
  if (VFOMULT > 1) {
    freq = freq * VFOMULT;
  }
  // устанавливаем расчитанную частоту на синтезаторе
  si5351.set_freq(freq*100, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0,1);
  old_freq = current_freq;
}
