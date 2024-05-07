#include <LiquidCrystal_I2C.h>  //библиотека дисплея
#include <max6675.h> //библиотека термопары
#include <Servo.h>  //библиотека сервоприводов

#include <GyverPID.h> //БИБЛИОТЕКА РЕГУЛЯТОРА
#include "PIDtuner.h" //РЕГУЛИРОВКА КОЭФФИЦИЕНТОВ !!!!!!!!!!!!!!!!!!!!!!!!!!!!
GyverPID regulator(5, 2, 2, 200);  //КОЭФФИЦИЕНТЫ ПИД, ПЕРИОД = 200 МС  //Коэф. были 1.6-0.4-0
Servo PIDServo; //устанавливаем ПИД-сервопривод
PIDtuner tuner;

#define SETPOINT 15.0 //УСТАНОВКА ПОДДЕРЖИВАЕМОЙ ТЕМПЕРАТУРЫ (задаётся вручную)
float value = 30;
float signal = 0;

#define RELAY_ON 1  //РЕЛЕ ВЫКЛЮЧЕНО
#define RELAY_OFF 0 //РЕЛЕ ВКЛЮЧЕНО
#define Relay 10  //ПОДАЧА СИГНАЛА НА РЕЛЕ ЧЕРЕЗ ПИН 10
#define Vent 8

LiquidCrystal_I2C lcd(0x27,16,2); // пины А4, А5 под дисплей И использование ячеек дисплея
MAX6675 thermocouple(2,3,4); // пины под термопару

void setup()
{
  lcd.init(); //подключение экрана
  lcd.backlight();  //подключение экрана

  Serial.begin(9600);
  Serial.setTimeout(50);

  pinMode(Relay, OUTPUT); //ПОДКЛЮЧЕНИЕ РЕЛЕ
  pinMode(Vent, OUTPUT);
  digitalWrite(Relay, RELAY_OFF); //РЕЛЕ ПО УМОЛЧАНИЮ ВЫКЛЮЧЕН
  digitalWrite(Vent, RELAY_OFF);

  PIDServo.attach(9); // ПОДКЛЮЧЕНИЕ ПИНА ДЛЯ ПИД-СЕРВОПРИВОДА
  regulator.setDirection(NORMAL); //НАПРАВЛЕНИЕ РЕГУЛИРОВАНИЯ
  regulator.setLimits(0, 180);  //ПРЕДЕЛЫ УСТАНОВКИ РЕГУЛЯТОРА
  regulator.setpoint = SETPOINT;  //УСТАНОВКА ДЛЯ ПОДДЕРЖАНИИ ТЕМПЕРАТУРЫ
  delay(500);
}

void loop()
{
  static uint32_t tmr;
  
  static byte val = 0;
  if (millis() - tmr > 300) {
    tmr = millis();


    int temp = thermocouple.readCelsius(); //переменная значения температуры
    regulator.input = temp; //СООБЩАЕМ РЕГУЛЯТОРУ ТЕКУЩУЮЮ ТЕМПЕРАТУРУ
    float value = regulator.output;
    regulator.getResult();  //РАСЧЁТ ВЫХОДНОГО СИГНАЛА НА ПИД-СЕРВОПРИВОД
 
    if (regulator.output > 0)
    {
      digitalWrite(Relay, RELAY_ON);  //ЕСЛИ УПРАВЛЯЮЩИЙ СИГНАЛ БОЛЬШЕ 0,ВКЛЮЧАЕПМ РЕЛЕ
    }
    else
    {
      digitalWrite(Relay, RELAY_OFF); //ЕСЛИ МЕНЬШЕ 0, ВЫКЛЮЧАЕМ РЕЛЕ
    }
    if (temp - SETPOINT > 0)
    {
      digitalWrite(Vent, RELAY_ON);  //ЕСЛИ УПРАВЛЯЮЩИЙ СИГНАЛ БОЛЬШЕ 0,ВКЛЮЧАЕПМ РЕЛЕ
    }
    else
    {
      digitalWrite(Vent, RELAY_OFF); //ЕСЛИ МЕНЬШЕ 0, ВЫКЛЮЧАЕМ РЕЛЕ
    }

    PIDServo.write(regulator.output);  //ЗАПИСЫВАЕМ УПРАВЛЯЮЩИЙ СИГНАЛ В СЕРВОПРИВОД

    Serial.print(regulator.input);
    Serial.print(' ' );
    Serial.print(regulator.Kp);
    Serial.print(' ' );
    Serial.print(regulator.Ki);
    Serial.print(' ' );
    Serial.print(regulator.Kd);
    Serial.println(' ' );

    lcd.setCursor(0,0);
    lcd.print("T: ");
    lcd.print(temp); //переедаем на экран температуру
    lcd.print((char) 223);
    lcd.print("C ");
    lcd.print("S: ");
    lcd.print((int) SETPOINT);  //передаем на экран установку
    lcd.print((char) 223);
    lcd.print("C");

    lcd.setCursor(0,1);
    lcd.print(value); //выводим показания положения сервопривода в терминал
    
  }
  parsing();
}

void parsing()  {
    if (Serial.available() > 1) {
    char incoming = Serial.read();
    float value = Serial.parseFloat();
    switch (incoming) {
      case 'p': regulator.Kp = value;
        break;
      case 'i': regulator.Ki = value;
        break;
      case 'd': regulator.Kd = value;
        break;      
   }
    }
}