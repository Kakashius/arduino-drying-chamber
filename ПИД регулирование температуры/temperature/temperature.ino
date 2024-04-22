//Инициализация библиотек, объектов и констант
#include "max6675.h"  //Инициализация библиотеки датчика-термопары
#define thermoCLK 10  // пин термопары  ВЫБРАТЬ САМОМУ!
#define thermoCS 11   // пин термопары  ВЫБРАТЬ САМОМУ!
#define thermoDO 12   // пин термопары  ВЫБРАТЬ САМОМУ!
MAX6675 thermo1(thermoCLK, thermoCS, thermoDO);  //Инициализация объекта датчика температуры №1

#include "DHT11.h"  //Инициализация библиотеки датчика температуры и влажности
DHT11 thermo2Humid1(<ВЫБРАТЬ ПИН>) //Инициализация объекта датчика температуры №2 и влажности
DHT11 humid2(<ВЫБРАТЬ ПИН>) //Второй датчик влажности

#define PID_OPTIMIZED_I //Оптимизация интегральной суммы
#include "GyverPID.h" //Инициализация библиотеки ПИД-регулирования
GuyverPID thermoRegulator; //Инициализация объекта регулирования температуры
#define THERMOSETPOINT 60.0 //Установочная температура - БУДЕТ МЕНЯТЬСЯ В ИНТЕРФЕЙСЕ!!!
GuyverPID humidRegulator; //Инициализация объекта регулирования влажности
#define HUMIDSETPOINT 50.0 //Установочная влажность - БУДЕТ МЕНЯТЬСЯ В ИНТЕРФЕЙСЕ!!!

#include "PWMrelay.h" //Инициализация библиотеки управления твердотельным реле
PWMrelay heater(<ВЫБРАТЬ ПИН>);  //Инициализация объекта твердотельного реле

#include "Servo.h"  //Инициализация библиотеки управления сервоприводами
Servo servo1; //Инициализация объекта регулирования сервоприводом №1
Servo servo2; //Инициализация объекта регулирования сервоприводом №2

#include <LiquidCrystal_I2C.h>  //Инициализация библиотеки дисплея I2C
LiquidCrystal_I2C lcd(0x27,16,2); // пины А4, А5 под дисплей I2C

uint32_t thermoTmr, humidTmr, lcdTmr; //Инициализация переменных таймера: нагрев, влажность, дисплей
float temperature, humidity;  //Объявление переменных для хранения значений температуры и влажности

void setup() {
  thermoRegulator.setDirection(NORMAL); //Направление регулирования: чем дальше установки, тем мощнее управляющий сигнал
  thermoRegulator.setLimits(0, 255);  // пределы для выходного сигнала ШИМ
  thermoRegulator.setpoint = THERMOSETPOINT;  //Установка для ПИД-регулирования нагрева - БУДЕТ МЕНЯТЬСЯ В ИНТЕРФЕЙСЕ!
  thermoRegulator.Kp = 5; //Коэффициент Kp
  thermoRegulator.Ki = 0; //Коэффициент Ki  - СНАЧАЛА КАЛИБРУЕМИ!!!
  thermoRegulator.Kd = 0; //Коэффициент Kd

  humidRegulator.setDirection(REVERSE);
  humidRegulator.setLimits(0, 90);
  humidRegulator.setpoint = HUMIDSETPOINT;
  humidRegulator.Kp = 5;  //Коэффициент Kp
  humidRegulator.Ki = 0;  //Коэффициент Ki  - СНАЧАЛА КАЛИБРУЕМИ!!!
  humidRegulator.Kd = 0;  //Коэффициент Kd

  heater.setPWM(255); //Инициализируем начальное хначение ШИМ-сигнала на реле

  servo1.attach(<ВЫБРАТЬ ПИН>); //Установка пина на сервопривод №1
  servo2.attach(<ВЫБРАТЬ ПИН>); //Установка пина на сервопривод №2

  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.flush();
  Serial.println("Setpoint CurrentTemp PWMsignal"); //Отладка и плоттер температуры камеры
  //Serial.println("Setpoint CurrentHumid servoSignal"); Отладка и плоттер влажности камеры

  lcd.init(); //Инициализация работы дисплея I2C
  lcd.backlight();

  delay(500); //Задержка перед началом работы цикла
}

void loop() {
  thermoRegulation();
  //humidRegulation();

}

float avrVal(float val1, float val2 = 0) {  //Функция ср.значения датчиков температур
  return (val1 + val2) / 2.0;
}

void thermoRegulation() { //Функция регулирования нагрева и её отладка
  if (millis() - thermoTmr >= 1000) {
    thermoTmr = millis();
    temperature = avrVal(thermo1.readCelsius(), thermo2Humid1.readTemperature());
    thermoRegulator.input = temperature;
    thermoRegulator.getResult();
    int pwmOutput = thermoRegulator.output;
    heater.setPWM(pwmOutput);
    heater.tick();

    Serial.print(THERMOSETPOINT);
    Serial.print(" ");
    Serial.print(temperature);
    Serial.print(" ");
    Serial.println(pwmOutput);
  }
}

void humidRegulation() {  //Функция регулирования влажности и её отладка
  if (millis() - humidTmr >= 50) {
    humidTmr = millis();
    humidity = avrVal(thermo2Humid1.readHumidity(), humid2.readHumidity());
    humidRegulator.input = humidity;
    humidRegulator.getResult();
    int servoOutput = humidRegulator.output;
    servo1.write(servoOutput);
    servo2.write(servoOutput);

    /* Serial.print(HUMIDSETPOINT);
    Serial.print(" ");
    Serial.print(humidity);
    Serial.print(" ");
    Serial.println(servoOutput); ДЛЯ ОТЛАДКИ ВЛАЖНОСТИ*/
  }
}

void lcdScreen() {  //Функция передачи значений дисплею I2C
  if (millis() - lcdTmr >= 1000) {
    lcdTmr = millis();
    lcd.setCursor(0, 0);
    lcd.print("Curr:")
    lcd.setCursor(6, 0);
    lcd.print((int) temperature);
    lcd.setCursor(9, 0);
    lcd.print((char) 223);
    lcd.print("C");
    lcd.setCursor(12, 0);
    lcd.print((int) humidity);
    lcd.setCursor(15, 0);
    lcd.print("%");

    lcd.setCursor(0,1);
    lcd.print("Set:")
    lcd.setCursor(6, 1);
    lcd.print((int) THERMOSETPOINT);
    lcd.setCursor(9, 1);
    lcd.print((char) 223);
    lcd.print("C");
    lcd.setCursor(12, 1);
    lcd.print((int) HUMIDSETPOINT);
    lcd.setCursor(15, 1);
    lcd.print("%");
  }
}