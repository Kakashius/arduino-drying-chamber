// Инициализация библиотек
#include <LiquidCrystal_I2C.h> // Дисплей
#include <max6675.h> // Термопара
#include <Servo.h>  // Сервопривод
#include <pidautotuner.h>

// Определение переменных
Servo PIDServo;  // Переменная сервопривода
#define SETPOINT 30.0  // УСТАНОВКА ДЛЯ РЕГУЛИРОВАНИЯ
#define RELAY_ON 1  // Реле включен
#define RELAY_OFF 0  // Реле выключен
#define Relay 10  // Пин под ПИД-реле
#define ventRelay 8  // Пин под вентимлятор
LiquidCrystal_I2C lcd(0x27,16,2); // Дисплей пины А4, А5
MAX6675 thermocouple(2,3,4); // Пины термопары

void setup() {
  // Инициализация дисплея
  lcd.init(); //подключение экрана
  lcd.backlight();  //подключение экрана

  // Инициализация порта Serial
  Serial.begin(9600);
  Serial.setTimeout(50);

  // Инициализация сервопривода
  PIDServo.attach(9);

  // Инициализация ПИД-реле и вентилятора 
  pinMode(Relay, OUTPUT);
  pinMode(ventRelay, OUTPUT);
  digitalWrite(Relay, RELAY_OFF);
  digitalWrite(ventRelay, RELAY_OFF);

  PIDAutotuner tuner = PIDAutotuner();
  tuner.setTargetInputValue(SETPOINT);  //Устанавливаем здесь SETPOINT
  tuner.setLoopInterval(200); // Устанавливаем время интервала выполнерния - должен совпадать с ПИД интервалом
  tuner.setOutputRange(0, 180); // Устанавливаем диапазон выходного значения для сервопривода
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID); // Метод Зиглера-Николсона
  tuner.startTuningLoop(micros());  // Старт тюнинга

  long microseconds;
  while (!tuner.isFinished()) {
    long prevMicroseconds = microseconds;
    microseconds = micros();
    double input = doSomethingToGetInput(); // Засовываем показания датчика
    double output = tuner.tunePID(input, microseconds); // Вывод регулирования
    doSomethingToSetOutput(output);

    while (micros() - microseconds < 200) delayMicroseconds(1); //Цикл выполнения программы должен совпадать со временем ПИД-контроля
  }  
  doSomethingToSetOutput(0);  //выключаем тюнер 

  // Получаем коэффициенты ПИД-регулятора установим их в наш регулятор
  double kp = tuner.getKp();
  double ki = tuner.getKi();
  double kd = tuner.getKd();
  delay(500);
}

void loop() {
  static uint32_t tmr;
  if (millis() - tmr > 300) {
    tmr = millis();

    int temp = thermocouple.readCelsius();  //Значение с датчика
    float value = computePID(temp, SETPOINT, kp, ki, kd, 0.2, 0, 180);
    if (value > 0)
    {
      digitalWrite(Relay, RELAY_ON);  //ЕСЛИ УПРАВЛЯЮЩИЙ СИГНАЛ БОЛЬШЕ 0,ВКЛЮЧАЕПМ РЕЛЕ
    }
    else
    {
      digitalWrite(Relay, RELAY_OFF); //ЕСЛИ МЕНЬШЕ 0, ВЫКЛЮЧАЕМ РЕЛЕ
    }
    if (temp > SETPOINT)
    {
      digitalWrite(ventRelay, RELAY_ON);  //ЕСЛИ УПРАВЛЯЮЩИЙ СИГНАЛ БОЛЬШЕ 0,ВКЛЮЧАЕПМ РЕЛЕ
    }
    else
    {
      digitalWrite(ventRelay, RELAY_OFF); //ЕСЛИ МЕНЬШЕ 0, ВЫКЛЮЧАЕМ РЕЛЕ
    }
    PIDServo.write(value);  //ЗАПИСЫВАЕМ УПРАВЛЯЮЩИЙ СИГНАЛ В СЕРВОПРИВОД

    Serial.print(kp);
    Serial.print(' ' );
    Serial.print(ki);
    Serial.print(' ' );
    Serial.print(kd);
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

// Функция ПИД-регулятора
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}