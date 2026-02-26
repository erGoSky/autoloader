#include <Arduino.h>

#define UB_DEB_TIME 50      // дебаунс
#define UB_HOLD_TIME 600    // время до перехода в состояние "удержание"
#define UB_STEP_TIME 400    // время до перехода в состояние "импульсное удержание"
#define UB_STEP_PRD 200     // период импульсов
#define UB_CLICK_TIME 500   // ожидание кликов
#include <uButton.h>

uButton A_IN(7);
uButton B_IN(8);
uButton A_OUT(12);
uButton B_OUT(13);
uButton COM_OUT(16);

// доступ за індексом (0..4) через цю функцію
static uButton &getBtn(uint8_t i) {
  switch (i) {
    case 0: return A_IN;
    case 1: return B_IN;
    case 2: return A_OUT;
    case 3: return B_OUT;
    case 4: return COM_OUT;
    default: return A_IN;
  }
}

// Архітектура: три задачі з чергами (Input -> Feedback -> Motors)
// Легка, кооперативна реалізація для Arduino (Nano ATmega328)

// --- Апаратура (підлаштуйте піни під вашу плату) ---
#define LEDPIN 10               // WS2812 data pin (1 адресний LED у прикладі)
#define BUZZER_PIN 11

// Драйвер моторів (DRV8833) — два мотори, кожен має два входи
/*
  A_IN1 | B_IN2 | FUNCTION
  PWM   | 0     | Forward PWM, fast decay
  1     | PWM   | Forward PWM, slow decay
  0     | PWM   | Reverse PWM, fast decay
  PWM   | 1     | Reverse PWM, slow decay
*/
const uint8_t M1_A = 3;   // PWM pin for motor1
const uint8_t M1_B = 5;   // PWM pin for motor1
const uint8_t M2_A = 6;   // PWM pin for motor2
const uint8_t M2_B = 9;   // PWM pin for motor2

// --- Коди подій та команд ---
enum EventCode : uint8_t {
  EVT_NONE = 0,
  EVT_LIMIT_CHANGED = 1, // параметр = індекс кінцевика (0..4), біт0=новий стан
};

struct Event {
  uint8_t code;
  uint8_t param; // наприклад: індекс кінцевика (0..4) та стан
};

struct MotorCmd {
  uint8_t motorId; // 1 або 2
  int8_t dir;      // -1, 0, +1
  uint32_t duration; // ms
};

// --- Простая кільцева черга (без динаміки) ---
template<typename T, size_t N>
struct RingQueue {
  T buf[N];
  volatile uint8_t head = 0;
  volatile uint8_t tail = 0;

  bool push(const T &v) {
    uint8_t next = (head + 1) % N;
    noInterrupts();
    if (next == tail) { // повна
      interrupts();
      return false;
    }
    buf[head] = v;
    head = next;
    interrupts();
    return true;
  }

  bool pop(T &out) {
    noInterrupts();
    if (head == tail) {
      interrupts();
      return false;
    }
    out = buf[tail];
    tail = (tail + 1) % N;
    interrupts();
    return true;
  }
};

// черги
RingQueue<Event, 16> eventQueue;
RingQueue<MotorCmd, 8> motorQueue;

// --- Стан кінцевиків для відстеження змін ---
bool limitsState[5];
unsigned long lastScan = 0;
const unsigned long SCAN_PERIOD = 50; // ms

// --- Стан моторів (нон-блокінг) ---
struct MotorState {
  bool running = false;
  uint32_t endTime = 0;
  int8_t dir = 0;
};
MotorState motors[2];

// --- Функції апаратних дій (сигнали) ---
void playBuzzerPattern(uint8_t code) {
  // Набір простих патернів: код визначає частоту/довжину
  if (code == 0) return;
  // Прості тональні сигнали
  tone(BUZZER_PIN, 1000, 100);
  delay(120);
}

// Прості LED-патерни для одного WS2812 (можна розширити через tinyLED)
void ledPatternForEvent(uint8_t code) {
  // Прості кольори через швидку реалізацію (якщо є бібліотека, підключіть її)
  if (code == 0) return;
  // Тут просто використаємо digital output як індикатор (placeholder)
  digitalWrite(LEDPIN, HIGH);
  delay(80);
  digitalWrite(LEDPIN, LOW);
}

// старі функції контролю моторів залишаємо заради сумісності, але
// вони тепер делегують до бібліотеки
void startMotor(const MotorCmd &cmd) {
  GMotor &m = (cmd.motorId == 1) ? motor1 : motor2;
  int16_t speed = 0;
  if (cmd.dir > 0) speed = 255;
  else if (cmd.dir < 0) speed = -255;
  m.setSpeed(speed);
}

void stopMotor(uint8_t motorId) {
  GMotor &m = (motorId == 1) ? motor1 : motor2;
  m.setSpeed(0);
}

// --- Завдання 1: Input/Logic ---
// тепер задача не сканує всі кінцевики подряд, а тільки ті, які
// необхідно опитати згідно з поточним станом машини.
// викликати `inputTask()` можна в loop() або зверху, але саме
// читання виконується через checkLimit(index).
void inputTask() {
  // в базовому каркасі нічого не робимо; логіку викликів треба
  // реалізувати у state-machine вищого рівня
}

// helper: перевірити конкретний кінцевик і відправити подію,
// якщо стан змінився
void checkLimit(uint8_t i) {
  if (i >= 5) return;
  uButton &btn = getBtn(i);
  bool changed = btn.tick();                // оновити стан за lib
  if (changed) {
    bool s = btn.pressing();                // чи зараз натиснута
    limitsState[i] = s;
    Event e;
    e.code = EVT_LIMIT_CHANGED;
    e.param = (i & 0x0F) | (s ? 0x80 : 0x00); // верхній біт = стан
    eventQueue.push(e);
  }
}

// --- Завдання 2: Feedback ---
void feedbackTask() {
  Event e;
  while (eventQueue.pop(e)) {
    if (e.code == EVT_LIMIT_CHANGED) {
      bool state = (e.param & 0x80);
      // Відобразити сигнал/LED
      playBuzzerPattern(e.code);
      ledPatternForEvent(e.code);
      // Перетворити подію в команду мотору — просте правило прикладу
      MotorCmd cmd;
      if (state) {
        // якщо кінцевик активований — зупинити мотор 1
        cmd.motorId = 1;
        cmd.dir = 0;
        cmd.duration = 0;
      } else {
        // якщо відпущено — рухати мотор 1 вперед 500ms
        cmd.motorId = 1;
        cmd.dir = 1;
        cmd.duration = 500;
      }
      motorQueue.push(cmd);
    }
  }
}

// --- Завдання 3: Motors ---
void motorsTask() {
  // обробка нових команд
  MotorCmd cmd;
  while (motorQueue.pop(cmd)) {
    if (cmd.motorId >= 1 && cmd.motorId <= 2) {
      uint8_t idx = cmd.motorId - 1;
      motors[idx].running = (cmd.dir != 0 && cmd.duration > 0);
      motors[idx].dir = cmd.dir;
      motors[idx].endTime = motors[idx].running ? (millis() + cmd.duration) : 0;
      // негайно застосувати напрям
      startMotor(cmd);
      if (!motors[idx].running) stopMotor(cmd.motorId);
    }
  }

  // завершення моторів по таймауту
  unsigned long now = millis();
  for (uint8_t i = 0; i < 2; ++i) {
    if (motors[i].running && (long)(motors[i].endTime - now) <= 0) {
      stopMotor(i + 1);
      motors[i].running = false;
    }
  }
}

void setupPins() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  // limit switches via uButton (named instances)
  for (uint8_t i = 0; i < 5; ++i) {
    uButton &btn = getBtn(i);
    limitsState[i] = btn.readButton(); // начальный физический уровень
  }
  // motors: initialize gyver objects
  motor1.setMode(STOP);
  motor2.setMode(STOP);
  // optionally set resolution/min duty etc
}

void setup() {
  setupPins();
}

void loop() {
  inputTask();
  feedbackTask();
  motorsTask();
}
