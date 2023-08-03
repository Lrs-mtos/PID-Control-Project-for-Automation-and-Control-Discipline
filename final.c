//****************************************
//  PID Control Project for Automation and Control Discipline
//  Demonstrating the use of controller coefficients (Kp, Ki, and Kd)
//****************************************

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 8 // WHITE
#define PWM 6
#define IN2 4
#define IN1 7
#define BTN 3 

int buttonState;
int lastButtonState = LOW;  

unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 300;  


void readEncoder();

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int btn_count = 0;

float kp = 0; 
float ki = 0; 
float kd = 0; 

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(BTN, INPUT);  
  attachInterrupt(digitalPinToInterrupt(BTN), interrupt_btn_fun, RISING); 

  Serial.println("target pos");
}

void loop() {
  int target = 0; -

  switch (btn_count) { 
    case 0:
      kp = 0;
      ki = 0;
      kd = 0;
      break;
    case 1:
      kp = 50;
      ki = 0;
      kd = 0;
      break;
    case 2:
      kp = 15;
      ki = 0.00055;
      kd = 0;
      break;
    case 3:
      kp = 15;
      ki = 0.35;
      kd = 20;
      break;
    default:
      break;
  }

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  int e = target - pos;

  float dedt = (e - eprev) / (deltaT);

  eintegral = eintegral + e * deltaT;

  float u = kp * e + kd * dedt + ki * eintegral;

  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  setMotor(dir, pwr, PWM, IN1, IN2);

  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void interrupt_btn_fun() {  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (btn_count >= 3) {
      btn_count = 0;
    } else {
      btn_count++;
    }
  }
  posi = 0;  pos
  lastDebounceTime = millis();
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}
