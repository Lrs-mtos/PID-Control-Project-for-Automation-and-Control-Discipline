//****************************************
//  Projeto de Automação e Controle:     *
//  Demonstração do uso dos coeficientes *
//  do controlador (Kp, Ki e Kd)         *
//**************************************** 

#define encA 2
#define encB 8
#define PWM 6
#define IN1 7
#define IN2 4
#define btn 8


int pos = 0;
long prevT = 0; 
float eprev = 0;
float eint = 0;

void interrupt_fun();
void set_motor(int dir, int pwm_value);
float calc_control_sig(int error,float deltaT, int kp,int kd, float ki);
void setup(){
  Serial.begin(115200);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(PWM,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encA), interrupt_fun, RISING);
}
 

void loop(){
  
  int pos_target = 0;
  int kp = 15;
  float ki =0.00055 ;
  int kd = 6;
  long currT;
  long prevT = 0; 
  
  currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;


//Calcular ERRO

  int e = pos - pos_target;


  float control_signal = calc_control_sig(e,deltaT,kp,kd,ki);

 
  float pwm_sig = fabs(control_signal);

  if(pwm_sig > 255){
    pwm_sig = 255;
  }
  int dir = 1;
  if(control_signal<0){
    dir = 0;
  }

  set_motor(dir,pwm_sig);

  eprev = e;

  Serial.print(pos);
  Serial.print(" ");
  Serial.print(eint);
  Serial.print(" ");
  //Serial.print(eint);
  //Serial.print(" ");
  Serial.println();

}



void set_motor(int dir, int pwm_value){
  
  if(dir == 1){
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    analogWrite(PWM,pwm_value);
  }else if(dir == 0){
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    analogWrite(PWM,pwm_value);
    
  }

}

float calc_control_sig(int error,float deltaT, int kp,int kd, float ki){

  float dedt = (error-eprev)/(deltaT);

  float ki_lim = 200000;

  if(!(eint > ki_lim || eint < -ki_lim)){
    
    eint += error*deltaT;
    
  }else{
    
    eint = 0;  
  }
  float sign = kp*error+kd*dedt+ki*eint;


  return sign;

}



void interrupt_fun(){

int b = digitalRead(encB);

if(b > 0){
pos++;
}else{
  pos--;
}