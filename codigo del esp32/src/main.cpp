#include <Arduino.h>
// Control PID de posicion de un motor DC usando el 
// Arduino Mega 
//  - Encoder en modo x4 usando interrupciones
//  - Driver TBG6612 modo brake
//  - PWM de 10bits, f=7.8kz aprox
//  - Se usa el timer 3 para generar la tasa de muestreo
//    del controlador
//  - Se usara los PWM del timer 1 (timer4)
//
//  JBG37-520   CPR=5,764

//---------------------------------------------------//
// 1. VARIABLES GLOBALES
//---------------------------------------------------//
// 1.1. ENCODER (Pines disponibles 2, 3, 18, 19)
int M1_encoderA = 2;     // Yellow
int M1_encoderB = 4;     // White
volatile long M1_enc_count = 0;
byte M1_A_old = 0;
byte M1_B_old = 0;

int M2_encoderA = 18;     // Yellow
int M2_encoderB = 19;     // White
volatile long M2_enc_count = 0;
byte M2_A_old = 0;
byte M2_B_old = 0;

// 1.2. CONFIGURACION DEL PWM
int pin11_PWM = 13;   
int M1_pin_AN1 = 12;      
int M1_pin_AN2 = 14;
int PWM_RESOLUTION = 65535;  // 16 bits
// 1.3. PARAMETROS DEL MOTOR
float M1_CPR = 5764;         // Conteo en modo x4, lee los canales A y B
//
float M2_CPR = 5764;
long te=0;
volatile bool FLAG_TIMER = false;
hw_timer_t *timer = NULL;


//-------------------------------------------------------------------------//
// 2. VARIABLES DEL CONTROLADOR PID
//-------------------------------------------------------------------------//
//  2.1. TIEMPO DE MUESTREO
float Tms = 20;         // No es recomendable poner un valor mas pequeno
                        // por la resolucion del encoder
float Ts  = Tms/1000;   // En segundos
//  2.2. GANANCIAS
float Kp = 10;     // 3
float Ki = 1;     // 1
float Kd = 0.2;   // 0.001
//  2.3. VALORES DE SATURACION DE LA SENAL DE CONTROL
float outMax =  3.3;    // Vmax de salida del micro
float outMin = -3.3;
//  2.4. VARIABLES DEL CONTROLADOR
float Input1, Output1, Setpoint1;
float ITerm1 = 0;
float lastInput1 = 0;  // Asumo que parte del reposo
//  2.5. PARA ALMACENAR LA REFERENCIA ENVIADA DESDE LA PC
float M1_th_ref = 0;    // Set point (Angulo en radianes)
float M1_ang_old = 0;

float M2_th_ref = 0;    // Set point (Angulo en radianes)
float M2_ang_old = 0;

// Funciones
void IRAM_ATTR onTimer();
int set_direccion(int val_pwm, int pin_AN1, int pin_AN2);
void send_float(float arg);
void IRAM_ATTR M1_encoder_isr();
void IRAM_ATTR M2_encoder_isr();
void setup() 
{
  Serial.begin(115200);
  ledcSetup(0,5000,10);
  ledcAttachPin(13, 0);

  //-------------------------------------------//
  // CONFIGURAMOS ENCODER (X4)
  //-------------------------------------------//
  // LEEMOS EL ESTADO DEL ENCODER DEL MOTOR 1
  pinMode(M1_encoderA, INPUT);
  pinMode(M1_encoderA, INPUT);
  pinMode(M1_pin_AN1, OUTPUT);
  pinMode(M1_pin_AN2, OUTPUT);

  pinMode(M2_encoderA, INPUT);
  pinMode(M2_encoderB, INPUT);
  M1_A_old = digitalRead(M1_encoderA);
  M1_B_old = digitalRead(M1_encoderB);
  M2_A_old = digitalRead(M2_encoderA);
  M2_B_old = digitalRead(M2_encoderB);
  // ESTABLECEMOS LAS INTERRUPCIONES
  attachInterrupt(digitalPinToInterrupt(M1_encoderA),
                  M1_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_encoderB),
                  M1_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_encoderA),
                M2_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_encoderB),
                  M2_encoder_isr, CHANGE);  
    timer = timerBegin(0, 80, true); 
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 5000, true); // Configura el temporizador para 5 ms
  timerAlarmEnable(timer);
  pinMode(21,OUTPUT);
  digitalWrite(21,HIGH);
}


//---------------------------------------------------//
// 3. FUNCION PRINCIPAL
//---------------------------------------------------//
void loop() 
{
  //-------------------------------------------//
  // RECIBIMOS COMANDOS DESDE LA PC
  //-------------------------------------------//
  if(Serial.available())
  {
    // LEEMOS ANGULO EN SEXAGESIMAL
    String str = Serial.readStringUntil('\n');
    int TH_SEXAGESIMAL = str.toInt();
    // ESTABLECEMOS LA POSICION DE REFERENCIA
    M1_th_ref = TH_SEXAGESIMAL*PI/180;
    Output1=TH_SEXAGESIMAL;
  }
  if(FLAG_TIMER)
  {
    FLAG_TIMER = 0;
    noInterrupts();
    float M1_counter_encoder = M1_enc_count;
    float M2_counter_encoder = M2_enc_count;
    interrupts();
    //  A.2 HALLAMOS EL ANGULO CORRESPONDIENTE EN RADIANES
    float M1_ang = (M1_counter_encoder / M1_CPR)*2*PI;
    //  A.3 APROXIMAMOS LA VELOCIDAD (rad/sec)
    float M1_delta_ang = M1_ang - M1_ang_old;
    float M1_w = M1_delta_ang/Ts;
    //  A.4 GUARDAMOS EL ANGULO
    M1_ang_old = M1_ang;
    // CONTROL
    int M1_pwm_motor=Output1;
    M1_pwm_motor = set_direccion(M1_pwm_motor, M1_pin_AN1, M1_pin_AN2);
    ledcWrite(0, M1_pwm_motor);
  
    float M2_ang = (M2_counter_encoder*1.0 / 1200) *PI;
    float M2_delta_ang = M2_ang - M2_ang_old;
    float M2_w = M2_delta_ang / Ts;
    M2_ang_old = M2_ang;
  // ENVIAMOS INFO A LA PC
  //Serial.println(M1_ang*180/PI, 6);
   Serial.println(M2_ang);
  }
}

//---------------------------------------------------//
// 4. RUTINAS DE INTERRUPCION
//---------------------------------------------------//
void M1_encoder_isr()
{
  long t1 = micros();
  // LEEMOS LOS VALORES DEL CANAL A Y B
  //byte A = digitalRead(encoderA);
  //byte B = digitalRead(encoderB);  // 24,32us
  byte A = digitalRead(M1_encoderA);
  byte B = digitalRead(M1_encoderB);
  // https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Arduino.h
  // #define bitRead(value, bit) (((value) >> (bit)) & 0x01)
  
  // APLICAMOS LA LOGICA
  if(M1_A_old==0 && M1_B_old==0)
  {
    if(A==1)  // Cambio en A
      M1_enc_count++;
    if(B==1)  // Cambio en B
      M1_enc_count--;
  }
  else if(M1_A_old==0 && M1_B_old==1)
  {
    if(A==1)  // Cambio en A
      M1_enc_count--;
    if(B==0)  // Cambio en B
      M1_enc_count++;
  }
  else if (M1_A_old==1 && M1_B_old==0)
  {
    if(A==0)  // Cambio en A
      M1_enc_count--;
    if(B==1)  // Cambio en B
      M1_enc_count++;
  }
  else if (M1_A_old==1 && M1_B_old==1)
  {
    if(A==0)  // Cambio en A
      M1_enc_count++;
    if(B==0)  // Cambio en B
      M1_enc_count--;
  }
  // ACTUALIZAMOS
  M1_A_old = A;
  M1_B_old = B;
  // ANALIZAMOS TIEMPO
  long t2 = micros();
  te = t2-t1;
}

void IRAM_ATTR onTimer() {
  FLAG_TIMER = true;
}

void IRAM_ATTR M2_encoder_isr() {
  long t1 = micros();
  byte A = digitalRead(M2_encoderA);
  byte B = digitalRead(M2_encoderB);

  if (M2_A_old == 0 && M2_B_old == 0) {
    if (A == 1)
      M2_enc_count++;
    if (B == 1)
      M2_enc_count--;
  } else if (M2_A_old == 0 && M2_B_old == 1) {
    if (A == 1)
      M2_enc_count--;
    if (B == 0)
      M2_enc_count++;
  } else if (M2_A_old == 1 && M2_B_old == 0) {
    if (A == 0)
      M2_enc_count--;
    if (B == 1)
      M2_enc_count++;
  } else if (M2_A_old == 1 && M2_B_old == 1) {
    if (A == 0)
      M2_enc_count++;
    if (B == 0)
      M2_enc_count--;
  }
  M2_A_old = A;
  M2_B_old = B;
  long t2 = micros();
  te = t2 - t1;
}

int set_direccion(int val_pwm, int pin_AN1, int pin_AN2) {
  if (val_pwm == 0) {
    digitalWrite(pin_AN1, 1);
    digitalWrite(pin_AN2, 1);
  } else if (val_pwm > 0) {
    digitalWrite(pin_AN1, 1);
    digitalWrite(pin_AN2, 0);
  } else {
    val_pwm = -val_pwm;
    digitalWrite(pin_AN1, 0);
    digitalWrite(pin_AN2, 1);
  }
  return val_pwm;
}

//---------------------------------------------------//
// 5. FUNCIONES AUXILIARES
//---------------------------------------------------//

void send_float(float arg)
{
  byte * data = (byte *) &arg;
  Serial.write(data, sizeof (arg));
}