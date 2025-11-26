#include <PID_v1.h>
#include <MeanFilterLib.h>
#include <TimerOne.h>

// ========== CONSTANTES ==========
const float Pi = 3.14159265;
const float dt = 2.5; // Milisegundos

// Motor 1 - Velocidad
const uint16_t cuentas_max_M1 = 6533;

// Motor 2 - Posición
const float cuentas_max_M2 = 145.1;
const uint16_t mm_por_rev_M2 = 8;

// ========== MOTOR 1 - VELOCIDAD ==========
// Variables lectura encoder Motor 1
volatile int32_t cuentas_M1 = 0;
volatile int8_t last_state_M1 = 0, current_state_M1 = 0;

// Variables cálculo velocidad Motor 1
MeanFilter<float> meanFilter_M1(10);
float current_position_M1 = 0.0, last_position_M1 = 0.0;
float velocidad_M1 = 0.0;
double velocidadf_M1 = 0.0;
double RPM_M1 = 0.0, RPM_s_M1 = 0.0;

// Variables Control PID Velocidad Motor 1
float Kp_M1 = 5.0, Ki_M1 = 10.0, Kd_M1 = 0.0;
double PWM_M1 = 0.0, Setpoint_M1 = 0.0, Abs_Setpoint_M1 = 0.0;
double Prcntg_PWM_M1 = 0.0, Voltaje_M1 = 0.0;
int U_M1 = 0;
PID VelPID_M1(&RPM_M1, &PWM_M1, &Abs_Setpoint_M1, Kp_M1, Ki_M1, Kd_M1, DIRECT);

// Pines Motor 1
int Pin_encoder_A_M1 = 3, Pin_encoder_B_M1 = 2;
int Pin_DIR_M1 = 6, Pin_PWM_M1 = 5;

// ========== MOTOR 2 - POSICIÓN ==========
// Variables lectura encoder Motor 2
volatile int32_t cuentas_M2 = 0;
volatile int8_t last_state_M2 = 0, current_state_M2 = 0;

// Variables cálculo posición Motor 2
float current_position_M2 = 0.0;
double posicionf_M2 = 0.0;

// Variables Control PID Posición Motor 2
float Kp_M2 = 40.0, Ki_M2 = 10.0, Kd_M2 = 0.0;
double PWM_M2 = 0.0, Setpoint_M2 = 0.0, Abs_Setpoint_M2 = 0.0;
double Error_pos_M2 = 0.0;
double tolerancia_M2 = 0.1;
double Prcntg_PWM_M2 = 0.0, Voltaje_M2 = 0.0;
int U_M2 = 0;
PID PosPID_M2(&posicionf_M2, &PWM_M2, &Abs_Setpoint_M2, Kp_M2, Ki_M2, Kd_M2, DIRECT);

// Pines Motor 2
int Pin_encoder_A_M2 = 20, Pin_encoder_B_M2 = 21;
int Pin_DIR_M2 = 8, Pin_PWM_M2 = 9;

// ========== COMUNICACIÓN SERIAL ==========
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

void setup()
{
  Serial.begin(9600);

  // ===== Configuración Motor 1 =====
  pinMode(Pin_encoder_A_M1, INPUT);
  pinMode(Pin_encoder_B_M1, INPUT);
  pinMode(Pin_DIR_M1, OUTPUT);
  pinMode(Pin_PWM_M1, OUTPUT);

  // ===== Configuración Motor 2 =====
  pinMode(Pin_encoder_A_M2, INPUT_PULLUP);
  pinMode(Pin_encoder_B_M2, INPUT_PULLUP);
  pinMode(Pin_DIR_M2, OUTPUT);
  pinMode(Pin_PWM_M2, OUTPUT);

  // Configurar frecuencias PWM
  TCCR0B = TCCR0B & B11111000 | B00000010;  // 7812.50 Hz para pines 5 y 6 (Motor 1)
  TCCR2B = TCCR2B & B11111000 | B00000001;  // ~31 kHz para pines 9 y 10 (Motor 2)

  // Configurar PID Motor 1 (Velocidad)
  VelPID_M1.SetMode(AUTOMATIC);
  VelPID_M1.SetTunings(Kp_M1, Ki_M1, Kd_M1);
  VelPID_M1.SetSampleTime(dt);
  VelPID_M1.SetOutputLimits(0, 255);

  // Configurar PID Motor 2 (Posición)
  PosPID_M2.SetMode(AUTOMATIC);
  PosPID_M2.SetTunings(Kp_M2, Ki_M2, Kd_M2);
  PosPID_M2.SetOutputLimits(-255, 255);

  // Configurar interrupciones
  cli();
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_A_M1), lectura_encoder_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_B_M1), lectura_encoder_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_A_M2), lectura_encoder_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_B_M2), lectura_encoder_M2, CHANGE);
  Timer1.initialize(dt * 1000);
  Timer1.attachInterrupt(ISR_Timer);
  sei();

  Serial.println("Sistema iniciado - Envíe comandos V1: o P2:");
}

void loop()
{
  LeerSerial();
  ProcesarComandos();
  EnviarDatos();
}

void ISR_Timer()
{
  // ===== CONTROL MOTOR 1 - VELOCIDAD =====
  VelPID_M1.Compute();
  U_M1 = PWM_M1;
  Prcntg_PWM_M1 = (PWM_M1 / 255.0) * 100.0;
  Voltaje_M1 = 12 * sqrt(PWM_M1 / 255.0);

  if (Setpoint_M1 > 0) {
    digitalWrite(Pin_DIR_M1, LOW);
    analogWrite(Pin_PWM_M1, U_M1);
  }
  else if (Setpoint_M1 < 0) {
    digitalWrite(Pin_DIR_M1, HIGH);
    analogWrite(Pin_PWM_M1, U_M1);
    Voltaje_M1 = -Voltaje_M1;
  }
  else {
    digitalWrite(Pin_DIR_M1, LOW);
    analogWrite(Pin_PWM_M1, 0);
  }

  // Calcular velocidad Motor 1
  current_position_M1 = (cuentas_M1 * 2 * Pi) / cuentas_max_M1;
  velocidad_M1 = ((current_position_M1 - last_position_M1) / (dt) * 1000);
  velocidad_M1 = meanFilter_M1.AddValue(velocidad_M1);
  RPM_s_M1 = (velocidad_M1 / (2 * PI)) * 60;
  velocidadf_M1 = abs(velocidad_M1);
  RPM_M1 = (velocidadf_M1 / (2 * PI)) * 60;
  last_position_M1 = current_position_M1;

  // ===== CONTROL MOTOR 2 - POSICIÓN =====
  PosPID_M2.Compute();
  U_M2 = abs(PWM_M2);
  Prcntg_PWM_M2 = (PWM_M2 / 255.0) * 100.0;
  Voltaje_M2 = 12 * sqrt(abs(PWM_M2) / 255.0);

  Error_pos_M2 = Setpoint_M2 - current_position_M2;

  if (PWM_M2 < 0.0 && abs(Error_pos_M2) > tolerancia_M2) {
    digitalWrite(Pin_DIR_M2, HIGH);
    analogWrite(Pin_PWM_M2, U_M2);
    Prcntg_PWM_M2 = -Prcntg_PWM_M2;
    Voltaje_M2 = -Voltaje_M2;
  }
  else if (PWM_M2 > 0.0 && abs(Error_pos_M2) > tolerancia_M2) {
    digitalWrite(Pin_DIR_M2, LOW);
    analogWrite(Pin_PWM_M2, U_M2);
  }
  else if (abs(Error_pos_M2) < tolerancia_M2) {
    digitalWrite(Pin_DIR_M2, LOW);
    analogWrite(Pin_PWM_M2, 0);
    Prcntg_PWM_M2 = 0.0;
    Voltaje_M2 = 0.0;
  }

  // Calcular posición actual Motor 2 en cm
  current_position_M2 = (cuentas_M2 / cuentas_max_M2) * mm_por_rev_M2 / 10.0;
  posicionf_M2 = abs(current_position_M2);
}

// ========== INTERRUPCIONES ENCODER ==========
void lectura_encoder_M1()
{
  if (digitalRead(Pin_encoder_A_M1) == 1)
    bitSet(current_state_M1, 1);
  else
    bitClear(current_state_M1, 1);

  if (digitalRead(Pin_encoder_B_M1) == 1)
    bitSet(current_state_M1, 0);
  else
    bitClear(current_state_M1, 0);

  if (last_state_M1 == 3 && current_state_M1 == 1) cuentas_M1++;
  if (last_state_M1 == 1 && current_state_M1 == 0) cuentas_M1++;
  if (last_state_M1 == 0 && current_state_M1 == 2) cuentas_M1++;
  if (last_state_M1 == 2 && current_state_M1 == 3) cuentas_M1++;

  if (last_state_M1 == 3 && current_state_M1 == 2) cuentas_M1--;
  if (last_state_M1 == 2 && current_state_M1 == 0) cuentas_M1--;
  if (last_state_M1 == 0 && current_state_M1 == 1) cuentas_M1--;
  if (last_state_M1 == 1 && current_state_M1 == 3) cuentas_M1--;

  last_state_M1 = current_state_M1;
}

void lectura_encoder_M2()
{
  if (digitalRead(Pin_encoder_A_M2) == 1)
    bitSet(current_state_M2, 1);
  else
    bitClear(current_state_M2, 1);

  if (digitalRead(Pin_encoder_B_M2) == 1)
    bitSet(current_state_M2, 0);
  else
    bitClear(current_state_M2, 0);

  if (last_state_M2 == 3 && current_state_M2 == 1) cuentas_M2++;
  if (last_state_M2 == 1 && current_state_M2 == 0) cuentas_M2++;
  if (last_state_M2 == 0 && current_state_M2 == 2) cuentas_M2++;
  if (last_state_M2 == 2 && current_state_M2 == 3) cuentas_M2++;

  if (last_state_M2 == 3 && current_state_M2 == 2) cuentas_M2--;
  if (last_state_M2 == 2 && current_state_M2 == 0) cuentas_M2--;
  if (last_state_M2 == 0 && current_state_M2 == 1) cuentas_M2--;
  if (last_state_M2 == 1 && current_state_M2 == 3) cuentas_M2--;

  last_state_M2 = current_state_M2;
}

// ========== COMUNICACIÓN SERIAL ==========
void LeerSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  if (Serial.available() > 0) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;
    }
  }
}

void ProcesarComandos() {
  if (newData == true) {
    newData = false;

    char* comando = receivedChars;
    
    // Buscar y procesar comando V1: (Velocidad Motor 1)
    char* pos_V1 = strstr(comando, "V1:");
    if (pos_V1 != NULL) {
      // Extraer el valor después de "V1:"
      float valor = atof(pos_V1 + 3);
      Setpoint_M1 = valor;
      Abs_Setpoint_M1 = abs(valor);
      Serial.print("V1 actualizado: ");
      Serial.println(Setpoint_M1);
    }

    // Buscar y procesar comando P2: (Posición Motor 2)
    char* pos_P2 = strstr(comando, "P2:");
    if (pos_P2 != NULL) {
      // Extraer el valor después de "P2:"
      float valor = atof(pos_P2 + 3);
      Setpoint_M2 = valor;
      Abs_Setpoint_M2 = abs(valor);
      Serial.print("P2 actualizado: ");
      Serial.println(Setpoint_M2);
    }

    // Si no se encuentra V1: ni P2:, mostrar mensaje
    if (pos_V1 == NULL && pos_P2 == NULL) {
      Serial.println("Comando no reconocido. Use V1: o P2:");
    }
  }
}

void EnviarDatos() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  
  // Enviar datos cada 100ms para no saturar el serial
  if (currentTime - lastPrintTime >= 100) {
    lastPrintTime = currentTime;
    
    Serial.print("M1[V:");
    Serial.print(Setpoint_M1);
    Serial.print(", RPM:");
    Serial.print(RPM_s_M1, 2);
    Serial.print(", PWM:");
    Serial.print(U_M1);
    Serial.print("] | M2[P:");
    Serial.print(Setpoint_M2);
    Serial.print(", Pos:");
    Serial.print(current_position_M2, 2);
    Serial.print("cm, Err:");
    Serial.print(Error_pos_M2, 2);
    Serial.println("]");
  }
}
