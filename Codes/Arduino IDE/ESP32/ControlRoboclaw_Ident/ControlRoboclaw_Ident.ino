#include <HardwareSerial.h>
#include "RoboClaw.h"

// ==================== CONFIGURACIÓN DE HARDWARE ====================

// Configuración de pines para ESP32
#define ROBOCLAW_RX 16  // Pin RX del ESP32 (conectar a TX del RoboClaw)
#define ROBOCLAW_TX 17  // Pin TX del ESP32 (conectar a RX del RoboClaw)
#define ROBOCLAW_ADDRESS 0x80  // Dirección del RoboClaw

// Configuración del encoder y motor 1 (rotacional - Crank)
const float ENCODER_CPR_M1 = 6533.0;   // CPR en la salida del gearbox

// Configuración del motor 2 (actuador lineal)
const float PULSES_PER_REV_M2 = 145.1;     // Pulsos por revolución del motor 2
const float CM_PER_REV = 0.8;              // 8mm = 0.8cm por revolución
const float PULSES_PER_CM = PULSES_PER_REV_M2 / CM_PER_REV;  // 181.375 pulsos/cm

// Límites del actuador lineal
const float MIN_POSITION_CM = 0.0;          // 0 cm (retraído)
const float MAX_POSITION_CM = 14.3;         // 14.3 cm (extendido)

// ==================== CONFIGURACIÓN DE PARÁMETROS ====================

// Tiempo que cada Setpoint se mantiene activo (en segundos)
const float tiempoPorSetpoint = 10;

// Intervalo de impresión por serial (en milisegundos)
const unsigned long PrintTime = 50;

// -------------------- SELECCIÓN DE MOTOR --------------------
// Descomentar UNA de las siguientes líneas según el motor a controlar

#define CONTROL_MOTOR1_VELOCIDAD    // Motor 1: Control de velocidad (RPM)
// #define CONTROL_MOTOR2_POSICION    // Motor 2: Control de posición (cm)

// -------------------- MODO MANUAL --------------------
// Descomentar esta sección para usar valores definidos manualmente
// Comentar la sección de MODO ALEATORIO

#define MODO_MANUAL
#ifdef MODO_MANUAL
  #ifdef CONTROL_MOTOR1_VELOCIDAD
    // Setpoints de VELOCIDAD en RPM para Motor 1 (Crank)
    // Valores positivos = sentido horario, negativos = antihorario
    float valoresSetpoint[] = {20, 30, 40, 50, 60, 50, 40, 30, 20};
  #endif
  
  #ifdef CONTROL_MOTOR2_POSICION
    // Setpoints de POSICIÓN en cm para Motor 2 (Actuador lineal)
    // Rango válido: 0 a 14.3 cm
    float valoresSetpoint[] = {2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 10.0, 8.0, 6.0, 4.0};
  #endif
  
  const int cantidadValores = sizeof(valoresSetpoint) / sizeof(valoresSetpoint[0]);
#endif

// -------------------- MODO ALEATORIO --------------------
// Descomentar esta sección para usar valores aleatorios
// Comentar la sección de MODO MANUAL

//#define MODO_ALEATORIO
#ifdef MODO_ALEATORIO
  #ifdef CONTROL_MOTOR1_VELOCIDAD
    // Rango de velocidad en RPM (considerar signo)
    const float SETPOINT_MIN = 20.0;    // RPM mínimo (puede ser negativo)
    const float SETPOINT_MAX = 60.0;     // RPM máximo
    const float ZONA_MUERTA = 10.0;      // Excluir valores entre -10 y 10 RPM
  #endif
  
  #ifdef CONTROL_MOTOR2_POSICION
    // Rango de posición en cm
    const float SETPOINT_MIN = 1.0;      // Posición mínima en cm
    const float SETPOINT_MAX = 13.0;     // Posición máxima en cm
    const float ZONA_MUERTA = 0.0;       // No hay zona muerta para posición
  #endif
  
  const int cantidadValores = 8;        // Cantidad de valores a generar
  float valoresSetpoint[cantidadValores];
#endif

// Velocidad del actuador lineal para cambios de posición (cm/s)
const float VELOCIDAD_ACTUADOR = 5.0;

// ==================== FIN CONFIGURACIÓN ====================

// Variables RoboClaw
int32_t enc1 = 0, enc2 = 0, speed1 = 0, speed2 = 0;
uint8_t status1, status2, status3, status4;
bool valid1, valid2, valid3, valid4;
// Variables para PWM
int16_t pwm1 = 0, pwm2 = 0;

// Configuración PID
const uint32_t ACCEL = 10000;   // Aceleración en pulsos/s^2
const uint32_t DECEL = 10000;   // Deceleración en pulsos/s^2

// Crear objeto de comunicación serial y RoboClaw
HardwareSerial RoboclawSerial(2);  // Usar UART2 del ESP32
RoboClaw roboclaw(&RoboclawSerial, 10000);  // 10ms timeout

// Variables Serial
String inputString = "";
bool stringComplete = false;

// Variables de tiempo
unsigned long currentTime = 0, lastPrintTime = 0;

// Variables para secuencia de Setpoints
bool secuenciaActiva = false;
int indiceActual = 0;
unsigned long tiempoInicioSetpoint = 0;
unsigned long tiempoInicioSecuencia = 0;
unsigned long duracionSetpoint_ms;
float setpointActual = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Inicializar semilla para números aleatorios
  randomSeed(analogRead(0));
  
  // Calcular duración en milisegundos
  duracionSetpoint_ms = (unsigned long)(tiempoPorSetpoint * 1000.0);
  
  // Iniciar UART2 con los pines especificados
  RoboclawSerial.begin(38400, SERIAL_8N1, ROBOCLAW_RX, ROBOCLAW_TX);
  
  // Iniciar comunicación con RoboClaw
  roboclaw.begin(38400);
  
  delay(2000);
  
  // Resetear encoders al iniciar
  roboclaw.ResetEncoders(ROBOCLAW_ADDRESS);
  
  // Generar valores aleatorios si está en modo aleatorio
  #ifdef MODO_ALEATORIO
    generarValoresAleatorios();
  #endif
  
  // Mostrar configuración inicial
  printConfig();
}

void loop() {
  // Leer comandos del monitor serial
  leerSerial();
  
  // Procesar comando cuando esté completo
  if (stringComplete) {
    procesarComando();
    inputString = "";
    stringComplete = false;
  }
  
  // Ejecutar secuencia si está activa
  ejecutarSecuencia();
  
  // Imprimir datos periódicamente
  currentTime = millis();
  if (currentTime - lastPrintTime >= PrintTime) {
    lastPrintTime = currentTime;
    if (secuenciaActiva) {
      enviarDatos();
    }
  }
}

void leerSerial() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void procesarComando() {
  inputString.trim();
  inputString.toUpperCase();
  
  if (inputString == "S") {
    iniciarSecuencia();
  }
  else if (inputString == "D") {
    detenerSecuencia();
  }
  else if (inputString == "R") {
    roboclaw.ResetEncoders(ROBOCLAW_ADDRESS);
    Serial.println("Encoders reseteados a 0");
  }
  else if (inputString == "L") {
    leerEstado();
  }
  else if (inputString == "H" || inputString == "?") {
    printConfig();
  }
}

void iniciarSecuencia() {
  if (!secuenciaActiva) {
    secuenciaActiva = true;
    indiceActual = 0;
    tiempoInicioSetpoint = millis();
    tiempoInicioSecuencia = millis();
    
    // Aplicar primer setpoint
    aplicarSetpoint(valoresSetpoint[indiceActual]);
    
    Serial.println("I"); // Indica inicio de secuencia
  }
}

void detenerSecuencia() {
  secuenciaActiva = false;
  indiceActual = 0;
  
  // Detener motores en posición actual
  detenerMotores();
  
  Serial.println("D"); // Confirma detención
}

void ejecutarSecuencia() {
  if (!secuenciaActiva) return;
  
  unsigned long tiempoActual = millis();
  
  // Verificar si es tiempo de cambiar al siguiente setpoint
  if (tiempoActual - tiempoInicioSetpoint >= duracionSetpoint_ms) {
    indiceActual++;
    
    // Verificar si terminó la secuencia
    if (indiceActual >= cantidadValores) {
      finalizarSecuencia();
      return;
    }
    
    // Aplicar siguiente setpoint
    tiempoInicioSetpoint = tiempoActual;
    aplicarSetpoint(valoresSetpoint[indiceActual]);
  }
}

void aplicarSetpoint(float valor) {
  setpointActual = valor;
  
  #ifdef CONTROL_MOTOR1_VELOCIDAD
    // Convertir RPM a pulsos por segundo
    int32_t pulsesPerSecond = rpmToPulsesPerSecond(valor);
    roboclaw.SpeedAccelM1(ROBOCLAW_ADDRESS, ACCEL, pulsesPerSecond);
  #endif
  
  #ifdef CONTROL_MOTOR2_POSICION
    // Verificar límites
    if (valor < MIN_POSITION_CM) valor = MIN_POSITION_CM;
    if (valor > MAX_POSITION_CM) valor = MAX_POSITION_CM;
    
    // Convertir cm a pulsos (negativo porque el motor va en reversa para extender)
    int32_t encoderCounts = -(int32_t)(valor * PULSES_PER_CM);
    uint32_t speed = (uint32_t)(VELOCIDAD_ACTUADOR * PULSES_PER_CM);
    
    roboclaw.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, 
                                         ACCEL/2,
                                         speed, 
                                         DECEL/2, 
                                         encoderCounts, 
                                         1);
  #endif
}

void finalizarSecuencia() {
  secuenciaActiva = false;
  indiceActual = 0;
  
  // Detener motores en posición actual
  detenerMotores();
  
  Serial.println("F"); // Indica fin de secuencia
}

void detenerMotores() {
  // Detener Motor 1 (velocidad = 0)
  roboclaw.SpeedAccelM1(ROBOCLAW_ADDRESS, ACCEL, 0);
  
  // Detener Motor 2 en posición actual
  roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
  
  setpointActual = 0;
}

void enviarDatos() {
  // Calcular tiempo transcurrido en segundos
  unsigned long tiempoTranscurrido = millis() - tiempoInicioSecuencia;
  float tiempoSegundos = tiempoTranscurrido / 1000.0;
  
  // Leer valores actuales
  float valorReal = 0.0;
  float pwmPorcentaje = 0.0;
  
  // Leer PWM de ambos motores
  roboclaw.ReadPWMs(ROBOCLAW_ADDRESS, pwm1, pwm2);
  
  #ifdef CONTROL_MOTOR1_VELOCIDAD
    speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status1, &valid1);
    if (valid1) {
      valorReal = pulsesPerSecondToRPM(speed1);
    }
    // PWM en porcentaje (-100% a 100%)
    pwmPorcentaje = (pwm1 / 32767.0) * 100.0;
  #endif
  
  #ifdef CONTROL_MOTOR2_POSICION
    enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
    if (valid2) {
      valorReal = -(float)enc2 / PULSES_PER_CM;
    }
    // PWM en porcentaje (-100% a 100%)
    pwmPorcentaje = (pwm2 / 32767.0) * 100.0;
  #endif
  
  // Imprimir: Tiempo Setpoint ValorReal PWM(%)
  Serial.print(tiempoSegundos, 3);
  Serial.print(" ");
  Serial.print(setpointActual, 2);
  Serial.print(" ");
  Serial.print(valorReal, 2);
  Serial.print(" ");
  Serial.println(pwmPorcentaje, 2);
}

void leerEstado() {
  Serial.println("\n--- Estado Actual ---");
  
  // Motor 1
  enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
  speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status3, &valid3);
  
  if (valid1) {
    float degrees = encoderCountsToDegrees(enc1);
    Serial.print("Motor 1: ");
    Serial.print(enc1);
    Serial.print(" pulsos (");
    Serial.print(degrees, 1);
    Serial.println(" grados)");
  }
  
  if (valid3) {
    float rpm = pulsesPerSecondToRPM(speed1);
    Serial.print("Velocidad Motor 1: ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");
  }
  
  // Motor 2
  enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
  speed2 = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &status4, &valid4);
  
  if (valid2) {
    float cm = -(float)enc2 / PULSES_PER_CM;
    Serial.print("Actuador: ");
    Serial.print(enc2);
    Serial.print(" pulsos (");
    Serial.print(cm, 2);
    Serial.println(" cm)");
  }
  
  if (valid4) {
    float cm_s = -(float)speed2 / PULSES_PER_CM;
    Serial.print("Velocidad Actuador: ");
    Serial.print(cm_s, 2);
    Serial.println(" cm/s");
  }
  
  Serial.println("--------------------\n");
}

// ==================== FUNCIONES DE CONVERSIÓN ====================

int32_t rpmToPulsesPerSecond(float rpm) {
  float revolutionsPerSecond = rpm / 60.0;
  return (int32_t)(revolutionsPerSecond * ENCODER_CPR_M1);
}

float pulsesPerSecondToRPM(int32_t pulsesPerSecond) {
  float revolutionsPerSecond = (float)pulsesPerSecond / ENCODER_CPR_M1;
  return revolutionsPerSecond * 60.0;
}

int32_t degreesToEncoderCounts(float degrees) {
  float countsPerDegree = ENCODER_CPR_M1 / 360.0;
  return (int32_t)(degrees * countsPerDegree);
}

float encoderCountsToDegrees(int32_t counts) {
  float countsPerDegree = ENCODER_CPR_M1 / 360.0;
  return (float)counts / countsPerDegree;
}

// ==================== GENERACIÓN DE VALORES ALEATORIOS ====================

#ifdef MODO_ALEATORIO
void generarValoresAleatorios() {
  for (int i = 0; i < cantidadValores; i++) {
    float valor = 0.0;
    
    // Generar valor aleatorio entre SETPOINT_MIN y SETPOINT_MAX
    // Excluyendo la zona muerta si aplica
    do {
      // random() trabaja con enteros, escalamos por 100 para tener decimales
      long valorInt = random((long)(SETPOINT_MIN * 100), (long)(SETPOINT_MAX * 100) + 1);
      valor = valorInt / 100.0;
    } while (ZONA_MUERTA > 0 && valor > -ZONA_MUERTA && valor < ZONA_MUERTA);
    
    valoresSetpoint[i] = valor;
  }
}
#endif

// ==================== MOSTRAR CONFIGURACIÓN ====================

void printConfig() {
  Serial.println("\n============================================");
  Serial.println("=== Sistema de Identificación - RoboClaw ===");
  Serial.println("============================================");
  
  #ifdef CONTROL_MOTOR1_VELOCIDAD
    Serial.println("Motor: 1 (Crank)");
    Serial.println("Tipo control: VELOCIDAD (RPM)");
  #endif
  
  #ifdef CONTROL_MOTOR2_POSICION
    Serial.println("Motor: 2 (Actuador Lineal)");
    Serial.println("Tipo control: POSICIÓN (cm)");
    Serial.print("Velocidad actuador: ");
    Serial.print(VELOCIDAD_ACTUADOR);
    Serial.println(" cm/s");
  #endif
  
  #ifdef MODO_MANUAL
    Serial.println("Modo: MANUAL");
  #endif
  
  #ifdef MODO_ALEATORIO
    Serial.println("Modo: ALEATORIO");
    Serial.print("Rango: ");
    Serial.print(SETPOINT_MIN);
    Serial.print(" a ");
    Serial.println(SETPOINT_MAX);
  #endif
  
  Serial.print("Cantidad de Setpoints: ");
  Serial.println(cantidadValores);
  
  Serial.print("Tiempo por Setpoint: ");
  Serial.print(tiempoPorSetpoint);
  Serial.println(" segundos");
  
  Serial.print("Intervalo de impresión: ");
  Serial.print(PrintTime);
  Serial.println(" ms");
  
  Serial.println("\nSetpoints configurados:");
  for(int i = 0; i < cantidadValores; i++) {
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(valoresSetpoint[i], 2);
    
    #ifdef CONTROL_MOTOR1_VELOCIDAD
      Serial.println(" RPM");
    #endif
    
    #ifdef CONTROL_MOTOR2_POSICION
      Serial.println(" cm");
    #endif
  }
  
  Serial.println("\nComandos disponibles:");
  Serial.println("  S - Iniciar secuencia");
  Serial.println("  D - Detener secuencia");
  Serial.println("  R - Resetear encoders");
  Serial.println("  L - Leer estado actual");
  Serial.println("  H - Mostrar esta ayuda");
  Serial.println("\nFormato de salida: Tiempo(s) Setpoint ValorReal PWM(%)");
  Serial.println("============================================\n");
}
