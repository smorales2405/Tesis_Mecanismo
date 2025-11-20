#include <HardwareSerial.h>
#include "RoboClaw.h"

// Configuración de pines para ESP32
#define ROBOCLAW_RX 16  // Pin RX del ESP32 (conectar a TX del RoboClaw)
#define ROBOCLAW_TX 17  // Pin TX del ESP32 (conectar a RX del RoboClaw)
#define ROBOCLAW_ADDRESS 0x80  // Dirección del RoboClaw

// Configuración del encoder y motor 1 (rotacional)
const float ENCODER_CPR_OUTPUT = 6533.0;   // CPR en la salida del gearbox

// Configuración del motor 2 (actuador lineal)
const float PULSES_PER_REV_M2 = 145.1;     // Pulsos por revolución del motor 2
const float CM_PER_REV = 0.8;              // 8mm = 0.8cm por revolución
const float PULSES_PER_CM = PULSES_PER_REV_M2 / CM_PER_REV;  // 181.375 pulsos/cm

// Límites del actuador lineal
const int32_t MIN_POSITION_PULSES = 0;      // Posición mínima (retraído)
const int32_t MAX_POSITION_PULSES = -2600;  // Posición máxima (extendido)
const float MIN_POSITION_CM = 0.0;          // 0 cm
const float MAX_POSITION_CM = 14.3;         // 14.3 cm

// Variables para el modo oscilatorio del actuador
bool oscillatingMode = false;          // Indica si está en modo oscilatorio
float oscillatingSpeed = 0.0;          // Velocidad de oscilación en cm/s
bool movingForward = true;             // Dirección actual del movimiento
unsigned long lastOscillationCheck = 0; // Para temporización

// Crear objeto de comunicación serial y RoboClaw
HardwareSerial RoboclawSerial(2);  // Usar UART2 del ESP32
RoboClaw roboclaw(&RoboclawSerial, 10000);  // 10ms timeout

// Variables para control
String inputString = "";
bool stringComplete = false;

// Configuración PID (ajustar según necesidad)
const uint32_t ACCEL = 10000;   // Aceleración en pulsos/s^2
const uint32_t DECEL = 10000;   // Deceleración en pulsos/s^2
const uint32_t MAX_SPEED = 5000; // Velocidad máxima en pulsos/s

void setup() {
  Serial.begin(115200);  // Monitor serial
  
  // Iniciar UART2 con los pines especificados
  RoboclawSerial.begin(38400, SERIAL_8N1, ROBOCLAW_RX, ROBOCLAW_TX);
  
  // Iniciar comunicación con RoboClaw
  roboclaw.begin(38400);
  
  delay(2000);
  
  // Resetear encoders al iniciar
  roboclaw.ResetEncoders(ROBOCLAW_ADDRESS);
  
  printMenu();
}

void loop() {
  // Leer comandos del monitor serial
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
  
  // Procesar comando cuando esté completo
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Monitorear oscilación del actuador
  if (oscillatingMode) {
    if (millis() - lastOscillationCheck > 100) {  // Revisar cada 100ms
      checkOscillation();
      lastOscillationCheck = millis();
      
      // Mostrar posición actual periódicamente
      static unsigned long lastPositionShow = 0;
      if (millis() - lastPositionShow > 2000) {  // Cada 2 segundos
        uint8_t status;
        bool valid;
        int32_t enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status, &valid);
        if (valid) {
          float cm = -(float)enc2 / PULSES_PER_CM;
          Serial.print("Posición actual: ");
          Serial.print(cm);
          Serial.println(" cm");
        }
        lastPositionShow = millis();
      }
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command.startsWith("P1:")) {
    // Control de posición Motor 1
    float degrees = command.substring(3).toFloat();
    moveToPosition(1, degrees);
  }
  else if (command.startsWith("P2:")) {
      // Detener oscilación si está activa
      oscillatingMode = false;
      // Control de posición Motor 2 en CENTÍMETROS
      float cm = command.substring(3).toFloat();
      moveLinearActuator(cm);
  }
  else if (command.startsWith("V1:")) {
    // Control de velocidad Motor 1
    float rpm = command.substring(3).toFloat();
    setMotorSpeed(1, rpm);
  }
  else if (command.startsWith("V2:")) {
      // Control de velocidad Motor 2 en CM/S
      float cm_per_sec = command.substring(3).toFloat();
      setLinearActuatorSpeed(cm_per_sec);  // Nueva función
  }
  else if (command == "S") {
    // Detener motores
    stopMotors();
  }
  else if (command == "R") {
    // Reset encoders
    resetEncoders();
  }
  else if (command == "E") {
    // Leer posición actual
    readEncoders();
  }
  else if (command == "H" || command == "?") {
    // Mostrar menú de ayuda
    printMenu();
  }
  else {
    Serial.println("Comando no reconocido. Escribe 'H' para ayuda");
  }
}

void moveToPosition(uint8_t motor, float degrees) {
  // Convertir grados a pulsos de encoder
  int32_t encoderCounts = degreesToEncoderCounts(degrees);
  
  Serial.print("Moviendo Motor ");
  Serial.print(motor);
  Serial.print(" a ");
  Serial.print(degrees);
  Serial.print(" grados (");
  Serial.print(encoderCounts);
  Serial.println(" pulsos)");
  
  // Usar las funciones de la librería RoboClaw
  if (motor == 1) {
    roboclaw.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 
                                         ACCEL, 
                                         MAX_SPEED, 
                                         DECEL, 
                                         encoderCounts, 
                                         1);  // buffer flag = 1 (ejecutar inmediatamente)
  } else {
    roboclaw.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, 
                                         ACCEL, 
                                         MAX_SPEED, 
                                         DECEL, 
                                         encoderCounts, 
                                         1);
  }
}

void setMotorSpeed(uint8_t motor, float rpm) {
  // Convertir RPM a pulsos por segundo
  int32_t pulsesPerSecond = rpmToPulsesPerSecond(rpm);
  
  Serial.print("Motor ");
  Serial.print(motor);
  Serial.print(" velocidad: ");
  Serial.print(rpm);
  Serial.print(" RPM (");
  Serial.print(pulsesPerSecond);
  Serial.println(" pulsos/s)");
  
  // Usar las funciones de velocidad con aceleración
  if (motor == 1) {
    roboclaw.SpeedAccelM1(ROBOCLAW_ADDRESS, ACCEL, pulsesPerSecond);
  } else {
    roboclaw.SpeedAccelM2(ROBOCLAW_ADDRESS, ACCEL, pulsesPerSecond);
  }
}

void moveLinearActuator(float cm) {
  // Verificar límites
  if (cm < MIN_POSITION_CM || cm > MAX_POSITION_CM) {
    Serial.print("ERROR: Posición fuera de límites (");
    Serial.print(MIN_POSITION_CM);
    Serial.print(" - ");
    Serial.print(MAX_POSITION_CM);
    Serial.println(" cm)");
    return;
  }
  
  // Convertir cm a pulsos (negativo porque el motor va en reversa para extender)
  int32_t encoderCounts = -(int32_t)(cm * PULSES_PER_CM);
  
  Serial.print("Moviendo actuador lineal a ");
  Serial.print(cm);
  Serial.print(" cm (");
  Serial.print(encoderCounts);
  Serial.println(" pulsos)");
  
  // Usar velocidad más lenta para el actuador lineal
  uint32_t linearSpeed = 1000;  // Ajustar según necesidad
  
  roboclaw.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, 
                                       ACCEL/2,      // Menor aceleración para el actuador
                                       linearSpeed, 
                                       DECEL/2, 
                                       encoderCounts, 
                                       1);
}

void setLinearActuatorSpeed(float cm_per_sec) {
  // Si la velocidad es 0, detener oscilación
  if (cm_per_sec == 0) {
    oscillatingMode = false;
    roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
    Serial.println("Actuador lineal detenido");
    return;
  }
  
  // Verificar límites de velocidad
  float max_cm_per_sec = 15.0;  // Ajustar según tu sistema
  if (abs(cm_per_sec) > max_cm_per_sec) {
    Serial.print("Velocidad limitada a ");
    Serial.print(max_cm_per_sec);
    Serial.println(" cm/s");
    cm_per_sec = max_cm_per_sec;
  }
  
  // Activar modo oscilatorio
  oscillatingMode = true;
  oscillatingSpeed = abs(cm_per_sec);  // Guardar velocidad absoluta
  movingForward = true;  // Empezar extendiendo
  
  Serial.print("Modo oscilatorio activado: ");
  Serial.print(oscillatingSpeed);
  Serial.println(" cm/s");
  Serial.println("El actuador oscilará entre 0 y 14.3 cm");
  Serial.println("Ingrese 'S' o V2:0 para detener");
  
  // Iniciar movimiento hacia el límite máximo
  startOscillation();
}

void startOscillation() {
  if (!oscillatingMode) return;
  
  int32_t targetPosition;
  
  if (movingForward) {
    // Mover hacia el límite máximo (extender)
    targetPosition = MAX_POSITION_PULSES;  // -2600 pulsos
    Serial.println("Extendiendo actuador...");
  } else {
    // Mover hacia el límite mínimo (retraer)
    targetPosition = MIN_POSITION_PULSES;  // 0 pulsos
    Serial.println("Retrayendo actuador...");
  }
  
  // Convertir velocidad cm/s a pulsos/s
  uint32_t speed = (uint32_t)(oscillatingSpeed * PULSES_PER_CM);
  
  // Enviar comando de posición
  roboclaw.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS,
                                       ACCEL/4,     // Aceleración suave
                                       speed,
                                       DECEL/4,     // Deceleración suave
                                       targetPosition,
                                       0);  // Buffer = 0 para poder cambiar dirección
}

void checkOscillation() {
  if (!oscillatingMode) return;
  
  uint8_t status;
  bool valid;
  int32_t currentPosition = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status, &valid);
  
  if (!valid) return;
  
  // Verificar si llegó al límite
  bool reachedLimit = false;
  
  if (movingForward && currentPosition <= (MAX_POSITION_PULSES + 50)) {
    // Llegó al límite máximo (con tolerancia de 50 pulsos)
    reachedLimit = true;
    movingForward = false;
    Serial.println("Límite máximo alcanzado - Cambiando dirección");
  } 
  else if (!movingForward && currentPosition >= (MIN_POSITION_PULSES - 50)) {
    // Llegó al límite mínimo (con tolerancia de 50 pulsos)
    reachedLimit = true;
    movingForward = true;
    Serial.println("Límite mínimo alcanzado - Cambiando dirección");
  }
  
  // Si llegó al límite, cambiar dirección
  if (reachedLimit) {
    delay(100);  // Pequeña pausa antes de cambiar dirección
    startOscillation();
  }
  
  // Verificar si el motor se detuvo (por si acaso)
  uint32_t speed;
  uint8_t speedStatus;
  bool speedValid;
  speed = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &speedStatus, &speedValid);
  
  if (speedValid && speed == 0) {
    // El motor se detuvo, reiniciar movimiento
    delay(500);  // Esperar medio segundo
    startOscillation();
  }
}

void stopMotors() {
  // Detener modo oscilatorio si está activo
  oscillatingMode = false;

  // Usar las funciones de la librería para detener
  roboclaw.ForwardM1(ROBOCLAW_ADDRESS, 0);
  roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
  Serial.println("Motores detenidos");
}

void resetEncoders() {
  roboclaw.ResetEncoders(ROBOCLAW_ADDRESS);
  Serial.println("Encoders reseteados a 0");
}

void readEncoders() {
  uint8_t status1, status2;
  bool valid1, valid2;
  
  // Motor 1 - mostrar en grados
  int32_t enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
  if (valid1) {
    float degrees1 = encoderCountsToDegrees(enc1);
    Serial.print("Motor 1: ");
    Serial.print(enc1);
    Serial.print(" pulsos (");
    Serial.print(degrees1);
    Serial.println(" grados)");
  }
  
  // Motor 2 - mostrar en centímetros
  int32_t enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
  if (valid2) {
    float cm = -(float)enc2 / PULSES_PER_CM;  // Negativo para conversión correcta
    Serial.print("Actuador lineal: ");
    Serial.print(enc2);
    Serial.print(" pulsos (");
    Serial.print(cm);
    Serial.print(" cm) ");
    
    // Mostrar si está en límites
    if (enc2 <= MAX_POSITION_PULSES) {
      Serial.print("[LÍMITE MÁXIMO]");
    } else if (enc2 >= MIN_POSITION_PULSES) {
      Serial.print("[LÍMITE MÍNIMO]");
    }
    Serial.println();
  }
  
  // Velocidades
  uint8_t status3, status4;
  bool valid3, valid4;
  uint32_t speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status3, &valid3);
  uint32_t speed2 = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &status4, &valid4);
  
  if (valid3) {
    float rpm1 = pulsesPerSecondToRPM(speed1);
    Serial.print("Velocidad Motor 1: ");
    Serial.print(rpm1);
    Serial.println(" RPM");
  }
  
  if (valid4) {
    float cm_per_sec = -(float)speed2 / PULSES_PER_CM;
    Serial.print("Velocidad actuador: ");
    Serial.print(cm_per_sec);
    Serial.println(" cm/s");
  }
}

// Funciones de conversión
int32_t degreesToEncoderCounts(float degrees) {
  float countsPerDegree = ENCODER_CPR_OUTPUT / 360.0;
  return (int32_t)(degrees * countsPerDegree);
}

float encoderCountsToDegrees(int32_t counts) {
  float countsPerDegree = ENCODER_CPR_OUTPUT / 360.0;
  return (float)counts / countsPerDegree;
}

int32_t rpmToPulsesPerSecond(float rpm) {
  float pulsesPerRevolution = ENCODER_CPR_OUTPUT;
  float revolutionsPerSecond = rpm / 60.0;
  return (int32_t)(revolutionsPerSecond * pulsesPerRevolution);
}

float pulsesPerSecondToRPM(int32_t pulsesPerSecond) {
  float pulsesPerRevolution = ENCODER_CPR_OUTPUT;
  float revolutionsPerSecond = (float)pulsesPerSecond / pulsesPerRevolution;
  return revolutionsPerSecond * 60.0;
}

void printMenu() {
  Serial.println("\n=== Control de Motor con RoboClaw ===");
  Serial.println("Comandos disponibles:");
  Serial.println("  P1:xxx - Posición Motor 1 en grados (ej: P1:90)");
  Serial.println("  P2:xxx - Posición Actuador en cm (0-14.3) (ej: P2:7.5)");
  Serial.println("  V1:xxx - Velocidad Motor 1 en RPM (ej: V1:30)");
  Serial.println("  V2:xxx - Velocidad Actuador oscilante (ej: V2:2)");
  Serial.println("         (El actuador oscilará entre límites)");
  Serial.println("         V2:0 detiene la oscilación");
  Serial.println("  S      - Detener ambos motores");
  Serial.println("  R      - Reset encoders");
  Serial.println("  E      - Leer posición y velocidad actual");
  Serial.println("  H o ?  - Mostrar este menú");
  Serial.println("\nLímites actuador: 0 cm (retraído) a 14.3 cm (extendido)");
  Serial.println("=====================================\n");
}

void showStatus() {
  Serial.println("\n--- Estado Actual ---");
  readEncoders();
  Serial.println("--------------------\n");
}