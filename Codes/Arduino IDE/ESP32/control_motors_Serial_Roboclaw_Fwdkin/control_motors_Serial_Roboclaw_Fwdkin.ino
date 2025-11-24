#include <HardwareSerial.h>
#include "RoboClaw.h"
#include "FiveBarKinematics.h"

// Configuración de pines para ESP32
#define ROBOCLAW_RX 16  // Pin RX del ESP32 (conectar a TX del RoboClaw)
#define ROBOCLAW_TX 17  // Pin TX del ESP32 (conectar a RX del RoboClaw)
#define ROBOCLAW_ADDRESS 0x80  // Dirección del RoboClaw

// Encoders
int32_t enc1 = 0;
int32_t enc2 = 0;

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
// Variables para límites personalizados de oscilación
float minOscillationLimit = MIN_POSITION_CM;  // Límite inferior (por defecto 0 cm)
float maxOscillationLimit = MAX_POSITION_CM;  // Límite superior (por defecto 14.3 cm)
float cm = 0.0;

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

// Crear instancia del mecanismo
FiveBarMechanism mechanism;
float th2 = 0.0, lDE = 0.0;
float th4 = 0.0, O_y = 0.0;
unsigned long lastKinematicsCheck = 0; // Para temporización
bool flag_offset = 0;
float offset_th4 = 0.0;
float offset_O_y = 0.0;

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
          //Serial.print("Posición actual: ");
          //Serial.print(cm);
          //Serial.println(" cm");
        }
        lastPositionShow = millis();
      }
    }
  }

  if (millis() - lastKinematicsCheck > 100) {
    // Calcular cinemática directa
    readEncoders();
    th2 = (encoderCountsToDegrees(enc1)-90)*(PI/180);
    lDE = -(float)enc2 / PULSES_PER_CM+6.0;
    KinematicsResult result = mechanism.forwardKinematics(th2, lDE);

    if (result.valid) {
      if (flag_offset == 0) {
        offset_th4 = result.theta4;
        offset_O_y = result.O_y;
        flag_offset = 1;
        Serial.println("Offset");
      }
      th4 = (result.theta4-offset_th4)*(180/PI);
      if (th4 > 100) {
        th4 = -(360-th4);
      } 
      O_y = result.O_y - offset_O_y;
      //Serial.print("Altura: ");
      Serial.print(O_y);
      //Serial.print(" cm / ");
      Serial.print(" ");
      //Serial.print("Inclinación: ");
      Serial.print(th4);
      Serial.print(" ");
      Serial.print("0");
      Serial.print(" ");
      Serial.print("15");
      Serial.println(" ");
      lastKinematicsCheck = millis();  
    } else {
      Serial.println("Solución no válida");
    }
  }

  /*
  readEncoders();
  if (enc1 > ENCODER_CPR_OUTPUT) {
    Serial.print("Enc 1: ");
    Serial.print(enc1);
    Serial.println( "Reset de mecanismo");
    roboclaw.SetEncM1(ROBOCLAW_ADDRESS, 0);
    mechanism.reset();
  }
  */

}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  // Buscar y procesar comandos en el string
  int pos = 0;
  
  while (pos < command.length()) {
    // Saltar espacios en blanco
    while (pos < command.length() && command.charAt(pos) == ' ') {
      pos++;
    }
    
    if (pos >= command.length()) break;
    
    // Identificar el tipo de comando
    int colonPos = command.indexOf(':', pos);
    
    if (colonPos != -1 && colonPos - pos <= 2) {
      // Es un comando con ":" (P1:, P2:, V1:, V2:)
      String cmdType = command.substring(pos, colonPos + 1);
      
      // Extraer el resto después de ':'
      int paramStart = colonPos + 1;
      
      // Saltar espacios después de ':'
      while (paramStart < command.length() && command.charAt(paramStart) == ' ') {
        paramStart++;
      }
      
      // Encontrar dónde termina este comando
      int endPos;
      if (cmdType == "V2:") {
        // V2 puede tener múltiples parámetros separados por espacios
        endPos = findNextCommand(command, paramStart);
      } else {
        // P1:, P2:, V1: tienen un solo parámetro numérico
        // Leer hasta el siguiente espacio seguido de un comando o fin de string
        endPos = findNextCommand(command, paramStart);
        
        // Si no encontró otro comando, usar todo lo que queda
        if (endPos == command.length()) {
          endPos = command.length();
        }
      }
      
      // Construir el comando completo incluyendo el tipo y los parámetros
      String params = command.substring(paramStart, endPos);
      params.trim(); // Eliminar espacios extra
      String fullCommand = cmdType + params;
      
      processSingleCommand(fullCommand);
      pos = endPos;
      
    } else {
      // Es un comando de una letra (S, R, E, H, ?)
      char cmd = command.charAt(pos);
      String singleCmd = String(cmd);
      processSingleCommand(singleCmd);
      pos++;
    }
  }
}

void processSingleCommand(String command) {
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
      // Control de velocidad Motor 2 en CM/S con límites opcionales
      parseAndSetLinearSpeed(command.substring(3));
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
    Serial.print("Comando no reconocido: ");
    Serial.println(command);
  }
}

// Función auxiliar para encontrar el siguiente comando
int findNextCommand(String command, int startPos) {
  // Buscar patrones de comandos: P1:, P2:, V1:, V2:
  // Debe haber un espacio antes del comando (o estar al inicio)
  for (int i = startPos; i < command.length() - 2; i++) {
    // Verificar si hay un espacio antes (o es el inicio del string)
    if (i > startPos) {  // No estamos al inicio de la búsqueda
      // Debe haber al menos un espacio antes del comando
      if (command.charAt(i) == ' ' || command.charAt(i-1) == ' ') {
        // Saltar espacios
        int checkPos = i;
        while (checkPos < command.length() && command.charAt(checkPos) == ' ') {
          checkPos++;
        }
        
        // Verificar si sigue un patrón de comando
        if (checkPos + 2 < command.length()) {
          String pattern = command.substring(checkPos, checkPos + 3);
          if (pattern == "P1:" || pattern == "P2:" || 
              pattern == "V1:" || pattern == "V2:") {
            return checkPos;
          }
        }
        
        // Verificar comandos de una letra
        if (checkPos < command.length()) {
          char c = command.charAt(checkPos);
          if (c == 'S' || c == 'R' || c == 'E' || c == 'H' || c == '?') {
            return checkPos;
          }
        }
      }
    }
  }
  return command.length();
}

void moveToPosition(uint8_t motor, float degrees) {
  // Convertir grados a pulsos de encoder
  int32_t encoderCounts = degreesToEncoderCounts(degrees);
  
  /*
  Serial.print("Moviendo Motor ");
  Serial.print(motor);
  Serial.print(" a ");
  Serial.print(degrees);
  Serial.print(" grados (");
  Serial.print(encoderCounts);
  Serial.println(" pulsos)");
  */
  
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
  
  /*
  Serial.print("Motor ");
  Serial.print(motor);
  Serial.print(" velocidad: ");
  Serial.print(rpm);
  Serial.print(" RPM (");
  Serial.print(pulsesPerSecond);
  Serial.println(" pulsos/s)");
  */
  
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
    /*
    Serial.print("ERROR: Posición fuera de límites (");
    Serial.print(MIN_POSITION_CM);
    Serial.print(" - ");
    Serial.print(MAX_POSITION_CM);
    Serial.println(" cm)");
    */
    return;
  }
  
  // Convertir cm a pulsos (negativo porque el motor va en reversa para extender)
  int32_t encoderCounts = -(int32_t)(cm * PULSES_PER_CM);
  
  /*
  Serial.print("Moviendo actuador lineal a ");
  Serial.print(cm);
  Serial.print(" cm (");
  Serial.print(encoderCounts);
  Serial.println(" pulsos)");
  */
  
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
    //Serial.println("Actuador lineal detenido");
    return;
  }
  
  // Verificar límites de velocidad
  float max_cm_per_sec = 15.0;  // Ajustar según tu sistema
  if (abs(cm_per_sec) > max_cm_per_sec) {
    /*
    Serial.print("Velocidad limitada a ");
    Serial.print(max_cm_per_sec);
    Serial.println(" cm/s");
    cm_per_sec = max_cm_per_sec;
    */
  }
  
  // Activar modo oscilatorio
  oscillatingMode = true;
  oscillatingSpeed = abs(cm_per_sec);  // Guardar velocidad absoluta
  movingForward = true;  // Empezar extendiendo
  
  /*
  Serial.print("Modo oscilatorio activado: ");
  Serial.print(oscillatingSpeed);
  Serial.println(" cm/s");
  Serial.print("El actuador oscilará entre ");
  Serial.print(minOscillationLimit);
  Serial.print(" cm y ");
  Serial.print(maxOscillationLimit);
  Serial.println(" cm");
  Serial.println("Ingrese 'S' o V2:0 para detener");
  */
  
  // Iniciar movimiento hacia el límite máximo
  startOscillation();
}

void parseAndSetLinearSpeed(String params) {
  // Parsear parámetros: velocidad [min_limit] [max_limit]
  params.trim();
  
  // Contar cuántos números hay
  int spaceCount = 0;
  for (int i = 0; i < params.length(); i++) {
    if (params.charAt(i) == ' ') {
      // Saltar espacios múltiples
      while (i < params.length() && params.charAt(i) == ' ') {
        i++;
      }
      if (i < params.length()) spaceCount++;
    }
  }
  
  float cm_per_sec;
  float minLimit = MIN_POSITION_CM;  // Valor por defecto
  float maxLimit = MAX_POSITION_CM;  // Valor por defecto
  
  if (spaceCount == 0) {
    // Solo velocidad, sin límites
    cm_per_sec = params.toFloat();
    /*
    Serial.print("Velocidad: ");
    Serial.print(cm_per_sec);
    Serial.println(" cm/s (límites por defecto)");
    */
    
  } else if (spaceCount == 1) {
    // Error: se necesitan 2 límites o ninguno
    /*
    Serial.println("ERROR: Debes especificar AMBOS límites (mínimo y máximo)");
    Serial.println("Formato correcto: V2:velocidad min max");
    Serial.println("Ejemplo: V2:5 2 12");
    */
    return;
    
  } else {
    // Velocidad + 2 límites
    int firstSpace = params.indexOf(' ');
    int secondSpace = params.indexOf(' ', firstSpace + 1);
    
    // Saltar espacios múltiples
    while (secondSpace < params.length() && params.charAt(secondSpace) == ' ') {
      secondSpace++;
    }
    secondSpace = params.indexOf(' ', secondSpace);
    if (secondSpace == -1) {
      // Buscar el último grupo de números
      int lastNonSpace = params.length() - 1;
      while (lastNonSpace > 0 && params.charAt(lastNonSpace) == ' ') {
        lastNonSpace--;
      }
      secondSpace = params.lastIndexOf(' ', lastNonSpace);
    }
    
    cm_per_sec = params.substring(0, firstSpace).toFloat();
    minLimit = params.substring(firstSpace + 1, secondSpace).toFloat();
    maxLimit = params.substring(secondSpace + 1).toFloat();
    
    // Validar límites
    if (minLimit < MIN_POSITION_CM || minLimit > MAX_POSITION_CM ||
        maxLimit < MIN_POSITION_CM || maxLimit > MAX_POSITION_CM) {
      /*
      Serial.print("ERROR: Límites fuera de rango (");
      Serial.print(MIN_POSITION_CM);
      Serial.print(" - ");
      Serial.print(MAX_POSITION_CM);
      Serial.println(" cm)");
      */
      return;
    }
    
    if (minLimit >= maxLimit) {
      Serial.println("ERROR: El límite mínimo debe ser menor que el máximo");
      return;
    }
    
    /*
    Serial.print("Velocidad: ");
    Serial.print(cm_per_sec);
    Serial.print(" cm/s, Límites: ");
    Serial.print(minLimit);
    Serial.print(" - ");
    Serial.print(maxLimit);
    Serial.println(" cm");
    */
  }
  
  // Actualizar límites globales de oscilación
  minOscillationLimit = minLimit;
  maxOscillationLimit = maxLimit;
  
  // Llamar a la función modificada
  setLinearActuatorSpeed(cm_per_sec);
}

void startOscillation() {
  if (!oscillatingMode) return;
  
  int32_t targetPosition;
  
  if (movingForward) {
    // Mover hacia el límite máximo personalizado (extender)
    targetPosition = -(int32_t)(maxOscillationLimit * PULSES_PER_CM);
    /*
    Serial.print("Extendiendo actuador hacia ");
    Serial.print(maxOscillationLimit);
    Serial.println(" cm...");
    */
  } else {
    // Mover hacia el límite mínimo personalizado (retraer)
    targetPosition = -(int32_t)(minOscillationLimit * PULSES_PER_CM);
    /*
    Serial.print("Retrayendo actuador hacia ");
    Serial.print(minOscillationLimit);
    Serial.println(" cm...");
    */
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
  
  // Convertir posiciones de límites a pulsos
  int32_t maxLimitPulses = -(int32_t)(maxOscillationLimit * PULSES_PER_CM);
  int32_t minLimitPulses = -(int32_t)(minOscillationLimit * PULSES_PER_CM);
  
  // Verificar si llegó al límite
  bool reachedLimit = false;
  
  if (movingForward && currentPosition <= (maxLimitPulses + 50)) {
    // Llegó al límite máximo (con tolerancia de 50 pulsos)
    reachedLimit = true;
    movingForward = false;
    //Serial.println("Límite máximo alcanzado - Cambiando dirección");
  } 
  else if (!movingForward && currentPosition >= (minLimitPulses - 50)) {
    // Llegó al límite mínimo (con tolerancia de 50 pulsos)
    reachedLimit = true;
    movingForward = true;
    //Serial.println("Límite mínimo alcanzado - Cambiando dirección");
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
  enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
  if (valid1) {
    float degrees1 = encoderCountsToDegrees(enc1);
    /*
    Serial.print("Motor 1: ");
    Serial.print(enc1);
    Serial.print(" pulsos (");
    Serial.print(degrees1);
    Serial.println(" grados)");
    */
  }
  
  // Motor 2 - mostrar en centímetros
  enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
  if (valid2) {
    cm = -(float)enc2 / PULSES_PER_CM;  // Negativo para conversión correcta
    /*
    Serial.print("Actuador lineal: ");
    Serial.print(enc2);
    Serial.print(" pulsos (");
    Serial.print(cm);
    Serial.print(" cm) ");
    */

    // Mostrar si está en límites
    if (enc2 <= MAX_POSITION_PULSES) {
      //Serial.print("[LÍMITE MÁXIMO]");
    } else if (enc2 >= MIN_POSITION_PULSES) {
      //Serial.print("[LÍMITE MÍNIMO]");
    }
    //Serial.println();
  }
  
  // Velocidades
  uint8_t status3, status4;
  bool valid3, valid4;
  uint32_t speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status3, &valid3);
  uint32_t speed2 = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &status4, &valid4);
  
  if (valid3) {
    float rpm1 = pulsesPerSecondToRPM(speed1);
    /*
    Serial.print("Velocidad Motor 1: ");
    Serial.print(rpm1);
    Serial.println(" RPM");
    */
  }
  
  if (valid4) {
    float cm_per_sec = -(float)speed2 / PULSES_PER_CM;
    /*
    Serial.print("Velocidad actuador: ");
    Serial.print(cm_per_sec);
    Serial.println(" cm/s");
    */
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
  Serial.println("  V2:vel [min] [max] - Velocidad Actuador oscilante");
  Serial.println("         V2:5      -> Oscila a 5cm/s entre 0-14.3cm (toda la carrera)");
  Serial.println("         V2:5 2 12 -> Oscila a 5cm/s entre 2cm y 12cm");
  Serial.println("         V2:0      -> Detiene la oscilación");
  Serial.println("\n  MÚLTIPLES COMANDOS:");
  Serial.println("  V1:30 P2:5   -> Velocidad M1 a 30 RPM y posición M2 a 5cm");
  Serial.println("  V1:50 V2:10  -> Velocidad M1 a 50 RPM y M2 oscilando a 10cm/s");
  Serial.println("\n  S      - Detener ambos motores");
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