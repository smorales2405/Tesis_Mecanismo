#include <HardwareSerial.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>

#include "RoboClaw.h"
#include "FiveBarKinematics.h"


// WEBSERVER (WiFi)
const char* ap_ssid     = "ESP32_Mecanismo";
const char* ap_password = "mecanismo2025";
AsyncWebServer server(80);

// Configuración de pines para ESP32
#define ROBOCLAW_RX 16  // Pin RX del ESP32 (conectar a TX del RoboClaw)
#define ROBOCLAW_TX 17  // Pin TX del ESP32 (conectar a RX del RoboClaw)
#define ROBOCLAW_ADDRESS 0x80  // Dirección del RoboClaw

//Variables Roboclaw
int32_t enc1 = 0, enc2 = 0, speed1 = 0, speed2 = 0;
uint8_t status1, status2, status3, status4;
bool valid1, valid2, valid3, valid4;

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

// Variables para retorno a posición inicial
bool returningToHome = false;          // Indica si está retornando a home
unsigned long homeReturnStart = 0;     // Para timeout de seguridad
int32_t tol_home1 = 1*ENCODER_CPR_OUTPUT/360.0;   // °
int32_t tol_home2 = 5*ENCODER_CPR_OUTPUT/360.0;   // °
int32_t tol_home3 = 0.5*PULSES_PER_CM/10.0;       // mm
int32_t PPSToHome = 20*ENCODER_CPR_OUTPUT/60;
int32_t targetPosition1 = 0;

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
  
  setupWiFi();

  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    return;
  }

  // Configurar servidor web
  setupWebServer();
  // Iniciar servidor
  server.begin();
  delay(1000);

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
    // Primero, limpiar el comando
    inputString.trim();
    
    // No procesar nuevos comandos si está retornando a home (excepto emergencia)
    if (returningToHome && inputString != "E") {
      Serial.println("Esperando retorno a posición inicial...");
    } else {
      processCommand(inputString);
    }
    inputString = "";
    stringComplete = false;
  }
  
  // Verificar retorno a home
  if (returningToHome) {
    static unsigned long lastHomeCheck = 0;
    if (millis() - lastHomeCheck > 200) {  // Revisar cada 200ms
      checkHomeReturn();
      lastHomeCheck = millis();
    }

    enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
    if (abs(abs(enc1)-abs(targetPosition1)) <= tol_home2) {
      roboclaw.ForwardM1(ROBOCLAW_ADDRESS, 0);
    }
  }
  
  // Monitorear oscilación del actuador (solo si no está retornando a home)
  if (oscillatingMode && !returningToHome) {
    if (millis() - lastOscillationCheck > 100) {
      checkOscillation();
      lastOscillationCheck = millis();
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
      /*
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
      */
      lastKinematicsCheck = millis();  
    } else {
      Serial.println("Solución no válida");
    }
  }

}

void setupWiFi() {

  // Configurar ESP32 como Access Point
  WiFi.mode(WIFI_AP);
  
  // Configurar canal y máximo de conexiones
  const int channel = 1;          // Canal WiFi (1-13)
  const int max_connection = 4;   // Máximo 4 dispositivos conectados
  const bool hidden = false;      // false = red visible, true = red oculta

  // Configurar IP estática para el AP (opcional)
  IPAddress local_IP(192, 168, 4, 2);
  IPAddress gateway(192, 168, 4, 2);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);

  // Crear AP con configuración completa
  bool result = WiFi.softAP(ap_ssid, ap_password, channel, hidden, max_connection);
  
  if (result) {
    Serial.println("WiFi AP OK");
    Serial.print("Red: ");
    Serial.println(ap_ssid);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
    delay(3000);
  } else {
    Serial.println("\nError al crear AP");
    delay(2000);
  }
}

void setupWebServer() {
  // Servir archivos estáticos desde la SD
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/Webserver/index.html", "text/html");
  });
  
  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/Webserver/styles.css", "text/css");
  });

  server.on("/chart.umd.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/Webserver/chart.umd.min.js", "application/javascript");
  });   

  server.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/Webserver/app.js", "application/javascript");
  });

  // ============ ENDPOINTS DE CONTROL ============

  // Motor 1 (Crank) - Velocidad en RPM
  server.on("/motor1/speed", HTTP_GET, [](AsyncWebServerRequest *request){
    if (returningToHome) {
      request->send(200, "application/json", "{\"error\":\"Retornando a home\"}");
      return;
    }
    if (request->hasParam("rpm")) {
      float rpm = request->getParam("rpm")->value().toFloat();
      setMotorSpeed(1, rpm);
      request->send(200, "application/json", "{\"status\":\"ok\",\"rpm\":" + String(rpm) + "}");
    } else {
      request->send(400, "application/json", "{\"error\":\"Falta parametro rpm\"}");
    }
  });

  // Motor 2 (Actuador) - Posición fija en cm
  server.on("/motor2/position", HTTP_GET, [](AsyncWebServerRequest *request){
    if (returningToHome) {
      request->send(200, "application/json", "{\"error\":\"Retornando a home\"}");
      return;
    }
    if (request->hasParam("cm")) {
      float cmPos = request->getParam("cm")->value().toFloat();
      // Detener oscilación si está activa
      oscillatingMode = false;
      moveLinearActuator(cmPos);
      request->send(200, "application/json", "{\"status\":\"ok\",\"position\":" + String(cmPos) + "}");
    } else {
      request->send(400, "application/json", "{\"error\":\"Falta parametro cm\"}");
    }
  });

  // Motor 2 (Actuador) - Modo oscilante
  server.on("/motor2/oscillate", HTTP_GET, [](AsyncWebServerRequest *request){
    if (returningToHome) {
      request->send(200, "application/json", "{\"error\":\"Retornando a home\"}");
      return;
    }
    if (request->hasParam("speed")) {
      float speed = request->getParam("speed")->value().toFloat();
      
      // Si velocidad es 0, detener oscilación
      if (speed == 0) {
        oscillatingMode = false;
        roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
        request->send(200, "application/json", "{\"status\":\"ok\",\"oscillating\":false}");
        return;
      }
      
      // Obtener límites (opcionales)
      float minLimit = MIN_POSITION_CM;
      float maxLimit = MAX_POSITION_CM;
      
      if (request->hasParam("min")) {
        minLimit = request->getParam("min")->value().toFloat();
      }
      if (request->hasParam("max")) {
        maxLimit = request->getParam("max")->value().toFloat();
      }
      
      // Validar límites
      if (minLimit < MIN_POSITION_CM || maxLimit > MAX_POSITION_CM || minLimit >= maxLimit) {
        request->send(400, "application/json", "{\"error\":\"Limites invalidos\"}");
        return;
      }
      
      // Configurar oscilación
      minOscillationLimit = minLimit;
      maxOscillationLimit = maxLimit;
      setLinearActuatorSpeed(speed);
      
      String json = "{\"status\":\"ok\",\"oscillating\":true,";
      json += "\"speed\":" + String(speed) + ",";
      json += "\"min\":" + String(minLimit) + ",";
      json += "\"max\":" + String(maxLimit) + "}";
      request->send(200, "application/json", json);
    } else {
      request->send(400, "application/json", "{\"error\":\"Falta parametro speed\"}");
    }
  });

  // Detener motores (retorno a home)
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stopMotors();
    request->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"stop\"}");
  });

  // Parada de emergencia
  server.on("/emergency", HTTP_GET, [](AsyncWebServerRequest *request){
    emergencyStop();
    request->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"emergency\"}");
  });

  // Reset encoders
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    resetEncoders();
    request->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"reset\"}");
  });

  // Obtener datos actuales (cinemática y posición)
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String state = "Detenido";
    if (returningToHome) {
      state = "Retornando";
    } else if (oscillatingMode) {
      state = "Oscilando";
    }
    
    String json = "{";
    json += "\"O_y\":" + String(O_y, 2) + ",";
    json += "\"th4\":" + String(th4, 2) + ",";
    json += "\"position\":" + String(cm, 1) + ",";
    json += "\"state\":\"" + state + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });
}

void resetEncoderM1() {
  enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
  if (enc1 > ENCODER_CPR_OUTPUT) {
    roboclaw.SetEncM1(ROBOCLAW_ADDRESS, 0);
  }
  else if (enc1 < 0 && enc1 < ENCODER_CPR_OUTPUT) {
    roboclaw.SetEncM1(ROBOCLAW_ADDRESS, 0);
  }
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
  else if (command == "E") {
    // Parada de emergencia - detiene inmediatamente sin retornar a home
    emergencyStop();
  }
  else if (command == "R") {
    // Reset encoders
    resetEncoders();
  }
  else if (command == "L") {
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

void emergencyStop() {
  // Detener todo inmediatamente
  oscillatingMode = false;
  returningToHome = false;
  
  roboclaw.ForwardM1(ROBOCLAW_ADDRESS, 0);
  roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
  
  Serial.println("\n⚠ PARADA DE EMERGENCIA ⚠");
  Serial.println("Motores detenidos inmediatamente");
  Serial.println("Use 'S' para retornar a posición inicial");
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
  
  Serial.print("Modo oscilatorio activado: ");
  Serial.print(oscillatingSpeed);
  Serial.println(" cm/s");
  Serial.print("El actuador oscilará entre ");
  Serial.print(minOscillationLimit);
  Serial.print(" cm y ");
  Serial.print(maxOscillationLimit);
  Serial.println(" cm");
  Serial.println("Ingrese 'S' o V2:0 para detener");
  
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
    Serial.print("Velocidad: ");
    Serial.print(cm_per_sec);
    Serial.println(" cm/s (límites por defecto)");
    
  } else if (spaceCount == 1) {
    // Error: se necesitan 2 límites o ninguno
    Serial.println("ERROR: Debes especificar AMBOS límites (mínimo y máximo)");
    Serial.println("Formato correcto: V2:velocidad min max");
    Serial.println("Ejemplo: V2:5 2 12");
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
      Serial.print("ERROR: Límites fuera de rango (");
      Serial.print(MIN_POSITION_CM);
      Serial.print(" - ");
      Serial.print(MAX_POSITION_CM);
      Serial.println(" cm)");
      return;
    }
    
    if (minLimit >= maxLimit) {
      Serial.println("ERROR: El límite mínimo debe ser menor que el máximo");
      return;
    }
    
    Serial.print("Velocidad: ");
    Serial.print(cm_per_sec);
    Serial.print(" cm/s, Límites: ");
    Serial.print(minLimit);
    Serial.print(" - ");
    Serial.print(maxLimit);
    Serial.println(" cm");
  }
  
  // Actualizar límites globales de oscilación
  minOscillationLimit = minLimit;
  maxOscillationLimit = maxLimit;
  
  // Llamar a la función modificada
  setLinearActuatorSpeed(cm_per_sec);
}

void startOscillation() {
  if (!oscillatingMode) return;
  
  int32_t targetPosition2;
  
  if (movingForward) {
    // Mover hacia el límite máximo personalizado (extender)
    targetPosition2 = -(int32_t)(maxOscillationLimit * PULSES_PER_CM);
    Serial.print("Extendiendo actuador hacia ");
    Serial.print(maxOscillationLimit);
    Serial.println(" cm...");
  } else {
    // Mover hacia el límite mínimo personalizado (retraer)
    targetPosition2 = -(int32_t)(minOscillationLimit * PULSES_PER_CM);
    Serial.print("Retrayendo actuador hacia ");
    Serial.print(minOscillationLimit);
    Serial.println(" cm...");
  }
  
  // Convertir velocidad cm/s a pulsos/s
  uint32_t speed = (uint32_t)(oscillatingSpeed * PULSES_PER_CM);
  
  // Enviar comando de posición
  roboclaw.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS,
                                       ACCEL/4,     // Aceleración suave
                                       speed,
                                       DECEL/4,     // Deceleración suave
                                       targetPosition2,
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
    Serial.println("Límite máximo alcanzado - Cambiando dirección");
  } 
  else if (!movingForward && currentPosition >= (minLimitPulses - 50)) {
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
  // Desactivar modo oscilatorio si está activo
  oscillatingMode = false;
  
  Serial.println("\n=== RETORNANDO A POSICIÓN INICIAL ===");
  returningToHome = true;
  homeReturnStart = millis();
  
  // Motor 1: Ir al múltiplo de 360° más cercano
  returnMotor1ToHome();
  
  // Motor 2: Ir a posición 0 cm
  returnActuatorToHome();
  
  Serial.println("Esperando que ambos motores lleguen a posición inicial...");
}

void returnMotor1ToHome() {

  int32_t revolutions = 0;

  // Leer posición actual del motor 1
  enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
  
  if (!valid1) {
    Serial.println("Error leyendo encoder Motor 1");
    return;
  }
  
  // Calcular el múltiplo de 6533 pulsos (360°) más cercano
  const int32_t PULSES_PER_REV = 6533;
  speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status3, &valid3);
  // Si la posición es positiva y no está exactamente en un múltiplo, ir al siguiente
  if (speed1 > 0) {
    if (enc1 > 0) {
      revolutions = enc1 / PULSES_PER_REV + 1;
    } 
    else if (enc1 < 0) {
      revolutions = enc1 / PULSES_PER_REV;
    }
    roboclaw.SpeedAccelM1(ROBOCLAW_ADDRESS, ACCEL, PPSToHome);
  }
  else if (speed1 < 0) {
    if (enc1 < 0) {
      revolutions = enc1 / PULSES_PER_REV - 1;
    } 
    else if (enc1 > 0) {
      revolutions = enc1 / PULSES_PER_REV;
    }
    roboclaw.SpeedAccelM1(ROBOCLAW_ADDRESS, ACCEL, -PPSToHome);
  }

  targetPosition1 = revolutions * PULSES_PER_REV;
  
  float currentDegrees = encoderCountsToDegrees(enc1);
  float targetDegrees = encoderCountsToDegrees(targetPosition1);
  
  Serial.print("Motor 1 - Posición actual: ");
  Serial.print(currentDegrees);
  Serial.print("° → Objetivo: ");
  Serial.print(targetDegrees);
  Serial.println("° (múltiplo de 360°)");

}

void returnActuatorToHome() {
  uint8_t status;
  bool valid;
  
  // Leer posición actual del actuador
  int32_t currentPosition = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status, &valid);
  
  if (!valid) {
    Serial.println("Error leyendo encoder Motor 2");
    return;
  }
  
  float currentCm = -(float)currentPosition / PULSES_PER_CM;
  
  Serial.print("Actuador - Posición actual: ");
  Serial.print(currentCm);
  Serial.println(" cm → Objetivo: 0 cm");
  
  // Mover a posición 0 (completamente retraído)
  roboclaw.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS,
                                       ACCEL/2,
                                       1000,  // Velocidad segura para retorno
                                       DECEL/2,
                                       MIN_POSITION_PULSES,  // 0 pulsos
                                       1);
}

void checkHomeReturn() {
  if (!returningToHome) return;
    
  // Leer posiciones actuales
  enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
  enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
  
  // Leer velocidades para saber si se están moviendo
  speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status1, &valid1);
  speed2 = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &status2, &valid2);
  
  bool motor1AtHome = false;
  bool motor2AtHome = false;
  
  // Verificar Motor 1 - está en múltiplo de 360° y detenido
  if (valid1) {
    const int32_t PULSES_PER_REV = 6533;
    // Tolerancia de ±10 pulsos
    if (abs(abs(enc1)-abs(targetPosition1)) <= tol_home1) {
      if (speed1 < 10) {  // Prácticamente detenido
        motor1AtHome = true;
      }
    }
  }
  
  // Verificar Motor 2 - está en posición 0 y detenido
  if (valid2) {
    // Tolerancia de ±10 pulsos desde 0
    if (abs(enc2 - MIN_POSITION_PULSES) <= tol_home3) {
      if (speed2 < 10) {  // Prácticamente detenido
        motor2AtHome = true;
      }
    }
  }
  
  // Si ambos están en home
  if (motor1AtHome && motor2AtHome) {
    returningToHome = false;
    Serial.println("\n✓ AMBOS MOTORES EN POSICIÓN INICIAL");
    Serial.print("Motor 1: ");
    Serial.print(encoderCountsToDegrees(enc1));
    Serial.println("°");
    Serial.print("Actuador: ");
    Serial.print(-(float)enc2 / PULSES_PER_CM);
    Serial.println(" cm");
    Serial.println("=====================================");
  }
  
  // Timeout de seguridad (10 segundos)
  if (millis() - homeReturnStart > 5000) {
    returningToHome = false;
    roboclaw.ForwardM1(ROBOCLAW_ADDRESS, 0);
    roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
    Serial.println("\n⚠ Timeout - Retorno a home cancelado");
  }
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
  Serial.println("  S      - Retornar a posición inicial");
  Serial.println("         (Motor 1 → 0°)");
  Serial.println("         (Actuador → 0 cm)");
  Serial.println("  E      - Parada de emergencia (detiene sin retornar)");
  Serial.println("  R      - Reset encoders");
  Serial.println("  L      - Leer posición y velocidad actual");
  Serial.println("  H o ?  - Mostrar este menú");
  Serial.println("\nLímites actuador: 0 cm (retraído) a 14.3 cm (extendido)");
  Serial.println("=====================================\n");
}

void showStatus() {
  Serial.println("\n--- Estado Actual ---");
  readEncoders();
  Serial.println("--------------------\n");
}