// =====================================================
// ESP32 - Control de Mecanismo via Arduino MEGA
// Comunicación: ESP32 <-> Arduino MEGA (Serial)
// Webserver para control y visualización de cinemática
// =====================================================

#include <HardwareSerial.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>

#include "FiveBarKinematics.h"

// ==================== CONFIGURACIÓN WiFi ====================
const char* ap_ssid     = "ESP32_Mecanismo";
const char* ap_password = "mecanismo2025";
AsyncWebServer server(80);

// ==================== COMUNICACIÓN SERIAL CON ARDUINO MEGA ====================
#define MEGA_RX 16  // Pin RX del ESP32 (conectar a TX del Arduino MEGA)
#define MEGA_TX 17  // Pin TX del ESP32 (conectar a RX del Arduino MEGA)

HardwareSerial MegaSerial(2);  // Usar UART2 del ESP32

// ==================== CONSTANTES DEL MECANISMO ====================
// Configuración del encoder y motor 1 (rotacional)
const float ENCODER_CPR_OUTPUT = 6533.0;   // CPR en la salida del gearbox

// Configuración del motor 2 (actuador lineal)
const float PULSES_PER_REV_M2 = 145.1;     // Pulsos por revolución del motor 2
const float CM_PER_REV = 0.8;              // 8mm = 0.8cm por revolución
const float PULSES_PER_CM = PULSES_PER_REV_M2 / CM_PER_REV;  // 181.375 pulsos/cm

// Límites del actuador lineal
const float MIN_POSITION_CM = 0.0;          // 0 cm
const float MAX_POSITION_CM = 14.3;         // 14.3 cm

// ==================== VARIABLES DE ESTADO ====================
// Variables recibidas del Arduino MEGA
float th2 = 0.0;            // Posición angular del motor 1 en radianes
float lDE = 0.0;            // Posición lineal del motor 2 en cm
float cm = 0.0;             // Posición actual del actuador en cm

// Variables de cinemática
float th4 = 0.0, O_y = 0.0;
bool flag_offset = false;
float offset_th4 = 0.0;
float offset_O_y = 0.0;

// Crear instancia del mecanismo
FiveBarMechanism mechanism;

// Variables de control
String inputString = "";
bool stringComplete = false;

// Variables para retorno a posición inicial
bool returningToHome = false;
unsigned long homeReturnStart = 0;

// Temporizadores
unsigned long lastKinematicsCheck = 0;
unsigned long lastMegaRequest = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);  // Monitor serial para debug
  
  // Iniciar comunicación serial con Arduino MEGA
  MegaSerial.begin(9600, SERIAL_8N1, MEGA_RX, MEGA_TX);
  
  delay(2000);
  
  // Configurar WiFi Access Point
  setupWiFi();

  // Iniciar tarjeta SD
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

// ==================== LOOP PRINCIPAL ====================
void loop() {
  // Leer comandos del monitor serial (debug/control directo)
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
  
  // Leer datos del Arduino MEGA periódicamente
  if (millis() - lastMegaRequest > 50) {
    readMegaData();
    lastMegaRequest = millis();
  }
  
  // Calcular cinemática directa periódicamente
  if (millis() - lastKinematicsCheck > 100) {
    calculateKinematics();
    lastKinematicsCheck = millis();
  }
  
  // Verificar retorno a home
  if (returningToHome) {
    checkHomeReturn();
  }
}

// ==================== CONFIGURACIÓN WiFi ====================
void setupWiFi() {
  WiFi.mode(WIFI_AP);
  
  const int channel = 1;
  const int max_connection = 4;
  const bool hidden = false;

  IPAddress local_IP(192, 168, 4, 2);
  IPAddress gateway(192, 168, 4, 2);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);

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

// ==================== CONFIGURACIÓN WEBSERVER ====================
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
      setMotorSpeed(rpm);
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
      moveLinearActuator(cmPos);
      request->send(200, "application/json", "{\"status\":\"ok\",\"position\":" + String(cmPos) + "}");
    } else {
      request->send(400, "application/json", "{\"error\":\"Falta parametro cm\"}");
    }
  });

  // Motor 2 (Actuador) - Modo oscilante (mantener endpoint pero sin funcionalidad)
  server.on("/motor2/oscillate", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Modo oscilante no disponible\"}");
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

  // Reset encoders (endpoint mantenido pero sin funcionalidad)
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Reset no disponible\"}");
  });

  // Obtener datos actuales (cinemática y posición)
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String state = "Detenido";
    if (returningToHome) {
      state = "Retornando";
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

// ==================== LECTURA DE DATOS DEL ARDUINO MEGA ====================
void readMegaData() {
  // Leer datos del Arduino MEGA
  // Formato esperado: "DATA:th2:X.XXX,lDE:Y.YYY\n"
  
  while (MegaSerial.available()) {
    String line = MegaSerial.readStringUntil('\n');
    line.trim();
    
    // Buscar formato de datos
    if (line.startsWith("DATA:")) {
      parseDataFromMega(line);
    }
  }
}

void parseDataFromMega(String data) {
  // Formato: "DATA:th2:X.XXX,lDE:Y.YYY"
  
  int th2Index = data.indexOf("th2:");
  int lDEIndex = data.indexOf("lDE:");
  
  if (th2Index != -1 && lDEIndex != -1) {
    // Extraer th2 (posición angular en radianes)
    int th2End = data.indexOf(",", th2Index);
    if (th2End == -1) th2End = data.length();
    String th2Str = data.substring(th2Index + 4, th2End);
    th2 = th2Str.toFloat();
    
    // Extraer lDE (posición lineal en cm)
    String lDEStr = data.substring(lDEIndex + 4);
    lDE = lDEStr.toFloat();
    cm = lDE;  // Actualizar posición del actuador
    
    /*
    Serial.print("Datos recibidos - th2: ");
    Serial.print(th2, 4);
    Serial.print(" rad, lDE: ");
    Serial.print(lDE, 2);
    Serial.println(" cm");
    */
  }
}

// ==================== CÁLCULO DE CINEMÁTICA ====================
void calculateKinematics() {
  // Calcular cinemática directa
  // th2 viene en radianes, ajustar offset de -90° (convertir a sistema del mecanismo)
  float th2_adjusted = th2 - PI/2;
  float lDE_adjusted = lDE + 6.0;  // Agregar offset base del actuador
  
  KinematicsResult result = mechanism.forwardKinematics(th2_adjusted, lDE_adjusted);

  if (result.valid) {
    if (!flag_offset) {
      offset_th4 = result.theta4;
      offset_O_y = result.O_y;
      flag_offset = true;
      Serial.println("Offset calculado");
    }
    
    th4 = (result.theta4 - offset_th4) * (180.0 / PI);
    if (th4 > 100) {
      th4 = -(360 - th4);
    } 
    O_y = result.O_y - offset_O_y;
    
    /*
    Serial.print("O_y: ");
    Serial.print(O_y, 2);
    Serial.print(" cm, th4: ");
    Serial.print(th4, 2);
    Serial.println("°");
    */
  }
}

// ==================== PROCESAMIENTO DE COMANDOS ====================
void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
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
      String cmdType = command.substring(pos, colonPos + 1);
      
      int paramStart = colonPos + 1;
      while (paramStart < command.length() && command.charAt(paramStart) == ' ') {
        paramStart++;
      }
      
      int endPos = findNextCommand(command, paramStart);
      
      String params = command.substring(paramStart, endPos);
      params.trim();
      String fullCommand = cmdType + params;
      
      processSingleCommand(fullCommand);
      pos = endPos;
      
    } else {
      char cmd = command.charAt(pos);
      String singleCmd = String(cmd);
      processSingleCommand(singleCmd);
      pos++;
    }
  }
}

void processSingleCommand(String command) {
  if (command.startsWith("V1:")) {
    // Control de velocidad Motor 1
    float rpm = command.substring(3).toFloat();
    setMotorSpeed(rpm);
  }
  else if (command.startsWith("P2:")) {
    // Control de posición Motor 2 en cm
    float cmPos = command.substring(3).toFloat();
    moveLinearActuator(cmPos);
  }
  else if (command == "S") {
    // Detener motores
    stopMotors();
  }
  else if (command == "E") {
    // Parada de emergencia
    emergencyStop();
  }
  else if (command == "L") {
    // Leer datos actuales
    showStatus();
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

int findNextCommand(String command, int startPos) {
  for (int i = startPos; i < command.length() - 2; i++) {
    if (i > startPos) {
      if (command.charAt(i) == ' ' || command.charAt(i-1) == ' ') {
        int checkPos = i;
        while (checkPos < command.length() && command.charAt(checkPos) == ' ') {
          checkPos++;
        }
        
        if (checkPos + 2 < command.length()) {
          String pattern = command.substring(checkPos, checkPos + 3);
          if (pattern == "V1:" || pattern == "P2:") {
            return checkPos;
          }
        }
        
        if (checkPos < command.length()) {
          char c = command.charAt(checkPos);
          if (c == 'S' || c == 'E' || c == 'H' || c == '?' || c == 'L') {
            return checkPos;
          }
        }
      }
    }
  }
  return command.length();
}

// ==================== FUNCIONES DE CONTROL DE MOTORES ====================

void setMotorSpeed(float rpm) {
  // Enviar comando de velocidad al Arduino MEGA
  String cmd = "V1:" + String(rpm, 1) + "\n";
  MegaSerial.print(cmd);
  
  Serial.print("Enviado a MEGA - Motor 1 velocidad: ");
  Serial.print(rpm);
  Serial.println(" RPM");
}

void moveLinearActuator(float cmPos) {
  // Verificar límites
  if (cmPos < MIN_POSITION_CM || cmPos > MAX_POSITION_CM) {
    Serial.print("ERROR: Posición fuera de límites (");
    Serial.print(MIN_POSITION_CM);
    Serial.print(" - ");
    Serial.print(MAX_POSITION_CM);
    Serial.println(" cm)");
    return;
  }
  
  // Enviar comando de posición al Arduino MEGA
  String cmd = "P2:" + String(cmPos, 2) + "\n";
  MegaSerial.print(cmd);
  
  Serial.print("Enviado a MEGA - Actuador posición: ");
  Serial.print(cmPos);
  Serial.println(" cm");
}

void stopMotors() {
  Serial.println("\n=== DETENIENDO MOTORES ===");
  returningToHome = true;
  homeReturnStart = millis();
  
  // Motor 1: Detener en posición actual (velocidad 0)
  String cmd1 = "V1:0\n";
  MegaSerial.print(cmd1);
  Serial.println("Motor 1: Detenido");
  
  // Motor 2: Enviar a posición inicial (0 cm)
  String cmd2 = "P2:0\n";
  MegaSerial.print(cmd2);
  Serial.println("Motor 2: Retornando a 0 cm");
  
  Serial.println("Esperando que el actuador llegue a posición inicial...");
}

void emergencyStop() {
  returningToHome = false;
  
  // Enviar comando de parada de emergencia al Arduino MEGA
  String cmd = "ESTOP\n";
  MegaSerial.print(cmd);
  
  Serial.println("\n⚠ PARADA DE EMERGENCIA ⚠");
  Serial.println("Motores detenidos inmediatamente");
  Serial.println("Use 'S' para retornar a posición inicial");
}

void checkHomeReturn() {
  if (!returningToHome) return;
  
  // Verificar si el actuador llegó a posición 0 (con tolerancia)
  float tolerance = 0.3;  // 3mm de tolerancia
  
  if (abs(cm) <= tolerance) {
    returningToHome = false;
    Serial.println("\n✓ ACTUADOR EN POSICIÓN INICIAL");
    Serial.print("Posición actual: ");
    Serial.print(cm);
    Serial.println(" cm");
    Serial.println("=====================================");
  }
  
  // Timeout de seguridad (10 segundos)
  if (millis() - homeReturnStart > 10000) {
    returningToHome = false;
    Serial.println("\n⚠ Timeout - Retorno a home cancelado");
  }
}

// ==================== FUNCIONES AUXILIARES ====================

void printMenu() {
  Serial.println("\n=== Control de Mecanismo via Arduino MEGA ===");
  Serial.println("Comandos disponibles:");
  Serial.println("  V1:xxx - Velocidad Motor 1 en RPM (ej: V1:30)");
  Serial.println("  P2:xxx - Posición Actuador en cm (0-14.3) (ej: P2:7.5)");
  Serial.println("\n  MÚLTIPLES COMANDOS:");
  Serial.println("  V1:30 P2:5   -> Velocidad M1 a 30 RPM y posición M2 a 5cm");
  Serial.println("\n  S      - Detener motores (Motor 1 para, Motor 2 a 0 cm)");
  Serial.println("  E      - Parada de emergencia (detiene ambos inmediatamente)");
  Serial.println("  L      - Leer estado actual");
  Serial.println("  H o ?  - Mostrar este menú");
  Serial.println("\nLímites actuador: 0 cm (retraído) a 14.3 cm (extendido)");
  Serial.println("=====================================\n");
}

void showStatus() {
  Serial.println("\n--- Estado Actual ---");
  Serial.print("th2: ");
  Serial.print(th2, 4);
  Serial.print(" rad (");
  Serial.print(th2 * 180.0 / PI, 1);
  Serial.println("°)");
  
  Serial.print("Actuador (lDE): ");
  Serial.print(lDE, 2);
  Serial.println(" cm");
  
  Serial.print("O_y (altura): ");
  Serial.print(O_y, 2);
  Serial.println(" cm");
  
  Serial.print("th4 (inclinación): ");
  Serial.print(th4, 2);
  Serial.println("°");
  
  Serial.print("Estado: ");
  if (returningToHome) {
    Serial.println("Retornando a home");
  } else {
    Serial.println("Normal");
  }
  Serial.println("--------------------\n");
}
