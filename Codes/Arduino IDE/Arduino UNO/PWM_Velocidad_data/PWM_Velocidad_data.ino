#include <TimerOne.h>
#include <MeanFilterLib.h>

// ==================== CONFIGURACIÓN DE PARÁMETROS ====================

// Tiempo que cada valor de PWM se mantiene activo (en segundos)
const float tiempoPorPWM = 12.0;

// Intervalo de impresión por serial (en milisegundos)
const unsigned long PrintTime = 100;

// -------------------- MODO MANUAL --------------------
// Descomentar esta sección para usar valores definidos manualmente
// Comentar la sección de MODO ALEATORIO


#define MODO_MANUAL
#ifdef MODO_MANUAL
  // Define aquí los valores de PWM en porcentaje (-100 a 100)
  // Evitar valores entre -10 y 10
  //float valoresPWM[] = {-30, -40, -50, -60, -70, -80};
  float valoresPWM[] = {-50};
  const int cantidadValores = sizeof(valoresPWM) / sizeof(valoresPWM[0]);
#endif


// -------------------- MODO ALEATORIO --------------------
// Descomentar esta sección para usar valores aleatorios
// Comentar la sección de MODO MANUAL
/*
#define MODO_ALEATORIO
#ifdef MODO_ALEATORIO
  const int PWM_MIN = 15;          // Mínimo porcentaje de PWM (positivo)
  const int PWM_MAX = 50;         // Máximo porcentaje de PWM
  const int cantidadValores = 10;  // Cantidad de valores a generar
  float valoresPWM[cantidadValores];
#endif
*/

// ==================== FIN CONFIGURACIÓN ====================

// Constantes
const uint16_t cuentas_max = 6533;
const float dt = 10;

// Variables lectura encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Variables calculo velocidad
MeanFilter<float> meanFilter(10);
float current_position = 0.0, last_position = 0.0;
double velocidad = 0.0; 
double velocidadf = 0.0, RPM = 0.0;

// Variables PWM
double PWM = 0.0;
int Abs_PWM = 0;

// Variables Serial
const byte numChars = 32;
char receivedChars[numChars];  
boolean newData = false;

// Variables de tiempo
unsigned long currentTime = 0, lastPrintTime = 0;

// Variables para secuencia de PWM
bool secuenciaActiva = false;
int indiceActual = 0;
unsigned long tiempoInicioPWM = 0;
unsigned long duracionPWM_ms;
unsigned long tiempoInicioSecuencia = 0;

// Pines
int Pin_encoder_A = 3, Pin_encoder_B = 2;
int Pin_DIR = 5, Pin_PWM = 6;

void setup()  
{
  Serial.begin(115200); 
  
  // Inicializar semilla para números aleatorios
  randomSeed(analogRead(A0));
  
  // Calcular duración en milisegundos
  const float factorCompensacion = 8.0;
  duracionPWM_ms = (unsigned long)(tiempoPorPWM * 1000.0 * factorCompensacion);
  
  // Generar valores aleatorios si está en modo aleatorio
  #ifdef MODO_ALEATORIO
    generarValoresAleatorios();
  #endif
  
  pinMode(Pin_encoder_A, INPUT_PULLUP); 
  pinMode(Pin_encoder_B, INPUT_PULLUP); 
  
  TCCR0B = TCCR0B & B11111000 | B00000010;   // Set PWM frequency of 7812.50 Hz for D5 & D6  

  cli();
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), lectura_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), lectura_encoder, CHANGE);
  Timer1.initialize(dt*1000); //Pines 9 y 10
  Timer1.attachInterrupt(ISR_Timer); 
  sei();
  
  // Mostrar configuración inicial
  Serial.println("=== Sistema listo ===");
  Serial.print("Valores de PWM a enviar: ");
  Serial.println(cantidadValores);
  Serial.print("Tiempo por cada PWM: ");
  Serial.print(tiempoPorPWM);
  Serial.println(" segundos");
  Serial.println("Enviar 'S' para iniciar, 'D' para detener");
  Serial.println("Valores PWM configurados:");
  for(int i = 0; i < cantidadValores; i++) {
    Serial.print(valoresPWM[i]);
    Serial.print("% ");
  }
  Serial.println();
  Serial.println("=====================");
}

void loop()
{ 
  LeerSerial();
  ProcesarComando();
  EjecutarSecuencia();

  currentTime = millis();
  if (currentTime - lastPrintTime >= PrintTime) {
    lastPrintTime = currentTime;
    if (secuenciaActiva) {
      EnviarDatos();
    }
  }
}

void ISR_Timer()
{      
  current_position = (cuentas * 2 * PI) / cuentas_max; 
  velocidad = (current_position - last_position) / (dt / 1000);
  velocidad = meanFilter.AddValue(velocidad);
  velocidadf = abs(velocidad);
  RPM = (velocidad / (2 * PI)) * 60; 

  last_position = current_position;
}

void lectura_encoder()
{    
  if(digitalRead(Pin_encoder_A) == 1)
    bitSet(current_state, 1);
  else
    bitClear(current_state, 1);
   
  if(digitalRead(Pin_encoder_B) == 1)
    bitSet(current_state, 0);
  else
    bitClear(current_state, 0);

  if(last_state == 3 && current_state == 1)
    cuentas++;
  if(last_state == 1 && current_state == 0) 
    cuentas++;
  if(last_state == 0 && current_state == 2)
    cuentas++;
  if(last_state == 2 && current_state == 3)
    cuentas++;   
     
  if(last_state == 3 && current_state == 2)
    cuentas--;
  if(last_state == 2 && current_state == 0)
    cuentas--;
  if(last_state == 0 && current_state == 1)
    cuentas--;
  if(last_state == 1 && current_state == 3)
    cuentas--;
    
  last_state = current_state; 
}

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

void ProcesarComando() {
  if (newData == true) {
    // Comando 'S' o 's' para iniciar secuencia
    if (receivedChars[0] == 'S' || receivedChars[0] == 's') {
      iniciarSecuencia();
    }
    // Comando 'D' o 'd' para detener
    else if (receivedChars[0] == 'D' || receivedChars[0] == 'd') {
      detenerSecuencia();
    }
    newData = false;
  }
}

void iniciarSecuencia() {
  if (!secuenciaActiva) {
    secuenciaActiva = true;
    indiceActual = 0;
    tiempoInicioPWM = millis();
    tiempoInicioSecuencia = millis();
    
    // Aplicar primer valor de PWM
    aplicarPWM(valoresPWM[indiceActual]);
    
    Serial.println("I"); // Indica inicio de secuencia
  }
}

void detenerSecuencia() {
  secuenciaActiva = false;
  indiceActual = 0;
  
  // Detener motor
  PWM = 0;
  Abs_PWM = 0;
  digitalWrite(Pin_DIR, LOW);      
  analogWrite(Pin_PWM, 0);

  Serial.println("D"); // Confirma detención
}

void EjecutarSecuencia() {
  if (!secuenciaActiva) return;
  
  unsigned long tiempoActual = millis();
  
  // Verificar si es tiempo de cambiar al siguiente valor de PWM
  if (tiempoActual - tiempoInicioPWM >= duracionPWM_ms) {
    indiceActual++;
    
    // Verificar si terminó la secuencia
    if (indiceActual >= cantidadValores) {
      finalizarSecuencia();
      return;
    }
    
    // Aplicar siguiente valor de PWM
    tiempoInicioPWM = tiempoActual;
    aplicarPWM(valoresPWM[indiceActual]);
  }
}

void aplicarPWM(float porcentaje) {
  PWM = porcentaje / 100.0 * 255.0;
  Abs_PWM = abs((int)PWM);
  
  // Aplicar PWM al motor
  if (PWM > 0) {
    digitalWrite(Pin_DIR, LOW);      
    analogWrite(Pin_PWM, Abs_PWM);
  }
  else if (PWM < 0) {
    digitalWrite(Pin_DIR, HIGH);      
    analogWrite(Pin_PWM, Abs_PWM);
  }
  else {
    digitalWrite(Pin_DIR, LOW);      
    analogWrite(Pin_PWM, 0);
  }
}

void finalizarSecuencia() {
  secuenciaActiva = false;
  indiceActual = 0;
  
  // Detener motor
  PWM = 0;
  Abs_PWM = 0;
  digitalWrite(Pin_DIR, LOW);      
  analogWrite(Pin_PWM, 0);
  
  Serial.println("F"); // Indica fin de secuencia
}

void EnviarDatos() {
  // Calcular tiempo transcurrido (compensado por factor Timer0)
  unsigned long tiempoTranscurrido = millis() - tiempoInicioSecuencia;
  float tiempoSegundos = (tiempoTranscurrido / 8.0) / 1000.0;
  
  float porcentajePWM = (PWM / 255.0) * 100.0;
  
  Serial.print(tiempoSegundos, 3);
  Serial.print(" ");
  Serial.print(porcentajePWM);
  Serial.print(" ");
  Serial.println(RPM);
}

#ifdef MODO_ALEATORIO
void generarValoresAleatorios() {
  for (int i = 0; i < cantidadValores; i++) {
    int valor = 0;
    
    // Generar valor aleatorio entre PWM_MIN y PWM_MAX
    // Excluyendo el rango -10 a 10 solo si el rango lo cruza
    do {
      valor = random(PWM_MIN, PWM_MAX + 1);
    } while (valor > -10 && valor < 10);
    
    valoresPWM[i] = (float)valor;
  }
}
#endif
