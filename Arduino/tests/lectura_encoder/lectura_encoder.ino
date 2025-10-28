#include <MeanFilterLib.h>

// Timer para ESP32
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Constantes
const float Pi = 3.14159265;
const uint16_t cuentas_max = 8400;
const float dt = 1; //Milisegundos
const int freq_timer = 1000000 / (dt * 1000);

// Variables lectura encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Variables calculo velocidad
MeanFilter<float> meanFilter(10);
float current_position = 0.0, last_position = 0.0;
float velocidad = 0.0;
double velocidadf = 0.0;

// Pines
int Pin_encoder_A = 16, Pin_encoder_B = 17;
int Pin_DIR = 32, Pin_PWM = 33;

// Configuración PWM para ESP32
const int freq = 7812;  // Frecuencia PWM en Hz
const int ledChannel = 0;  // Canal PWM
const int resolution = 8;  // Resolución de 8 bits (0-255)
int PWM = 0;

void IRAM_ATTR ISR_Timer()
{      
  portENTER_CRITICAL_ISR(&timerMux);  
  current_position = (cuentas*2*Pi)/cuentas_max; 
  velocidad = ((current_position - last_position)/(dt)*1000);
  velocidad = meanFilter.AddValue(velocidad);
  velocidadf = abs(velocidad);
  
  last_position = current_position;
  Serial.println("ISR_Timer");
  portEXIT_CRITICAL_ISR(&timerMux);
    
}

void IRAM_ATTR lectura_encoder()
{
    if(digitalRead(Pin_encoder_A)==1)
      bitSet(current_state,1);
    else
      bitClear(current_state,1);
     
    if(digitalRead(Pin_encoder_B)==1)
      bitSet(current_state,0);
    else
      bitClear(current_state,0);
  
    if(last_state==3 && current_state==1)
      cuentas++;
    if(last_state==1 && current_state==0) 
      cuentas++;
    if(last_state==0 && current_state==2)
      cuentas++;
    if(last_state==2 && current_state==3)
      cuentas++;   
       
    if(last_state==3 && current_state==2)
      cuentas--;
    if(last_state==2 && current_state==0)
      cuentas--;
    if(last_state==0 && current_state==1)
      cuentas--;
    if(last_state==1 && current_state==3)
      cuentas--;
      
    last_state = current_state; 
}

void setup()  
{
    Serial.begin(115200);
    
    pinMode(Pin_encoder_A, INPUT); 
    pinMode(Pin_encoder_B, INPUT); 
    pinMode(Pin_DIR, OUTPUT); 
    pinMode(Pin_PWM, OUTPUT);
    
    // Configurar interrupciones del encoder
    attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), lectura_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), lectura_encoder, CHANGE);
    
    // Configurar Timer para ESP32 (nueva API v3.x)
    // Frecuencia = 1000000 / (dt * 1000) = 1000 / dt Hz
    timer = timerBegin(freq_timer);  // Frecuencia en Hz
    timerAttachInterrupt(timer, &ISR_Timer);
    timerAlarm(timer, 100, true, 0);

    // Configurar PWM para ESP32
    ledcAttach(Pin_PWM, freq, resolution);
}

void loop()
{  
  PWM = 255*0.25;
  digitalWrite(Pin_DIR,LOW);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.print(cuentas); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(5000);

  PWM = 255*0.5;
  digitalWrite(Pin_DIR,LOW);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.print(cuentas); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(5000);

  PWM = 255*1;
  digitalWrite(Pin_DIR,LOW);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.print(cuentas); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(5000);

  PWM = 255*0.25;
  digitalWrite(Pin_DIR,HIGH);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.print(cuentas); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(5000);

  PWM = 255*0.5;
  digitalWrite(Pin_DIR,HIGH);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.print(cuentas); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(5000);

  PWM = 255*1;
  digitalWrite(Pin_DIR,HIGH);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.print(cuentas); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(5000);
}
