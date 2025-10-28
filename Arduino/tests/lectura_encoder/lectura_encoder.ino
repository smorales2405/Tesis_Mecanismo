#include <MeanFilterLib.h>

// Agregar después de #include <MeanFilterLib.h>
hw_timer_t *Timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Constantes
const float Pi = 3.14159265;
const uint16_t cuentas_max = 8400;
const float dt = 2.5;

// Variables lectura encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Variables calculo velocidad
MeanFilter<float> meanFilter(10);
float current_position = 0.0, last_position = 0.0;
float velocidad = 0.0;
double velocidadf = 0.0;
int PWM = 0;

// Pines
int Pin_encoder_A = 16, Pin_encoder_B = 17;
int Pin_DIR = 32, Pin_PWM = 33;

void IRAM_ATTR lectura_encoder();
void IRAM_ATTR ISR_Timer();

void setup()  
{
	Serial.begin(9600); 
	
	pinMode(Pin_encoder_A,INPUT_PULLUP); 
	pinMode(Pin_encoder_B,INPUT_PULLUP);

  pinMode(Pin_DIR,OUTPUT); 
  pinMode(Pin_PWM,OUTPUT); 
	
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), lectura_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), lectura_encoder, CHANGE);

  // Configurar timer: frecuencia = 1MHz / (dt*1000) = 1000000/2500 = 400 Hz
  Timer = timerBegin(1000000);  // 1 MHz (1 microsegundo de resolución)
  timerAttachInterrupt(Timer, &ISR_Timer);
  timerAlarm(Timer, dt * 1000, true, 0);  // 2500 µs, auto-reload, unlimited

}

void loop()
{  
  PWM = 255*0.25;
  digitalWrite(Pin_DIR,LOW);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(500);

  PWM = 255*0.5;
  digitalWrite(Pin_DIR,LOW);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(500);

  PWM = 255*1;
  digitalWrite(Pin_DIR,LOW);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(500);

  PWM = 255*0.25;
  digitalWrite(Pin_DIR,HIGH);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(500);

  PWM = 255*0.5;
  digitalWrite(Pin_DIR,HIGH);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(500);

  PWM = 255*1;
  digitalWrite(Pin_DIR,HIGH);      
  analogWrite(Pin_PWM,PWM);
  Serial.print(PWM); 
  Serial.print(",");
  Serial.println(velocidad);
  delay(500);
}

void IRAM_ATTR ISR_Timer()
{      

    current_position = (cuentas*2*Pi)/cuentas_max; 
    velocidad = ((current_position - last_position)/(dt)*1000);
    velocidad = meanFilter.AddValue(velocidad);
    velocidadf = abs(velocidad);
    
    last_position = current_position;

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
