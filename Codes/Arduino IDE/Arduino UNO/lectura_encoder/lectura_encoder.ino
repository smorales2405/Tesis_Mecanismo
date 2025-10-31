#include <TimerOne.h>
#include <MeanFilterLib.h>

// Constantes
const uint16_t cuentas_max = 8400;
const float dt = 100;

// Variables lectura encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Variables calculo velocidad
MeanFilter<float> meanFilter(10);
float current_position = 0.0, last_position = 0.0;
double velocidad = 0.0; 
double velocidadf = 0.0, RPM = 0.0;

// Pines
int Pin_encoder_A = 3, Pin_encoder_B = 2;

void setup()  
{
	Serial.begin(9600); 
	
	pinMode(Pin_encoder_A,INPUT_PULLUP); 
	pinMode(Pin_encoder_B,INPUT_PULLUP); 
	
	cli();
	attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), lectura_encoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), lectura_encoder, CHANGE);
	Timer1.initialize(dt*1000); //Pines 9 y 10
	Timer1.attachInterrupt(ISR_Timer) ; 
	sei();
}

void loop()
{  
  Serial.print("Cuentas: "); 
  Serial.print(cuentas);
  Serial.print(" Velocidad: ");
  Serial.print(RPM);
  //Serial.print(abs(Dato2-velocidad));
  //Serial.print(" ");
  //Serial.print(0); 
  //Serial.print(" ");
  //Serial.print(10); 
  //Serial.print(" ");
  //Serial.print(-10);
  Serial.println(" ");
}

void ISR_Timer()
{      

  current_position = (cuentas*2*PI)/cuentas_max; 
  velocidad = (current_position - last_position)/(dt/1000);
  velocidad = meanFilter.AddValue(velocidad);
  velocidadf = abs(velocidad);
  RPM = (velocidad/(2*PI))*60; 

  last_position = current_position;

}

void lectura_encoder()
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
