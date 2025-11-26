#include <TimerOne.h>
#include <MeanFilterLib.h>

// Constantes
const uint16_t cuentas_max = 145.1;
const float dt = 100;

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
int Abs_PWM = 0.0;

// Variables Serial
const byte numChars = 32;
char receivedChars[numChars];  
boolean newData = false;
float dataNumber = 0.0, Setpoint = 0.0;
unsigned long PrintTime = 50;
unsigned long currentTime = 0, lastPrintTime = 0;

// Pines
int Pin_encoder_A = 3, Pin_encoder_B = 2;
int Pin_DIR = 5, Pin_PWM = 6;

void setup()  
{
	Serial.begin(115200); 
	
	pinMode(Pin_encoder_A,INPUT_PULLUP); 
	pinMode(Pin_encoder_B,INPUT_PULLUP); 
	
  TCCR0B = TCCR0B & B11111000 | B00000010;   // Set PWM frequency of 7812.50 Hz for D5 & D6

	cli();
	attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), lectura_encoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), lectura_encoder, CHANGE);
	Timer1.initialize(dt*1000); //Pines 9 y 10
	Timer1.attachInterrupt(ISR_Timer) ; 
	sei();
}

void loop()
{ 

  LeerSerial();
  ObtenerPWM();

  currentTime = millis();
  if (currentTime - lastPrintTime >= PrintTime) {
    lastPrintTime = currentTime;
    EnviarDatos();
  }

}

void ISR_Timer()
{      

    if (PWM > 0) {

        digitalWrite(Pin_DIR,LOW);      
        analogWrite(Pin_PWM,Abs_PWM);

    }
    else if (PWM < 0) {

        digitalWrite(Pin_DIR,HIGH);      
        analogWrite(Pin_PWM,Abs_PWM);

    }
    else {

        digitalWrite(Pin_DIR,LOW);      
        analogWrite(Pin_PWM,0);

    }

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

void ObtenerPWM() {
    if (newData == true) {
        dataNumber = 0;             
        dataNumber = atof(receivedChars);
        PWM = dataNumber/100.0*255.0;
        Abs_PWM = abs(PWM);
        newData = false;
    }
}

void EnviarDatos(){
  //Serial.print("Cuentas: "); 
  //Serial.print(cuentas);
  //Serial.print(" Velocidad: ");
  Serial.print(RPM);
  //Serial.print(abs(Dato2-velocidad));
  Serial.print(" ");
  Serial.print(PWM); 
  //Serial.print(" ");
  //Serial.print(-500); 
  //Serial.print(" ");
  //Serial.print(500); 
  //Serial.print(" ");
  //Serial.print(-10);
  Serial.println(" ");
}