#include <PID_v1.h>
#include <TimerOne.h>

// Constantes
const float Pi = 3.14159265;
const float cuentas_max = 145.1;
const uint16_t mm_por_rev = 8;
const float dt = 2.5;

// Variables lectura encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Variables calculo posicion
float current_position = 0.0, last_position = 0.0;
double posicionf = 0.0;

// Variables Control PID
float Kp = 40.0, Ki = 10.0, Kd = 0.0;
double PWM = 0.0, Abs_Setpoint = 0.0, Error_pos = 0.0; 
double tolerancia = 0.1;
double Prcntg_PWM = 0.0, Voltaje = 0.0;
int U = 0;
PID PosPID(&posicionf, &PWM, &Abs_Setpoint, Kp, Ki, Kd, DIRECT); 

// Variables Serial
const byte numChars = 32;
char receivedChars[numChars];  
boolean newData = false;
float dataNumber = 0.0, Setpoint = 0.0;

// Pines
int Pin_encoder_A = 20, Pin_encoder_B = 21;
int Pin_DIR = 8, Pin_PWM = 9;

void setup()  
{
	Serial.begin(9600); 

	pinMode(Pin_encoder_A,INPUT_PULLUP); 
	pinMode(Pin_encoder_B,INPUT_PULLUP); 

	pinMode(Pin_DIR,OUTPUT); 
  pinMode(Pin_PWM,OUTPUT);

	//TCCR0B = TCCR0B & B11111000 | B00000010;   // Set PWM frequency of 7812.50 Hz for D5 & D6  
	TCCR2B = TCCR2B & B11111000 | B00000001;  // Prescaler = 1 para pines 9 y 10
	//TCCR2B = TCCR2B & B11111000 | B00000010;     // Set PWM frequency of 3921.16 Hzfor D3 & D11 

	PosPID.SetMode(AUTOMATIC);
	PosPID.SetTunings(Kp,Ki,Kd);
	PosPID.SetOutputLimits(-255, 255);

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
  ObtenerSetpoint();
  EnviarDatos();
}

void ISR_Timer()
{      

    PosPID.Compute();
    U = abs(PWM);
    Prcntg_PWM = (PWM/255.0)*100.0;
    Voltaje = 12*sqrt(abs(PWM)/255.0);

    Error_pos = dataNumber - current_position;

    if (PWM < 0.0 && abs(Error_pos) > tolerancia) {

        digitalWrite(Pin_DIR,HIGH);      
        analogWrite(Pin_PWM,U);
        Prcntg_PWM = -Prcntg_PWM;
        Voltaje = -Voltaje;

    }
    else if (PWM > 0.0 && abs(Error_pos) > tolerancia) {

        digitalWrite(Pin_DIR,LOW);      
        analogWrite(Pin_PWM,U);

    }
    else if (abs(Error_pos) < tolerancia) {

        digitalWrite(Pin_DIR,LOW);      
        analogWrite(Pin_PWM,0);
        Prcntg_PWM = 0.0;
        Voltaje = 0.0;
    }

    // Posicion actual en cm
    current_position = (cuentas/cuentas_max)*mm_por_rev/10; 
    posicionf = abs(current_position);

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

void ObtenerSetpoint() {
    if (newData == true) {
        dataNumber = 0;             
        dataNumber = atof(receivedChars);   
        Setpoint = dataNumber;
        Abs_Setpoint = abs(dataNumber);
        newData = false;
    }
}

void EnviarDatos(){
  Serial.print(Setpoint); 
  Serial.print(",");
  Serial.print(current_position);
  Serial.print(",");
  Serial.print(Voltaje);
  Serial.print(" ");
  Serial.print(0); 
  Serial.print(" ");
  Serial.print(15); 
  Serial.println(" ");
  //Serial.print(-10);
  //Serial.println(" ");
}