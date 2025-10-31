#include <PID_v1.h>
#include <MeanFilterLib.h>

// Timer para ESP32
hw_timer_t * timer = NULL;
volatile bool timer_flag = false;

// Constantes
const float Pi = 3.14159265;
const uint16_t cuentas_max = 6533;
const float dt = 2.5; //Milisegundos

// Variables lectura encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Variables calculo velocidad
MeanFilter<float> meanFilter(10);
float current_position = 0.0, last_position = 0.0;
float velocidad = 0.0;
double velocidadf = 0.0;

// Variables Control PID
float Kp = 5.0, Ki = 10.0, Kd = 0.0;
double PWM = 0.0, Abs_Setpoint = 0.0, Prcntg_PWM = 0.0, Voltaje = 0.0;
int U = 0;
PID VelPID(&velocidadf, &PWM, &Abs_Setpoint, Kp, Ki, Kd, DIRECT); 

// Variables Serial
const byte numChars = 32;
char receivedChars[numChars];  
boolean newData = false;
float dataNumber = 0.0, Setpoint = 0.0;

// Pines
int Pin_encoder_A = 16, Pin_encoder_B = 17;
int Pin_DIR = 32, Pin_PWM = 33;

// Configuración PWM para ESP32
const int freq = 7812;  // Frecuencia PWM en Hz
const int ledChannel = 0;  // Canal PWM
const int resolution = 8;  // Resolución de 8 bits (0-255)

void IRAM_ATTR lectura_encoder();
void IRAM_ATTR ISR_Timer();

void setup()  
{
    Serial.begin(9600); 
    
    pinMode(Pin_encoder_A, INPUT); 
    pinMode(Pin_encoder_B, INPUT); 
    pinMode(Pin_DIR, OUTPUT); 
    pinMode(Pin_PWM, OUTPUT);

    // Configurar PWM para ESP32
    ledcAttachChannel(Pin_PWM, freq, resolution, ledChannel);
    
    VelPID.SetMode(AUTOMATIC);
    VelPID.SetTunings(Kp, Ki, Kd);
    VelPID.SetSampleTime(dt);
    
    // Configurar interrupciones del encoder
    attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), lectura_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), lectura_encoder, CHANGE);
    
    // Configurar Timer para ESP32 (nueva API v3.x)
    // Frecuencia = 1000000 / (dt * 1000) = 1000 / dt Hz
    timer = timerBegin(1000000 / (dt * 1000));  // Frecuencia en Hz
    timerAttachInterrupt(timer, &ISR_Timer);
    timerStart(timer);
}

void loop()
{ 

  LeerSerial();
  ObtenerSetpoint();
  EnviarDatos();

}

void IRAM_ATTR ISR_Timer()
{      
    
    VelPID.Compute();
    U = PWM;
    Prcntg_PWM = (PWM/255.0)*100.0;
    Voltaje = 12*sqrt(PWM/255.0);

    if (Setpoint > 0) {
        digitalWrite(Pin_DIR, LOW);      
        ledcWrite(Pin_PWM, U);  // Cambiar analogWrite por ledcWrite
    }
    else if (Setpoint < 0) {
        digitalWrite(Pin_DIR, HIGH);      
        ledcWrite(Pin_PWM, U);  // Cambiar analogWrite por ledcWrite
        Voltaje = -Voltaje;
    }
    else {
        digitalWrite(Pin_DIR, LOW);      
        ledcWrite(Pin_PWM, 0);  // Cambiar analogWrite por ledcWrite
    }
 
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
  Serial.print(velocidad);
  Serial.print(",");
  Serial.println(U);
  //Serial.print(" ");
  //Serial.print(0); 
  //Serial.print(" ");
  //Serial.print(10); 
  //Serial.print(" ");
  //Serial.print(-10);
  //Serial.println(" ");
}