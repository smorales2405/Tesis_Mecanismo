#include <MeanFilterLib.h>

// Timer para ESP32
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Constantes
const float Pi = 3.14159265;
const uint16_t cuentas_max = 8400;
const float dt = 100; //Milisegundos
const int freq_timer = 1000000 / (dt * 1000);

// Variables lectura encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Variables calculo velocidad
MeanFilter<float> meanFilter(10);
float current_position = 0.0, last_position = 0.0;
double velocidad = 0.0; 
double velocidadf = 0.0, RPM = 0.0;

// Pines
int Pin_encoder_A = 26, Pin_encoder_B = 27;
int Pin_DIR = 32, Pin_PWM = 33;

// Configuración PWM para ESP32
const int freq = 7812;  // Frecuencia PWM en Hz
const int ledChannel = 8;  // Canal PWM
const int resolution = 8;  // Resolución de 8 bits (0-255)
int PWM = 0;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  
  current_position = (cuentas*2*Pi)/cuentas_max; 
  velocidad = (current_position - last_position)/(dt/1000);
  //velocidad = meanFilter.AddValue(velocidad);
  velocidadf = abs(velocidad);
  RPM = (velocidad/2*PI)*60; 

  last_position = current_position;
  
  isrCounter = isrCounter + 1;
  lastIsrAt = millis();
  
  /*
  Serial.print("onTimer no. ");
  Serial.print(isrCounter);
  Serial.print(" at ");
  Serial.print(lastIsrAt);
  Serial.print(" ms ");
  Serial.print("cuentas: "); 
  //Serial.print(",");
  Serial.println(cuentas); 
  //Serial.print(",");
  //Serial.println(velocidad);
  */
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  //xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
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

void setup() {
  Serial.begin(115200);

  pinMode(Pin_encoder_A, INPUT_PULLUP); 
  pinMode(Pin_encoder_B, INPUT_PULLUP); 
  pinMode(Pin_DIR, OUTPUT); 
  pinMode(Pin_PWM, OUTPUT);

  ledcAttachChannel(Pin_PWM, freq, resolution, ledChannel);

  // Configurar interrupciones del encoder
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), lectura_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), lectura_encoder, CHANGE);

  // Set BTN_STOP_ALARM to input mode
  //pinMode(BTN_STOP_ALARM, INPUT_PULLUP);

  // Create semaphore to inform us when the timer has fired
  //timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 1Mhz
  timer = timerBegin(1000000);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, dt*1000, true, 0);
}

void loop() {
  // If Timer has fired
  //if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    //uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    //portENTER_CRITICAL(&timerMux);
    //isrCount = isrCounter;
    //isrTime = lastIsrAt;

    //PWM = 255*1;
    //digitalWrite(Pin_DIR,HIGH);      
    //ledcWrite(Pin_PWM,PWM);

    Serial.print("onTimer no. ");
    Serial.print(isrCounter);
    Serial.print(" at ");
    Serial.print(lastIsrAt);
    Serial.print(" ms ");
    Serial.print("cuentas: "); 
    //Serial.print(",");
    Serial.print(cuentas); 
    Serial.print(" Velocidad: ");
    Serial.println(RPM);

    //portEXIT_CRITICAL(&timerMux);
    // Print it
    //Serial.print("onTimer no. ");
    //Serial.print(isrCount);
    //Serial.print(" at ");
    //Serial.print(isrTime);
    //Serial.println(" ms");
  //}
  // If button is pressed
  //if (digitalRead(BTN_STOP_ALARM) == LOW) {
    // If timer is still running
  //  if (timer) {
  //    // Stop and free timer
  //    timerEnd(timer);
  //    timer = NULL;
  //  }
  //}
}