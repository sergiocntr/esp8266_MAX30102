#ifndef ALGORITHM_H_
#define ALGORITHM_H_
#include <Arduino.h>
//#include "QuickStats.h"
//#include "max30102.h"
//#include <CircularBuffer.h>
#include <Wire.h>
#include "debugutils.h"
//#define DEBUGMIO
#include "MAX30105.h"

MAX30105 particleSensor;
// Interrupt pin
const byte interruptPin = 16; // pin connected to MAX30102 INT
const uint8_t BUFFER_SIZE  = 400; //numero di campionamenti
const uint8_t DELAY_BETWEEN_DATA  = 20; //numero di campionamenti
//CircularBuffer<data::record, BUFFER_SIZE> stack;
int bufferRed[BUFFER_SIZE] ;
int bufferIr[BUFFER_SIZE] ;
//QuickStats stats; //initialize an instance of this class
uint32_t f_ir_mean,f_red_mean;

long samplesTaken = 0;
float average(int* samples)
{

  float total1=0.0;
  for(int i=0;i<BUFFER_SIZE;i++){
    total1=total1+samples[i];
  }
  return total1/(float)BUFFER_SIZE;
}
float slope(int* samples)  //calculate the slope (dsamples/dx)
{
  //long startTime=millis();
  float xavg=100;
  float yavg=average(samples);
  float numerator = 0.0;
  float denominator = 0.0;
  for(int i=0;i<BUFFER_SIZE;i++){
    if(i-xavg!=0){ // protect against dividing by zero
      numerator = numerator + (i-xavg)*(samples[i]-yavg);
      denominator = denominator + ((i-xavg)*(i-xavg));
    }
  }
  //startTime =millis() - startTime;
  //DEBUG_PRINT("spent time: " + String(startTime));
  float myslope = numerator/denominator;
  return myslope;
}

void readSensor(){
  //long startTime=millis();
  samplesTaken = 0;
  while (1)//do we have new data?
  {
    bufferRed[samplesTaken] = f_red_mean - particleSensor.getRed();
    bufferIr[samplesTaken]= f_ir_mean - particleSensor.getIR();
    delay(DELAY_BETWEEN_DATA);
    samplesTaken++;
    if (samplesTaken==BUFFER_SIZE) break;
  }
  //startTime =millis() - startTime;
  //DEBUG_PRINT("spent time: " + String(startTime));
  float myslope=0.0;
  myslope=slope(bufferRed);
  DEBUG_PRINT("Slope RED: " + String(myslope));
  myslope=slope(bufferIr);
  DEBUG_PRINT("Slope IR: " + String(myslope));
}
void calculate_DC(){
  // calculates DC mean and subtracts DC from ir and red
  f_ir_mean=0;
  f_red_mean=0;
  long startTime = millis();
  samplesTaken = 0;
  particleSensor.check(); //Check the sensor, read up to 3 samples
  while (1)//do we have new data?
  {
    f_red_mean += particleSensor.getRed();
    f_ir_mean += particleSensor.getIR();
    delay(DELAY_BETWEEN_DATA);
    samplesTaken++;
    if (samplesTaken==BUFFER_SIZE) break;

  }
  startTime =millis()-startTime;
  f_ir_mean= f_ir_mean/samplesTaken  ;
  delay(5);
  f_red_mean= f_red_mean/samplesTaken ;
  delay(5);
  DEBUG_PRINT("Media IR: " + String(f_ir_mean));
  DEBUG_PRINT("Media RED: " + String(f_red_mean));
  DEBUG_PRINT("Sample Taken : " + String(samplesTaken));
  DEBUG_PRINT("Time Taken : " + String(startTime));

};// serve a trovare il valore medio,nel buffer metteremo solo la differenza
void initFirst(){
  pinMode(interruptPin, INPUT);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    DEBUG_PRINT("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Let's configure the sensor to run fast so we can over-run the buffer and cause an interrupt
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  //particleSensor.setup();
  //particleSensor.enableAFULL(); //Enable the almost full interrupt (default is 32 samples)

  //particleSensor.setFIFOAlmostFull(3); //Set almost full int to fire at 29 samples


  DEBUG_PRINT("Init: OK");
}
//questo non è usato
void bubbleSort(int A[],int len)
{
  unsigned long newn;
  unsigned long n=len;
  int temp=0;
  do {
    newn=1;
    for(int p=1;p<len;p++){
      if(A[p-1]>A[p]){
        temp=A[p];           //swap places in array
        A[p]=A[p-1];
        A[p-1]=temp;
        newn=p;
      } //end if
    } //end for
    n=newn;
  } while(n>1);
}
#endif
