/*
 * P5_Software_FlashV2
 * written by Tomek Steenbock
 * One measure Block is 20 Bytes big
 * Voltage is stated in Volts
 * Current is stated in Amps
 * Time is stated in Seconds
*/

//includes
#include <SPIMemory.h>
#include <Servo.h>

//#define Servo
//#define BatteryCutOffFunction
#define live
#define debug

#define ESC_PIN 9  
#define Reciver_Pin_Motor 3
#define AUX_Pin 2
#define taster_Pin 6 

//global Variables
int g_measureBlocks = 0;
boolean g_passthroughmode = true;
boolean g_overwriteSecureFunctions = false;
boolean g_flyMode = false;
float g_elapseTime = 0;
int g_oldPwmValue = 0;
int g_oldPwmSignalAux = 0;

//Objects
SPIFlash flash(4);
#ifdef Servo
  Servo motor; 
#endif
/*
 * 
 * setup, loop function
 * 
 */
void setup() {
  Serial.begin(9600);
  flash.begin();

  #ifdef Servo
    motor.attach(ESC_PIN);
  #endif
}

void loop() {
  boolean erase = false;
  //If g_flyMode is true it will erase the flash chip and start loggings data and forward throttle signal.
  while(g_flyMode){
    if(!erase){
      flash.eraseChip();
      erase= false;
    }
    dataLogging();
    forwardRecvSignal();
    #ifdef BatteryCutOffFunction
      activateDeactivateOverwrite();
    #endif
  }
  //chechks if flightmode switch is pressed. If yes it will set g_flyMode to true.
  if(digitalRead(taster_Pin) == LOW && !g_flyMode){
    g_flyMode = true;
    #ifdef debug
      Serial.println("Hi i am in flight mode now!\nErasing chip......");
    #endif
  }
  //Will read back every measure block if a byte is received via a serial connection.
  if(Serial.available() > 0 && !g_flyMode){
    readFromFlash();
    flushSerialBuffer();
  }
  
}
/*
 * 
 * functions
 * 
 */
//will forward pwm signal from a reciver to a ESC if g_passthroughmode true.
void forwardRecvSignal(){// reads the pwm Signal from the reciver and transfers it to the ESC_PIN(if passthroughmode is true)
  if(g_passthroughmode){
    int pwmValue = pulseIn(Reciver_Pin_Motor, HIGH);
     #ifdef Servo
       int pwmSignal = map(pwmValue,1100,1800,20,160);
        if(pwmSignal > 160){
              motor.write(160);
        }else if(pwmSignal < 20){
              motor.write(20);
        }else{
              motor.write(pwmSignal);
        }
        
    #else
      int pwmSignal = map(pwmValue,1100,1800,20,200);
        if(pwmSignal > 200){
              analogWrite(ESC_PIN,200);
        }else if(pwmSignal < 20){
              analogWrite(ESC_PIN,20);
        }else{
              analogWrite(ESC_PIN,pwmSignal);
        }
    #endif
  }
}
//will read every byte in Serial buffer to clear it.
void flushSerialBuffer(){
  while(Serial.available() > 0){
    char t  = Serial.read();
  }
}
//measures 5 float variabkes every 200ms and writes it to flash. If live is defined it will output all measured values via a serial connection to the connected computer
void dataLogging(){
    if(g_elapseTime <= millis()){
     float timeStamp = millis() / 1000.00;
    
     float solarLow  = measureSolarLow();
     float solarHigh = measureSolarHigh();
     float MDL = measureMDL();
     float batteryVoltage = measureBattery();
      
     checkBatteryVoltage(batteryVoltage);
//|------------------------------------------------------|
     uint32_t addr = g_measureBlocks * 20; 

     writeFloatToFlash(addr, timeStamp);
     writeFloatToFlash(addr + 4, solarLow);
     writeFloatToFlash(addr + 8, solarHigh);
     writeFloatToFlash(addr + 12, MDL);
     writeFloatToFlash(addr + 16, batteryVoltage);
     g_measureBlocks +=1;
     
     #ifdef live
       Serial.print("Timespamp: ");
       Serial.print(timeStamp);
       Serial.print("V SolarLow: ");
       Serial.print(solarLow);
       Serial.print("V SolarHigh: ");
       Serial.print(solarHigh);
       Serial.print("V MDL: ");
       Serial.print(MDL);
       Serial.print("V Battery Voltage: ");
       Serial.print(batteryVoltage);
       Serial.println("V");
     #endif

     g_elapseTime = millis() + 200; // Datenmessen mit 5Hz
    }
}

/*
 * 
 * mesure functions
 * 
 */
//measures solar voltage thats loading the low cell
float measureSolarLow(){
   float u = analogRead(A0) / 1024.0 * 5.0; 
   return u;
}
//measures voltage of one cell
float measureMDL(){
   float u = analogRead(A2) / 1024.0 * 5.0; 
   return u;
}
//measures voltage between both battery cellls
float measureBattery(){
  float u2 = analogRead(A3);
  float u = ((u2 / 1024.00 * 5.0) / 0.6);
  return u;
}
//measures solar voltage thats loading the high cell
float measureSolarHigh(){
  float u2 = analogRead(A1);
  float u = ((u2 / 1024.00 * 5.0) / 0.53003533568904593639575971731449);
  return u;
}

/*
 * 
 * write/read to Flash functions
 * 
 */
//writes a float variable to a given address in the flash chip
void writeFloatToFlash(uint32_t addr, float data){//writes a Float to the flash chip
  flash.writeFloat(addr,data);
}
//reads all measure blocks and sents it via Serial to a computer
void readFromFlash(){
  float measureBlock[5]; 
  Serial.println("Time stamp,SolarLow,SolarHigh,MDL,BatteryVoltage");//sents the headlines for the csv file
  int readPointer = 0;
  while(true){
    //reads in one measure block
    for(int i = 0; i < 5; i++){
      measureBlock[i] = flash.readFloat(readPointer * 20 + i * 4);
    }
    //checks if there are more measure blocks to read out
    if(isnan(measureBlock[0])){
      break;
    }
    sentData(measureBlock);// sents the data in csv format to a computer
    readPointer++;
  }
}

/*
 * 
 * Sending Functions
 * 
 */
//sents a measure block in csv format via a serial connection to a computer
void sentData(float measureblock[]){
  Serial.print(measureblock[0]);
  for(int i = 1; i < 5; i++){
    Serial.print(",");
    Serial.print(measureblock[i]);
  }
  Serial.println();
}

/*
 * 
 * Hardware controll functions
 * 
 */
//checks if the battery voltage to low and cuts off Motor if it is
void checkBatteryVoltage(float batteryVoltage){
  if((batteryVoltage) < 6.8 && !g_overwriteSecureFunctions){
      g_passthroughmode = false;
      shutdownMotor();
    } else if((batteryVoltage > 7 && !g_passthroughmode) || (!g_passthroughmode && g_overwriteSecureFunctions)){
      g_passthroughmode = true;
    }
}
//activates or deactivates motor cut off function
void activateDeactivateOverwrite(){// will activate the overwriteSecureFunctions variable, if it was false and deactivate it, if it was true
  int pwm_value = pulseIn(AUX_Pin, HIGH);
  if(pwm_value - g_oldPwmSignalAux > 10 ||pwm_value - g_oldPwmSignalAux < -10){
    if(g_overwriteSecureFunctions){
      g_overwriteSecureFunctions = false;
     }else{
      g_overwriteSecureFunctions = true;
     }
    g_oldPwmSignalAux = pwm_value;
  }
}
//shutsdown Motor
void shutdownMotor(){
  #ifdef Servo
    motor.write(0);
  #else
    analogWrite(ESC_PIN,0);
  #endif
}
