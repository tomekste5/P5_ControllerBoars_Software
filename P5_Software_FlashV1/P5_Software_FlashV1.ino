/* 
 * P5_Software_FlashV1 
 * written by Tomek Steenbock
 * One measure Block is 24 Bytes big
 * Voltage is stated in Volt
 * Current is stated in Ampere
 * Time is stated in Seconds
*/

//includes
#include <SPIMemory.h> 
#include <Servo.h>


#define live
#define debug
//defines
#define circut_voltage_messure_Pin A0 // measure!
#define solar_voltage_messure_Pin A1
#define messure_voltage_pin A2
#define battery_voltage_messure_Pin A3
#define motor_voltage_messure_Pin A6
#define ESC_PIN 9  
#define Reciver_Pin_Motor 3
#define AUX_Pin 2 // ändern
#define taster_Pin 6 // ändern
#define writePointerAddr 8388603

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
Servo motor;
/*
 * 
 * setup, loop function
 * 
 */
void setup() {
  Serial.begin(9600);
  flash.begin();
  motor.attach(ESC_PIN);
}

void loop() {
  boolean erase = true;
  //If g_flyMode is true it will erase the flash chip and start loggings data and forward throttle signal.
  while(g_flyMode){
    if(erase){
      flash.eraseChip();
      erase= false;
    }
    dataLogging();
    reciverSignalToESC();
    //activateDeactivateOverwrite();
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
void reciverSignalToESC(){
  if(g_passthroughmode){
    int pwmValue = pulseIn(Reciver_Pin_Motor, HIGH);
       int pwmSignal = map(pwmValue,1100,1800,20,160);
        if(pwmSignal > 160){
          motor.write(160);
        }else if(pwmSignal < 20){
          motor.write(20);
        }
        else{
          motor.write(pwmSignal);
        }
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
    
      float solarPanelVoltage = measureVoltageAtSolarPanel();
      float currentAtSolar = measureCurrent(measureVoltageInCircut(), solarPanelVoltage); // float oder short

      float motorVoltage = measureVoltageAtMotor();
      float motorCurrent = measureCurrentMotor((motorVoltage), measureVoltageAtBattery());
    
      float batteryVoltage = measureVoltageAtBattery();
      float current = measureCurrent(measureVoltage(), batteryVoltage);
      
      checkBatteryVoltage(batteryVoltage);
//|------------------------------------------------------|
      uint32_t addr = g_measureBlocks * 24; 
     // probably 160 (if there is not enough space between the addr where the writePointer is written and the addr of the new measureblock, the system will stop to datalog
       writeFloatToFlash(addr, timeStamp);
       writeFloatToFlash(addr + 4, solarPanelVoltage);
       writeFloatToFlash(addr + 8, currentAtSolar);//65
       writeFloatToFlash(addr + 12, batteryVoltage);//97 etcv
       writeFloatToFlash(addr + 16, current); //batteryCurrent
       writeFloatToFlash(addr + 20, motorCurrent);
       g_measureBlocks +=1;
       #ifdef live
       Serial.print(" MotorCurrent: ");
       Serial.print(motorCurrent);
       Serial.print("MotorVoltage: ");
       Serial.print(motorVoltage);
       Serial.print("Timespamp: ");
       Serial.print(timeStamp);
       Serial.print(" SolarVoltage: ");
       Serial.print(solarPanelVoltage);
       Serial.print("CurrentAtSolar: ");
       Serial.print(currentAtSolar);
       Serial.print(" batteryVoltage: ");
       Serial.print(batteryVoltage);
       Serial.print(" System voltage: ");
       Serial.print(measureVoltageAtBattery());
       Serial.print(" CircuteCurrent: ");
       Serial.println(current);
       #endif

     
     /* Serial.print("MeasureBlocks: ");
      Serial.println(g_measureBlocks);*/
      
      g_elapseTime = millis() + 200; // Datenmessen mit 2Hz
    }
}
/*
 * 
 * mesure functions
 * 
 */
//measures solar panel voltage
float measureVoltageAtSolarPanel(){//measures the Voltage at the solar Panel
   float u2 = analogRead(solar_voltage_messure_Pin);
   float u = ((u2 / 1024.00 * 5.0) / 0.24859813084112149532710280373832);
   return u;
}
//measures voltage at motor in order to calculate current
float measureVoltageAtMotor(){//measures the voltage at the motor
   float u = analogRead(motor_voltage_messure_Pin) / 1024.0 * 5.0; 
   return u;
}
//measures battery voltage
float measureVoltageAtBattery(){//measures the voltage at the battery
   float u = analogRead(battery_voltage_messure_Pin) / 1024.0 * 5.0; 
}
//mesaures volatge in ciruit
float measureVoltageInCircut(){
  float u2 = analogRead(circut_voltage_messure_Pin); 
  float u = ((u2 / 1024.00 * 5.0) / 0.24859813084112149532710280373832);
  return u;
}
//measures voltage in order to calculate current
float measureVoltage(){
  float u = analogRead(messure_voltage_pin) / 1024.0 * 5.0;
  return u;
}

//mesures current in ciruit
float measureCurrent(float u1, float u2){//measures the current with two voltages
  float i = ((u1 - u2) / 0.051);
  
  return i;
}
//measures the drawn current of motor
float measureCurrentMotor(float u1, float u2){//measures the current with two voltages
  float i = ((u1 - u2) / 0.001);
  
  return i;
}
/*
 * 
 * write/read to Flash functions
 * 
 */
//writes a float variable to a given address in the flash chip
void writeFloatToFlash(uint32_t addr, float data){
  flash.writeFloat(addr,data);
}
//reads all measure blocks and sents it via Serial to a computer
void readFromFlash(){
  float measureBlock[6]; 
  Serial.println("Time stamp,solarPanelVoltage,currentAtSolar,batteryVoltage,current,MotorCurrent");//sents the headlines for the csv file
  int readPointer = 0;
  while(true){
    //reads in one measure block
    for(int i = 0; i <= 5; i++){
      measureBlock[i] = flash.readFloat(readPointer * 24 + i * 4);
    }
    //checks if there are more measure blocks to read out
    if(isnan(measureBlock[0])){
      break;
    }
    sentData(measureBlock);// sents the data in csv format to the computer
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
  for(int i = 1; i < 6; i++){
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
void checkBatteryVoltage(float batteryVoltage){//Checks the batteryVoltage(if its too low)
  if((batteryVoltage) < 3.5 && !g_overwriteSecureFunctions){//if the voltage drops below 3.5V and the overwriteSecurefunction isnt activated, the motor will shutdown
      g_passthroughmode = false;
      shutdownMotor();
    } else if((batteryVoltage > 3.99 && !g_passthroughmode) || (!g_passthroughmode && g_overwriteSecureFunctions)){//if the overwriteSecureFunctions is true the Passthroughmode will be activaded even if the voltage is below 3.99V
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
  motor.write(0);
}
