/*
 * P5_Software_EEPROMV1
 * written by Tomek Steenbock
 * One messure Block is 12 Bytes big
 * The voltage is stated in deci Volt
 * The current is stated in mili Ampere
 * Time is stated in Seconds
*/
//includes
#include <Wire.h>
#include <Servo.h>
#include <SPIMemory.h>

//defines
#define solar_voltage_messure_Pin A1
#define battery_voltage_messure_Pin A3
#define circut_voltage_messure_Pin A0
#define motor_voltage_messure_Pin A6
#define messure_voltage_pin A2
#define ESC_PIN A3
#define Reciver_Pin_Motor 3 
#define EEPROM_addr B1010000

//global Variables
unsigned short write_index_addr = 65.534;
unsigned short mesureBlocks = 0;
boolean passthroughmode = true;

//Objects
Servo motor;
/*
 * 
 * setup function
 * 
 */
void setup() {
  Serial.begin(9600);
  Wire.begin();
  flash.begin(MB(64));
  
  motor.attach(ESC_PIN);
  attachInterrupt(Reciver_Pin_Motor, pwmToESC, CHANGE);
}
/*
 * 
 * loop function
 * 
 */
void loop() {
  //logs data to EEPROM
  while(!Serial){
    dataLogging();
  }
  //reads back all measure blocks if bytes a are received via serial connection (not implemented)
  if(Serial.available() > 0){
    //implement
  }
}
/*
 * 
 * functions
 * 
 */
 //forwards pwm signal from receiver to ESC
void pwmToESC(){
  if(passthroughmode){
    int pwm_value = pulseIn(Reciver_Pin_Motor, HIGH);
    int pwmSignal = map(pwm_value,0,1337,0,180);
    motor.write(pwmSignal); 
  }
}
//measures 6 values and writes them to EEPROM chip every 2 Seconds 
void dataLogging(){
    unsigned short timestamp = millis() / 1000;
    
    unsigned short solarPanelVoltage = mesureSolarPanelVoltage();
    unsigned short currentAtSolar = mesureCurrent(mesureCircuteVoltage(), solarPanelVoltage);
    
    unsigned short batteryVoltage = mesureBatteryVoltage;
    unsigned short current = mesureCurrent(mesureVoltage(), batteryVoltage);
    
    unsigned short height;
    
    if((batteryVoltage * 10) < 717){
      shutdownMotor();
      passthroughmode = false;
    } else if((batteryVoltage * 10) > 819 && !passthroughmode){
      passthroughmode = true;
    }
    
    unsigned short addr = 0 + mesureBlocks*12;
    if(addr != (write_index_addr - 13)){
      writeShortToEEPROM(timestamp, addr);
      writeShortToEEPROM(solarPanelVoltage, (addr + 2));
      writeShortToEEPROM(currentAtSolar, (addr + 4));
      writeShortToEEPROM(batteryVoltage, (addr + 6));
      writeShortToEEPROM(current, (addr + 8));
      writeShortToEEPROM(height, (addr + 10));
    
      mesureBlocks += 1;
      setWritePointer();
    }
    
    delay(2000); // Datenmessen mit 0.5Hz
}
/*
 * 
 * mesure functions
 * 
 */
//measures solar panel voltage
unsigned short measureSolarPanelVoltage(){
   short u2 = analogRead(solar_voltage_messure_Pin);
   unsigned short uGes = (((u2 / 1024.00 * 5) / 0.24859813084112149532710280373832) * 1024 / 5) * 10;
   return uGes;
}
//measures voltage at motor in order to calculate current
unsigned short measureMotorVoltage(){
   short u2 = analogRead(motor_voltage_messure_Pin);
   return uGes;
}
//measures battery voltage
unsigned short  measureBatteryVoltage(){
   short u2 = analogRead(battery_voltage_messure_Pin);
   return uGes;
}
//mesaures volatge in ciruit
unsigned short measureCircuteVoltage(){
  short u2 = analogRead(circut_voltage_messure_Pin);
  unsigned short uGes = (((u2 / 1024.00 *5) / 0.24859813084112149532710280373832) * 1024 / 5) * 10;
  return uGes;
}
//measures voltage in order to calculate current
unsigned short measureVoltage(){
  short u2 = analogRead(messure_voltage_pin);
  return uGes;
}

//mesures current in ciruit
unsigned short mesureCurrent(short u1, short u2){
  unsigned short i = ((((u1/10.00) - (u2/10.00)) / 1024.00 * 5.00) * 0.051) * 1000;
  return i;
}
//measures Height (not implemented)
unsigned short messureHeight(){
  //implement
}
/*
 * 
 * write/read to EEPROM functions
 * 
 */
//writes a unsigned short to given address on EEPROM chip 
void writeShortToEEPROM(unsigned short number,unsigned short addr){
    byte* data = (byte*) &number;
    
    Wire.beginTransmission(EEPROM_addr);
    Wire.write((unsigned short)(addr >> 8)); //sending the higher bytes of the addres (Page address)
    Wire.write((unsigned short) (addr & 0xFF)); //sending the lower bytes of the adress(Byte address)
    
    Wire.write(data, 2);
    
    Wire.endTransmission();
     
    delay(5);
}
//writes a float to given address on EEPROM chip 
void writeFloatToEEPROM(float number,unsigned short addr){
    byte* data = (byte*) &number;
    
    Wire.beginTransmission(EEPROM_addr);
    Wire.write((unsigned short)(addr >> 8)); //sending the higher bytes of the addres (Page address)
    Wire.write((unsigned short) (addr & 0xFF)); //sending the lower bytes of the adress(Byte address)
    
    Wire.write(data, 4);
    
    Wire.endTransmission();
     
    delay(5);
}
//writes the number of meassured blocks at a static address on EEPROM chip
void setWritePointer(){
   Wire.beginTransmission(EEPROM_addr);
   Wire.write((unsigned short) (write_index_addr >> 8));
   Wire.write((unsigned short) (write_index_addr & 0xFF));
   
   Wire.write((unsigned short)(mesureBlocks >> 8)); 
   Wire.write((unsigned short) (mesureBlocks & 0xFF)); 
 
   Wire.endTransmission();
   
   delay(5);
}
//reads back all measure blocks via serial connection to connected computer
void readDataFromEEPROM(){
  //gets the write_pointer
  Wire.beginTransmission(EEPROM_addr);
  Wire.write((unsigned short) (write_index_addr >> 8)); 
  Wire.write((unsigned short) (write_index_addr & 0xFF));
  Wire.endTransmission();
  
  Wire.requestFrom(EEPROM_addr, 2);
  int index = 0;
  byte a[2];
  
  while(Wire.available()){
    a[index] = Wire.read();
    index++;
  }
  
  //reading the EEPROM data
  index = 0;
  int* read_pointer = (int*) a;
  while(index < *read_pointer){
    unsigned short addr = 0 + index * 12;
    Wire.beginTransmission(EEPROM_addr);
    Wire.write((unsigned short) (addr >> 8));
    Wire.write((unsigned short) (addr & 0xFF));
    Wire.endTransmission();
    
    Wire.requestFrom(EEPROM_addr, 12);
    int i = 0;
    byte b[12];
    while(Wire.available() > 0){
      b[i] = Wire.read();
      i++;
    }
    sentDataSerial(b); //sending the messureBlock via serial connection to connected PC
    index++;
  }
}
/*
 * 
 * Sending Functions
 * 
 */
//sents one measure block as byte array to connected computer
void sentDataSerial(byte data[]){
    Serial.write(data,12);
}
/*
 * 
 * Hardware controll functions
 * 
 */
 //shutsdown Motor
void shutdownMotor(){
  motor.write(0);
}
