/**
 ******************************************************************************
 * @file           : V2.1
 * @author         : ALy Mustafa
 * @brief          : Secret Box Program
 ******************************************************************************
 **/
 
// ================================================================
// ===                       INCLUDES                           ===
// ================================================================
#include <Arduino.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include <U8x8lib.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
// ================================================================
// ===                    General_Variables                     ===
// ================================================================
#define Buzzer 23
/*======= Massages =======*/
char s1[] = "Box is opened";
char s2[] = "Box is falled";
/*======= counters =======*/
unsigned long OpenCounter=0;
unsigned long FallCounter=0;
/*======= Flags =======*/
byte FallFlag = 0;
byte OpenFlag = 0;
/*======= Timer =======*/
unsigned long currentTime;
unsigned long lastTime;
unsigned long dead_time; // Duration in milliseconds

// ================================================================
// ===                    EEPROM_Variables                     ===
// ================================================================
#define EEPROM_SIZE 4096 // Example EEPROM size in bytes
#define MAX_STRING_LENGTH 20 // Maximum length of each string to be stored in EEPROM
//MAX_STRING_LENGTH --> can be used if wanna clear eeprom//
#define address1 0
#define address2 1
#define address3 2
byte eeprom_Flag;
byte savedNumber;
// ================================================================
// ===                 OLED_Display_Variables                   ===
// ================================================================
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// ================================================================
// ===                    ULTRASONIC_Parametars                 ===
// ================================================================
#define trig 18
#define echo 5
#define BoxLength 20
float distance=0,t=0;
float distanCe;
// ================================================================
// ===                    MPU6050_Parametars                    ===
// ================================================================
/* ==  Aceleration  == */
float aX;
float aY;
float aZ;
// ==  Angles  ==/
/*float Yaw;
float Pitch;
float Roll;*/
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define SDA 21 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SCL 22 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
// ===               INTERRUPT DETECTION ROUTINE                ===
volatile bool mpuInterrupt = true;     // indicates whether MPU interrupt pin has gone high
/*void dmpDataReady() {
    mpuInterrupt = true;
    
}*/
//====================================================================================================================
//====================================================================================================================

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
 void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
      Serial.begin(115200);
      eeprom_init();
      u8x8.begin();
      u8x8.setFont(u8x8_font_7x14B_1x2_r); 
      u8x8.noInverse();   
      u8x8.setCursor(0,0);
      pinMode(Buzzer, OUTPUT);
      UltraSonic_init();
    // ================================
    // ===         MPU_INIT         ===
    // ================================
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(SDA,SCL,400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    //Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    //pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(480); 
    mpu.setYAccelOffset(-1147); 
    mpu.setZAccelOffset(1060); 
    mpu.setXGyroOffset(-653);
    mpu.setYGyroOffset(-52);
    mpu.setZGyroOffset(0);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        /*Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
*/
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
      millis();
     // =================================================
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() 
{
  app_Start();
}

// ================================================================
// ===                      APPLICATION                         ===
// ================================================================
void app_Start() 
{
  View_From_EEPROM();
  mpu_update();
  aX = mpu_get_aX();
  //Serial.print("  Pitch:\t");
  aY = mpu_get_aY();
  //Serial.print("  Roll:\t");
  aZ = mpu_get_aZ();
while (checkBox_isOpened() || checkBox_isFalling() )
  {
    //View_From_EEPROM();
  }
  //when Nothing Happened
  eeprom_Flag = readNumFromEEPROM(address3);
  digitalWrite(Buzzer, LOW);
}
// ============================================================================================================
// ============================================================================================================

// ================================================================
// ===                        Main APIs                         ===
// ================================================================
void View_From_EEPROM(){
  /*Just tests
    Serial.print("OpenCounter ");
    Serial.println(OpenCounter);
    Serial.print("FallCounter ");
    Serial.println(FallCounter);*/

switch (eeprom_Flag) {
  case 0 : {
    Serial.print(F("Safe Box !!  "));
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
    u8x8.draw1x2String(0, 0, "Safe Box !!  ");
    u8x8.clearLine(5);
    u8x8.clearLine(6);
    u8x8.clearLine(7);
  break;
  }
  case 1 : {
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);   
  u8x8.draw1x2String(0, 0, s1);
  currentTime = millis();
  dead_time = currentTime - lastTime;
  if (dead_time > 2000){
      /*    for frequently happening actions you want     */
  lastTime = currentTime;
  }
  break;
  }
  case 2 : {
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f); 
  u8x8.draw1x2String(0, 5, s2);
  currentTime = millis();
  dead_time = currentTime - lastTime;
  if (dead_time > 2000){
      /*    for frequently happening actions you want     */
  lastTime = currentTime;
  }
  break;
  }
}
}

char checkBox_isOpened()
{
distanCe = Ultrasonic_get_distance();      
  if (distanCe > BoxLength){      
    if (OpenFlag == 0){
        OpenCounter++;
        eeprom_Flag = readNumFromEEPROM(address1);
        digitalWrite(Buzzer, HIGH);
        View_From_EEPROM();
        OpenFlag = 1;    
        }
        return 1;  
  }
  else {
    OpenFlag = 0;
    return 0;
  }
}

char checkBox_isFalling()
{
if( (aX > 3000) || (aX < -3000) || (aY > 2000) || (aY < -2000) || (aZ > 4500) || (aZ < -4500))
  {
    while( (aX > 3000) || (aX < -3000) || (aY > 2000) || (aY < -2000) || (aZ > 4500) || (aZ < -4500))
    {
      /*if (checkBox_isOpened())*/

       if (FallFlag == 0){
          FallCounter++;
          eeprom_Flag = readNumFromEEPROM(address2);
          digitalWrite(Buzzer, HIGH);
          View_From_EEPROM();
          FallFlag = 1 ;
       }
    mpu_update(); //to update the values
    aX = mpu_get_aX();
    //Serial.print("  Pitch:\t");
    aY = mpu_get_aY();
    //Serial.print("  Roll:\t");
    aZ = mpu_get_aZ();
    return 1;
    }
    }
else{
    FallFlag = 0;
    return 0;

    }
    }

// ================================================================
// ===                         Helper_APIs                            ===
// ================================================================
float mpu_get_aX()
{
   /* Serial.print("X: ");
    Serial.print(aaWorld.x);
    Serial.print("\t");*/
    return (aaWorld.x);
}
float mpu_get_aY()
{
  /* Serial.print("Y: ");
    Serial.print(aaWorld.y);
    Serial.print("\t");*/
    return (aaWorld.y);    
}
float mpu_get_aZ()
{
  /* Serial.print("Z: ");
    Serial.println(aaWorld.z);*/
    return (aaWorld.z);
}
float Ultrasonic_get_distance(void){
digitalWrite(trig,LOW);
delayMicroseconds(5);
digitalWrite(trig,HIGH);
delayMicroseconds(10);
digitalWrite(trig,LOW);
t=pulseIn(echo,HIGH);
distance=t*0.0175;  //Distance = (Speed of Sound * Time/2) = t/(1/((350*0.0001)/2))
//Serial.println(distance);
delay(50);  
return distance;
}

void eeprom_init(){
  EEPROM.begin(EEPROM_SIZE);     //Initialize EEPROM with specified size
  saveNumToEEPROM(address1 , 1); //Indication for Open
  saveNumToEEPROM(address2 , 2); //Indication for Fall
  saveNumToEEPROM(address3 , 0); //Indication for Nothing happened
}
void saveNumToEEPROM(int address, byte Num) {
  EEPROM.write(address, Num);
  EEPROM.commit();
}

byte readNumFromEEPROM(byte address) {
  savedNumber = EEPROM.read(address);
  return savedNumber;
}

void clearByteEEPROM(byte address ) {
  // Fill the specified EEPROM address range with null characters ('\0')
  EEPROM.write(address , '\0');
  EEPROM.commit();  // Save changes to EEPROM
  //Serial.println("cleared!");
}
void UltraSonic_init(void)
{
pinMode(trig,OUTPUT);
pinMode(echo,INPUT);
}

void mpu_update(void){
// ================================
// ===         MPU_START        ===
// ================================
// if programming failed, don't try to do anything
if (!dmpReady) return;
// read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // display angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //===============display Angles================//
      /*Yaw   =  mpu_get_Yaw();
      //Serial.print("  Pitch:\t");
      Pitch =  mpu_get_Pitch();
      //Serial.print("  Roll:\t");
      Roll  =  mpu_get_Roll();*/
      //==============================================//
      // display Acceleration
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }
}