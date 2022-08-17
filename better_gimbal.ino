#include <MemoryFree.h>
#include <SPI.h>
#include <Servo.h>
#include <SFE_MicroOLED.h>


// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
//I2Cdev device library code is placed under the MIT license
//Copyright (c) 2012 Jeff Rowberg

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_RESET 5  // Connect RST to pin 9 (req. for SPI and I2C)
#define PIN_DC    8  // Connect DC to pin 8 (required for SPI)
#define PIN_CS    10 // Connect CS to pin 10 (required for SPI)
#define DC_JUMPER 0 // Set to either 0 (default) or 1 based on jumper, matching the value of the DC Jumper

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
// Declare a MicroOLED object. The parameters include:
// 1 - Reset pin: Any digital pin
// 2 - D/C pin: Any digital pin (SPI mode only)
// 3 - CS pin: Any digital pin (SPI mode only, 10 recommended)
MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS); //Example SPI declaration,

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//PID Variables
int p_0 = 0; //ideal position value
int pError = 0; //difference between actual and p_0
int rError = 0;
int r_0 = 0;
int P = 0; //pitch
int R = 0;
float Y = 0;
int pError_i = 0; // initial Error
int rError_i = 0;
float KP = 1; //proportional constant
float KI = 0.007; // integral constant
float KD = 0.7; // derivative constant
float p_t = 0; //error function over time
float r_t = 0;
int sum_p = 0;
int sum_pi = 0; //integral of pError
int sum_r = 0;
int sum_ri = 0;
float t = 10; //time between measurements

//Servo Variables
Servo pitch;
Servo roll;

//Button variables
const int butt1 = 4;
const int butt2 = 7;
int counter = 0;

//Cube Drawing variables
int d = 3;
int x[] = {
  -d,  d,  d, -d, -d,  d,  d, -d
};
int y[] = {
  -d, -d,  d,  d, -d, -d,  d,  d
};
int z[] = {
  -d, -d, -d, -d,  d,  d,  d,  d
};

int x_2[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
int y_2[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

float r[] = {
  0, 0, 0
};

#define SHAPE_SIZE 600

void(* resetFunc) (void) = 0;//declare reset function at address 0


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
      // join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)M
        //TWBR = two wire baud rate
        Serial.begin(115200);
        while (!Serial);

//OLED initialization
    
    oled.begin();  
    oled.clear(ALL);
    oled.clear(PAGE);
    oled.setFontType(0);  // Set the text to small (10 columns, 6 rows worth of characters).
    //oled.setFontType(1);  // Set the text to medium (6 columns, 3 rows worth of characters).
    //oled.setFontType(2);  // Set the text to medium/7-segment (5 columns, 3 rows worth of characters).
    //oled.setFontType(3);  // Set the text to large (5 columns, 1 row worth of characters).
    oled.setCursor(0, 0);
    //oled screen is 64 wide*48 tall pixels
    oled.print("initializing");
    oled.display();
    
//==================================================
//==================================================

    mpu.initialize();   //initialize mpu
     Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

   
    devStatus = mpu.dmpInitialize();        // load and configure the DMP
    

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

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
        resetFunc();       
    }
//Button assignments
pinMode(butt1, INPUT);
pinMode(butt2, INPUT);

//Servo Assignments
pitch.attach(9);
pitch.write(90);
roll.attach(6);
roll.write(90);

//OLED ready
oled.clear(PAGE);
oled.setCursor(0, 0);
oled.println("ready");
oled.clear(ALL);
delay(500);

}


// ================================================================
// ===                      Main Loop                           ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) resetFunc();

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) 
        fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
// track FIFO count here in case there is > 1 packet available
// (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

// display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //oled.clear(PAGE);
     Y = ypr[0] * 180/3.14;
     P = ypr[1] * 180/3.14;
     R = ypr[2] * 180/3.14;
     Serial.print("ypr\t");
     Serial.print(Y);
     Serial.print("\t");
     Serial.print(P);
     Serial.print("\t");
     Serial.print(R);
     Serial.print("\t");
     Serial.println();
     PID_calc();
     if(digitalRead(butt1) == HIGH){
        counter++;
        if(counter >=2){
          counter = 0; 
        }
     }
     if(counter == 0){
      oled.clear(PAGE);
        OLED_YPR_val();
     }
     if(counter == 1){
      oled.clear(PAGE);
        OLED_virt();
     }
     mpu.resetFIFO();         
   }
   
}

// ================================================================
// ===                      Functions                           ===
// ================================================================

//relays yaw, pitch, roll values of the gyro
void OLED_YPR_val(){
   oled.setCursor(0, 0);
   oled.print("Y ");
   oled.print(Y);
   oled.println();
   oled.print("P ");
   oled.print(P);
   oled.println();
   oled.print("R ");
   oled.print(R);
   oled.display();
}

//depicts a 3D image of the gyro orientation
void OLED_virt(){
  r[0] = Y * PI/180;
  r[1] = R * PI/180; 
  r[2] = P * PI/180; 
  if (r[0] >= 2 * PI) r[0] = 0;
  if (r[1] >= 2 * PI) r[1] = 0;
  if (r[2] >= 2 * PI) r[2] = 0;

  for (int i = 0; i < 8; i++)
  {
    float px2 = x[i];
    float py2 = cos(r[0]) * y[i] - sin(r[0]) * z[i];
    float pz2 = sin(r[0]) * y[i] + cos(r[0]) * z[i];

    float px3 = cos(r[1]) * px2 + sin(r[1]) * pz2;
    float py3 = py2;
    float pz3 = -sin(r[1]) * px2 + cos(r[1]) * pz2;

    float ax = cos(r[2]) * px3 - sin(r[2]) * py3;
    float ay = sin(r[2]) * px3 + cos(r[2]) * py3;
    float az = pz3 - 150;

    x_2[i] = 32 + ax * SHAPE_SIZE / az;
    y_2[i] = 24 + ay * SHAPE_SIZE / az;
  }

  oled.clear(PAGE);
  for (int i = 0; i < 3; i++)
  {
    oled.line(x_2[i], y_2[i], x_2[i + 1], y_2[i + 1]);
    oled.line(x_2[i + 4], y_2[i + 4], x_2[i + 5], y_2[i + 5]);
    oled.line(x_2[i], y_2[i], x_2[i + 4], y_2[i + 4]);
  }
    oled.line(x_2[3], y_2[3], x_2[0], y_2[0]);
    oled.line(x_2[7], y_2[7], x_2[4], y_2[4]);
    oled.line(x_2[3], y_2[3], x_2[7], y_2[7]);
    oled.display();
}

void PID_calc(){
       pError = (p_0 - (P));
     rError = (r_0 - (R));
      if(rError > 90){
        rError = 90;
       }
      if(rError < -90){
        rError= -90;
       }
      if(pError > 90){
        pError = 90;
       }
      if(pError < -90){
        pError = -90;
       }
     sum_p = sum_pi + pError;
     sum_r = sum_ri + rError;
     p_t = KP*pError + KI*(sum_p)*t + KD*((pError - pError_i)/t);
     r_t = KP*rError + KI*(sum_r)*t + KD*((rError - rError_i)/t);
     /*Serial.print("pError ");
     Serial.print(pError);
     Serial.print("\t");  
     Serial.print(p_t);
     Serial.print("\t");*/
      if(p_t > 180){
        p_t = 180;
       }
       if(p_t < -180){
        p_t = -180;
       }  
       if(r_t > 180){
        r_t = 180;
       }
       if(r_t < -180){
        r_t = -180;
       }
     pitch.write(map(p_t, -180, 180, 0, 180));
     roll.write(map(r_t, -180, 180, 0, 180));
     /*Serial.print("\t");
     Serial.print("servo speed ");
     Serial.print(pitch.read());
     Serial.println();*/
     delay(t);
     sum_pi = sum_p;
     sum_ri = sum_r;
     pError_i = pError;
     rError_i = rError;     
}
