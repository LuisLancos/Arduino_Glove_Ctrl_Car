


// this is the final version of the code for the Car side, using a servvo control board 
// 28 April 2018
// Luis Lancos

//========== ESC =====================================================================

// this is the code for the Glove controled car. This is the car code. The Car has a servo for left and right and a 25A ESC with reverse. 
// The ESC uses neutral position 90 degree or 1500 µs to arm 
 // This pulse is usually between 1000 to 2000 µs or 1.0 to 2.0 ms, with 1500 µs or 1.5 ms being the center. 
 // Most servomotors actually support signals between 500 to 2500 µs or 0.5 to 2.5 ms. 
 // Actually, if the control signal was at 500 Hz (instead of 50 Hz), it would mean it would repeat every 2 ms,
// https://www.robotshop.com/blog/en/rc-speed-controller-esc-arduino-library-20470
// https://github.com/lobodol/ESC-calibration
// The ESC is attached to pin 6 of Arduino
// 1-1.5ms pulses for reverse, 1.5-2ms to go forwards.


 
 #include "ESC.h"     
#include <SoftwareSerial.h>                               // for the ESC
//#include <Servo.h>                                        // for the servo

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> 

#define LED_PIN (13)                                      // Pin for the LED 
#define SERVOMIN  50 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  650 // this is the 'maximum' pulse length count (out of 4096) - 650

#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds
#define ARM_VALUE 1500

// #define SERVO_PIN (10)

#define SERIAL_BRATE (9600)
#define S_SERVO 0                                           // we are using the steering servo on channel 0 on the servo driver board
#define ESC_Channel 1                                       // using the driver board in channle 1 to drive the ESC



 unsigned int       data = 0;   // variable used to store received data
 const unsigned int upperThreshold = 70;  //upper threshold value
 const unsigned int lowerThreshold = 50;  //lower threshold value
 const byte         MAX_STRING_LEN = 40;
 char               message[MAX_STRING_LEN];  // a string to hold incoming data
 
 ESC myESC (6, SPEED_MIN, SPEED_MAX, 1500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
 int oESC;                                                 // Variable for the speed sent to the ESC


// HC-06
// ===============================================================================================
// Bluetooth connection. I will be using the HC-06 and HC-05 in master-slave configuration to estabilish the comunication bettwen both deviced
// Range bettwen 5m and 15m in pure line of sight

// The HC-05 is configured as Master and defaults to commincation mode when first powered on.
// Needs to be placed in to AT mode
//  Pins
//    BT VCC to Arduino 5V out. 
//    BT GND to GND
//    BT RX to Arduino pin 3 (through a voltage divider)
//    BT TX to Arduino pin 2 (no need voltage divider)

// The HC-06 is confugres as slave and its the component used in the Car side.
//
//  Pins
//    BT VCC to Arduino 5V out. 
//    BT GND to GND
//    BT RX to Arduino pin 8 (through a voltage divider)
//    BT TX to Arduino pin 9 (no need voltage divider)
//    Baurd rate : 9600 - did try with higher ones but allways get some noise in the middle, so this was the most reliable speed
//    Will connect to HC-06 , that is a slave module

//  These HC-06 modules require capital letters and no line ending
//   example : https://gyazo.com/8267b1cd9833ea52591d5dedf7274adb
// toInt() convert char to int


// Importante note 
// Arduino sends and receives all information as ASCII over serial, regardless of data type. 
// to send Integers, receive all the bytes into an array and run atoi() (literally stands for 'array to integer') 
// To send a int ,  have to do is send it over serial via BlueTooth, Serial.print() will convert it automatically.
// When it gets to the other Arduino, that's where atoi()

// HC-05 Datasheet https://www.gme.cz/data/attachments/dsh.772-148.1.pdf
// HC-06 Dayasheet http://silabs.org.ua/bc4/hc06.pdf


 //AT Commands for test:
 //HC 06 AT commands are limited, all I could find are given here.
   // (source http://www.instructables.com/id/AT-command-mode-of-HC-05-Bluetooth-module/step5/AT-commands/)
   //     AT : check the connection
   //     AT+NAME: Change name. No space between name and command.
   //     AT+UART=XXX,X,X: change baud rate, x is baud rate code, no space between command and code.
   //     AT+PIN: change pin, xxxx is the pin, again, no space.
   //     AT+VERSION
   // mySerial.print("AT+PIN1234")



// SERVO - using the PCA 9685 to
//===============================================================================================


// I decided to use the Servo control board, as when using the servo.h library to control directly I was having significant Jitter. 
// Then I learned that this Jotter was due to conflictibg librarys such as serial.h . 
// The other problem that this setup is adressing is the power consuption on the servos, when subject to forces from the CAR sterring can consume significant amount of power
// the way of solving this is to use an external driver for the ESC and Servos
// The servo sits in channel 0 and the ESC  sits in channel 1
// Using A4 and A5 pins to drive the control board

// called this way, it uses the default address 0x40

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!


  // the broad divides the frequency into 4096 ticks , so for 50 MHZ



#define MIN_PMW_T_VAL (204)              // Send a signal of 1000 us under 50 HHZ
#define NEUTRAL_90_PMW_T_VAL (300)              //  at 1460 aprox
#define ARM_PMW_T_VAL (300)              // This is a vlue specifically for the ESC -Trottle at 1460 aprox, needs to be adjusted for diferent ESC
#define STOP_PMW_T_VAL (245)            // ESC, this is the stop value usually 
#define MAX_PMW_T_VAL (408)              //  at 2000 aprox  - full 




// Car position and action control 
// ===================================================================================================
//Horizontal , rest position
//scheme https://forum.arduino.cc/index.php?topic=370222.0

//X ( movement in direction of the shorter sides of the rectangle with led to the back )
//
//eg: angle x:  -0.05 , angle y:  0.03 , angle z:  0.00 ,   other  
//( stable position , x =/- 5 and z=+/- 10 (ignore z in this case as its up and down)
//
//angle x:  12.44 , angle y:  -1.79 , angle z:  -2.22 ,  
//Any angle X > 10 means stopping ( Hand pushed back )
//
//eg: angle x:  -12.49 , angle y:  -23.64 , angle z:  -13.53 ,   Speed: 12.49 Steering LETFT
//angle x < -10 ( pushing forward )
//angle Y < -5Turning Left
//
//eg: Angle x:  -14.51 , angle y:  5.12 , angle z:  -10.63 ,   Speed: 14.51 Steering RIGHT
//angle x < -10 ( pushing forward )
//angle Y < 5 Turning Right
//
//eg: angle x:  -10.74 , angle y:  3.49 , angle z:  -13.77 ,   Speed: 10.74 Straight
//angle x < -10 ( pushing forward )
//angle -5<Y < 5  Straight 
//
//No signal = stop

//======== CODE =========================================================================================


char inChar = ' ';
    String inString;
    String inStringX;
    String inStringY;
    String inStringZ;
    char sig = ' ';
    char action =' ';


    int angle_x;
    int angle_y;
    int angle_z; 


    struct CarEstrutura {
          int p_speed;         // stores the previous speed
          int sspeed;           // Stores the mapped speed for teh ESC
          int Steering;        // stores the mapped value to the servo
          int p_Steering;      // stores the previous mapped value to the servo
          char    Steering_action;  // L, left, R, Right 
          char    Direction;        // S, stop, R- reverse, F, ahead 
          float   ang_x;            // store the orginal angles as received from the glove
          float   ang_y; 
          float   ang_z;
    };

   CarEstrutura CarVar ;  

   

SoftwareSerial BTSerial(8, 9); // RX | TX             // RX and TX pins 
uint16_t  ReceivedNr =0;

// Servo CarServo;                             // Car Servo instance declaration
uint8_t ServoPosition = 0;                  // Global variable storing current servo position

void setup() {

  Serial.begin(SERIAL_BRATE);                                    // Init Serial 
  BTSerial.begin(SERIAL_BRATE);                                 // Init Bluetooth connection
  
  pwm.begin();
  pwm.setPWMFreq(50);                                         // Analog servos run at 50 Hz update
 
 
 
  delay(5000);                                           // let connect the car
  pinMode(LED_PIN, OUTPUT);                               // LED Visual Output
//   myESC.arm();                                            // Send the Arm value so the ESC will be ready to take commands
  Esc_arm();
  digitalWrite(LED_PIN, HIGH);                            // LED High Once Armed
  Serial.print("armed");
  //delay(5000);                                            // Wait for a while

Serial.print("start:");
 
}  




void loop() {

     char copy[50];
    inStringX = "";
    inStringY = "";
    inStringZ = "";
    sig = ' ';
    int wordcount=1;
    inString ="";

 // read Bluettoth and construct the string with the format "#action Y-angle y X-angle x  Z-anglez $'
 // separate the string into X, Y and Z components   
  
  while (BTSerial.available()>0){  
      inChar = BTSerial.read();
       inString += inChar;

        if  (( inChar == '$' ) || ( inChar == '\n' )  ) { inString += "\0";  break;}

            if ( inChar == 'X' ) sig ='X'; 
            if ( inChar == 'Y' ) sig ='Y';
            if ( inChar == 'Z' ) sig ='Z';                  // z value
            if ( inChar == '#' ) sig ='A';                  // the sig var is used to identified what component we are reading Y, Z, Y 
            if ( inChar == "" ) sig ='V';                   // action like, S-top, F-oward, R-ight, L-eft ( received from the glove)

            switch ( sig ) {                                // if sig equals X, means we will be reading the X float value
                case 'X':
                    if (inChar !='X') inStringX += inChar;  // The first character is letter X itself, so we ignore
                    break;
                case 'Y':         
                    if (inChar !='Y') inStringY += inChar;
                    break;      
                case 'Z':
                    if (inChar !='Z') inStringZ += inChar;
                    break;
                case 'A':
                    if (inChar !='#') action = inChar;
                    break;
                case 'V':                                   // if we have spaces the received string was not complete, so ignore
                     break;
                } 
                
         delay(5);                                          // without this delay doesn't work
  }

  
     
   int strl =   inString.length();
   if  (( inString != '$' ) && ( inString != '\n' ) && ( inString != '\0' )  && (int)inString.length() >=10) // this filters some garbage that still persists 
     {
        // convert the read values to float, their original format. 
        CarVar.ang_x = inStringX.toInt();
        CarVar.ang_y = inStringY.toInt();
        CarVar.ang_z = inStringZ.toInt();

        CalculateCarActionAngles(CarVar.ang_x,CarVar.ang_y,CarVar.ang_z); // use the angles to calculate the action of the car, speed and steering
        
        Serial.print("steering value:"); Serial.println(CarVar.ang_y); // debug only 
        
        CarSteering( CarVar.ang_y, CarVar.p_Steering ); // Will turn the weels of the Car in the correct direction
        SpeedCar(CarVar.sspeed, CarVar.p_speed );   

// The prints bellow are for Debug purpose mainly
      //  Serial.println(" "); // force new line
      //  Serial.print("inString:");Serial.println(inString);
      //  Serial.print("Str lenght:");Serial.println(strl);
     //   Serial.print("X:"); Serial.println(CarVar.ang_x);
     //   Serial.print("Y:"); Serial.println(CarVar.ang_y);
     //   Serial.print("Z:"); Serial.println(CarVar.ang_z);
     //   Serial.print("action:"); Serial.println(CarVar.Direction);

     }
}                     


// ========== Control Steering Servo ==============================================================

float  CarSteering ( int angles, int p_angles  ) {
  //   uses the global struct and to steer
  //   basically it  controls de steering servo 
  //   Servo is attached to Pin 10 and uses the Servo.h library to control de motion in angles

 //   Serial.print("Value to convert:");Serial.println(angles); // debug info
 //   Serial.print("Mapped value:");Serial.println(map(angles , -170, 170, SERVOMIN, SERVOMAX)); // debug info
 //   CarServo.attach (SERVO_PIN);   

  pwm.setPWM(S_SERVO, 0, map(angles , -170, 170, SERVOMIN, SERVOMAX));
 
  // delay(5);               // wait the servo to move into position
  
}


// ========== Control Speed - ESC ==============================================================

void Esc_arm( ){

// this can varie for the various ESC devices
// the current Hobby King ESC. arms at 1470-1500 so the process is 
// process, tottlre must be in minimum position --> battery connect and wait 
// push trollt to 1470-1500 position, wait for the servo to arm
// ready

Serial.print("arming INIT");

pwm.setPWM(ESC_Channel, 0, ARM_PMW_T_VAL);
delay(5000); // 2 sec for  ESC to arm

Serial.print("arming FIM");
  
}


float  SpeedCar ( int sspeed, int p_speed) {
  // uses the global struct and to steer
  // basically it  controls de ESC
  // https://www.robotshop.com/blog/en/rc-speed-controller-esc-arduino-library-20470


  if (sspeed == 0) pwm.setPWM(ESC_Channel, 0, STOP_PMW_T_VAL); // if Spped contains its because of the stop action, so we should stop the ESC.
   else  pwm.setPWM(ESC_Channel,0, map(-sspeed, -170, 170, MIN_PMW_T_VAL, MAX_PMW_T_VAL )); 

}


 // ========== Control Speed - ESC ==============================================================

void CalculateCarActionAngles(int angulox, int anguloy, int anguloz){
  
    // Store and converts the angles in the struct 
    // The MU6050 using the wire.h library, as used by the glove, return angles bettwen -170 and 170 so it required mapping to the servo angles bettwen 0 and 180
    // X axis mesures foward and bckward mov and Y the steering angle

    // store previous values
     CarVar.p_speed = CarVar.sspeed; 
     CarVar.p_Steering = CarVar.Steering;

    // copies base values to Steering and Speed 
    CarVar.sspeed = angulox; // this will be mapped later on into pulses ( 1000-2000 range ) 

    CarVar.Steering = (int)anguloy; // will be mapped alter on
    //Serial.print("sInside function:"); Serial.println(CarVar.Steering);

    // based on the input from the glove, it clculates the actions expected from the car
    // those are stored in a struct 
    //struct CarEstrutura {
    //      uint16_t p_speed;       // stores the previous speed
    //      uint16_t speed;         // Stores the mapped speed for teh ESC
    //      uint16_t Steering;      // stores the mapped value to the servo
    //      uint16_t p_Steering;    // stores the previous mapped value to the servo
    //      char    Steering_action;           // L, left, R, Right 
    //      char    Direction;            // S, stop, R- reverse, F, ahead 
    //      float   ang_x;          // store the orginal angles as received from the glove
    //      float   ang_y; 
    //      float   ang_z;
    // 
    //      map the X and Y angles to the action. Moving ahead, Left or Right, Stop and mantain current course
   
   if (angulox > 10 && angulox < 45 )
      {
        Serial.print(" STOP -> ");
        CarVar.Direction = 'S';
        CarVar.Steering_action='S';
        CarVar.sspeed = 0;                   // is is to the speed must be zero. If that ESC would have breaks this would be the point we would activate the breaks
      }
   else {
       if (angulox > 45 ) 
              {
                Serial.print(" Steer reverse  -> "); 
                CarVar.Direction = 'B';
                CarVar.Steering_action='B';
                CarVar.sspeed =120; 
                }
        else 
          {
                
            if (angulox <-10 && anguloy > 5 ) 
                  {
                    Serial.print(" Steer RIGHT  -> "); 
                    CarVar.Direction = 'F';
                    CarVar.Steering_action='R';
         
                    }
            if  (angulox<-10  && anguloy <-5 )
                  {
                    Serial.print(" Steer LETFT -> ");
                    CarVar.Direction = 'F';
                    CarVar.Steering_action='L';
    
                    }
             if ( angulox <-10 && anguloy <5 && anguloy >-5 )
                  {
                    Serial.print(" Straight -> ");
                    CarVar.Direction = 'F';
                    CarVar.Steering_action='M';     // Go straight so goes
                    CarVar.Steering = 90;           // Force neutral 
    
                  }
             if ( angulox <10 && angulox >-10 ) 
                  {
                    Serial.print(" Stable -> ");
                    CarVar.Direction = 'O';
                    CarVar.Steering_action=' ';
                    CarVar.sspeed = CarVar.p_speed ;     // if its stable, we maintain the speed
                   }
       
   }

  
}
}


float checkpositive( int value ) {

 //   if (value < 0 ) value = -value;
return value;
}




