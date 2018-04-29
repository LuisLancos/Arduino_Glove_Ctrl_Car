// this is the final version of the code for the Car side, using a servvo control board 
// 28 April 2018
// Luis Lancos

#include <Wire.h>


// Digiole - OLED  Display definitions  
// =============================================================================================================
    //Gyro - Arduino UNO R3
    //VCC  -  5V  
    //GND  -  GND
    //SDA  -  A4
    //SCL  -  A5
     //INT - port-2
     // functions : cleardisplay() , print() , setPrintPos()
     // http://www.digole.com/index.php?productID=540
     

#include <sampledata.h>
#define _Digole_Serial_I2C_ //To tell compiler compile the special communication only, 
//all available are:_Digole_Serial_UART_, _Digole_Serial_I2C_ 
#include <DigoleSerial.h>

//--------Software Serial setup
#if defined(_Digole_SoftSerial_UART_)
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial(10, 11); // RX, TX
  DigoleSerialDisp mydisp(&mySerial, 9600); 
#endif

//--------I2C setup
// the glove uses the IC2 to comunicate with IMU and the OLED screen 
// https://howtomechatronics.com/tutorials/arduino/how-i2c-communication-works-and-how-to-use-it-with-arduino/
// about pull ups resistors in Ic2 bus https://rheingoldheavy.com/i2c-pull-resistors/
// https://playground.arduino.cc/Main/I2CBi-directionalLevelShifter


#if defined(_Digole_Serial_I2C_)
 // #include <Wire.h>
  DigoleSerialDisp mydisp(&Wire,'\x27');  //I2C:Arduino UNO: SDA (data line) is on analog input pin 4, and SCL (clock line) is on analog input pin 5 on UNO and Duemilanove
#endif

#define LCDCol 16
#define LCDRow 2

// HC-05 Bluetooth 
//===================================================================================================================

/*HC 05 AT commands are limited, all I could find are given here.
 * the HC-06 is configured as master and is paired with the HC-06 on the CAR
 *  The HC-05 is configured as Master and defaults to commincation mode when first powered on.
  Needs to be placed in to AT mode
    Pins
      BT VCC to Arduino 5V out. 
      BT GND to GND
      BT RX to Arduino pin 8 (through a voltage divider)
      BT TX to Arduino pin 9 (no need voltage divider)

 Setup example : https://exploreembedded.com/wiki/Setting_up_Bluetooth_HC-05_with_Arduino
      
(source http://www.instructables.com/id/AT-command-mode-of-HC-05-Bluetooth-module/step5/AT-commands/)
AT : check the connection
AT+NAME: Change name. No space between name and command.
AT+BAUD: change baud rate, x is baud rate code, no space between command and code.
AT+PIN: change pin, xxxx is the pin, again, no space.
AT+VERSION

// Importante note 
// Arduino sends and receives all information as ASCII over serial, regardless of data type. 
// to send Integers, receive all the bytes into an array and run atoi() (literally stands for 'array to integer') 
// To send a int ,  have to do is send it over serial via BlueTooth, Serial.print() will convert it automatically.
// When it gets to the other Arduino, that's where atoi()

// HC-05 Datasheet https://www.gme.cz/data/attachments/dsh.772-148.1.pdf
// HC-06 Dayasheet http://silabs.org.ua/bc4/hc06.pdf
*/

#include <SoftwareSerial.h>
SoftwareSerial BTserial(8, 9); // RX, TX   for the HC-05

char c=' ';
boolean NL = true;

// MPU-6060 declarations
// ================================================================================================================

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
// PINs SDLC --> A5 and SDCA --> A4 and INT --> D2
//
// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of 
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte 
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally, 
// and are swapped in code.
//
// http://www.instructables.com/id/Arduino-MPU6050-GY521-6-Axis-Accelerometer-Gyro-3D/
// https://maker.pro/arduino/tutorial/how-to-interface-arduino-and-the-mpu-6050-sensor
// https://www.filipeflop.com/blog/tutorial-acelerometro-mpu6050-arduino/


#define MPU6050_I2C_ADDRESS 0x68


typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;




inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;


#define MPU6050_ACCEL_XOUT_H       0x3B   // R 
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W

String  StrToSend ="";



void setup()
{      
  int error;
  uint8_t c;


  Serial.begin(9600); 
  Wire.begin();                                                     // Starts the IC2 as a master 
  mydisp.begin();                                                   // Starts Digiole display 
  mydisp.clearScreen();                                             //CLear screen 
  BTserial.begin(9600);                                              // Init HC-05 60 9600 Baud rate

  // MU Setup section
  
  MPU6050_read (MPU6050_WHO_AM_I, &c, 1);  
  MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0); 
  //Initialize the angles
  calibrate_sensors();  
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
  
}


void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
  
  unsigned long t_now = millis();   

  // Convert gyro values to degrees/sec
  float FS_SEL = 131; 
  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
 
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;
  
  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180/3.14159;
  //float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  
  float accel_angle_z = atan(sqrt(pow(accel_x,2) + pow(accel_y,2))/accel_z)*RADIANS_TO_DEGREES;;
 
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();
 
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
  
  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
  
  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

     // print to display
    mydisp.setPrintPos(0, 1);
    mydisp.print(" | Angle Pych (x) = "); mydisp.print(angle_x);
    mydisp.setPrintPos(1, 1);
    mydisp.print(" | Angle Roll (y) = "); mydisp.print(angle_y);

    // to remove bit later 
    Serial.print("angle x:  "); 
    Serial.print(angle_x, 2);
    Serial.print(F(" , "));

    Serial.print("angle y:  "); 
    Serial.print(angle_y, 2);
    Serial.print(F(" , "));

    Serial.print("angle z:  "); 
    Serial.print(angle_z, 2);
    Serial.print(F(" ,  "));

// its important to understand the direction of the MU unit. up and down works whe you move vetrtically in the direction of elnght of the rectangle 
// the angles to detect movment are pure experimental 

  mydisp.clearScreen();
  mydisp.setPrintPos(0, 1);
  
  float Turningangle = 0;
  char action=""; 

  // This function display the action, will display to the Digiole just for guidance
  action='+';
  if (angle_x>10)
        Serial.println(" Stop");
        mydisp.print(" Stop");
        action ='S';
  if (angle_x<-10)
       { Serial.print(" Speed"); // this tels about the speed of the car, as steep the angle the faster it moves. 
          float CarSpeed = -angle_x;
          Serial.print( "Car Speed: "); Serial.print( CarSpeed );
          mydisp.print(" Speed");
       // once the car is moving lets check the directions. 
       // I'm giving some leanway and if y axe is +/- 30 degrees, will go stright, otherwise will turn left or right 
        mydisp.setPrintPos(1, 1);
        if (angle_y>5) 
              {
                Serial.print(" Steering RIGHT  - "); 
                action ='R';
                Turningangle = angle_y;
                Serial.print( "Turning angle: "); Serial.println( Turningangle );
                mydisp.print(" Steering RIGHT  - "); mydisp.print(angle_y);
                }
        if  (angle_y<-10)
              {
                Serial.print(" Steering LETFT - ");
                action ='L';
                Turningangle = -angle_y;
                Serial.print( "Turning angle: "); Serial.println( Turningangle );
                mydisp.print(" Steering LEFT  - "); mydisp.print(angle_y);
                }
         else {
                Serial.println(" Straight "); 
                mydisp.print(" Straight "); 
                action ='T';
         }

       }
  else { Serial.println(" other  ");  }


   // The code bellow sends the angles to the CAR bluetooth 
   // message format something like "#RX12344X45678Y637363Z12233$"

   // BTSerial.write("#");    // start with # and ends with $ of the message and a way to guarantee that there is no data corruption. 

    StrToSend ="";

    // StrToSend.concat("*"); // garbadge
    BTserial.print("#");
    BTserial.print(action); BTserial.print('X');BTserial.print(angle_x);BTserial.print('Y');BTserial.print(angle_y);
    BTserial.print('Z');BTserial.print(angle_z); 
    BTserial.print('$');
    BTserial.print('\0');
  // StrToSend.concat('\0');
    //BTserial.write(StrToSend);

    BTserial.print("\n");                        
   // BTserial.print(StrToSend); 
   // BTserial.print("\0"); 
    //BTserial.print("\n"); 
    BTserial.flush();
    //BTSerial.write('X');  
   // BTSerial.write(angle_x);
   // BTSerial.write('Y');  
   // BTSerial.write(angle_y);
   // BTSerial.write('Z');  
   // BTSerial.write(angle_z);
   // BTSerial.write('$');   
   //'\0'

   // Debug only 
   //Serial.print("StrToSend:");Serial.println(StrToSend);
   // Serial.print("#");    // start with # and ends with $ of the message and a way to guarantee that there is no data corruption. 
   //                           // On the receiving part, if the message does not start and ends correctly should be discarted 
   //
  //  Serial.print(action); 
  //  Serial.print("X");  
  //  Serial.print(angle_x);
  //  Serial.print("Y");  
  //  Serial.print(angle_y);
  //  Serial.print("Z");  
  //  Serial.print(angle_z);
  //  Serial.println("$");   


        
  // Delay so we don't swamp the serial port
  delay(100);

}



// Functions - MU 6050
//=========================================================================================================================



void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}


// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}



int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
    
    // Read the raw values.
    // Read 14 bytes at once, 
    // containing acceleration, temperature and gyro.
    // With the default settings of the MPU-6050,
    // there is no filter enabled, and the values
    // are not very stable.  Returns the error value
  
    accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
   
    int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

    // Swap all high and low bytes.
    // After this, the registers values are swapped, 
    // so the structure name like x_accel_l does no 
    // longer contain the lower byte.
    uint8_t swap;
    
    #define SWAP(x,y) swap = x; x = y; y = swap

    SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
    SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
    SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
    SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
    SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
    SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
    SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}


void calibrate_sensors() {
  
    int                   num_readings = 10;
    float                 x_accel = 0;
    float                 y_accel = 0;
    float                 z_accel = 0;
    float                 x_gyro = 0;
    float                 y_gyro = 0;
    float                 z_gyro = 0;
    accel_t_gyro_union    accel_t_gyro;
  
   //Serial.println("Starting Calibration");

   // Discard the first set of values read from the IMU
   // The sensor should be motionless on a horizontal surface 
   //  while calibration is happening
  
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
  
  // Read and average the raw values from the IMU
  
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  
  //Serial.println("Finishing Calibration");
}

