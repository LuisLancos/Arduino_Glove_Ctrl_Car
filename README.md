# Arduino_Glove_Ctrl_Car

Introduction

This was my first project in Arduino. The project uses an IMU with accelerator and gyroscope to detect the hand position. This is done using the IMU 6050. 

The position of te IMU is then translated into angles along the x and y axes (the z axe is calculated but for now ignored). The X axis to acceleration, forward and backward movement and, Y axis to fine left and right. 

The action ( Forward, Stop, Left, Right ) is concatnated together with the angle on both axes and then sent to the car via bluetooth.

Using the HC-05 and HC-06 bluetooth modules paired as master and slave to establish the communication between glove and car. 

I did add a OLED display on the glove to monitor the measured angle and action. Although not essencial is helps to debug and monitor the behaviour of the glove/IMU 6050. 


The Car uses a HC-06 bluetooth module paired with the glove Bluetooth. Its receives and parses the input stream and then translates this into PWM impulses for the Steering Servo and the Speed controller, ESC. 

Both Servo and ESC are controlled via a PCA9685 Adafruit 16-Channel 12-bit PWM/Servo Driver module. The X axis angles is then translated into a proportional speed instruction to the ESC. The Y axis angles are translated into PWM signals to the servo, allowing to steer left and right as per measured angle. 


