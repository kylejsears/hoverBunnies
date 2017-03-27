/*
 * Code to control a hovercraft via bluetooth connection w/ a PS4 controller
 * Original setup uses Arduino Uno + Arduino USB Host Shield + OSEPP Motor and Servo Shield
 * Written by: Kyle Sears
 * PS4BT example sketch used to create a connection w/ controller
 * Problems: Controller does connect, however input from controller fails to be read
 */
 
//PS4BT example sketch by Kristian Lauszus gives framework for PS4 controller connection
#include <PS4BT.h>
#include <usbhub.h>
#include <AFMotor.h>
#include <Wire.h>
#include <Servo.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
/*
 * Initializing objects
 */
USB usb; //usb instance
BTD Btd(&usb); // bluetooth dongle instance at usb port
//PS4BT PS4(&Btd, PAIR); //uncomment if a new controller is going to be paired
PS4BT PS4(&Btd);

AF_DCMotor leftThrust(1);
AF_DCMotor rightThrust(2);
Servo steeringServo;
Servo payloadServo;

uint8_t oldR2Value = 0;
int steeringPos = 90;
int payloadPos = 0;
int drop = 60;

//variables for timing
long next_time = 0;
long next_time01 = 0;
long next_time02 = 0;

void setup() {
  Serial.begin(115200);

  
  #if !defined(__MIPSEL__)
   while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif

  leftThrust.setSpeed(0);
  rightThrust.setSpeed(0);

  steeringServo.attach(9);
  payloadServo.attach(10);
  steeringServo.write(steeringPos);
  payloadServo.write(payloadPos);
  
  if (usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}
void loop() {
  usb.Task();

  //will run as long as controller is connected
  if (PS4.connected()){

    long current_time = millis();

    /*
     * Steering control
     */
    if (next_time < current_time){
      next_time = current_time + 500;
     
      if (PS4.getAnalogHat(LeftHatX) > 137){
         steeringPos += 22;
         steeringServo.write(steeringPos);
         Serial.print(F("\r\nright"));
      }
      if (PS4.getAnalogHat(LeftHatX) < 117){
          steeringPos -= 22;
          steeringServo.write(steeringPos);
          Serial.print(F("\r\nleft"));
      }
    }
    
    /*
     * Thrust Control
     */
    if (next_time01 < current_time){
      next_time01 = current_time + 500;

      if (PS4.getAnalogButton(R2) != oldR2Value){
        Serial.print(F("\r\nChanging speed"));
        oldR2Value = PS4.getAnalogButton(R2);
        leftThrust.setSpeed(oldR2Value);
        rightThrust.setSpeed(oldR2Value);
      }
    }
 }  
  usb.Task();

  if (PS4.connected()){
    delay(100);
    if (PS4.getAnalogHat(LeftHatX) > 137){
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(PS4.getAnalogHat(LeftHatX));
      steeringPos += 1;
      steeringServo.write(steeringPos);
    }
    if (PS4.getAnalogHat(LeftHatX) < 117){
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(PS4.getAnalogHat(LeftHatX));
      steeringPos -= 1;
      steeringServo.write(steeringPos);
    }
    if (PS4.getAnalogHat(LeftHatX) >= 117 && PS4.getAnalogHat(LeftHatX) <= 137 && steeringPos != 90){
      Serial.print(F("\r\nResetting Steering"));
      steeringPos = 0;
      steeringServo.write(steeringPos);
    }

    if (PS4.getAnalogButton(R2) && PS4.getAnalogButton(R2) != oldR2Value) { // These are the only analog buttons on the PS4 controller
      Serial.print(F("\r\nR2: "));
      Serial.print(PS4.getAnalogButton(R2));
      PS4.setRumbleOn(RumbleLow);
      oldR2Value = PS4.getAnalogButton(R2);
      leftThrust.setSpeed(oldR2Value);
      rightThrust.setSpeed(oldR2Value);
    }
    if (PS4.getAnalogButton(R2) == 0){
      oldR2Value = 0;
      leftThrust.setSpeed(oldR2Value);
      rightThrust.setSpeed(oldR2Value);
    }
    leftThrust.run(FORWARD);
    rightThrust.run(FORWARD);

    //disconnect controller when PS button is pressed
    if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      Serial.print(F("\r\nGoodbye"));
      PS4.disconnect();
    }
    else {
      //payload delivery
      if (PS4.getButtonClick(CROSS)) {
        Serial.print(F("\r\nCross"));
        Serial.print(F("\r\nDropping payload"));
        payloadPos += drop;
        payloadServo.write(payloadPos);
      }
      //reset payload postion to starting
      if (PS4.getButtonClick(SQUARE)) {
        Serial.print(F("\r\nSquare"));
        Serial.print(F("\r\nZeroing payload system"));
      }
      //change LED colors... just for fun
      if (PS4.getButtonClick(UP)) {
        Serial.print(F("\r\nUp"));
        PS4.setLed(Red);
      } if (PS4.getButtonClick(RIGHT)) {
        Serial.print(F("\r\nRight"));
        PS4.setLed(Blue);
      } if (PS4.getButtonClick(DOWN)) {
        Serial.print(F("\r\nDown"));
        PS4.setLed(Yellow);
      } if (PS4.getButtonClick(LEFT)) {
        Serial.print(F("\r\nLeft"));
        PS4.setLed(Green);
      }
      //release motors and then run in reverse at half of original speed, this will brake
      if (PS4.getButtonClick(R1)){
        Serial.print(F("\r\nR1"));
        leftThrust.run(RELEASE);
        rightThrust.run(RELEASE);
        leftThrust.run(BACKWARD);
        rightThrust.run(BACKWARD);
      }
    }
  }  

}
