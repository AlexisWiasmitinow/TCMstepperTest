#include <TMCStepper.h>
//#include <SpeedyStepper.h> //Simple & good stepper library, get it.
#include <SpeedyStepperNay.h>       // Edited the library to accomodate expander I2C communication
#include "PCF8575.h"
#include <Arduino.h>

//#define digitalWrite(uint8_t pin, uint8_t val) nativeDigitalWrite(uint8_t pin, uint8_t val);        // Duplicate digitalWrite function
#define DIR_PIN          2 // Direction
//#define STEP_PIN         4 // Step
#define STEP_PIN          P5 // Step
#define SERIAL_PORT Serial2 // HardwareSerial port 
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11        // Match to your driver
#define STALL_VALUE 8      // Stall value
#define TestLED 18          // LED test
#define DIAGpin 23          // Stall detection Pin
#define ENN 19              // Enable mottor pin

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
SpeedyStepper stepper;
PCF8575 Pcf8575(0x20);

bool shaft = false;  // ONLY NEEDED FOR CHANGING DIRECTION VIA UART, NO NEED FOR DIR PIN FOR THIS
bool shaftDirection;
bool MotorRun = true;       // Start motor rotation
uint16_t msread;
uint16_t rms_current;
uint16_t CurrentDirection;
uint16_t StallValue;
int nativeDigitalRead(u_int8_t pin);
int digitalRead(u_int8_t pin);
void nativeDigitalWrite(uint8_t pin, uint8_t val);
void digitalWrite(uint8_t pin, uint8_t val);
void DiagInputSetup();
void IRAM_ATTR Diag_ISR();

// Override the digitalWrite and digitalRead functions for stepper and multiplexer compatability
/*
int nativeDigitalRead(u_int8_t pin) {
  u_int8_t theValue = ::digitalRead(pin);
  return theValue;
}

int digitalRead(u_int8_t pin) {
  u_int8_t theValue = pcf8575.digitalRead(pin);
  return theValue;
}

void nativeDigitalWrite(uint8_t pin, uint8_t val) {
  ::digitalWrite(pin, val);
}

void digitalWrite(uint8_t pin, uint8_t val) {
    pcf8575.digitalWrite(pin, val);
}
*/

void DiagInputSetup() {
  attachInterrupt(digitalPinToInterrupt(DIAGpin), Diag_ISR,
                  HIGH);  // Enable diag pin for interrupt
}

void IRAM_ATTR Diag_ISR() {  
  delayMicroseconds(50);                 // Debouncer

  if (!(DIAGpin == LOW)) {
    digitalWrite(TestLED, 1);
    Serial.println("Stall detected!");
    digitalWrite(ENN, 1);                 // Reset the driver error condition
    delayMicroseconds(1000);
    digitalWrite(ENN, 0);                 // Enable driver
    MotorRun = false;                     // Change motor rotation state
  }
  else {
    Serial.println("False Alarm");
    digitalWrite(ENN, 1);                 // Reset the driver error condition
    delayMicroseconds(1000);
    digitalWrite(ENN, 0);                 // Enable driver
  }         
}

void setup() {
  //pinMode(DIR_PIN, OUTPUT);
  //pinMode(STEP_PIN,OUTPUT);
  //delay(5000);                      // Let driver stabilize
  //pinMode(15,OUTPUT);
  //pcf8575.pinMode(DIR_PIN, OUTPUT);
  //Pcf8575.pinMode(STEP_PIN, OUTPUT);
  //Pcf8575.pinMode(P5, OUTPUT);
  pinMode(TestLED, OUTPUT);
  pinMode(ENN, OUTPUT);
  digitalWrite(ENN, 0);       // Enable driver
  pinMode(DIAGpin, INPUT_PULLDOWN);
  stepper.connectToPins(STEP_PIN, DIR_PIN); // INITIALIZE SpeedyStepper
  //stepper.connectToPins((pcf8575.STEP_PIN), DIR_PIN); // INITIALIZE SpeedyStepper

  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209
  Serial.begin(115200);
  Pcf8575.begin();                // Initialize I2C communication
  delay(500);
  Serial.println(F("Serial Initialized"));


  driver.pdn_disable(true);         // Enable driver UART
  driver.begin();                   // Initialize driver  
  driver.toff(5);                   // Enables driver in software
  driver.blank_time(24);            // Comparator blank time. This time needs to safely cover the switching
  driver.rms_current(300);          // Set motor RMS current
  driver.microsteps(1);             // Set microsteps to 1
  driver.pwm_autoscale(true);       // Needed for stealthChop
  driver.en_spreadCycle(false);     // Toggle spreadCycle for smooth & silent operation and enable stallgaurd 
  driver.TCOOLTHRS(0xFFFFF);        // Prevents from switching to spreadcycle when exceeding this velocity, hence max speed  
  driver.SGTHRS(STALL_VALUE);       // The stall output becomes active if SG_RESULT fall below this 2x value.     

  DiagInputSetup();                 // Initialize interrupt setup
  //MotorRun = true;                  // Setup motor run
  stepper.setCurrentPositionInSteps(0);                   // Set zero position
  stepper.setSpeedInStepsPerSecond(400);                  // Set Speed
  stepper.setAccelerationInStepsPerSecondPerSecond(400);  // Set acceleration, smaller value for super smooth direction changing
}

void loop() {

  if (MotorRun == true) {
    stepper.moveRelativeInSteps(800);
    //stepper.moveToPositionInSteps(1600);
    delay(2000);
    

    msread = driver.microsteps();
    rms_current = driver.rms_current();
    //CurrentPosition = stepper.moveToPositionInSteps();


    driver.microsteps(2);           
    msread = driver.microsteps();
    Serial.print(F("Read microsteps: "));
    Serial.println(msread);
  
    shaft = !shaft; // REVERSE DIRECTION
    driver.shaft(shaft); // SET DIRECTION
    shaftDirection = driver.shaft();
    Serial.print("Shaft direction: ");
    Serial.println(shaftDirection);
    //digitalWrite(TestLED, shaft);
    digitalWrite(TestLED, 0);

    driver.rms_current(500);
    rms_current = driver.rms_current();
    Serial.print(F("Read current slow: "));
    Serial.println(rms_current);

    Serial.print(F("Stall Guard value: "));
    Serial.println(driver.SG_RESULT());

    Serial.print("Threshold :");
    Serial.println(driver.SGTHRS());

    digitalWrite(TestLED, HIGH);
    delay(1000);
    digitalWrite(TestLED, LOW);

    /*nativeDigitalWrite(TestLED, HIGH);
    delay(1000);
    nativeDigitalWrite(TestLED, LOW);*/

  }
  else {
    stepper.moveRelativeInSteps(0);       // Stop motor
    Serial.println("Motor driver rotation stopped!");
    Serial.print("Stall detection current position: ");
    //Serial.println(stepper.getCurrentPositionInSteps());            //Get current position is steps
    Serial.println(stepper.getCurrentPositionInMillimeters());        // Get cuurent position in millimeters
    Serial.print("Shaft direction: ");
    Serial.println(shaftDirection);
    delay(2000);
  }  
}