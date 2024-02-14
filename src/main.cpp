#include <Arduino.h>
#include <TMCStepper.h>
#include <SpeedyStepper.h> 

#define DIR_PIN          2    // Direction
#define STEP_PIN         4    // Step
#define SERIAL_PORT Serial2   // HardwareSerial port 
#define DRIVER_ADDRESS 0b00   // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11          // Match to your driver

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
SpeedyStepper stepper;

bool shaft = false;  // ONLY NEEDED FOR CHANGING DIRECTION VIA UART, NO NEED FOR DIR PIN FOR THIS
uint16_t msread;
uint16_t rms_current;
uint16_t CurrentPosition;

void setup() {
  stepper.connectToPins(STEP_PIN, DIR_PIN); // INITIALIZE SpeedyStepper

  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209
  Serial.begin(115200);
  delay(500);
  Serial.println(F("Serial Initialized"));

  driver.begin();                 // Initialize driver
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(300);        // Set motor RMS current
  driver.microsteps(16);          // Set microsteps to 1/2
  driver.pwm_autoscale(true);     // Needed for stealthChop
  // driver.en_spreadCycle(true); // Toggle spreadCycle for smooth & silent operation

  stepper.setCurrentPositionInSteps(0);                   // Set zero position
  stepper.setSpeedInStepsPerSecond(400);                  //Set Speed
  stepper.setAccelerationInStepsPerSecondPerSecond(400);  //Set acceleration, smaller value for super smooth direction changing
}

void loop() {

  stepper.moveRelativeInSteps(1600);
  delay(2000);  

  msread = driver.microsteps();
  rms_current = driver.rms_current();
  //CurrentPosition = stepper.moveToPositionInSteps();

  msread = driver.microsteps();
  Serial.print(F("Read microsteps via UART to test UART receive : "));      // Test if UART is working
  Serial.println(msread);                                                   // If working, sends microstep value, otherwise, default is 256
  Serial.print("Check position: ");                       
  Serial.println(CurrentPosition);


  Serial.println(F("Move 6400 steps forward at 600ma"));
  stepper.moveToPositionInSteps(6400);
  driver.rms_current(600);
  rms_current = driver.rms_current();                                     // Increase current and test
  Serial.print(F("Read current current high: "));
  Serial.println(rms_current);

  driver.microsteps(4);           
  msread = driver.microsteps();
  Serial.print(F("Read microsteps: "));
  Serial.println(msread);
 
  shaft = !shaft;               // REVERSE DIRECTION
  driver.shaft(shaft);          // SET DIRECTION
  delay(50);
  Serial.print("Shaft direction: ");
  Serial.println(shaft);

  driver.rms_current(500);
  rms_current = driver.rms_current();
  Serial.print(F("Read current current slow: "));
  Serial.println(rms_current);
}