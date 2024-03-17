#include <Arduino.h>
/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2130 or TMC2660 motor load using the stallGuard value.
 * You can finetune the reading by changing the STALL_VALUE.
 * This will let you control at which load the value will read 0
 * and the stall flag will be triggered. This will also set pin DIAG1 high.
 * A higher STALL_VALUE will make the reading less sensitive and
 * a lower STALL_VALUE will make it more sensitive.
 *
 * You can control the rotation speed with
 * 0 Stop
 * 1 Resume
 * + Speed up
 * - Slow down
 */
#include <SoftwareSerial.h>
#include <SpeedyStepper.h>
#include <TMCStepper.h>

#define MAX_SPEED 40  // In timer value
#define MIN_SPEED 1000

#define STALL_VALUE 10  // [0..255]

#define EN_PIN 32    // Enable
#define DIR_PIN 17   // Direction
#define STEP_PIN 27  // Step
#define RXD2 17      // TMC2208/TMC2224 SoftwareSerial receive pin
#define TXD2 12      // TMC2208/TMC2224 SoftwareSerial transmit pin
#define DRIVER_ADDRESS 0b00
#define DIAG 13
#define R_SENSE 0.1f

#define stepDelay 50

using namespace TMC2208_n;

SoftwareSerial softSerial(RXD2, TXD2);
TMC2209Stepper driver(&softSerial, R_SENSE, DRIVER_ADDRESS);
SpeedyStepper stepper;

void testTMC2209() {
  Serial.printf("current: %d\n", driver.rms_current());
  Serial.printf("microsteps: %d\n", driver.microsteps());
  Serial.printf("shaft: %d\n", driver.shaft());
  driver.microsteps(4);
  driver.rms_current(300);
  driver.shaft(true);
  driver.push();
  delay(1000);
  Serial.printf("current: %d\n", driver.rms_current());
  Serial.printf("microsteps: %d\n", driver.microsteps());
  Serial.printf("shaft: %d\n", driver.shaft());
  driver.microsteps(2);
  driver.rms_current(600);
  driver.shaft(false);
  driver.push();
  delay(1000);
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIAG, INPUT);
  digitalWrite(DIR_PIN, 0);
  digitalWrite(EN_PIN, 0);
  stepper.connectToPins(STEP_PIN, RXD2);
  Serial.begin(115200);  // Init serial port and set baudrate
  softSerial.begin(57600);
  // softSerial.begin(115200);

  //
  delay(1000);

  Serial.println("\nSetup...");

  driver.begin();
  // delay(1000);

  // driver.beginSerial(115200);

  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(2400);
  // driver.rms_current(400); // mA
  driver.microsteps(1);

  driver.TCOOLTHRS(1048575);  // 20bit max

  // driver.semin(0);
  // driver.semax(2);
  // driver.sedn(1);
  driver.pwm_autoscale(false);
  driver.intpol(true);
  driver.SGTHRS(STALL_VALUE);
  driver.en_spreadCycle(false);  // enables stealthchop and thus also stallguard

  bool clockwise = true;
  int dir = clockwise;
  driver.shaft(dir);
  // driver.TCOOLTHRS(STALL_VALUE); another stallguard threashold...
  // https://www.youtube.com/watch?v=J3U3e2WiPLg
  // https://valarsystems.com/blogs/val-2000/section-7-stallguard

  stepper.setCurrentPositionInSteps(0);    // Set zero position
  stepper.setSpeedInStepsPerSecond(2000);  // Set Speed
  stepper.setAccelerationInStepsPerSecondPerSecond(10000);
  Serial.println("\nStart...");
}
bool runMotor = true;
bool startMotor = true;
bool diag = false;
int loopCounter = 0;
void loop() {
  testTMC2209();
  // driver.moveTo(100);
  // stepper.moveToPositionInSteps(6400);
  /*if (runMotor) {
    digitalWrite(STEP_PIN, 1);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, 0);
    // delay(500);
    diag = digitalRead(DIAG);
    if (diag) {
      runMotor = false;
    }
    Serial.print("diag: ");
    Serial.println(diag);

    delayMicroseconds(stepDelay);
  } else if (loopCounter < 10) {
    digitalWrite(STEP_PIN, 1);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, 0);
    delayMicroseconds(stepDelay);
    startMotor = false;
    loopCounter++;
  }*/
}
