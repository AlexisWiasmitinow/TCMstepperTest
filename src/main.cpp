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
// #include <Streaming.h>
#include <SpeedyStepper.h>
#include <TMCStepper.h>

#define MAX_SPEED 40 // In timer value
#define MIN_SPEED 1000

#define STALL_VALUE 1 // [0..255]

#define EN_PIN 15           // Enable
#define DIR_PIN 19          // Direction
#define STEP_PIN 21         // Step
#define RXD2 18             // TMC2208/TMC2224 SoftwareSerial receive pin
#define TXD2 16             // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define DIAG 4
#define stepDelay 500

#define R_SENSE 0.1f
SoftwareSerial softSerial(RXD2, TXD2);
// TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver(&softSerial, R_SENSE, DRIVER_ADDRESS);
SpeedyStepper stepper;

void setup() {

    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(DIAG, INPUT);
    digitalWrite(DIR_PIN, 0);
    // digitalWrite(EN_PIN, 0);
    stepper.connectToPins(STEP_PIN, DIR_PIN);
    // Serial.begin(250000); // Init serial port and set baudrate
    Serial.begin(115200); // Init serial port and set baudrate

    // SERIAL_PORT.begin(115200);
    // Serial2.begin(115200);
    softSerial.begin(115200);
    //

    while (!Serial)
        ; // Wait for serial port to connect
    driver.begin();
    delay(1000);
    Serial.println("\nStart...");

    // driver.beginSerial(115200);

    driver.toff(5);
    driver.blank_time(24);
    driver.rms_current(2400); // mA
    // driver.rms_current(800); // mA
    driver.microsteps(4);
    driver.SGTHRS(STALL_VALUE);
    driver.shaft(0);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    driver.intpol(true);

    stepper.setCurrentPositionInSteps(0);                    // Set zero position
    stepper.setSpeedInStepsPerSecond(2000);                  // Set Speed
    stepper.setAccelerationInStepsPerSecondPerSecond(10000); // Set acceleration, smaller value for super smooth direction changing

    delay(500);
    // driver.VACTUAL(2000);
    // driver.VACTUAL(0);
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.semin(0);
    driver.semax(2);
    driver.sedn(0b01);
}
bool runMotor = true;
bool diag = false;
void loop() {
    // driver.moveTo(100);
    // stepper.moveToPositionInSteps(6400);
    if (runMotor) {
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
    }
}
