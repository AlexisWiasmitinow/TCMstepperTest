#include <TMCStepper.h>
#include <SpeedyStepper.h> 

#define DIR_PIN          2 // Direction
#define STEP_PIN         4 // Step
#define SERIAL_PORT Serial2 // HardwareSerial port 
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11        // Match to your driver
#define STALL_VALUE 8      // Stall value
#define TestLED 18          // LED test
#define DIAGpin 23          // Stall detection Pin
#define ENN 22              // Enable mottor pin

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
SpeedyStepper stepper;

bool shaft = false;  // ONLY NEEDED FOR CHANGING DIRECTION VIA UART, NO NEED FOR DIR PIN FOR THIS
bool shaftPosition;
bool MotorRun = true;       // Start motor rotation
uint16_t msread;
uint16_t rms_current;
uint16_t CurrentPosition;
uint16_t StallValue;
void DiagInputSetup();
void IRAM_ATTR Diag_ISR();

void DiagInputSetup() {
  attachInterrupt(digitalPinToInterrupt(DIAGpin), Diag_ISR,
                  HIGH);  // Enable dia pin for interrupt
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
  pinMode(TestLED, OUTPUT);
  pinMode(ENN, OUTPUT);
  digitalWrite(ENN, 0);               // Enable driver
  pinMode(DIAGpin, INPUT_PULLDOWN);
  stepper.connectToPins(STEP_PIN, DIR_PIN); // INITIALIZE SpeedyStepper

  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209
  Serial.begin(115200);
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
    shaftPosition = driver.shaft();
    Serial.print("Shaft direction: ");
    Serial.println(shaftPosition);
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
  }
  else {
    stepper.moveRelativeInSteps(0);       // Stop motor
    Serial.println("Motor driver rotation stopped!");
    Serial.print("Stall detection current position: ");
    //Serial.println(stepper.getCurrentPositionInSteps());            //Get current position is steps
    Serial.println(stepper.getCurrentPositionInMillimeters());        // Get curent position in millimeters
    Serial.print("Shaft direction: ");                                // Get direction in which motor was running
    Serial.println(shaftPosition);
    delay(2000);
  }  
}