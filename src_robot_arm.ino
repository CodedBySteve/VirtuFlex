#include <Servo.h>
#include <Stepper.h>
#include <DFRobot_BMI160.h>
#define BMI160_ACCEL_RANGE_2G
#define BMI160_GYRO_RANGE_250_DPS
#define BMI160_ACCEL_BW_OSR4_AVG1
//#define BMI160_ACCEL_BW_RES_AVG128
// ----- Joystick
Servo Elbow; 
Servo ShoulderRight;
Servo ShoulderLeft;
int joystick_X = A3;
int joystick_Y = A6;
float RightStep_Count = 60;  // Initial right shoulder position
float LeftStep_Count = 52;   // Initial left shoulder position
float ElbowStep_Count = 70;
int threshold_Forward = 900;
int threshold_Backward = 200;
// ----- Accelerometer
DFRobot_BMI160 BMI;
Servo Wrist_UPDOWN;
Servo Wrist_Rota;
const int8_t i2c_addr = 0x69;
int threshold_P = 200;
int threshold_N = -200;
int deadband = 2.5;
int Last_Wrist_Rota_Pos = 0;
int LastWrist_UPDOWN_Pos = 0;

// ------ Stepper 
const int stepsPerRevolution = 200;
int Flex_thumb = A0;
int Flex_middle = A2;
int threshold_thumb = 705;
int threshold_middle = 730;
Stepper Base(stepsPerRevolution, 2, 3, 4, 5); // Changed pins for the stepper motor

// ------ Servo
Servo Claw;
float Index_pos = 0;
float current_Index_pos = 0;
int Flex_Index = A1;
int deadband_Index = 2;

void setup() {
  Serial.begin(9600);
  Claw.attach(6); // Changed pin for Claw servo
  Wrist_Rota.attach(7); // Changed pin for Wrist_Rota servo
  Wrist_UPDOWN.attach(8); // Changed pin for Wrist_UPDOWN servo
  Elbow.attach(9);
  ShoulderLeft.attach(10);
  ShoulderRight.attach(11);
  Base.setSpeed(70);
  // Initial position
  ShoulderRight.write(60);
  ShoulderLeft.write(52);
  delay(1000);
  Elbow.write(70);
  // Initialize the BMI160 sensor
  if (BMI.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("Initialization failed");
    while(1);
  }
  delay(3000);
}

void loop() {
  // **** Accelerometer Variables
  int16_t data[6] = {0};
  int result = BMI.getAccelGyroData(data);
  float Ax = data[3] / 16384.0;
  float Ay = data[4] / 16384.0;
  float Az = data[5] / 16384.0;

  // ---- Wrist UPDOWN Code -----
  float tiltAngleUPDOWN = atan2(Ax, Az) * 180 / PI;

  // Map the tilt angle to servo position
  int Wrist_UPDOWN_Pos = map(tiltAngleUPDOWN, 50, -50, 0, 170);
  Serial.println(Wrist_UPDOWN_Pos);

  // Apply deadband to avoid twitching
  if (abs(Wrist_UPDOWN_Pos - LastWrist_UPDOWN_Pos) > deadband) {
    if (Wrist_UPDOWN_Pos <= 0) {
      Wrist_UPDOWN.write(0);
      LastWrist_UPDOWN_Pos = 0;
    } else if (Wrist_UPDOWN_Pos >= 170) {
      Wrist_UPDOWN.write(170);
      LastWrist_UPDOWN_Pos = 170;
    } else {
      Wrist_UPDOWN.write(Wrist_UPDOWN_Pos);
      LastWrist_UPDOWN_Pos = Wrist_UPDOWN_Pos;
    }
  }

  // ---- Wrist rota Code -----
  float tiltAngle_Rota = atan2(Ay, Az) * 180 / PI;

  // Map the tilt angle to servo position
  int Wrist_Rota_Pos = map(tiltAngle_Rota, 90, -90, 0, 210);

  // Apply deadband to avoid twitching
  if (abs(Wrist_Rota_Pos - Last_Wrist_Rota_Pos) > deadband) {
    if (Wrist_Rota_Pos <= 0) {
      Wrist_Rota.write(0);
      Last_Wrist_Rota_Pos = 0;
    } else if (Wrist_Rota_Pos >= 190) {
      Wrist_Rota.write(190);
      Last_Wrist_Rota_Pos = 190;
    } else {
      Wrist_Rota.write(Wrist_Rota_Pos);
      Last_Wrist_Rota_Pos = Wrist_Rota_Pos;
    }
  }

  // Serial.print("Tilt Angle: ");
  // Serial.println(tiltAngle_Rota);
  // Serial.print("Servo Pos: ");
  // Serial.println(Wrist_Rota_Pos);

  // ---- Flex Code ------
  // Index Finger Code 
  int Index = analogRead(Flex_Index);
  int mapping = map(Index, 666, 800, 130, 0);

  if (mapping < 0) {
    Claw.write(0);
    mapping = 0;
  } else if (mapping >= 130) {
    Claw.write(130);
    mapping = 130;
  } else {
    if (abs(mapping - current_Index_pos) > deadband_Index) {
      Claw.write(mapping);
      current_Index_pos = mapping;
    }
    Claw.write(mapping);
  }
  current_Index_pos = mapping;

  // ------- Stepper motor Code
// const int stepsPerRevolution = 200;
// int Flex_thumb = A0;
// int Flex_middle = A2;
// int threshold_thumb = 705;
// int threshold_middle = 730;
//   if (Flex_thumb >= threshold_thumb);
//   Serial.println("Clockwise");
//   Stepper1.step(stepsPerRevolution);
// } else if (Flex_middle >= threshold_middle) {
//   Serial.println("anticlockwise");
//   Stepper1.step(-stepsPerRevolution);
// } else if (Flex_thumb >= threshold_thumb && Flex_middle >= threshold_middle){
//   Serial.println("No movement");

   // ------- Joystick Code

   int read_X = analogRead(joystick_X);
   int read_Y = analogRead(joystick_Y);
  //  Serial.println(read_Y);
  //  Serial.println(read_X);
   // *** Elbow ***
  // Elbow Forward Movement
  if (read_Y >= threshold_Forward) {
    ElbowStep_Count -= 1;

    // Constrain values within valid range
    ElbowStep_Count = constrain(ElbowStep_Count, 0, 70);

    Elbow.write(ElbowStep_Count);

    if (ElbowStep_Count <= 0){
      Serial.println("Elbow MAX reached limit");
    }
  }
  // Elbow Backward Movement
  if (read_Y <= threshold_Backward) {
    ElbowStep_Count += 1;

    // Constrain values within valid range
    ElbowStep_Count = constrain(ElbowStep_Count, 0, 70);

    Elbow.write(ElbowStep_Count);

    if (ElbowStep_Count >= 70){
      Serial.println("Elbow MIN reached limit");
    }
  }

   // **** SHOULDERS CODE *****
   // Forward Movement
  if (read_X >= threshold_Forward) {
    RightStep_Count -= 1;
    LeftStep_Count += 1;
    
    // Constrain values within valid range
    RightStep_Count = constrain(RightStep_Count, 60, 93);
    LeftStep_Count = constrain(LeftStep_Count, 20, 52);

    ShoulderRight.write(RightStep_Count);
    ShoulderLeft.write(LeftStep_Count);

    if (RightStep_Count <= 60) {
      Serial.println("Right MAX reached limit");
    }
    if (LeftStep_Count >= 52) {
      Serial.println("Left MAX reached limit");
    }
  }

  // Backward Movement  
  if (read_X <= threshold_Backward) {
    RightStep_Count += 1;
    LeftStep_Count -= 1;
    
    // Constrain values within valid range
    RightStep_Count = constrain(RightStep_Count, 60, 93);
    LeftStep_Count = constrain(LeftStep_Count, 20, 52);

    ShoulderRight.write(RightStep_Count);
    ShoulderLeft.write(LeftStep_Count);

    if (RightStep_Count >= 93) {
      Serial.println("Right MIN reached limit");
    }
    if (LeftStep_Count <= 20) {
      Serial.println("Left MIN reached limit");
    }
  //  int ShoulderRight_Pos = map(read_X, 1023, 519, 93, 60); *This is so I don't forget the ranges*
  //  int ShoulderLeft_Pos = map(read_X, 1023, 519, 20, 52); *This is so I don't forget the ranges*
  }
}



