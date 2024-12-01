#include <Servo.h>
#include <SoftwareSerial.h>

// Constants for buffer size
const byte Num_Char = 64; // Increased size to handle multiple commands
char Receive_Com[Num_Char]; // Buffer to store received commands

// Sensor Arduino Serial Communication Pins
int Ard_RX_Pin = 0;
int Ard_TX_Pin = 1;

// DC Motor Driver Control Pins
int DCM1_ENB_Pin = 11;
int DCM1_IN4_Pin = 7;
int DCM1_IN3_Pin = 6;
int DCM2_IN2_Pin = 5;
int DCM2_IN1_Pin = 4;
int DCM2_ENA_Pin = 3;

// Servo Motor Control Pin
Servo servo;
int Servo_Control_Pin = 10;

// Motor Speed
int DCM1_Speed = 0;
int DCM2_Speed = 0;

// Software Serial
SoftwareSerial ArdSerial(Ard_RX_Pin, Ard_TX_Pin);

void setup() {

  // I/O assignment
  pinMode(DCM1_ENB_Pin, OUTPUT);
  pinMode(DCM1_IN4_Pin, OUTPUT);
  pinMode(DCM1_IN3_Pin, OUTPUT);
  pinMode(DCM2_IN2_Pin, OUTPUT);
  pinMode(DCM2_IN1_Pin, OUTPUT);
  pinMode(DCM2_ENA_Pin, OUTPUT);

  // Initialize servo to 0 deg
  servo.attach(Servo_Control_Pin);
  servo.write(0);

  Serial.begin(9600); // Start serial communication with sensor Arduino
}

void loop() {
  if (Receive_Data()) {
    //Serial.println(Receive_Com);
    Process_Com(Receive_Com);
  }
}

// Receive data from Sensor Arduino
bool Receive_Data() {
  static byte index = 0;
  char data;
  bool New_Data = false;

  while (Serial.available() > 0 && !New_Data) {
    data = Serial.read();

    // Skip the starting bracket '['
    if (data == '[' && index == 0) {
      continue; // Ignore the starting bracket
    }

    // Store data until ']' is encountered or buffer is full
    if (data != '\n' && data != '\r') {
      if (data != ']' && index < Num_Char - 1) {
        Receive_Com[index++] = data;
      } else if (data == ']') {
        Receive_Com[index] = '\0'; // Null-terminate the string
        New_Data = true; // Mark data as ready to process
        index = 0; // Reset the index for the next command
      }
    }
  }
  return New_Data;
}

// Process command based on python instruction
void Process_Com(char* Com) {
  char* segment = strtok(Com, ",");
  while (segment != NULL) {
    // Process motor commands (e.g., "M1:100")
    int motorNumber, value;
    if (sscanf(segment, "M%d:%d", &motorNumber, &value) == 2) {
      DCM_On(motorNumber, value);
    }
    // Process servo commands (e.g., "S:45")
    else if (segment[0] == 'S') {
      int angle;
      if (sscanf(segment, "S:%d", &angle) == 1) {
        Servo_Pos(angle);
      }
    }

    segment = strtok(NULL, ","); // Get next command
  }
}

// Turn motors on
void DCM_On(int DCM, int SPEED) {
  if (DCM == 1) {
    digitalWrite(DCM1_IN3_Pin, SPEED >= 0 ? LOW : HIGH);
    digitalWrite(DCM1_IN4_Pin, SPEED >= 0 ? HIGH : LOW);
    analogWrite(DCM1_ENB_Pin, abs(SPEED));
  } else if (DCM == 2) {
    digitalWrite(DCM2_IN1_Pin, SPEED >= 0 ? LOW : HIGH);
    digitalWrite(DCM2_IN2_Pin, SPEED >= 0 ? HIGH : LOW);
    analogWrite(DCM2_ENA_Pin, abs(SPEED));
  }
}

// Actuate servo to desired angle based on given command
void Servo_Pos(int targetAngle) {
  int currentAngle = servo.read();
  if (targetAngle > currentAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      servo.write(pos);
      delay(15); // Adjust delay for smooth motion
    }
  } else {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      servo.write(pos);
      delay(15);
    }
  }
}
