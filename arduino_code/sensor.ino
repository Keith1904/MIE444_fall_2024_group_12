#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>

#define NUM_SENSORS 7
#define _USE_MATH_DEFINES

#include <LiquidCrystal_I2C.h>

// Initialize the LCD (address 0x27) for a 16x2 display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Add a global variable to track the last command received
char lastReceiveCom[32] = ""; // To store the last command received

// Define custom LCD characters 
byte circleTopLeft[8] = {
  B00111,
  B01100,
  B01000,
  B01000,
  B01000,
  B01000,
  B01100,
  B00111
};

byte circleTopRight[8] = {
  B11100,
  B00110,
  B00010,
  B00010,
  B00010,
  B00010,
  B00110,
  B11100
};

byte arrowRight[8] = {
  B00000,
  B00100,
  B00110,
  B11111,
  B00110,
  B00100,
  B00000,
  B00000
};

byte arrowLeft[8] = { 
  B00000,
  B00100,
  B01100,
  B11111,
  B01100,
  B00100,
  B00000,
  B00000
};

// DC Motor Encoder Outputs
int DCM1_EncA_Pin = 3;
int DCM2_EncA_Pin = 2;
int DCM1_EncB_Pin = 4;
int DCM2_EncB_Pin = 5;

// TOFs Pins
VL53L0X tof0;
VL53L0X tof6;
const uint8_t sensorAddresses[] = {0x30, 0x31};

//FRONT TOP
int XSHUT_PIN0 = 6;
//FRONT BOTTOM
int XSHUT_PIN6 = 10;
//RIGHT BACK
int U1_TRIG_PIN = 7;
int U1_ECHO_PIN = A2;
//BACK
int U2_TRIG_PIN = 8;
int U2_ECHO_PIN = 9;
//LEFT BACK
int U3_TRIG_PIN = A1;
int U3_ECHO_PIN = 11;
//RIGHT FRONT
int U4_TRIG_PIN = 8;
int U4_ECHO_PIN = A0;
//LEFT FRONT
int U5_TRIG_PIN = 8;
int U5_ECHO_PIN = A3; 

// Actuation Arduino Serial Communication Pins
int Ard_RX_Pin = 13;
int Ard_TX_Pin = 12;

// Definition of constants
float V_Sound = 0.0135; //Velocity of sounds in in/us
float Wheel_Dia = 2.55906; //Wheel diameter in inches
int Enc_Res = 11; //Encoder pulses per full revolution
int Gear_Ratio = 34; //DC motor gear ratio
float Dist_Per_Pulse_M1 = 0.012; //Distance travelled per encoder pulse in inches for M1
float Dist_Per_Pulse_M2 = 0.012; //Distance travelled per encoder pulse in inches for M2
float Wheel_Dist = 7.25; //wheel to wheel distance in inches

// Array to store US/DRIVE command
const byte Num_Char = 32; //Size of byte array
char Receive_Com[Num_Char]; //Character array to store commands WITHOUT BRACKETS
float Drive_Val; //Store extracted drive command numerical value
float Rotate_Val; //Store extracted rotate command numerical value
float Servo_Val; //Store numerical angle value for servo

// DC Motor Encoder Counts and Movement Tracking
int DCM1_Enc_Count = 0;
int DCM2_Enc_Count = 0;
int DCM1_Target = 0;
int DCM2_Target = 0;
int DCM1_Dir = 0;
int DCM2_Dir = 0;

SoftwareSerial ArdSerial(Ard_RX_Pin, Ard_TX_Pin); //PINS ON SENSOR THAT SEND TO ACTUATION

void setup() {
  
  ArdSerial.begin(9600); //Start serial communication with actuation arduino
  Serial.begin(9600); //Start serial communication with bluetooth module

  // Define output and input pins
  
  pinMode(DCM1_EncA_Pin, INPUT);
  pinMode(DCM2_EncA_Pin, INPUT);
  pinMode(DCM1_EncB_Pin, INPUT);
  pinMode(DCM2_EncB_Pin, INPUT);

  // initialize encoder interrupts
  attachInterrupt(digitalPinToInterrupt(DCM1_EncA_Pin), DCM1_Enc_Update, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(DCM2_EncA_Pin), DCM2_Enc_Update, CHANGE); 

  // I/O assignment
  pinMode(XSHUT_PIN0, OUTPUT);
  pinMode(XSHUT_PIN6, OUTPUT);
  pinMode(U1_TRIG_PIN, OUTPUT);
  pinMode(U1_ECHO_PIN, INPUT);
  pinMode(U2_TRIG_PIN, OUTPUT);
  pinMode(U2_ECHO_PIN, INPUT);
  pinMode(U3_TRIG_PIN, OUTPUT);
  pinMode(U3_ECHO_PIN, INPUT);
  pinMode(U4_TRIG_PIN, OUTPUT);
  pinMode(U4_ECHO_PIN, INPUT);
  pinMode(U5_TRIG_PIN, OUTPUT);
  pinMode(U5_ECHO_PIN, INPUT);

  // explicitly set all output pins to LOW
  digitalWrite(XSHUT_PIN0, LOW);
  digitalWrite(XSHUT_PIN6, LOW);
  digitalWrite(U1_TRIG_PIN, LOW);
  digitalWrite(U2_TRIG_PIN, LOW);
  digitalWrite(U3_TRIG_PIN, LOW);
  digitalWrite(U4_TRIG_PIN, LOW);
  digitalWrite(U5_TRIG_PIN, LOW);

  //Serial.println("Starting TOF Sensor...");
  
  Wire.begin();
  Wire.setClock(50000);

  digitalWrite(XSHUT_PIN0, HIGH);
  delay(10);  // Allow sensor to power up
  
  if (!tof0.init()) {
    //Serial.println("Failed to detect TOF0 sensor! Check wiring.");
    while (1);
  }
  tof0.setAddress(sensorAddresses[0]);
  tof0.setTimeout(500);
  //Serial.println("TOF0 sensor initialized successfully.");

  digitalWrite(XSHUT_PIN6, HIGH);
  delay(10);  // Allow sensor to power up

  if (!tof6.init()) {
    //Serial.println("Failed to detect TOF6 sensor! Check wiring.");
    while (1);
  }
  tof6.setAddress(sensorAddresses[6]);
  tof6.setTimeout(500);
  //Serial.println("TOF6 sensor initialized successfully.");

  ArdSerial.write("[M1:0,M2:0]");
  Receive_Com[0] = '\0';

  //LCD Setup 
  lcd.begin(16, 2);
  lcd.backlight();
  //lcd.clear();
  lcd.createChar(0, circleTopLeft);
  lcd.createChar(1, circleTopRight);
  lcd.createChar(2, arrowRight);
  lcd.createChar(3, arrowLeft);

}

void loop() {
  if (Receive_Data()) {
    //Serial.println(Receive_Com);
    Com_Type(Receive_Com);
  }
  LCD_UpdateIfNewData();
}

void DCM1_Enc_Update()
{
  // updates DC motor 1 encoder value after interrupt
  if (digitalRead(DCM1_EncA_Pin) == HIGH)
  {
    if (digitalRead(DCM1_EncB_Pin) == LOW)
    {
      DCM1_Enc_Count++;
      DCM1_Target--;
    }
    else
    {
      DCM1_Enc_Count--;
      DCM1_Target++;
    }
  }
  else
  {
    if (digitalRead(DCM1_EncB_Pin) == LOW)
    {
      DCM1_Enc_Count--;
      DCM1_Target++;
    }
    else
    {
      DCM1_Enc_Count++;
      DCM1_Target--;
    }
  }
  // Serial.println(DCM1_Enc_Count);
  // Serial.println(DCM1_Target);
  if (DCM1_Target == 0)
  {
    // Serial.println("M1 STOPPED");
    ArdSerial.write("[M1:0,M2:0]"); // *** Needs to be updated when commands are finalized
  }
}

void DCM2_Enc_Update()
{
  // updates DC motor 2 encoder value after interrupt
  if (digitalRead(DCM2_EncA_Pin) == HIGH)
  {
    if (digitalRead(DCM2_EncB_Pin) == LOW)
    {
      DCM2_Enc_Count++;
      DCM2_Target--;
    }
    else
    {
      DCM2_Enc_Count--;
      DCM2_Target++;
    }
  }
  else
  {
    if (digitalRead(DCM2_EncB_Pin) == LOW)
    {
      DCM2_Enc_Count--;
      DCM2_Target++;
    }
    else
    {
      DCM2_Enc_Count++;
      DCM2_Target--;
    }
  }
  // Serial.println(DCM2_Enc_Count);
  // Serial.println(DCM2_Target);
  if (DCM2_Target == 0)
  {
    // Serial.println("M2 STOPPED");
    ArdSerial.write("[M1:0,M2:0]"); // *** Needs to be updated when commands are finalized
  }
}

// Calculate target pulses for each motor based on distance given
void Drive_Pulse_Target()
{
  DCM1_Target = int(Drive_Val / Dist_Per_Pulse_M1);
  DCM2_Target = int(Drive_Val / Dist_Per_Pulse_M2);
  //Serial.println(DCM1_Target);
  //Serial.println(DCM2_Target);
  if (DCM1_Target > 0)
  {
    DCM1_Dir = 1;
  }
  else
  {
    DCM1_Dir = -1;
  }
  if (DCM2_Target > 0)
  {
    DCM2_Dir = 1;
  }
  else
  {
    DCM2_Dir = -1;
  }
}

// Calculate target pulses for given rotation angle
void Rot_Pulse_Target()
{
  DCM1_Target = -int(((Rotate_Val * M_PI / 180) * Wheel_Dist / 2) / Dist_Per_Pulse_M1);
  DCM2_Target = int(((Rotate_Val * M_PI / 180) * Wheel_Dist / 2) / Dist_Per_Pulse_M2);
}

// Receive python instructions
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

// Send motor actuation instructions to Actuation Arduino
void Send_Com()
{
  // Serial.println("SENDING COM");
  if (DCM1_Target > 0 && DCM2_Target > 0)
  {
    ArdSerial.write("[M1:200,M2:200]");
  }
  else if (DCM1_Target > 0 && DCM2_Target < 0)
  {
    ArdSerial.write("[M1:200,M2:-200]");
  }
  else if (DCM1_Target < 0 && DCM2_Target > 0)
  {
    ArdSerial.write("[M1:-200,M2:200]");
  }
  else if (DCM1_Target < 0 && DCM2_Target < 0)
  {
    ArdSerial.write("[M1:-200,M2:-200]");
  }
}

// Check if command is sensor, drive, or stop
void Com_Type(char* Receive_Com)
{
  if (Receive_Com[0] == 'u' || Receive_Com[0] == 'm')
  {
    Process_Sensor_Com();
    Serial.println("[SENSOR COMMAND RECEIVED]");
  }
  else if (Receive_Com[0] == 'w' || Receive_Com[0] == 'r')
  {
    Process_Drive_Com();
    Serial.println("[DRIVE COMMAND RECEIVED]");
  }
  else if (Receive_Com[0] == 's')
  {
    Process_Servo_Com();
    Serial.println("[SERVO COMMAND RECEIVED]");
  }
  else if (Receive_Com[0] == 'x')
  {
    ArdSerial.write("[M1:0,M2:0]");
    Serial.println("[STOP COMMAND RECEIVED]");
  }
}

// Process drive commands 
void Process_Drive_Com()
{
  char Drive_Com[strlen(Receive_Com) - 3];
  for (int i = 0; i < strlen(Receive_Com) - 3; i++)
  {
    Drive_Com[i] = Receive_Com[i + 3];
  }
  if (Receive_Com[0] == 'w')
  {
    Drive_Val = atof(Drive_Com);
    //Serial.println(Drive_Val);
    Drive_Pulse_Target();
    Send_Com();
  }
  else if (Receive_Com[0] == 'r')
  {
    Rotate_Val = atof(Drive_Com);
    Rot_Pulse_Target();
    Send_Com();
  }
}

// Process servo commands
void Process_Servo_Com() {
  char Drive_Com[strlen(Receive_Com) - 1];  // Adjust to safely hold the target values
  for (int i = 0; i < strlen(Receive_Com) - 2; i++) {
    Drive_Com[i] = Receive_Com[i + 2];
  }
  Drive_Com[strlen(Receive_Com) - 2] = '\0'; // Add a null terminator

  Drive_Val = atof(Drive_Com);  // Convert to a floating-point number
  String servo_string = "[S:" + String(Drive_Val) + "]";
  ArdSerial.write(servo_string.c_str());
}

// Process sensor commands
void Process_Sensor_Com()
{
  String result = "["; // Start result string
  char *token = strtok(Receive_Com, ","); // Split Receive_Com by commas

  while (token != NULL)
  {
    float output = 0; // Reset output for each token
    String tokenStr = String(token);

    if (tokenStr == "u0") {
      output = tof0.readRangeSingleMillimeters() / 25.4 - 1.1;
    } 
    else if (tokenStr == "u1") {
      output = Ultrasonic_Check(U1_TRIG_PIN, U1_ECHO_PIN);
    }
    else if (tokenStr == "u2") {
      output = Ultrasonic_Check(U2_TRIG_PIN, U2_ECHO_PIN);
    }
    else if (tokenStr == "u3") {
      output = Ultrasonic_Check(U3_TRIG_PIN, U3_ECHO_PIN);
    }   
    else if (tokenStr == "u4") {
      output = Ultrasonic_Check(U4_TRIG_PIN, U4_ECHO_PIN);
    }
    else if (tokenStr == "u5") {
      output = Ultrasonic_Check(U5_TRIG_PIN, U5_ECHO_PIN);
    }  
    else if (tokenStr == "u6") {
      output = tof6.readRangeSingleMillimeters() / 25.4 - 1;
    } 
    else if (tokenStr == "m0") {
      output = M2_Check();
    } 
    else if (tokenStr == "m1") {
      output = M1_Check();
    }

    // Append to result in proper format
    result += tokenStr + ":" + String(output, 2) + ",";

    token = strtok(NULL, ","); // Move to the next token
  }

  // Remove trailing comma and close the result
  if (result.endsWith(",")) {
    result.remove(result.length() - 1);
  }
  result += "]";

  // Output result
  Serial.println(result);
}

// Get ultrasonic sensor distance
float Ultrasonic_Check(int TRIG_PIN, int ECHO_PIN) {
  float distance = 0.0;

  // Retry until a non-zero distance is obtained
  while (distance < 1.0) {
    // Trigger the ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read echo and convert to distance
    distance = pulseIn(ECHO_PIN, HIGH) / 2 * 0.0135;
  }

  return distance;
}

// M1 distance travelled
float M1_Check()
{
  return DCM1_Enc_Count * Dist_Per_Pulse_M1;
}

// M2 distance travelled
float M2_Check()
{
  return DCM2_Enc_Count * Dist_Per_Pulse_M2;
}

// Function to handle LCD display logic based on Receive_Com
void LCD_Display() {
  lcd.clear();

  // Check if the command is for "r" (arrow display)
  if (Receive_Com[0] == 'r') {
    if (Receive_Com[3] == '-') {
      // Display arrow bending right (this is ccw)
      
      // for (int i=0 ; i<15 ; i=i+3){
      //   lcd.setCursor(i, 0); // Set cursor to the first character of the top row for the circle
      //   lcd.write(byte(0));  // Top-left of the circle
      //   lcd.write(byte(2));  // Arrow right
      //   lcd.write(byte(1));  // Bottom-right of the circle
      //   delay(200);   
      //   }
      lcd.setCursor(6, 0); // Set cursor to the first character of the top row for the circle
      lcd.write(byte(0));  // Top-left of the circle
      lcd.write(byte(2));  // Arrow right
      lcd.write(byte(1));  // Bottom-right of the circle
      delay(200); 

      
    } else {
      // Display arrow bending left (this is cw)

      // for (int i=16 ; i>=0 ; i=i-3){
      //   lcd.setCursor(i, 0); // Set cursor to the first character of the top row for the circle
      //   lcd.write(byte(0));  // Top-left of the circle
      //   lcd.write(byte(3));  // Arrow left
      //   lcd.write(byte(1));  // Bottom-right of the circle
      //   delay(200);   
      //   }
      lcd.setCursor(6, 0); // Set cursor to the first character of the top row for the circle
      lcd.write(byte(0));  // Top-left of the circle
      lcd.write(byte(3));  // Arrow left
      lcd.write(byte(1));  // Bottom-right of the circle
      delay(200);   

    }
  }
  // Check if the command is for "w" (display specific patterns)
  else if (Receive_Com[0] == 'w') {
    if (Receive_Com[3] != '-') {
      // goes forward, dispay arrows going forward

      lcd.setCursor(0, 0);   // Move cursor to the next block in the first row
      lcd.print(">>>>>>>");        // Print a character (e.g., "*") to "light up" the block
      delay(200);            // Adjust delay for speed (200ms between each block)
      
    //   for (int i = 0; i < 16; i++) {
    //     lcd.setCursor(i, 0);   // Move cursor to the next block in the first row
    //     lcd.print(">");        // Print a character (e.g., "*") to "light up" the block
    //     delay(200);            // Adjust delay for speed (200ms between each block)
        
    // }
    } else {
      // goes backward, display arrows going back
      lcd.setCursor(0, 0);   // Move cursor to the next block in the first row
      lcd.print("<<<<<<<");        // Print a character (e.g., "*") to "light up" the block
      delay(200);            // Adjust delay for speed (200ms between each block)


    //   for (int i = 16; i >= 0; i--) {
    //     lcd.setCursor(i, 0);   // Move cursor to the next block in the first row
    //     lcd.print("<");        // Print a character (e.g., "*") to "light up" the block
    //     delay(200);            // Adjust delay for speed (200ms between each block)
        
    // }

    }
  }  else {
    
    if (Receive_Com[0] == 'L') {
      lcd.print("Arrived at Loading Zone!");
    }

    else if (Receive_Com[0] == 'D') {
      lcd.print("Arrived at Delivery Zone!");
    }
  }
 }

// Updates LCD if new data is sent
void LCD_UpdateIfNewData() {
  // Check if the current command is different from the last one
  if (strcmp(Receive_Com, lastReceiveCom) != 0) {
    // Update lastReceiveCom to the current command
    strcpy(lastReceiveCom, Receive_Com);
    // Call LCD_Display function only if there is new data
    LCD_Display();
  }
} 