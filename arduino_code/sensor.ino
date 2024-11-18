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

// Bluetooth Serial Communication Pins
//int BT_RX_Pin = 0;
//int BT_TX_Pin = 1;

// DC Motor Encoder Outputs
int DCM1_EncA_Pin = 3;
int DCM2_EncA_Pin = 2;
int DCM1_EncB_Pin = 4;
int DCM2_EncB_Pin = 5;

// TOFs Pins
VL53L0X tof0;
VL53L0X tof1;
VL53L0X tof2;
VL53L0X tof3;
VL53L0X tof4;
VL53L0X tof5;
VL53L0X tof6;
const uint8_t sensorAddresses[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36};

int XSHUT_PIN0 = 6;
int XSHUT_PIN1 = 7;
int XSHUT_PIN2 = 8;
int XSHUT_PIN3 = 9;
int XSHUT_PIN4 = A3;
int XSHUT_PIN5 = A1;
int XSHUT_PIN6 = 10; 

// Infrared Output Pin
// int IR_Out_Pin = 11;

// Actuation Arduino Serial Communication Pins
int Ard_RX_Pin = 13;
int Ard_TX_Pin = 12;

// Definition of constants
float V_Sound = 0.0135; //Velocity of sounds in in/us
float Wheel_Dia = 2.55906; //Wheel diameter in inches
int Enc_Res = 11; //Encoder pulses per full revolution
int Gear_Ratio = 34; //DC motor gear ratio
float Dist_Per_Pulse_M1 = 0.015; //Distance travelled per encoder pulse in inches for M1
float Dist_Per_Pulse_M2 = 0.015; //Distance travelled per encoder pulse in inches for M2
float Wheel_Dist = 7.25; //wheel to wheel distance in inches
//int Motor_Speed = 50; //motor speed

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

  pinMode(XSHUT_PIN0, OUTPUT);
  pinMode(XSHUT_PIN1, OUTPUT);
  pinMode(XSHUT_PIN2, OUTPUT);
  pinMode(XSHUT_PIN3, OUTPUT);
  pinMode(XSHUT_PIN4, OUTPUT);
  pinMode(XSHUT_PIN5, OUTPUT);
  pinMode(XSHUT_PIN6, OUTPUT);

  // Start with the sensor enabled
  digitalWrite(XSHUT_PIN1, LOW);
  digitalWrite(XSHUT_PIN2, LOW);
  digitalWrite(XSHUT_PIN3, LOW);
  digitalWrite(XSHUT_PIN4, LOW);
  digitalWrite(XSHUT_PIN5, LOW);
  digitalWrite(XSHUT_PIN6, LOW);
  digitalWrite(XSHUT_PIN0, HIGH);
  delay(10);  // Allow sensor to power up

  //Serial.println("Starting TOF Sensor...");
  
  Wire.begin();
  Wire.setClock(50000);
  
  if (!tof0.init()) {
    //Serial.println("Failed to detect TOF0 sensor! Check wiring.");
    while (1);
  }
  tof0.setAddress(sensorAddresses[0]);
  tof0.setTimeout(500);
  //Serial.println("TOF0 sensor initialized successfully.");

  digitalWrite(XSHUT_PIN1, HIGH);
  delay(10);  // Allow sensor to power up

  if (!tof1.init()) {
    //Serial.println("Failed to detect TOF1 sensor! Check wiring.");
    while (1);
  }
  tof1.setAddress(sensorAddresses[1]);
  tof1.setTimeout(500);
  //Serial.println("TOF1 sensor initialized successfully."); 

  digitalWrite(XSHUT_PIN2, HIGH);
  delay(10);  // Allow sensor to power up 

  if (!tof2.init()) {
    //Serial.println("Failed to detect TOF2 sensor! Check wiring.");
    while (1);
  }
  tof2.setAddress(sensorAddresses[2]);
  tof2.setTimeout(500);
  //Serial.println("TOF2 sensor initialized successfully.");

  digitalWrite(XSHUT_PIN3, HIGH);
  delay(10);  // Allow sensor to power up

  if (!tof3.init()) {
    //Serial.println("Failed to detect TOF3 sensor! Check wiring.");
    while (1);
  }
  tof3.setAddress(sensorAddresses[3]);
  tof3.setTimeout(500);
  //Serial.println("TOF3 sensor initialized successfully."); 

  digitalWrite(XSHUT_PIN4, HIGH);
  delay(10);  // Allow sensor to power up

  if (!tof4.init()) {
    //Serial.println("Failed to detect TOF4 sensor! Check wiring.");
    while (1);
  }
  tof4.setAddress(sensorAddresses[1]);
  tof4.setTimeout(500);
  //Serial.println("TOF4 sensor initialized successfully.");

  digitalWrite(XSHUT_PIN5, HIGH);
  delay(10);  // Allow sensor to power up

  if (!tof5.init()) {
    //Serial.println("Failed to detect TOF5 sensor! Check wiring.");
    while (1);
  }
  tof5.setAddress(sensorAddresses[5]);
  tof5.setTimeout(500);
  //Serial.println("TOF5 sensor initialized successfully.");

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
  //Wire.begin();
  //Wire.setClock(50000);  // Set I2C speed to 50kHz
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.print("Hello!");
  //lcd.clear();
  lcd.createChar(0, circleTopLeft);
  lcd.createChar(1, circleTopRight);
  lcd.createChar(2, arrowRight);
  lcd.createChar(3, arrowLeft);
}

void loop()
{
  Receive_Data();
  delay(25);
  if (Receive_Com[0] == 'x')
  {
    ArdSerial.write("[M1:0,M2:0]");
    Receive_Com[0] = '\0';
  }

  // Update LCD only if there's new data in Receive_Com
  //LCD_UpdateIfNewData();
  //LCD_Display();

  Receive_Com[0] = '\0'; 
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

void Drive_Pulse_Target()
{
  DCM1_Target = int(Drive_Val / Dist_Per_Pulse_M1);
  DCM2_Target = int(Drive_Val / Dist_Per_Pulse_M2);
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

void Rot_Pulse_Target()
{
  DCM1_Target = -int(((Rotate_Val * M_PI / 180) * Wheel_Dist / 2) / Dist_Per_Pulse_M1);
  DCM2_Target = int(((Rotate_Val * M_PI / 180) * Wheel_Dist / 2) / Dist_Per_Pulse_M2);
}

void Receive_Data()
{
  char data;
  byte count = 0;
  bool Receive_Inpr = false;
  bool New_Data = false;
  char Com_Start = '['; // Command start character marker
  char Com_End = ']';   // Command end character marker
  while (Serial.available() > 0 && New_Data == false)
  {
    data = Serial.read();
    if (Receive_Inpr == true)
    {
      if (data != Com_End)
      {
        Receive_Com[count] = data;
        count++;
      }
      else
      {
        Receive_Com[count] = '\0';
        Receive_Inpr = false;
        count = 0;
        New_Data = true;
        //Serial.print("Receive_Com in Receive_Data: ");
        //Serial.println(Receive_Com);
      }
    }
    else if (data == Com_Start)
    {
      Receive_Inpr = true;
    }
  }
  Com_Type();
}

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

void Com_Type()
{
  if (Receive_Com[0] == 'u' || Receive_Com[0] == 'm')
  {
    Process_Sensor_Com();
  }
  else if (Receive_Com[0] == 'w' || Receive_Com[0] == 'r')
  {
    Process_Drive_Com();
    Serial.println("[COMMAND RECEIVED]");
  }
  else if (Receive_Com[0] == 's')
  {
    Process_Servo_Com();
  }
  else if (Receive_Com[0] == 'x')
  {
    ArdSerial.write("[M1:0,M2:0]");
  }
}

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

void Process_Sensor_Com()
{
  //Serial.print("Receive_Com in Process_Sensor_Com: ");
  //Serial.println(Receive_Com);
  String result = "[";
  char *token = strtok(Receive_Com, ","); // Split Receive_com by delimiter (commas)

  while (token != NULL)
  {
    String tokenStr = String(token);
    float output = 0;
    if (tokenStr == "u0") {
      while(output < 1) {
        output = tof0.readRangeSingleMillimeters()/25.4;
      }
    } 
    else if (tokenStr == "u1") {
      while(output < 1) {
        output = tof1.readRangeSingleMillimeters()/25.4;
      }
    } 
    else if (tokenStr == "u2") {
      while(output < 1) {
        output = tof2.readRangeSingleMillimeters()/25.4;
      }
    } 
    else if (tokenStr == "u3") {
      while(output < 1) {
        output = tof3.readRangeSingleMillimeters()/25.4;
      }
    } 
    else if (tokenStr == "u4") {
      while(output < 1) {
        output = tof4.readRangeSingleMillimeters()/25.4;
      }
    } 
    else if (tokenStr == "u5") {
      while(output < 1) {
        output = tof5.readRangeSingleMillimeters()/25.4;
      }
    } 
    else if (tokenStr == "u6") {
      while(output < 1) {
        output = tof6.readRangeSingleMillimeters()/25.4;
      }
    } 
    else if (tokenStr == "m0")
    {
      output = M2_Check();
    }
    else if (tokenStr == "m1")
    {
      output = M1_Check();
    }

    // Append to result in proper format
    result += tokenStr + ":" + String(output, 2) + ",";

    token = strtok(NULL, ","); // Move to the next token
  }

  // Remove the last comma and close the bracket
  result.remove(result.length() - 1);
  result += "]";
  Receive_Com[0] = '\0';
  Serial.println(result);
}

float M1_Check()
{
  return DCM1_Enc_Count * Dist_Per_Pulse_M1;
}

float M2_Check()
{
  return DCM2_Enc_Count * Dist_Per_Pulse_M2;
}

// Function to handle LCD display logic based on Receive_Com
/*void LCD_Display() {
  lcd.clear();

  // Check if the command is for "r" (arrow display)
  if (Receive_Com[0] == 'r') {
    if (Receive_Com[3] == '-') {
      // Display arrow bending right (this is ccw)
      
      for (int i=0 ; i<15 ; i=i+3){
        lcd.setCursor(i, 0); // Set cursor to the first character of the top row for the circle
        lcd.write(byte(0));  // Top-left of the circle
        lcd.write(byte(2));  // Arrow right
        lcd.write(byte(1));  // Bottom-right of the circle
        delay(200);   
        }

      
    } else {
      // Display arrow bending left (this is cw)

      for (int i=16 ; i>=0 ; i=i-3){
        lcd.setCursor(i, 0); // Set cursor to the first character of the top row for the circle
        lcd.write(byte(0));  // Top-left of the circle
        lcd.write(byte(3));  // Arrow left
        lcd.write(byte(1));  // Bottom-right of the circle
        delay(200);   
        }
      // lcd.setCursor(6, 0); // Set cursor to the first character of the top row for the circle

    }
  }
  // Check if the command is for "w" (display specific patterns)
  else if (Receive_Com[0] == 'w') {
    if (Receive_Com[3] != '-') {
      // goes forward, dispay arrows going forward
      for (int i = 0; i < 16; i++) {
        lcd.setCursor(i, 0);   // Move cursor to the next block in the first row
        lcd.print(">");        // Print a character (e.g., "*") to "light up" the block
        delay(200);            // Adjust delay for speed (200ms between each block)
        
    }
    } else {
      // goes backward, display arrows going back
      for (int i = 16; i >= 0; i--) {
        lcd.setCursor(i, 0);   // Move cursor to the next block in the first row
        lcd.print("<");        // Print a character (e.g., "*") to "light up" the block
        delay(200);            // Adjust delay for speed (200ms between each block)
        
    }
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

void LCD_UpdateIfNewData() {
  // Check if the current command is different from the last one
  if (strcmp(Receive_Com, lastReceiveCom) != 0) {
    // Update lastReceiveCom to the current command
    strcpy(lastReceiveCom, Receive_Com);
    // Call LCD_Display function only if there is new data
    LCD_Display();
  }
} */