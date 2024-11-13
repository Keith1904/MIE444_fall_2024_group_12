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
VL53L0X ToF_Sensors[NUM_SENSORS];
int ToF_XSHUT_Pins[NUM_SENSORS] = {6, 7, 8, 9, A3, A1, 10}; // XSHUT pins for each sensor
byte ToF_Addresses[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36}; // Unique I2C addresses

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
  //Wire.begin(); //I2C communication

  // Define output and input pins
  
  pinMode(DCM1_EncA_Pin, INPUT);
  pinMode(DCM2_EncA_Pin, INPUT);
  pinMode(DCM1_EncB_Pin, INPUT);
  pinMode(DCM2_EncB_Pin, INPUT);

  // initialize encoder interrupts
  attachInterrupt(digitalPinToInterrupt(DCM1_EncA_Pin), DCM1_Enc_Update, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(DCM2_EncA_Pin), DCM2_Enc_Update, CHANGE); 

  // Set up each ToF sensor
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(ToF_XSHUT_Pins[i], OUTPUT);
    digitalWrite(ToF_XSHUT_Pins[i], LOW); // Turn off each sensor initially
  }

  // Initialize each sensor one by one
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(ToF_XSHUT_Pins[i], HIGH); // Power on sensor
    delay(10); // Wait for the sensor to power up

    ToF_Sensors[i].init(); // Initialize sensor with default address
    ToF_Sensors[i].setAddress(ToF_Addresses[i]); // Assign unique address to sensor
  }

  // After initialization, re-enable all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(ToF_XSHUT_Pins[i], HIGH);
  }

  ArdSerial.write("[M1:0,M2:0]");
  Receive_Com[0] = '\0';

  //LCD Setup 
  Wire.begin();
  Wire.setClock(50000);  // Set I2C speed to 50kHz
  //lcd.begin();
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
  Receive_Com[0] = '\0';

  // Update LCD only if there's new data in Receive_Com
  LCD_UpdateIfNewData();

  Receive_Com[0] = '\0';
  //delay(5000);
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
      }
    }
    else if (data == Com_Start)
    {
      Receive_Inpr = true;
    }
  }
  // Serial.println(Receive_Com);
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
  else if (Receive_Com[0] == 's')
  {
    Servo_Val = atof(Drive_Com);
    ArdSerial.write("[S:%d]", &Servo_Val);
  }
}

void Process_Sensor_Com()
{
  String result = "[";
  char *token = strtok(Receive_Com, ","); // Split Receive_com by delimiter (commas)

  while (token != NULL)
  {
    String tokenStr = String(token);
    float output = 0;

    if (tokenStr == "u0") {
      output = ToF_Sensors[0].readRangeSingleMillimeters();
    } 
    else if (tokenStr == "u1") {
      output = ToF_Sensors[1].readRangeSingleMillimeters();
    } 
    else if (tokenStr == "u2") {
      output = ToF_Sensors[2].readRangeSingleMillimeters();
    } 
    else if (tokenStr == "u3") {
      output = ToF_Sensors[3].readRangeSingleMillimeters();
    } 
    else if (tokenStr == "u4") {
      output = ToF_Sensors[4].readRangeSingleMillimeters();
    } 
    else if (tokenStr == "u5") {
      output = ToF_Sensors[5].readRangeSingleMillimeters();
    } 
    else if (tokenStr == "u6") {
      output = ToF_Sensors[6].readRangeSingleMillimeters();
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
void LCD_Display() {
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
      // lcd.write(byte(0));  // Top-left of the circle
      // lcd.write(byte(3));  // Arrow left
      // lcd.write(byte(1));  // Bottom-right of the circle
      // delay(200);   

      // lcd.setCursor(0, 0);
      // lcd.createChar(0, cw_customChar);
      // lcd.home();
      // lcd.write(0);

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

  // else {
    
  //   if (Receive_Com[0] == 'L') {
  //     lcd.print("Arrived at Loading Zone!");
  //   }

  //   slse if (Receive_Com[0] == 'L') {
  //     lcd.print("Arrived at Delivery Zone!");
  //   }
  // }
  
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
}