#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define NUM_SENSORS 7
#define _USE_MATH_DEFINES

// Bluetooth Serial Communication Pins
//int BT_RX_Pin = 0;
//int BT_TX_Pin = 1;

// DC Motor Encoder Outputs
int DCM1_EncA_Pin = 3;
int DCM2_EncA_Pin = 2;
int DCM1_EncB_Pin = 4;
int DCM2_EncB_Pin = 5;

// Ultrasonic Sensors Echo Pins
Adafruit_VL53L0X ToF_Sensors[NUM_SENSORS];
int ToF_XSHUT_Pins[NUM_SENSORS] = {6, 7, 8, 9, A3, A1, 10}; // XSHUT pins for each sensor
byte ToF_Addresses[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36}; // Unique I2C addresses

// Infrared Output Pin
// int IR_Out_Pin = 11;

// Actuation Arduino Serial Communication Pins
int Ard_RX_Pin = A4;
int Ard_TX_Pin = A5;

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

// ToF results 
float V0_Length = 0;
float V1_Length = 0;
float V2_Length = 0;
float V3_Length = 0;
float V4_Length = 0;
float V5_Length = 0;
float V6_Length = 0;

SoftwareSerial ArdSerial(Ard_RX_Pin, Ard_TX_Pin); //PINS ON SENSOR THAT SEND TO ACTUATION

void setup() {
  // put your setup code here, to run once:

  // Define output and input pins
  
  pinMode(DCM1_EncA_Pin, INPUT);
  pinMode(DCM2_EncA_Pin, INPUT);
  pinMode(DCM1_EncB_Pin, INPUT);
  pinMode(DCM2_EncB_Pin, INPUT);

  // initialize encoder interrupts
  attachInterrupt(digitalPinToInterrupt(DCM1_EncA_Pin), DCM1_Enc_Update, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(DCM2_EncA_Pin), DCM2_Enc_Update, CHANGE); 

  ArdSerial.begin(9600); //Start serial communication with actuation arduino
  Serial.begin(9600); //Start serial communication with bluetooth module
  Wire.begin(); //I2C communication

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    pinMode(ToF_XSHUT_Pins[i], OUTPUT);
    digitalWrite(ToF_XSHUT_Pins[i], LOW); // Turn off each sensor initially
  }

  // Initialize each sensor one by one
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    digitalWrite(ToF_XSHUT_Pins[i], HIGH); // Enable the sensor
    delay(10); // Wait for the sensor to power up

    // Initialize sensor with default address
    if (!ToF_Sensors[i].begin())
    {
      Serial.print("Failed to initialize VL53L0X sensor ");
      Serial.println(i);
      while (1); // Stop if initialization fails
    }

    // Serial.print("Sensor ");
    // Serial.print(i);
    // Serial.print(" initialized at address 0x");
    // Serial.println(sensorAddresses[i], HEX);
    
    ToF_Sensors[i].setAddress(ToF_Addresses[i]); //Assign address
    digitalWrite(ToF_XSHUT_Pins[i], HIGH); //Set sensor to LOW again
  }

  // After initialization, re-enable all sensors
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    digitalWrite(ToF_XSHUT_Pins[i], HIGH);
  }

  ArdSerial.write("[M1:0,M2:0]");
  Receive_Com[0] = '\0';
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
  VL53L0X_RangingMeasurementData_t ToF_Data;
  String result = "[";
  char *token = strtok(Receive_Com, ","); // Split Receive_com by delimiter (commas)

  while (token != NULL)
  {
    String tokenStr = String(token);
    float output = 0;

    if (tokenStr == "u0")
    {
      while (output == 0)
      {
        ToF_Sensors[0].rangingTest(&ToF_Data, false);
        output = ToF_Data.RangeMilliMeter;
      }
    }
    else if (tokenStr == "u1")
    {
      while (output == 0)
      {
        ToF_Sensors[1].rangingTest(&ToF_Data, false);
        output = ToF_Data.RangeMilliMeter;
      }
    }
    else if (tokenStr == "u2")
    {
      while (output == 0)
      {
        ToF_Sensors[2].rangingTest(&ToF_Data, false);
        output = ToF_Data.RangeMilliMeter;
      }
    }
    else if (tokenStr == "u3")
    {
      while (output == 0)
      {
        ToF_Sensors[3].rangingTest(&ToF_Data, false);
        output = ToF_Data.RangeMilliMeter;
      }
    }
    else if (tokenStr == "u4")
    {
      while (output == 0)
      {
        ToF_Sensors[4].rangingTest(&ToF_Data, false);
        output = ToF_Data.RangeMilliMeter;
      }
    }
    else if (tokenStr == "u5")
    {
      while (output == 0)
      {
        ToF_Sensors[5].rangingTest(&ToF_Data, false);
        output = ToF_Data.RangeMilliMeter;
      }
    }
    else if (tokenStr == "u6")
    {
      while (output == 0)
      {
        ToF_Sensors[6].rangingTest(&ToF_Data, false);
        output = ToF_Data.RangeMilliMeter;
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