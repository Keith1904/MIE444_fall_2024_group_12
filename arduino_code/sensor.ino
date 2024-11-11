#include <SoftwareSerial.h>
#include <math.h>
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
int U0_Echo_Pin = 6;
int U1_Echo_Pin = 7;
int U2_Echo_Pin = 8;
int U3_Echo_Pin = 9;
int U4_Echo_Pin = A3;
int U5_Echo_Pin = A1;

// Ultrasonic Sensors Trigger Pins
int U0_Trig_Pin = 10;
int U1_Trig_Pin = 10;
int U2_Trig_Pin = 10;
int U3_Trig_Pin = 10;
int U4_Trig_Pin = A2;
int U5_Trig_Pin = A0;

// Infrared Output Pin
int IR_Out_Pin = 11;

// Time of Flight Output Pins
int TofF_OutA_Pin = 12;
int TofF_OutB_Pin = 13;

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

// DC Motor Encoder Counts and Movement Tracking
int DCM1_Enc_Count = 0;
int DCM2_Enc_Count = 0;
int DCM1_Target = 0;
int DCM2_Target = 0;
int DCM1_Dir = 0;
int DCM2_Dir = 0;

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

  pinMode(U0_Echo_Pin, INPUT);
  pinMode(U1_Echo_Pin, INPUT);
  pinMode(U2_Echo_Pin, INPUT);
  pinMode(U3_Echo_Pin, INPUT);
  pinMode(U4_Echo_Pin, INPUT);
  pinMode(U5_Echo_Pin, INPUT);
  
  pinMode(U0_Trig_Pin, OUTPUT);
  pinMode(U1_Trig_Pin, OUTPUT);
  pinMode(U2_Trig_Pin, OUTPUT);
  pinMode(U3_Trig_Pin, OUTPUT);
  pinMode(U4_Trig_Pin, OUTPUT);
  pinMode(U5_Trig_Pin, OUTPUT);
  
  pinMode(IR_Out_Pin, INPUT);
  
  pinMode(TofF_OutA_Pin, INPUT);
  pinMode(TofF_OutB_Pin, INPUT);

  ArdSerial.begin(9600); //Start serial communication with actuation arduino
  Serial.begin(9600); //Start serial communication with bluetooth module

  ArdSerial.write("[M1:0,M2:0]");
  Receive_Com[0] = '\0';
}

void loop() {
  Receive_Data();
  delay(25);
  if(Receive_Com[0] == 'x')
  {
    ArdSerial.write("[M1:0,M2:0]");
    Receive_Com[0] = '\0';
  }
  Receive_Com[0] = '\0';
}

void DCM1_Enc_Update() {
// updates DC motor 1 encoder value after interrupt
  if (digitalRead(DCM1_EncA_Pin) == HIGH) {
    if (digitalRead(DCM1_EncB_Pin) == LOW) {
      DCM1_Enc_Count++;
      DCM1_Target--;
    } else {
      DCM1_Enc_Count--;
      DCM1_Target++;
    }
  } else {
    if (digitalRead(DCM1_EncB_Pin) == LOW) {
      DCM1_Enc_Count--;
      DCM1_Target++;
    } else {
      DCM1_Enc_Count++;
      DCM1_Target--;
    }
  }
  //Serial.println(DCM1_Enc_Count);
  //Serial.println(DCM1_Target);
  if(DCM1_Target == 0) {
    //Serial.println("M1 STOPPED");
    ArdSerial.write("[M1:0,M2:0]"); // *** Needs to be updated when commands are finalized
  }
}

void DCM2_Enc_Update() {
// updates DC motor 2 encoder value after interrupt
  if (digitalRead(DCM2_EncA_Pin) == HIGH) {
    if (digitalRead(DCM2_EncB_Pin) == LOW) {
      DCM2_Enc_Count++;
      DCM2_Target--;
    } else {
      DCM2_Enc_Count--;
      DCM2_Target++;
    }
  } else {
    if (digitalRead(DCM2_EncB_Pin) == LOW) {
      DCM2_Enc_Count--;
      DCM2_Target++;
    } else {
      DCM2_Enc_Count++;
      DCM2_Target--;
    }
  }
  //Serial.println(DCM2_Enc_Count);
  //Serial.println(DCM2_Target);
  if(DCM2_Target == 0) {
    //Serial.println("M2 STOPPED");
    ArdSerial.write("[M1:0,M2:0]"); // *** Needs to be updated when commands are finalized
  }
}

void Drive_Pulse_Target() {
  DCM1_Target = int(Drive_Val/Dist_Per_Pulse_M1);
  DCM2_Target = int(Drive_Val/Dist_Per_Pulse_M2);
  if(DCM1_Target > 0) {
    DCM1_Dir = 1;
  }
  else {
    DCM1_Dir = -1;
  }
  if(DCM2_Target > 0) {
    DCM2_Dir = 1;
  }
  else {
    DCM2_Dir = -1;
  }
}

void Rot_Pulse_Target() {
  DCM1_Target = -int(((Rotate_Val*M_PI/180)*Wheel_Dist/2)/Dist_Per_Pulse_M1);
  DCM2_Target = int(((Rotate_Val*M_PI/180)*Wheel_Dist/2)/Dist_Per_Pulse_M2);
}

void Receive_Data() {
  char data;
  byte count = 0;
  bool Receive_Inpr = false;
  bool New_Data = false;
  char Com_Start = '['; // Command start character marker
  char Com_End = ']';   // Command end character marker
  while(Serial.available() > 0 && New_Data == false)
  {
    data = Serial.read();
    if(Receive_Inpr == true)
    {
      if(data != Com_End)
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
    else if(data == Com_Start)
    {
      Receive_Inpr = true;
    } 
  } 
  //Serial.println(Receive_Com);
  Com_Type();
}

void Send_Com() {
  //Serial.println("SENDING COM");
  if(DCM1_Target > 0 && DCM2_Target > 0)
  {
    ArdSerial.write("[M1:200,M2:200]");
  }
  else if(DCM1_Target > 0 && DCM2_Target < 0)
  {
   ArdSerial.write("[M1:200,M2:-200]");
  }
  else if(DCM1_Target < 0 && DCM2_Target > 0)
  {
   ArdSerial.write("[M1:-200,M2:200]");
  }
  else if(DCM1_Target < 0 && DCM2_Target < 0)
  {
   ArdSerial.write("[M1:-200,M2:-200]");
  }
}

void Com_Type() {
  if(Receive_Com[0] == 'u' || Receive_Com[0] == 'm')
  {
    Process_Sensor_Com();
  }
  else if(Receive_Com[0] == 'w' || Receive_Com[0] == 'r')
  {
    Process_Drive_Com();
    Serial.println("[COMMAND RECEIVED]");
  }
  else if(Receive_Com[0] == 'x')
  {
    ArdSerial.write("[M1:0,M2:0]");
  }
}

void Process_Drive_Com() {
  char Drive_Com[strlen(Receive_Com)-3];
  for(int i=0; i<strlen(Receive_Com)-3; i++)
  {
    Drive_Com[i] = Receive_Com[i+3];
  }
  if(Receive_Com[0] == 'w')
  {
    Drive_Val = atof(Drive_Com);
    Drive_Pulse_Target();
    Send_Com();
  }
  else
  {
    Rotate_Val = atof(Drive_Com);
    Rot_Pulse_Target();
    Send_Com();
  }
}

void Process_Sensor_Com() {
    String result = "[";
    char* token = strtok(Receive_Com, ",");  // Split Receive_com by delimiter (commas)

    while (token != NULL) {
        String tokenStr = String(token);
        float output = 0;

        if (tokenStr == "u0") {
            while(output == 0) {
              output = UltrasonicMedianCheck(U0_Trig_Pin,U0_Echo_Pin);
            }
        } else if (tokenStr == "u1") {
            while(output == 0) {
              output = UltrasonicMedianCheck(U1_Trig_Pin,U1_Echo_Pin);
            }
        } else if (tokenStr == "u2") {
            while(output == 0) {
              output = UltrasonicMedianCheck(U2_Trig_Pin,U2_Echo_Pin);
            }
        } else if (tokenStr == "u3") {
            while(output == 0) {
              output = UltrasonicMedianCheck(U3_Trig_Pin,U3_Echo_Pin);
            }
        } else if (tokenStr == "u4") {
            while(output == 0) {
              output = UltrasonicMedianCheck(U4_Trig_Pin,U4_Echo_Pin);
            }
        } else if (tokenStr == "u5") {
            while(output == 0) {
              output = UltrasonicMedianCheck(U5_Trig_Pin,U5_Echo_Pin);
            }
        }
          else if (tokenStr == "m0") {
            output = M2_Check();
        }
          else if (tokenStr == "m1") {
            output = M1_Check();
        }

        // Append to result in proper format
        result += tokenStr + ":" + String(output, 2) + ",";
        
        token = strtok(NULL, ",");  // Move to the next token
    }

    // Remove the last comma and close the bracket
    result.remove(result.length() - 1);
    result += "]";
    Receive_Com[0] = "\0";
    Serial.println(result);
}

float UltrasonicMedianCheck(int trigPin, int echoPin) {
    const int numReadings = 5;
    float readings[numReadings];
    int validReadings = 0;

    for (int i = 0; i < numReadings; i++) {
        digitalWrite(trigPin, LOW);         // Reset trigger pin
        delayMicroseconds(2000);            // Adjustable delay
        digitalWrite(trigPin, HIGH);        // Trigger for 10 µs
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        unsigned long pingTime = pulseIn(echoPin, HIGH);  // Reads echo pin
        float length = pingTime * V_Sound * 0.5;          // Calculate length in inches

        // Check if the reading is within bounds
        if (length >= 0 && length <= 70) {
            readings[validReadings] = length;
            validReadings++;
        }

        delay(50); // Short delay between measurements
    }

    // Return -1 if no valid readings were found
    if (validReadings == 0) {
        return 0;
    }

    // Manually sort the array of valid readings using selection sort
    for (int i = 0; i < validReadings - 1; i++) {
        int minIndex = i;
        for (int j = i + 1; j < validReadings; j++) {
            if (readings[j] < readings[minIndex]) {
                minIndex = j;
            }
        }
        // Swap the minimum element with the first unsorted element
        float temp = readings[i];
        readings[i] = readings[minIndex];
        readings[minIndex] = temp;
    }

    // Calculate the median
    if (validReadings % 2 == 1) {
        // If odd, return the middle element
        return readings[validReadings / 2];
    } else {
        // If even, return the average of the two middle elements
        return (readings[validReadings / 2 - 1] + readings[validReadings / 2]) / 2.0;
    }
}


float M1_Check() {
  return DCM1_Enc_Count * Dist_Per_Pulse_M1;
}

float M2_Check() {
  return DCM2_Enc_Count * Dist_Per_Pulse_M2;
}