#include <SoftwareSerial.h>
#include <math.h>
#define _USE_MATH_DEFINES

// Bluetooth Serial Communication Pins
int BT_RX_Pin = 0;
int BT_TX_Pin = 1;

// DC Motor Encoder Outputs
int DCM1_EncA_Pin = 2;
int DCM2_EncA_Pin = 3;
int DCM1_EncB_Pin = 4;
int DCM2_EncB_Pin = 5;

// Ultrasonic Sensors Echo Pins
int U0_Echo_Pin = 6;
int U1_Echo_Pin = 7;
int U2_Echo_Pin = 8;
int U3_Echo_Pin = 9;

// Ultrasonic Sensors Trigger Pins
int U0_Trig_Pin = 10;
int U1_Trig_Pin = 11;
int U2_Trig_Pin = 12;
int U3_Trig_Pin = 13;

// Infrared Output Pin
int IR_Out_Pin = A0;

// Time of Flight Output Pins
int TofF_OutA_Pin = A1;
int TofF_OutB_Pin = A2;

// Actuation Arduino Serial Communication Pins
int Ard_RX_Pin = A3;
int Ard_TX_Pin = A4;

// Definition of constants
float V_Sound = 0.0135; //Velocity of sounds in in/us
float Wheel_Dia = 2.55906; //Wheel diameter in inches
int Enc_Res = 11; //Encoder pulses per full revolution
int Gear_Ratio = 34; //DC motor gear ratio
float Dist_Per_Pulse = 0.0197; //Distance travelled per encoder pulse in inches

//Array to store US/DRIVE command
const byte Num_Char = 32; //Size of byte array
char Receive_Com[Num_Char]; //Character array to store commands WITHOUT BRACKETS
float Drive_Val; //Store extracted drive command numerical value

// DC Motor Encoder Counts and Movement Tracking
int DCM1_Enc_Count = 0;
int DCM2_Enc_Count = 0;
int DCM1_Target = 0;
int DCM2_Target = 0;
bool DCM1_in_progress = false;
bool DCM2_in_progress = false;

int pingTime;
int U0_Length;
int U1_Length;
int U2_Length;
int U3_Length;

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
  
  pinMode(U0_Trig_Pin, OUTPUT);
  pinMode(U1_Trig_Pin, OUTPUT);
  pinMode(U2_Trig_Pin, OUTPUT);
  pinMode(U3_Trig_Pin, OUTPUT);
  
  pinMode(IR_Out_Pin, INPUT);
  
  pinMode(TofF_OutA_Pin, INPUT);
  pinMode(TofF_OutB_Pin, INPUT);

  SoftwareSerial ArdSerial(Ard_RX_Pin, Ard_TX_Pin);
  ArdSerial.begin(9600); //Start serial communication with actuation arduino
  Serial.begin(9600); //Start serial communication with bluetooth module
}

void loop() {
  // put your main code here, to run repeatedly:
  Receive_Data();
  if (Check_Com(Receive_Com)) {
    Process_Sensor_Com();
  } else {
    // Call to function to process data for sensor command goes here
  }
}

void DCM1_Enc_Update() {
// updates DC motor 1 encoder value after interrupt
  if (digitalRead(DCM1_EncA_Pin) == HIGH) {
    if (digitalRead(DCM1_EncB_Pin) == LOW) {
      DCM1_Enc_Count++;
      DCM1_Target++;
    } else {
      DCM1_Enc_Count--;
      DCM1_Target--;
    }
  } else {
    if (digitalRead(DCM1_EncB_Pin) == LOW) {
      DCM1_Enc_Count--;
      DCM1_Target--;
    } else {
      DCM1_Enc_Count++;
      DCM1_Target++;
    }
  }
  if (DCM1_in_progress && DCM1_Target == 0) {
    Transmit_mot_command(stop motor 1) // *** Needs to be updated when commands are finalized
    DCM1_in_progress = false
  }
}

void DCM2_Enc_Update() {
// updates DC motor 2 encoder value after interrupt
  if (digitalRead(DCM2_EncA_Pin) == HIGH) {
    if (digitalRead(DCM2_EncB_Pin) == LOW) {
      DCM2_Enc_Count++;
      DCM2_Target++;
    } else {
      DCM2_Enc_Count--;
      DCM2_Target--;
    }
  } else {
    if (digitalRead(DCM2_EncB_Pin) == LOW) {
      DCM2_Enc_Count--;
      DCM2_Target--;
    } else {
      DCM2_Enc_Count++;
      DCM2_Target++;
    }
  }
  if (DCM2_in_progress && DCM2_Target == 0) {
    Transmit_mot_command(stop motor 2) // *** Needs to be updated when commands are finalized
    DCM2_in_progress = false
  }
}

int Pulse_Target() {
  DCM1_Target = int(round(Drive_Val/Dist_Per_Pulse));
  DCM2_Target = DCM1_Target;
}

void Receive_Data() {
  char data;
  static byte count = 0;
  static bool Receive_Inpr = false;
  static bool New_Data = false;
  char Com_Start = '['; // Command start character marker
  char Com_End = ']';   // Command end character marker

  while(Serial.available() > 0 && New_Data == false)
  {
    data = Serial.read();
    if(Receive_Inpr == true)
    {
      if(data != Com_End)
      {
        Recieve_Com[count] = data;
        count++; 
        if(count >= Num_Char)
        {
          count = Num_Char - 1;
        }
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
}

void Process_Drive_Com(Receive_Com) {
  char Drive_Com[];
  for(int i=0; i<strlen(Receive_Com)-3, i++)
  {
    Drive_Com[i] = Receive_Com[i+3];
  }
  Drive_Val = atof(Drive_Com);
}

bool Check_Com() {
  //Check and return true if drive command, false if sensor command
  if(Receive_Com[0] == 'u')
  {
    return false;
  }
  else
  {
    return true;
  }
}

void Process_Sensor_Com() {
    String result = "[";
    char* token = strtok(Receive_Com, ",");  // Split Receive_com by delimiter (commas)

    while (token != NULL) {
        String tokenStr = String(token);
        float output;

        if (tokenStr == "u0") {
            output = U0_check();
        } else if (tokenStr == "u1") {
            output = U1_check();
        } else if (tokenStr == "u2") {
            output = U2_check();
        } else if (tokenStr == "u3") {
            output = U3_check();
        }
        else if (tokenStr == "m0") {
            output = M2_check();
        }
        else if (tokenStr == "m1") {
            output = M1_check();
        }

        // Append to result in proper format
        result += tokenStr + ":" + String(output, 2) + ",";
        
        token = strtok(NULL, ",");  // Move to the next token
    }

    // Remove the last comma and close the bracket
    result.remove(result.length() - 1);
    result += "]";
    Receive_Com[0] = "\0"
    Serial.print(result)
}

float U0_Check() {
// When called will check the forward ultrasonic sensor
// returns a length float

  digitalWrite(U0_Trig_Pin, LOW);   // resets trigger pin
  delayMicroseconds(2000);          //adjustable delay
  digitalWrite(U0_Trig_Pin, HIGH);  //trigger for 10 us
  delayMicroseconds(10);
  digitalWrite(U0_Trig_Pin, LOW);

  pingTime = pulseIn(U0_Echo_Pin, HIGH);  // reads echo pin

  U0_Length = pingTime * V_Sound * 0.5  // U0 length in inches

  return U0_Length;

}

float U1_Check() {
// When called will check the right ultrasonic sensor
// returns a length float

  digitalWrite(U1_Trig_Pin, LOW);   // resets trigger pin
  delayMicroseconds(2000);          //adjustable delay
  digitalWrite(U1_Trig_Pin, HIGH);  //trigger for 10 us
  delayMicroseconds(10);
  digitalWrite(U1_Trig_Pin, LOW);

  pingTime = pulseIn(U1_Echo_Pin, HIGH);  // reads echo pin

  U1_Length = pingTime * V_Sound * 0.5  // U1 length in inches

  return U1_Length;

}

float U2_Check() {
// When called will check the back ultrasonic sensor
// returns a length float

  digitalWrite(U2_Trig_Pin, LOW);   // resets trigger pin
  delayMicroseconds(2000);          //adjustable delay
  digitalWrite(U2_Trig_Pin, HIGH);  //trigger for 10 us
  delayMicroseconds(10);
  digitalWrite(U2_Trig_Pin, LOW);

  pingTime = pulseIn(U2_Echo_Pin, HIGH);  // reads echo pin

  U2_Length = pingTime * V_Sound * 0.5  // U2 length in inches

  return U2_Length;

}

float U3_Check() {
// When called will check the left ultrasonic sensor
// returns a length float

  digitalWrite(U3_Trig_Pin, LOW);   // resets trigger pin
  delayMicroseconds(2000);          //adjustable delay
  digitalWrite(U3_Trig_Pin, HIGH);  //trigger for 10 us
  delayMicroseconds(10);
  digitalWrite(U3_Trig_Pin, LOW);

  pingTime = pulseIn(U3_Echo_Pin, HIGH);  // reads echo pin

  U3_Length = pingTime * V_Sound * 0.5  // U3 length in inches

  return U3_Length;

}

float M1_Check() {
  return DCM1_Enc_Count
}

float M2_check() {
  return DCM2_Enc_Count
}