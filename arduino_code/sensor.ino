#include <SoftwareSerial.h>

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
int Wheel_Dia = 65; //Wheel diameter in mm
int Enc_Res = 11; //Encoder pulses per full revolution
int Gear_Ratio = 34; //DC motor gear ratio
float Dist_Per_Pulse; //Distance travelled per encoder pulse

//Array to store US/DRIVE command
const byte Num_Char = 32; //Size of byte array
char Receive_Com[Num_Char]; //Character array to store commands WITHOUT BRACKETS

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



}

void DCM1_Enc_Update() {
// updates DC motor 1 encoder value after interrupt
  if (digitalRead(DCM1_EncA_Pin) == HIGH) {
    if (digitalRead(DCM1_EncB_Pin) == LOW) {
      DCM1_Enc_Count++;
    } else {
      DCM1_Enc_Count--;
    }
  } else {
    if (digitalRead(DCM1_EncB_Pin) == LOW) {
      DCM1_Enc_Count--;
    } else {
      DCM1_Enc_Count++;
    }
  }
}

void DCM2_Enc_Update() {
// updates DC motor 2 encoder value after interrupt
  if (digitalRead(DCM2_EncA_Pin) == HIGH) {
    if (digitalRead(DCM2_EncB_Pin) == LOW) {
      DCM2_Enc_Count++;
    } else {
      DCM2_Enc_Count--;
    }
  } else {
    if (digitalRead(DCM2_EncB_Pin) == LOW) {
      DCM2_Enc_Count--;
    } else {
      DCM2_Enc_Count++;
    }
  }
}

float Enc_Dist() {
  //Converts encoder pulses to distance based on wheel diameter
  float Wheel_Circ = 3.14159*Wheel_Dia;
  int Pulse_Per_Rev = Enc_Res*Gear_Ratio;
  Dist_Per_Pulse = Wheel_Circ/Pulse_Per_Rev;
  return Dist_Per_Pulse;
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

bool Check_Command(Receive_Com) {
  //Check and return true if drive command, false if sensor command
  if(Receive_Com[0] == 'w')
  {
    return true;
  }
  else
  {
    return false;
  }
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