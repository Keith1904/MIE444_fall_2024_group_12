#include <Servo.h>
#include <SoftwareSerial.h>

// Array to store US/DRIVE command
const byte Num_Char = 32; //Size of byte array
char Receive_Com[Num_Char]; //Character array to store commands WITHOUT BRACKETS

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
int Servo_Control_Pin = 9;

// LCD Screen Control Pins
int LCD_SDA_Pin = A4;
int LCD_SCL_Pin = A5;

// Motor Speed
int DCM1_Speed = 0;
int DCM2_Speed = 0;

//Software Serial
SoftwareSerial ArdSerial(Ard_RX_Pin, Ard_TX_Pin);

void setup() {
  // put your setup code here, to run once:

// Define output and input pins

  pinMode(DCM1_ENB_Pin, OUTPUT);
  pinMode(DCM1_IN4_Pin, OUTPUT);
  pinMode(DCM1_IN3_Pin, OUTPUT);
  pinMode(DCM2_IN2_Pin, OUTPUT);
  pinMode(DCM2_IN1_Pin, OUTPUT);
  pinMode(DCM2_ENA_Pin, OUTPUT);
  
  servo.attach(Servo_Control_Pin);
  servo.write(0);
  
  pinMode(LCD_SDA_Pin, OUTPUT);
  pinMode(LCD_SCL_Pin, OUTPUT);
  Serial.begin(9600); //Start serial communication with sensor arduino
}

void loop() {
  // put your main code here, to run repeatedly:
  //Process_Com(Receive_Data());
  if(Receive_Data())
  {
    Process_Com(Receive_Com);
  }
  Receive_Com[0] = '\0';
  delay(100);
}

bool Receive_Data() {
  char data;
  byte count = 0;
  bool Receive_Inpr = false;
  bool New_Data = false;
  char Com_Start = '['; // Command start character marker
  char Com_End = ']';   // Command end character marker
  while(Serial.available() > 0 && New_Data == false)
  {
    data = Serial.read();
    Serial.println(data);
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
  return New_Data;
}

void Process_Com(char* Com) {
  // Pointer to keep track of the current position in the string
  char* segment = strtok(Com, ",");
  //Serial.println(segment);
  // Loop through each segment, which should look like "M1:50" or "M2:-50"
  while (segment != NULL) {
    // Extract the motor number and its value using sscanf
    int motorNumber, value;
    if (sscanf(segment, "M%d:%d", &motorNumber, &value) == 2) {
      // Call the DCM function with the extracted motor number and value
      //Serial.println(motorNumber);
      //Serial.println(value);
      DCM_On(motorNumber, value);
    }

    // Get the next segment
    segment = strtok(NULL, ",");
  } 
}

void DCM_On(int DCM, int SPEED) {
  //Serial.println("MOTOR ON");
  if(DCM == 1)
  {
    if(SPEED >= 0)
    {
    digitalWrite(DCM1_IN3_Pin, LOW);
    digitalWrite(DCM1_IN4_Pin, HIGH);
    analogWrite(DCM1_ENB_Pin, SPEED);
    }
    if(SPEED < 0)
    {
      digitalWrite(DCM1_IN3_Pin, HIGH);
      digitalWrite(DCM1_IN4_Pin, LOW);
      analogWrite(DCM1_ENB_Pin, abs(SPEED));
    }
  }
  //motor 2 on
  if(DCM == 2)
  {
    if(SPEED >= 0)
    {
      digitalWrite(DCM2_IN1_Pin, LOW);
      digitalWrite(DCM2_IN2_Pin, HIGH);
      analogWrite(DCM2_ENA_Pin, SPEED);
    }
    if(SPEED < 0)
    {
      digitalWrite(DCM2_IN1_Pin, HIGH);
      digitalWrite(DCM2_IN2_Pin, LOW);
      analogWrite(DCM2_ENA_Pin, abs(SPEED));
    }
  }
}