#include <Servo.h>
#include <SoftwareSerial.h>

// Sensor Arduino Serial Communication Pins
int Ard_RX_Pin = 0;
int Ard_TX_Pin = 1;

// DC Motor Driver Control Pins
int DCM1_ENB_Pin = 2;
int DCM1_IN4_Pin = 3;
int DCM1_IN3_Pin = 5;
int DCM2_IN2_Pin = 6;
int DCM2_IN1_Pin = 8;
int DCM2_ENA_Pin = 9;

// Servo Motor Control Pin
Servo servo;
int Servo_Control_Pin = 11;

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
  
  pinMode(LCD_SDA_Pin, OUTPUT);
  pinMode(LCD_SCL_Pin, OUTPUT);
  ArdSerial.begin(9600); //Start serial communication with sensor arduino

}

void loop() {
  // put your main code here, to run repeatedly:
  Process_Com(Receive_Data());
}

char* Receive_Data() {
  char data;
  static byte count = 0;
  static bool Receive_Inpr = false;
  static bool New_Data = false;
  char Com_Start = '['; // Command start character marker
  char Com_End = ']';   // Command end character marker
  const byte Num_Char = 32; //Size of byte array
  static char Receive_Com[Num_Char]; //Character array to store commands WITHOUT BRACKETS

  while(ArdSerial.available() > 0 && New_Data == false)
  {
    data = ArdSerial.read();
    if(Receive_Inpr == true)
    {
      if(data != Com_End)
      {
        Receive_Com[count] = data;
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
        return Receive_Com;
      }
    }
    else if(data == Com_Start)
    {
      Receive_Inpr = true;
    }
  }
}

void Process_Com(char* Com) {
  // Pointer to keep track of the current position in the string
  char* segment = strtok(Com, ",");

  // Loop through each segment, which should look like "M1:50" or "M2:-50"
  while (segment != NULL) {
    // Extract the motor number and its value using sscanf
    int motorNumber, value;
    if (sscanf(segment, "M%d:%d", &motorNumber, &value) == 2) {
      // Call the DCM function with the extracted motor number and value
      DCM_On(motorNumber, value);
    }

    // Get the next segment
    segment = strtok(NULL, ",");
  } 
}

void DCM_On(int DCM, int SPEED) {
  //motor 1 on
  if(DCM == 1)
  {
    if(SPEED >= 0)
    {
    digitalWrite(DCM1_IN3_Pin, HIGH);
    digitalWrite(DCM1_IN4_Pin, LOW);
    analogWrite(DCM1_ENB_Pin, SPEED);
    }
    if(SPEED < 0)
    {
      digitalWrite(DCM1_IN3_Pin, LOW);
      digitalWrite(DCM1_IN4_Pin, HIGH);
      analogWrite(DCM1_ENB_Pin, abs(SPEED));
    }
  }
  //motor 2 on
  if(DCM == 2)
  {
    if(SPEED >= 0)
    {
      digitalWrite(DCM2_IN1_Pin, HIGH);
      digitalWrite(DCM2_IN2_Pin, LOW);
      analogWrite(DCM2_ENA_Pin, SPEED);
    }
    if(SPEED < 0)
    {
      digitalWrite(DCM2_IN1_Pin, LOW);
      digitalWrite(DCM2_IN2_Pin, HIGH);
      analogWrite(DCM2_ENA_Pin, abs(SPEED));
    }
  }
}