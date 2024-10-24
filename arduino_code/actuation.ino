#include <Servo.h>

// Sensor Arduino Serial Communication Pins
int Ard_RX_Pin = 0;
int Ard_TX_Pin = 1;

// DC Motor Driver Control Pins
int DCM_ENB_Pin = 2;
int DCM_IN4_Pin = 3;
int DCM_IN3_Pin = 5;
int DCM_IN2_Pin = 6;
int DCM_IN1_Pin = 8;
int DCM_ENA_Pin = 9;

// Servo Motor Control Pin
Servo servo;
int Servo_Control_Pin = 11;

// LCD Screen Control Pins
int LCD_SDA_Pin = A4;
int LCD_SCL_Pin = A5;

void setup() {
  // put your setup code here, to run once:

// Define output and input pins

  pinMode(DCM_ENB_PIN, OUTPUT);
  pinMode(DCM_IN4_PIN, OUTPUT);
  pinMode(DCM_IN3_PIN, OUTPUT);
  pinMode(DCM_IN2_PIN, OUTPUT);
  pinMode(DCM_IN1_PIN, OUTPUT);
  pinMode(DCM_ENA_PIN, OUTPUT);
  
  servo.Attach(Servo_Control_Pin);
  
  pinMode(LCD_SDA_Pin, OUTPUT);
  pinMode(LCD_SCL_Pin, OUTPUT);
  SoftwareSerial ArdSerial(Ard_RX_Pin, Ard_TX_Pin);
  ArdSerial.begin(9600); //Start serial communication with sensor arduino

}

void loop() {
  // put your main code here, to run repeatedly:
  Receive_Data();
  Process_Com();
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
        return Receive_Com
      }
    }
    else if(data == Com_Start)
    {
      Receive_Inpr = true;
    }
  }
}

void Process_Com(Com){
  
}
