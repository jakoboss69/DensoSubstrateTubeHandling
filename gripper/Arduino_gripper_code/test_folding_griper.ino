#include <Servo.h>

int hall0 = A1;
int hall1 = A2;
int hall2 = A3;
int hall3 = A4;

int servo1pin= 6; //svart
int servo2pin= 5; //blå

//tuned positions for the servos
int servo1_open = 70;
int servo1_close = 138;
int servo2_open = 158;
int servo2_close = 95;

Servo servo1;
Servo servo2;


void setup()
{
    pinMode(servo1pin, OUTPUT);
    pinMode(servo2pin, OUTPUT);
    pinMode(hall0, INPUT);
    pinMode(hall1, INPUT);
    pinMode(hall2, INPUT);
    pinMode(hall3, INPUT);

    servo1.attach(servo1pin);
    servo2.attach(servo2pin);
    
    servo1.write(servo1_open);
    servo2.write(servo2_open);
    Serial.begin(115200 );
}

void loop()
{

  if (Serial.available() > 0) {
    delay(50);
    int input = Serial.read();

    if (input == '2') //return hall data
    {
      int hall0_val = analogRead(hall0);
      int hall1_val = analogRead(hall1);
      int hall2_val = analogRead(hall2);
      int hall3_val = analogRead(hall3);   
      Serial.println( hall0_val + String(",") + hall1_val + String(",") + hall2_val + String(",") + hall3_val);
    }
    else if (input == '1') { //åpne
      servo1.write(servo1_open);
      servo2.write(servo2_open);
    }
    else if (input == '0'){ //lukke
      servo1.write(servo1_close);
      servo2.write(servo2_close);
    }
  } 
}