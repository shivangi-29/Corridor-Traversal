#define L_MOTOR_POSITIVE 9
#define L_MOTOR_NEGATIVE 10
#define L_MOTOR_ENABLE 3
#define R_MOTOR_POSITIVE 6
#define R_MOTOR_NEGATIVE 11
#define R_MOTOR_ENABLE 5
#define LED_POSITIVE 13
#define LED_NEGATIVE 12
#define LED_PIN 13
#define MAX_SPEED_R 170
#define MAX_SPEED_L 150
/*
 * Movements:
 *     W := forward
 *     S := stop
 *     D := right
 *     A := left
 *     R := extreme right
 *     L := extreme left
 *     X := reverse
 */

void setup() 
{
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

int incomingByte = Serial.read();

void loop()
{
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();

    Serial.print("I received: ");
    Serial.println(incomingByte);
    \
    if (incomingByte == 'A')
    {
      analogWrite(L_MOTOR_POSITIVE, 0);
      analogWrite(L_MOTOR_NEGATIVE, 0);

      analogWrite(R_MOTOR_POSITIVE, 200);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, 200);
      
      Serial.println("Turn left");
    }
    else if (incomingByte == 'D')
    {
      analogWrite(L_MOTOR_POSITIVE, 200);
      analogWrite(L_MOTOR_NEGATIVE, 0);

      analogWrite(R_MOTOR_POSITIVE, 0);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
      Serial.println("Turn right");
    }
    else if (incomingByte == 'W') 
    {
      analogWrite(L_MOTOR_POSITIVE, MAX_SPEED_L);
      analogWrite(L_MOTOR_NEGATIVE, 0);

      analogWrite(R_MOTOR_POSITIVE, MAX_SPEED_R);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
      Serial.println("Straight");
    }
    else if (incomingByte == 'S')
    {
      analogWrite(L_MOTOR_POSITIVE, 0);
      analogWrite(L_MOTOR_NEGATIVE, 0);

      analogWrite(R_MOTOR_POSITIVE, 0);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
      Serial.println("Move Back");
    }
    else if (incomingByte == 'R')
    {
      analogWrite(R_MOTOR_POSITIVE, 0);
      analogWrite(R_MOTOR_NEGATIVE, MAX_SPEED_R);

      analogWrite(L_MOTOR_POSITIVE, MAX_SPEED_L);
      analogWrite(L_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);

      delay(500);

      analogWrite(R_MOTOR_POSITIVE, 0);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_POSITIVE, 0);
      analogWrite(L_MOTOR_POSITIVE, 0);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    else if (incomingByte == 'L')
    {
      analogWrite(R_MOTOR_POSITIVE, MAX_SPEED_R);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_POSITIVE, 0);
      analogWrite(L_MOTOR_NEGATIVE, MAX_SPEED_L);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);

      delay(500);

      analogWrite(R_MOTOR_POSITIVE, 0);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_POSITIVE, 0);
      analogWrite(L_MOTOR_POSITIVE, 0);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    
    else if (incomingByte == 'B') 
    {
      analogWrite(R_MOTOR_POSITIVE, 0);
      analogWrite(R_MOTOR_NEGATIVE, 0);

      analogWrite(L_MOTOR_POSITIVE, 0);
      analogWrite(L_MOTOR_NEGATIVE, 0);

      delay(1000);
    
      digitalWrite(12, HIGH);
      delay(1000);
      digitalWrite(12, LOW);
      delay(1000);
      Serial.println("Blink");
    } 
  }
}



