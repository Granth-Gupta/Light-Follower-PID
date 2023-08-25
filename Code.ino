#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
#define ir6 A5

float Kp = 2.8;
float Ki = 0;
float Kd = 0;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error=0,previous_I =0;

int sen1,sen2,sen3,sen4,sen5,sen6;

// motor one - left motor
int MotorAip1 = 3;
int MotorAip2 = 5;
int maxA =162;
// motor two - right motor
int MotorBip2 = 9;
int MotorBip1 = 11;
int maxB = 180;

void setup()
{
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
  pinMode(ir6, INPUT);
  pinMode(MotorAip1, OUTPUT);
  pinMode(MotorAip2, OUTPUT);
  pinMode(MotorBip1, OUTPUT);
  pinMode(MotorBip2, OUTPUT);

  Serial.begin(9600);
}

void readIR() {
  sen1 = analogRead(A0);
  sen1 = map(sen1, 85, 505, 255, 0);
  if (sen1 < 0){
    sen1 = 0;
  }
  else if(sen1 > 255){
    sen1 = 255;
  }
  sen2 = analogRead(A1);
  sen2 = map(sen2, 70, 550, 255, 0);
  if (sen2 < 0){
    sen2 = 0;
  }
  else if(sen2 > 255){
    sen2 = 255;
  }
  sen3 = analogRead(A2);
  sen3 = map(sen3, 30, 280, 255, 0);
  if (sen3 < 0){
    sen3 = 0;
  }
  else if(sen3 > 255){
    sen3 = 255;
  }
  sen4 = analogRead(A3);
  sen4 = map(sen4, 50, 460, 255, 0);
  if (sen4 < 0){
    sen4 = 0;
  }
  else if(sen4 > 255){
    sen4 = 255;
  }
  sen5 = analogRead(A4);
  sen5 = map(sen5, 55, 450, 255, 0);
  if (sen5 < 0){
    sen5 = 0;
  }
  else if(sen5 > 255){
    sen5 = 255;
  }
  sen6 = analogRead(A5);
  sen6 = map(sen6, 55, 480, 255, 0);
  if (sen6 < 0){
    sen6 = 0;
  }
  else if(sen6 > 255){
    sen6 = 255;
  }
}

void calculate_pid()
{
  int error = (3 * sen1) + (2 * sen2) + (1 * sen3) - (1 * sen4) - (2 * sen5) - (3 * sen6);
  error = map(error,-1100,1100,-40,40);
  Serial.println(error);

  if(abs(error) < 1){
    motor_control(0);
  }

  else{
    
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
  motor_control(PID_value);
  }
}
void motor_control(int value)
{
  int speedA = maxA + PID_value;
  int speedB = maxB - PID_value;

  analogWrite(MotorAip1, speedA);
  analogWrite(MotorAip2, 0);
  analogWrite(MotorBip1, speedB);
  analogWrite(MotorBip2, 0);
}

void loop()
{
  readIR();
  calculate_pid();
}
