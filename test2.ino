const int Kp = 3.7;   // Kp value that you have to change was 4.2 when last working
const int Kd = 4;   // Kd value that you have to change
const int Ki = 0.7;
const int setPoint = 35;    // Middle point of sensor array
const int lowSpeed = 0;
const int baseSpeed = 100;    // Base speed for your motors 100
const int maxSpeed = 255;   // Maximum speed for your motors
int integral=0;

const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte serialEn = 2;    // Connect UART output enable of LSA08 to pin 2
const byte junctionPulse = 4;   // Connect JPULSE of LSA08 to pin 4
const byte dir1 = 8;   // Connect DIR1 of motor driver to pin 13
const byte dir2 = 7;   // Connect DIR2 of motor driver to pin 12
const byte dir3 = 6;   // Connect DIR1 of motor driver to pin 13
const byte dir4 = 5;   // Connect DIR2 of motor driver to pin 12

const byte pwm1 = 9;   // Connect PWM1 of motor driver to pin 11
const byte pwm2 = 10;   // Connect PWM2 of motor driver to pin 10

void setup() {
  pinMode(serialEn,OUTPUT);   // Setting serialEn as digital output pin
  pinMode(junctionPulse,INPUT);   // Setting junctionPulse as digital input pin

  // Setting pin 10 - 13 as digital output pin
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(dir3,OUTPUT);
  pinMode(dir4,OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);

  // Setting initial condition of serialEn pin to HIGH
  digitalWrite(serialEn,HIGH);

  // Setting the initial condition of motors
  // make sure both PWM pins are LOW
  digitalWrite(pwm1,LOW);
  digitalWrite(pwm2,LOW);
  

  // State of DIR pins are depending on your physical connection
  // if your robot behaves strangely, try changing thses two values
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  digitalWrite(dir3,HIGH);
  digitalWrite(dir4,LOW);

  // Begin serial communication with baudrate 9600
  Serial.begin(9600);

}

int lastError = 0;    // Declare a variable to store previous error

void loop() {
  digitalWrite(serialEn,LOW);   // Set serialEN to LOW to request UART data
  while(Serial.available() <= 0);   // Wait for data to be available
  int positionVal = Serial.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn,HIGH);    // Stop requesting for UART data


  Serial.println(positionVal);
  //Serial.println(analogRead(an_Pin));

  // If no line is detected, stay at the position
  if(positionVal == 255) {

    delay(10);
    digitalWrite(serialEn,LOW);   // Set serialEN to LOW to request UART data
    while(Serial.available() <= 0);   // Wait for data to be available
    int positionVal = Serial.read();    // Read incoming data and store in variable positionVal
    digitalWrite(serialEn,HIGH);    
    if(positionVal==255)
    {

      if(lastError>0)
      {
          digitalWrite(dir1,LOW);
          digitalWrite(dir2,HIGH);
          digitalWrite(dir3,HIGH);
          digitalWrite(dir4,LOW);
          analogWrite(pwm1,70); //was 60
          analogWrite(pwm2,70);
          delay(30); //was 50 when 60
      }
      else if(lastError<0)
      {
          digitalWrite(dir1,HIGH);
          digitalWrite(dir2,LOW);
          digitalWrite(dir3,LOW);
          digitalWrite(dir4,HIGH);
          analogWrite(pwm1,70);
          analogWrite(pwm2,70);
          delay(30);
      }
      else
      {
            digitalWrite(dir1,LOW);
            digitalWrite(dir2,HIGH);
            digitalWrite(dir3,LOW);
            digitalWrite(dir4,HIGH);
            analogWrite(pwm1,150);
            analogWrite(pwm2,150);
            delay(50);
      }
    }
      

  }

  // Else if line detected, calculate the motor speed and apply
  else {
    digitalWrite(dir1,HIGH);
    digitalWrite(dir2,LOW);
    digitalWrite(dir3,HIGH);
    digitalWrite(dir4,LOW);
    int error = positionVal - setPoint;   // Calculate the deviation from position to the set point
    int motorSpeed = Kp * error + Kd * (error - lastError) ;   // Applying formula of PID
    lastError = error;    // Store current error as previous error for next iteration use

    // Adjust the motor speed based on calculated value
    // You might need to interchange the + and - sign if your robot move in opposite direction
    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;

    // If the speed of motor exceed max speed, set the speed to max speed
    if(rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if(leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

    // If the speed of motor is negative, set it to 0
    if(rightMotorSpeed < lowSpeed) 
    { 
      /*
      digitalWrite(dir1,LOW);
      digitalWrite(dir2,HIGH);
      rightMotorSpeed = 130;//100    -rightMotorSpeed;
      */
      rightMotorSpeed =0;
    }
    if(leftMotorSpeed < lowSpeed) 
    {
      /*
      digitalWrite(dir3,LOW);
      digitalWrite(dir4,HIGH);
      leftMotorSpeed = 130;//100     -leftMotorSpeed;   130 for lesser kp and more base speed
      */
      leftMotorSpeed =0;
    }
    // Writing the motor speed value as output to hardware motor
    analogWrite(pwm1,rightMotorSpeed);
    analogWrite(pwm2,leftMotorSpeed);
}

}
