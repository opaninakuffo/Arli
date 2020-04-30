#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  115 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  560 // this is the 'maximum' pulse length count (out of 4096)
#define S0 6
#define S1 5
#define S2 3
#define S3 4
#define sensorOut 2

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //initializing the LCD display

int Rw = 35;  // color sensor red diodes low val
int Rb = 300; // color sensor red diodes high val
int Gw = 40; // color sensor green diodes low val
int Gb = 320; // color sensor green diodes high val
int Bw = 35; // color sensor blue diodes low val
int Bb = 255; // color sensor blue diodes high val
unsigned long interval = 4000;
unsigned long previousMillis = 0;
unsigned long currentMillis;
int buttonPushCounter = 0;   // counter for the number of mode button presses
int buttonState = 0;         // current state of the mode button
int lastButtonState = 0;     // previous state of the mode button
int buttonStepperPushCounter = 0;   // counter for the number of stepper button presses
int buttonStepperState = 0;         // current state of the stepper button
int lastButtonStepperState = 0;     // previous state of the stepper button
int potpinwaist = A0;  // analog pin used to connect the waist potentiometer
int potvalwaist;    // variable to read the value from the waist potentiometer
int potpinshoulder = A1;  // analog pin used to connect the shoulder potentiometer
int potvalshoulder;    // variable to read the value from the shoulder potentiometer
int potpinelbow = A2; // analog pin used to connect the elbow potentiometer
int potvalelbow; // variable to read the value from the elbow potentiometer
int potpingripper = A3; // analog pin used to connect the gripper potentiometer
int potvalgripper; // variable to read the value from the gripper potentiometer
const int  buttonPin = 8;    // the pin that the mode pushbutton is attached to
const int  buttonStepperPin = 13;    // the pin that the stepper pushbutton is attached to
const int stepPin = 9; //stepper motor step pin number
const int dirPin = 10; //stepper motor dir pin number
const int trigPin = 12; //ultrasonic sensor trig pin number
const int echoPin = 11; //ultrasonic sensor echo pin number
const int ultrasonicLED = 7; //ultrasonic sensor LED pin number
const int waist = 1; // Waist(Round) servo at port 1 on PCA9685
const int shoulder = 3; // Shoulder(Front_Back) servo at port 3 on PCA9685
const int elbow = 5;  // Elbow(Up_Down) servo at port 5 on PCA9685
const int gripper = 6; // Gripper(Open_Close) servo at port 6 on PCA9685
int i, distance, R, G, B; //initializing some variables that would be used later
int frequency = 0;
long duration;
static int mode = 0;
String color;

void setup() {
  //PinModes
  pinMode(S0, OUTPUT); //color sensor
  pinMode(S1, OUTPUT); //color sensor
  pinMode(S2, OUTPUT); //color sensor
  pinMode(S3, OUTPUT); //color sensor
  pinMode(sensorOut, INPUT); //color sensor
  digitalWrite(S0, HIGH); // color sensor
  digitalWrite(S1, LOW); //color sensor
  pinMode(ultrasonicLED, OUTPUT); //ultrasonic sensor
  pinMode(buttonPin, INPUT); // initialize the button pin as a input
  pinMode(buttonStepperPin, INPUT); // initialize the button pin as a input
  pinMode(stepPin, OUTPUT); //stpper motor
  pinMode(dirPin, OUTPUT); //stepper motor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(1);
  Serial.begin(9600);     // Set the boud rate to 9600

  mode = 1;  // Default state of operation

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
}

void loop() {
  buttonState = digitalRead(buttonPin); // read the pushbutton input pin
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
      Serial.print("number of mode button pushes: ");
      Serial.println(buttonPushCounter);
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;

  // changes robotic arm mode every button push by checking the modulo of the
  // button push counter. the modulo function gives you the remainder of the
  // division of two numbers:
  if (buttonPushCounter % 2 == 0) {
    mode = 1;
    Serial.println("Manual mode activated");
  } else {
    mode = 2;
    Serial.println("Automatic mode activated");
  }

  // If performing manual control use this
  if (mode == 1) {
    buttonStepperState = digitalRead(buttonStepperPin); // read the stepper pushbutton input pin
    // compare the stepper buttonState to its previous state
    if (buttonStepperState != lastButtonStepperState) {
      // if the state has changed, increment the counter
      if (buttonStepperState == HIGH) {
        // if the current state is HIGH then the button went from off to on:
        buttonStepperPushCounter++;
        Serial.print("number of stepper button pushes: ");
        Serial.println(buttonStepperPushCounter);
      }
      // Delay a little bit to avoid bouncing
      delay(50);
    }
    // save the current state as the last stepper state, for next time through the loop
    lastButtonStepperState = buttonStepperState;
    // initially the stepper motor moves the belt forward
    if (buttonStepperPushCounter == 0) {
      digitalWrite(dirPin, HIGH);
      for (int x = 0; x < 200; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);
      }
    }
    //when it is clicked the belt stops
    if (buttonStepperPushCounter == 1) {
      buttonStepperPushCounter = 1;
    }
    //when clicked again, the belt reverses
    if (buttonStepperPushCounter == 2) {
      digitalWrite(dirPin, LOW);
      for (int x = 0; x < 200; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);
      }
    }
    //if the button has been pushed 3 times, the button resets to 0 and the loop continues
    if (buttonStepperPushCounter > 2) {
      buttonStepperPushCounter = 0;
    }
    //Ultrasonic Sensor
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    // Prints the distance on the Serial Monitor and LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.print(distance);
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance > 15) {
      // If an object is not sensed, nothing happens, light is off
      digitalWrite(ultrasonicLED, LOW);
    }
    else {
      // If an object is sensed, the belt stops
      // LED turns on
      digitalWrite(ultrasonicLED, HIGH);
      // Objects color is read and printed on serial monitor and LCD
      color = readcolor();
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(color);
      lcd.print(" ");
      Serial.print(color);
      Serial.println("  ");
    }
    potvalwaist = analogRead(potpinwaist);            // reads the value of the waist potentiometer (value between 0 and 1023)
    potvalwaist = map(potvalwaist, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    pwm.setPWM(waist, 0, angletopulse(potvalwaist));                // sets the waist servo position according to the scaled value
    delay(15);                           // waits for the servo to get there
    potvalshoulder = analogRead(potpinshoulder);            // reads the value of the shoulder potentiometer (value between 0 and 1023)
    potvalshoulder = map(potvalshoulder, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    pwm.setPWM(shoulder, 0, angletopulse(potvalshoulder));                // sets the shoulder servo position according to the scaled value
    delay(15);                           // waits for the servo to get there
    potvalelbow = analogRead(potpinelbow);            // reads the value of the elbow potentiometer (value between 0 and 1023)
    potvalelbow = map(potvalelbow, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    pwm.setPWM(elbow, 0, angletopulse(potvalelbow));                // sets the elbow servo position according to the scaled value
    delay(15);
    potvalgripper = analogRead(potpingripper);            // reads the value of the grippert potentiometer (value between 0 and 1023)
    potvalgripper = map(potvalgripper, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    pwm.setPWM(gripper, 0, angletopulse(potvalgripper));                // sets the gripper servo position according to the scaled value
    delay(15);
  }

  // If performing automatic control use this
  if (mode == 2) {
    Serial.println("Starting auto mode");
    //Conyeyor belt
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    // Prints the distance on the Serial Monitor and LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.print(distance);
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance > 15) {
      // If an object is not sensed, keep the conveyor belt in motion
      digitalWrite(ultrasonicLED, LOW);
      digitalWrite(dirPin, HIGH);
      for (int x = 0; x < 200; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);
      }
    }
    else {
      // If an object is sensed, the belt stops
      // LED turns on
      delay(2000);
      digitalWrite(ultrasonicLED, HIGH);
      // Objects color is read and printed on serial monitor and LCD
      color = readcolor();
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(color);
      lcd.print(" ");
      Serial.print(color);
      Serial.println("  ");
      currentMillis = millis();
      //Belt moves small distance (quater stepper motor cycle) to allow room for robotic arm to pick object
      if (currentMillis - previousMillis >= interval) {
        digitalWrite(dirPin, HIGH);
        for (int x = 0; x < 50; x++) {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(2000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(2000);
        }
        previousMillis = millis();
      }
      if (color == "Red") {
        // Robot moves specific positions based on color sensor's output. For Red:
        for (int i = 100; i < 110; i++) {
          pwm.setPWM(elbow, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i < 180; i++) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 70; i < 110; i++) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 180; i > 100; i--) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 110; i > 70; i--) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 110; i > 100; i--) {
          pwm.setPWM(elbow, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 0; i < 170; i++) {
          pwm.setPWM(waist, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 70; i < 100; i++) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i > 180; i++) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 180; i > 100; i--) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i > 70; i--) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 170; i > 0; i--) {
          pwm.setPWM(waist, 0, angletopulse(i));
          delay(40);
        }
      }
      if (color == "Green") {
        // Robot moves specific positions based on color sensor's output. For Green:
        for (int i = 100; i < 110; i++) {
          pwm.setPWM(elbow, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i < 180; i++) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 70; i < 110; i++) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 180; i > 100; i--) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 110; i > 70; i--) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 110; i > 100; i--) {
          pwm.setPWM(elbow, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 0; i < 140; i++) {
          pwm.setPWM(waist, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 70; i < 100; i++) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i > 180; i++) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 180; i > 100; i--) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i > 70; i--) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 140; i > 0; i--) {
          pwm.setPWM(waist, 0, angletopulse(i));
          delay(40);
        }
      }
      if (color == "Blue") {
        // Robot moves specific positions based on color sensor's output. For Blue:
        for (int i = 100; i < 110; i++) {
          pwm.setPWM(elbow, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i < 180; i++) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 70; i < 110; i++) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 180; i > 100; i--) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 110; i > 70; i--) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 110; i > 100; i--) {
          pwm.setPWM(elbow, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 0; i < 100; i++) {
          pwm.setPWM(waist, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 70; i < 100; i++) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i > 180; i++) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 180; i > 100; i--) {
          pwm.setPWM(gripper, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i > 70; i--) {
          pwm.setPWM(shoulder, 0, angletopulse(i));
          delay(40);
        }
        for (int i = 100; i > 0; i--) {
          pwm.setPWM(waist, 0, angletopulse(i));
          delay(40);
        }
      }
    }
  }
}
//Function that converts angle to pulses for PCA9685 motor driver
int angletopulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

//Function that checks color of object using the color sensor
String readcolor() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //mapping output frequency from 0 to 255
  R = map(frequency, Rw, Rb, 0, 255);
  //Prints mapped value to serial monitor
  Serial.print("R= ");
  Serial.print(R);
  Serial.println("  ");
  delay(50);
  // Setting green filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //mapping output frequency from 0 to 255
  G = map(frequency, Gw, Gb, 0, 255);
  //Prints mapped value to serial monitor
  Serial.print("G= ");
  Serial.print(G);
  Serial.println("  ");
  delay(50);
  // Setting green filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //mapping output frequency from 0 to 255
  B = map(frequency, Bw, Bb, 0, 255);
  //Prints mapped value to serial monitor
  Serial.print("B= ");
  Serial.print(B);
  Serial.println("  ");
  delay(50);

  //Depending on different color frequencies, conditions are used to determine whether object is red, green or blue.
  //If red is less than both green and blue, color must be red
  if (R < G && R < B) {
    color = "Red"; // Red
  }
  //If green is less than both red and blue, color must be green 
  if (G < R && G < B) {
    color = "Green"; // Green
  }
  //If blue is less than both green and red, color must be red
  if (B < R && B < G) {
    color = "Blue"; // Blue
  }
  return color;
}
