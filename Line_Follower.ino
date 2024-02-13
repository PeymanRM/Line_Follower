#define EN1 4
#define IN1 5
#define IN2 6
#define EN2 8
#define IN3 9
#define IN4 10

const int sensorCount = 5;
bool sensorValues[sensorCount] ;
const int sensorPins[] = {14, 15, 16, 17, 18};
bool allActive = true; // Flag for detecting when all sensors detect the line
bool allDeactive = true; // Flag for detecting when no sensor detects the line

int position, error, previousError;
int setpoint = 2000; // Represents the middle of our proccess variable: position
float Kp = 0; // Proportional Gain
float Ki = 0; // Integral Gain
float Kd = 0; // Derivative Gain

int P, I, D, PIDvalue; 
int leftMotorSpeed, rightMotorSpeed;
const int normalSpeed = 230; //Standard speed of the line follower
const int numChars = 7;
char recievedChars[numChars];


void setup() {
  // Setting pin modes
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  for(int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Enabling Motors
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);

  // Beginning serial com
  Serial.begin(9600);
}

void loop() {
  getGains();
  readSensors(); // Stores digital sensor values in sensorValues[]
  position = getPosition(); // Gives approximated position of the line using weighted average in range of 0 to (sensorCount-1)*1000
  error = setpoint - position;
  
  if(allActive) { // End of track
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
  } else if(allDeactive && previousError >= 0) { // Out of Line
    leftMotorSpeed = (-1) * normalSpeed;
    rightMotorSpeed = normalSpeed;
  } else if(allDeactive && previousError < 0) {
    leftMotorSpeed = normalSpeed;
    rightMotorSpeed = (-1) * normalSpeed;
  } else { // Line somewhere in the middle
    PIDController();
  }

  drive();
  log();
  delay(2000);
}

void PIDController() {
  P = error; // e(t) = error
  I = I + error; // Running sum (Integral) of error
  D = error - previousError; // Differene (Derivitive) of error
  PIDvalue = Kp*P + Ki*I + Kd*D; 
  previousError = error;
  leftMotorSpeed = constrain(normalSpeed - PIDvalue, -255, 255);
  rightMotorSpeed = constrain(normalSpeed + PIDvalue, -255, 255);
}

void readSensors() {
  noInterrupts();
  for(int i = 0; i < sensorCount; i++) {
    sensorValues[i] = !digitalRead(sensorPins[i]);
  }
  interrupts();
}

int getPosition() {
  int sum = 0;
  int activeCount = 0; 
  for(int i = 0; i < sensorCount; i++) {
    sum += sensorValues[i] * i * 1000;
    if(sensorValues[i]) {
      activeCount++;
      allDeactive = false; 
    } else if(allActive) {
      allActive = false;
    }
  }
  return sum/activeCount; // Weighted average of sensors
}

void drive() {
  if(rightMotorSpeed == 0) { // Stop
    digitalWrite(EN1, LOW);
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  } else if(rightMotorSpeed > 0) { // Forward
    analogWrite(IN1, rightMotorSpeed);
    analogWrite(IN2, 0);
  } else { // Backward
    analogWrite(IN1, 0);
    analogWrite(IN2, (-1) * rightMotorSpeed);
  }
 
  if(leftMotorSpeed == 0) { // Stop
    digitalWrite(EN2, LOW);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  } else if(leftMotorSpeed > 0) { // Forward
    analogWrite(IN3, 0);
    analogWrite(IN4, leftMotorSpeed);
  } else { // Backward
    analogWrite(IN3, (-1) * leftMotorSpeed);
    analogWrite(IN4, 0);
  }
}

void log() {
  Serial.print("Sensor Values: ");
  for(int i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  Serial.print("Motors: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" ");
  Serial.println(rightMotorSpeed);
  Serial.println("--------------------------------------------------");
}

void getGains() {
  if(Serial.available()) {
    Serial.read();
    Serial.flush();
    while(!Serial.available()) {}  
    Serial.readBytes(recievedChars, numChars);
    Kp = atof(recievedChars);
    Serial.print("Kp is set to: ");
    Serial.println(Kp, 4);

    while(!Serial.available()) {}  
    Serial.readBytes(recievedChars, numChars);
    Ki = atof(recievedChars);
    Serial.print("Ki is set to: ");
    Serial.println(Ki, 4);

    while(!Serial.available()) {}  
    Serial.readBytes(recievedChars, numChars);
    Kd = atof(recievedChars);
    Serial.print("Kd is set to: ");
    Serial.println(Kd, 4);
  }
}