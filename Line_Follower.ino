#define EN1 4
#define IN1 5
#define IN2 6
#define EN2 8
#define IN3 9
#define IN4 10

const int sensorCount = 5;
bool sensorValues[sensorCount];
const int sensorPins[] = {14, 15, 16, 17, 18};
bool allActive = true;
bool allDeactive = true; 

int position, error, previousError;
int setpoint = 2000; 
const float Kp = 0;
const float Ki = 0;
const float Kd = 0;

int P, I, D, PIDvalue; 
int leftMotorSpeed, rightMotorSpeed;
const int normalSpeed = 230;


void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  for(int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);

  Serial.begin(9600);
}

void loop() {
  readSensors();
  position = getPosition();
  error = setpoint - position;
  
  if(allActive) {
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
  } else if(allDeactive && previousError >= 0) {
    leftMotorSpeed = (-1) * normalSpeed;
    rightMotorSpeed = normalSpeed;
  } else if(allDeactive && previousError < 0) {
    leftMotorSpeed = normalSpeed;
    rightMotorSpeed = (-1) * normalSpeed;
  } else {
    PIDController();
  }

  drive();
  log();
  delay(2000);
}

void PIDController() {
  P = error; 
  I = I + error; 
  D = error - previousError;
  PIDvalue = Kp*P + Ki*I + Kd*D; 
  previousError = error;
  leftMotorSpeed = constrain(normalSpeed - PIDvalue, -255, 255);
  rightMotorSpeed = constrain(normalSpeed + PIDvalue, -255, 255);
}

void readSensors() {
  noInterrupts();
  for(int i = 0; i < sensorCount; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
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
  return sum/activeCount;
}

void drive() {
  if(rightMotorSpeed == 0) {
    digitalWrite(EN1, LOW);
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  } else if(rightMotorSpeed > 0) {
    analogWrite(IN1, rightMotorSpeed);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, rightMotorSpeed);
  }
 
  if(leftMotorSpeed == 0) {
    digitalWrite(EN2, LOW);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  } else if(leftMotorSpeed > 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, leftMotorSpeed);
  } else {
    analogWrite(IN3, leftMotorSpeed);
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
  Serial.println(Kp*P);
  Serial.println("--------------------------------------------------");
}
