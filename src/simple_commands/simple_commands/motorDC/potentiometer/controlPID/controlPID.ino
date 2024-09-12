int inputPin = 26;
int anglePot = 27;
int pinA = 33; // IN1
int pinB = 25; // IN2
int voltageADC = 0;
float integral = 0;
float derivative = 0;

// Angulos
int thetaRef = 90;
float thetaMin = 0;
float thetaMax = 180;
int theta = 0;
int thetaError = 0;
int prevThetaError = 0;

// Ganancias
int Kp = 1;
int Ki = 0.9;
int Kd = 0.2;

// Control
int u = 0;
int v = 0;
int signOfV = 0;
int M = 100;
int timeOfOperation = 0;
int currTime = 0;
int prevTime = 0;


int sign(int num) {
  if (num > 0) return 1;
  if (num < 0) return -1;
  return 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  voltageADC = analogRead(inputPin);
  thetaRef = (float(voltageADC) / 1024.0) * (thetaMax - thetaMin);
  // Serial.println(voltageADC);
  // Serial.println(theta);
  thetaError = thetaRef - theta;
  Serial.println(thetaError);

  currTime = millis();
  timeOfOperation = currTime - prevTime;
  prevTime = currTime;

  derivative = (thetaError - prevThetaError) / timeOfOperation;
  prevThetaError = thetaError;
  integral += thetaError * timeOfOperation;
  u = Kp * thetaError + Kd * derivative + Ki * integral;
  v = abs(u);
  signOfV = sign(u);

  if (v > M) {
    v = M;
  }

  for (int i = 0; i <= M; i++) {
    if(signOfV == 1) {
      if (i <= v) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
      } else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
      }
    } else if(signOfV == -1) {
      if (i <= v) {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
      } else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
      }
    } else {
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, LOW);
    }
    delay(1);
  }
  theta = float(analogRead(anglePot)) / 1024.0 * thetaMax;
}