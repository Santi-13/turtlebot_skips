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
float lambda = -200.0;
float k = 2.0;

// Control
float u = 0;
float s = 0;
float ds = 0;
float signOfS = 0;
int M = 400;
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
  thetaRef = (float(voltageADC) / 4095.0) * (thetaMax - thetaMin);
  // Serial.println(voltageADC);
  // Serial.println(theta);
  thetaError = thetaRef - theta;
  Serial.println(thetaError);

  currTime = millis();
  timeOfOperation = currTime - prevTime;
  prevTime = currTime;

  s = thetaError;
  ds = (s - prevThetaError) / timeOfOperation;
  prevThetaError = s;

  u = -lambda * sqrt(abs(s)) * sign(s) - k * sign(ds);
  int v = abs(u);
  signOfS = sign(u);
  // Serial.println(u);

  if (v > M) {
    v = M;
  }

  for (int i = 0; i <= M; i++) {
    if(signOfS == 1) {
      if (i <= v) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
      } else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
      }
    } else if(signOfS == -1) {
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
  theta = float(analogRead(anglePot)) / 4095.0 * thetaMax;
}