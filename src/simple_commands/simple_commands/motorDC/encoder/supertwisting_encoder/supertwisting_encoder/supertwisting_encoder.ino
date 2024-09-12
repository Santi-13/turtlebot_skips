// Definición de pines
const int PinIn1 = 25;  // Pin de dirección IN1 del motor
const int PinIn2 = 35;  // Pin de dirección IN2 del motor
const int motorPWM = 27;   // Pin de control PWM del motor
const int encoderPin = 39; // Pin del encoder

// Variables para el control Super-Twisting
float velref = 400.0;       // Velocidad deseada (ajusta según sea necesario)
float velActual = 0.0;      // Velocidad actual del motor
float u = 0.0;              // Señal de control
float integralSignE = 0.0;  // Integral del signo del error
float e_prev = 0.0;         // Error anterior
unsigned long tiempoAnterior = 0; // Tiempo del último ciclo
volatile int conteoEncoder = 0; // Contador de pulsos del encoder
int conteoEncoderAnterior = 0;   // Contador anterior del encoder

// ISR para contar los pulsos del encoder
void IRAM_ATTR encoderISR() {
  conteoEncoder++;
}

void setup() {
  Serial.begin(9600);  // Inicializar comunicación serial

  // Configuración de pines
  pinMode(PinIn1, OUTPUT);
  pinMode(PinIn2, OUTPUT);
  
  // Configuración del PWM
  ledcSetup(0, 5000, 8); // Canal 0, 5 kHz, 8 bits de resolución
  ledcAttachPin(motorPWM, 0); // Asignar el pin de control PWM al canal 0

  pinMode(encoderPin, INPUT);

  // Interrupción para contar pulsos del encoder
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
  
  // Asegurarte de que el motor esté detenido al inicio
  ledcWrite(0, 0); // Establecer PWM a 0
  digitalWrite(PinIn1, LOW);
  digitalWrite(PinIn2, LOW);

  tiempoAnterior = millis();  // Inicializar el tiempo del último ciclo
}

void loop() {
  // Calcular la velocidad del motor en pulsos por segundo
  unsigned long tiempoActual = millis();
  unsigned long tiempoTranscurrido = tiempoActual - tiempoAnterior;

  if (tiempoTranscurrido >= 100) {  // Actualización de velocidad cada 100 ms
    velActual = (conteoEncoder - conteoEncoderAnterior) * (1000.0 / tiempoTranscurrido);
    conteoEncoderAnterior = conteoEncoder;
    tiempoAnterior = tiempoActual;

    // Calcular el error
    float error = velref - velActual;

    // Termino 1: Ganancia proporcional ajustada
    float k_ps = 3.0; // Aumentar la ganancia para respuesta más rápida
    float term1 = k_ps * sign(error) * sqrt(abs(error));

    // Termino 2: Integral del signo del error
    integralSignE += sign(error) * (tiempoTranscurrido / 1000.0); // Integral con respecto al tiempo
    float k_is = 28.0; // Aumentar la ganancia integral para respuesta más rápida
    float term2 = k_is * integralSignE;
    // Señal de control
    u = term1 + term2;

    // Saturar la señal de control a los valores de PWM (0-255)
    int pwmValue = constrain(abs(u), 0, 255);

    // Controlar la dirección del motor
    if (u > 0) {
      digitalWrite(PinIn1, HIGH);
      digitalWrite(PinIn2, LOW);  // Motor hacia adelante
    } else {
      digitalWrite(PinIn1, LOW);
      digitalWrite(PinIn2, HIGH);  // Motor hacia atrás
    }

    // Aplicar la señal PWM al motor
    ledcWrite(0, pwmValue); // Enviar el valor de PWM al canal 0

    // Enviar datos al Serial Plotter
    Serial.print("velActual:");
    Serial.print(velActual);
    Serial.print(" velref:");
    Serial.print(velref);
    Serial.print(" error:");
    Serial.print(error);
    Serial.print(" pwmValue:");
    Serial.println(pwmValue);

    // Guardar error anterior para el próximo ciclo
    e_prev = error;
  }
}

// Función signo
int sign(float x) {
  if (x > 0) return 1;
  else if (x < 0) return -1;
  else return 0;
}