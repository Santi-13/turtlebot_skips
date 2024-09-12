// 150:1 Micro Metal Gearmotor HP 6V with Extended Motor Shaft
// https://www.pololu.com/product/2386
// 240 rpm max
// Definición de pines
const int PinIn1 = 25;  // Pin de dirección IN1 del motor
const int PinIn2 = 35;  // Pin de dirección IN2 del motor
const int motorPWM = 27;   // Pin de control PWM del motor
const int encoderPin = 39; // Pin del encoder

// Variables para el control PID
float velref = 0.0;        // Velocidad inicial deseada en PPS (comienza en 0)
float velObjetivo = 400.0; // Velocidad objetivo deseada en PPS (250 PPS)
float incremento = 1.0;    // Incremento de velocidad gradual
float velActual = 0.0;     // Velocidad actual del motor
// gear_motor_operation = 150 
// una vuelta completa del iman equivale a 3 pulsos del encoder.
// resolucion del encoder = 3 * gear_motor_operation = 450 pulsos por revolución, es decir, por cada 450 pulsos del encoder, el eje de salida del motor gira 360 grados.   
// RPS_revoluciones por segundo = PPS_ref/resolucion del encoder
// RPM_revoluciones por minuto = RPS * 60 
float Kp = 1.0;             // Ganancia proporcional
float Ki = 0.1;             // Ganancia integral
float Kd = 0.01;            // Ganancia derivativa
float errorIntegral = 0.0;  // Integral del error
float errorAnterior = 0.0;  // Último error calculado
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

    // Incrementar gradualmente la velocidad de referencia hasta alcanzar la velocidad objetivo
    if (velref < velObjetivo) {
      velref += incremento;
      if (velref > velObjetivo) velref = velObjetivo;  // Limitar a la velocidad objetivo
    }

    // Calcular el error
    float error = velref - velActual;

    // Calcular el término integral
    errorIntegral += error * (tiempoTranscurrido / 1000.0);  // Integral del error

    // Calcular el término derivativo
    float errorDerivativo = (error - errorAnterior) / (tiempoTranscurrido / 1000.0);

    // Señal de control PID
    float u = Kp * error + Ki * errorIntegral + Kd * errorDerivativo;

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

    // Guardar el último error para el próximo ciclo
    errorAnterior = error;

    // Enviar datos al Serial Plotter
    Serial.print("velActual:");
    Serial.print(velActual);
    Serial.print(" velref:");
    Serial.print(velref);
    Serial.print(" error:");
    Serial.print(error);
    Serial.print(" pwmValue:");
    Serial.println(pwmValue);
  }
}
