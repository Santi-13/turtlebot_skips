#include <WiFi.h>

// Configuración de red Wi-Fi
const char* ssid     = "TurtleBot4";
const char* password = "TurtleBot4";

// Dirección IP y puerto del servidor TCP (ESP32 actuando como servidor)
WiFiServer server(80);

// Variable para almacenar el ángulo de orientación (theta)
String receivedTheta = "";

void setup() {
  // Iniciar la comunicación serie
  Serial.begin(115200);

  // Conectarse a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");

  // Iniciar el servidor TCP
  server.begin();
  Serial.println("Servidor TCP iniciado");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Escuchar clientes entrantes
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Cliente conectado");

    // Mientras el cliente esté conectado
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {  // Si se recibe un salto de línea, significa que se ha enviado todo el dato
          Serial.print("Ángulo de orientación (theta) recibido: ");
          Serial.println(receivedTheta);

          // Aquí puedes realizar alguna acción con el ángulo recibido
          // Por ejemplo, convertirlo en número y controlarlo
          float theta = receivedTheta.toFloat();
          // Realizar alguna acción con el ángulo, como controlar un LED, servo, etc.
          // ...
          
          // Limpiar la variable para la siguiente recepción
          receivedTheta = "";
        } else {
          // Acumular los caracteres recibidos
          receivedTheta += c;
        }
      }
    }

    // Cerrar la conexión
    client.stop();
    Serial.println("Cliente desconectado");
  }
}
