## Control a brushless motor with esp32 using web server 
### 0.1 Control IoT Interface for a ESP32 Drone

#include <WiFi.h>
#include <ESP32Servo.h>

// ======================== CONFIGURAÇÕES Wi-Fi ========================
const char* ssid = "K MIGUEL";      // 🔹 nome da rede Wi-Fi
const char* password = "13761730";  // 🔹 senha

// ======================== CONFIGURAÇÕES DO ESC ========================
Servo esc;
const int escPin = 18;  // pino de sinal do ESC
int speedValue = 1000;  // valor inicial (mínimo)
WiFiServer server(80);  // servidor HTTP

// ======================== HTML DA PÁGINA ========================
String getHTML() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta charset="UTF-8">
    <title>Controle ESC - ESP32</title>
    <style>
      body { font-family: Arial; text-align: center; background: #111; color: white; }
      a { display: inline-block; margin: 20px; padding: 15px 30px; background: #0f0; color: black; text-decoration: none; border-radius: 10px; font-weight: bold; }
      a.off { background: red; color: white; }
    </style>
  </head>
  <body>
    <h2> Controle do Motor Brushless</h2>
    <a href="/ON">LIGAR</a>
    <a href="/OFF" class="off">DESLIGAR</a>
  </body>
  </html>
  )rawliteral";
  return html;
}

void setup() {
  Serial.begin(115200);
  esc.attach(escPin, 1000, 2000);
  esc.writeMicroseconds(1000);

  Serial.println("Conectando ao Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.print("Acesse: http://");
  Serial.println(WiFi.localIP());

  server.begin();
  delay(3000);  // tempo para armar o ESC
  Serial.println("ESC armado!");
}

// ======================== LOOP PRINCIPAL ========================
void loop() {
  WiFiClient client = server.available();
  if (!client) return;

  String req = client.readStringUntil('\r');
  client.flush();

  if (req.indexOf("/ON") != -1) {
    speedValue = 1250;                 // valor máximo (acelera)
    esc.writeMicroseconds(speedValue);
    Serial.println("Motor LIGADO!");
  } 
  else if (req.indexOf("/OFF") != -1) {
    speedValue = 1000;                 // valor mínimo (para)
    esc.writeMicroseconds(speedValue);
    Serial.println("Motor DESLIGADO!");
  }

  // Resposta com a página HTML
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.print(getHTML());
  delay(1);
}
