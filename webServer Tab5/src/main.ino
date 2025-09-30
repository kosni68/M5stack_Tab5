#include <Arduino.h>
#include <M5Unified.h>
#include <M5GFX.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#define SDIO2_CLK GPIO_NUM_12
#define SDIO2_CMD GPIO_NUM_13
#define SDIO2_D0  GPIO_NUM_11
#define SDIO2_D1  GPIO_NUM_10
#define SDIO2_D2  GPIO_NUM_9
#define SDIO2_D3  GPIO_NUM_8
#define SDIO2_RST GPIO_NUM_15

// ⚡ Paramètres WiFi
const char* ssid     = "...";
const char* password = "...";

// Serveur web async sur port 80
AsyncWebServer server(80);

// Page HTML
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>M5Stack Tab5</title>
  <style>
    body { font-family: Arial; text-align: center; margin-top: 50px; }
    button { font-size: 20px; padding: 15px; margin: 10px; }
  </style>
</head>
<body>
  <h1>Serveur Web M5Stack Tab5</h1>
  <button onclick="fetch('/on')">LED ON</button>
  <button onclick="fetch('/off')">LED OFF</button>
</body>
</html>
)rawliteral";

void setup() {
  M5.begin();
  Serial.begin(115200);

  M5.Display.setFont(&fonts::FreeMonoBoldOblique24pt7b);

  WiFi.setPins(SDIO2_CLK, SDIO2_CMD, SDIO2_D0, SDIO2_D1, SDIO2_D2, SDIO2_D3, SDIO2_RST);

  // If you select the M5Tab5 board in Arduino IDE, you could use the default pins defined.
  // WiFi.setPins(BOARD_SDIO_ESP_HOSTED_CLK, BOARD_SDIO_ESP_HOSTED_CMD, BOARD_SDIO_ESP_HOSTED_D0,
  //              BOARD_SDIO_ESP_HOSTED_D1, BOARD_SDIO_ESP_HOSTED_D2, BOARD_SDIO_ESP_HOSTED_D3,
  //              BOARD_SDIO_ESP_HOSTED_RESET);

  // STA MODE
  WiFi.mode(WIFI_STA);
  M5.Display.println("WiFi mode set to STA");
  WiFi.begin(ssid, password);
  M5.Display.print("Connecting to ");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      M5.Display.print(".");
  }
  M5.Display.println("");
  M5.Display.print("Connected to ");
  M5.Display.println(ssid);
  M5.Display.print("IP address: ");
  M5.Display.println(WiFi.localIP());

  // AP MODE
  // WiFi.mode(WIFI_MODE_AP);
  // Serial.println("WiFi mode set to AP");
  // WiFi.softAP(ssid, password);
  // Serial.println("AP started");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.softAPIP())

  // Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){
    // Ici tu pourrais piloter un GPIO
    request->send(200, "text/plain", "LED ON");
  });

  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){
    // Ici aussi
    request->send(200, "text/plain", "LED OFF");
  });

  server.begin();
}

void loop() {
  M5.update();
}
