#include <M5Unified.h>
#include <M5GFX.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#define VERSION "1.0.0"

#define SDIO2_CLK GPIO_NUM_12
#define SDIO2_CMD GPIO_NUM_13
#define SDIO2_D0  GPIO_NUM_11
#define SDIO2_D1  GPIO_NUM_10
#define SDIO2_D2  GPIO_NUM_9
#define SDIO2_D3  GPIO_NUM_8
#define SDIO2_RST GPIO_NUM_15

// ⚡ Paramètres WiFi
const char* ssid     = "*************";
const char* password = "*************";

// Serveur web async sur port 80
AsyncWebServer server(80);

void setup() {
  M5.begin();
  Serial.begin(115200);

  M5.Display.setFont(&fonts::FreeMonoBoldOblique24pt7b);

  
  if (!LittleFS.begin(true)) {
      M5.Display.println("Error LittleFS");
      return;
  }

  File root = LittleFS.open("/");
  File file = root.openNextFile();

  while (file)
  {
    M5.Display.print("File: ");
    M5.Display.println(file.name());

    file.close();
    file = root.openNextFile();

  }

  WiFi.setPins(SDIO2_CLK, SDIO2_CMD, SDIO2_D0, SDIO2_D1, SDIO2_D2, SDIO2_D3, SDIO2_RST);

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

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
    M5.Display.print("Request main page");
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/style.css", "text/css");
  });

  server.begin();
}

void loop() {
  M5.update();
}
