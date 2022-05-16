#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <SoftwareSerial.h>
#include <ctype.h>

#define DEBUG 0
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define RXp2 16
#define TXp2 17

String message = "";

// Replace with your network credentials
const char *ssid = "Hotspot";
const char *password = "Rebel1x1";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Create a software serial object
SoftwareSerial s;

// Variables to save values from HTML form
String routes;
String deliveries;
String heights = "";
String state = "";

// Initialize WiFi
void initWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.println(WiFi.localIP());
}

void notifyClients(String state)
{
  ws.textAll(state);
}

boolean isValidNumber(String str)
{
  for (byte i = 0; i < str.length(); i++)
  {
    if (isDigit(str.charAt(i)))
      return true;
  }
  return false;
}

bool stringIsLowercase(String s, int n)
{
  for (int i = 0; i < n; i++)
  {
    if (!islower(s.charAt(i)))
    {
      return false;
    }
  }
  return true;
}

void SameLengthRH(String *heights, int n)
{
  if (heights->length() == n)
  {
    return;
  }
  else
  {
    for (int i = heights->length(); i < n; i++)
    {
      *heights += "0";
    }
    return;
  }
}
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    message = (char *)data;
    deliveries = message.substring(0, message.indexOf("@"));
    routes = message.substring(message.indexOf("@") + 1, message.indexOf("&"));
    heights = message.substring(message.indexOf("&") + 1, message.length());
    // Conditions that Enforce the Message format being send to the Robot

    SameLengthRH(&heights, deliveries.toInt());

    if (isValidNumber(deliveries) && stringIsLowercase(routes, deliveries.toInt()))
    {
      if (deliveries.toInt() < 9 && deliveries.toInt() > 0)
      {
        debug("# of Deliveries: ");
        debugln(deliveries);
        debug("Routes: ");
        debugln(routes);
        debug("Heights: ");
        debugln(heights);
        debugln("Message Sent over UART:");
        debugln(deliveries + routes + heights);
        Serial2.println(deliveries + routes + heights);

        notifyClients(state);
      }
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    // Notify client of the Current State of The Robot
    notifyClients(state);
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  else
  {
    Serial.println("SPIFFS mounted successfully");
  }
}

void handleReceivedMessage(String msg)
{
  // debug("String: ");
  // //  debugln(msg);
  // if (msg.toInt() == 6)
  // {
  //   Serial.println(msg);
  // }
  Serial.println(msg);
  notifyClients(msg);
}

void handleSerial()
{
  // Read serial input:
  while (Serial2.available() > 0)
  {
    int inChar = Serial2.read();
    if (isDigit(inChar))
    {
      // convert the incoming byte to a char and add it to the string:
      state += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n')
    {

      handleReceivedMessage(state);
      // clear the string for new input:
      state = "";
    }
  }
}
void setup()
{
  // Serial port for debugging purposes

  Serial.begin(115200);
  Serial2.begin(57600);
  // s.begin(57600, SWSERIAL_8N1, RXp2, TXp2);
  // s.enableIntTx(false);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  initWiFi();
  initWebSocket();
  initSPIFFS();
  digitalWrite(2, HIGH);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.serveStatic("/", SPIFFS, "/");

  server.begin();
}

void loop()
{
  handleSerial();
  ws.cleanupClients();
}
