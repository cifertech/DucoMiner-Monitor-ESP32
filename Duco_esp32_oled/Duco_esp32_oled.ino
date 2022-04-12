const char *wifi_ssid = "C1F3R";
const char *wifi_password = "314159265";
const char *username = "CiferTech";
const char *rig_identifier = "None";

#define LED_BUILTIN 2
#define BLINK_SHARE_FOUND    1
#define BLINK_SETUP_COMPLETE 2
#define BLINK_CLIENT_CONNECT 3
#define BLINK_RESET_DEVICE   5

#define WDT_TIMEOUT 60

#include "hwcrypto/sha.h"
//#include "sha/sha_parallel_engine.h"  

/* If you would like to use mqtt monitoring uncomment
   the ENABLE_MQTT defition line(#define ENABLE_MQTT).
   NOTE: enabling MQTT could slightly decrease hashrate */
// #define ENABLE_MQTT
// Change this to specify MQTT server (ip only no prefixes)
const char *mqtt_server = "";
// Port mqtt server is listening at (default: 1883)
const int mqtt_port = 1883;

/* If you're using the ESP32-CAM board or other board
  that doesn't support OTA (Over-The-Air programming)
  comment the ENABLE_OTA definition line (#define ENABLE_OTA)
   NOTE: enabling OTA support could decrease hashrate (up to 40%) */
// #define ENABLE_OTA

/* If you don't want to use the Serial interface comment
  the ENABLE_SERIAL definition line (#define ENABLE_SERIAL)*/
#define ENABLE_SERIAL
/****************** END OF MINER CONFIGURATION SECTION ******************/

#ifndef ENABLE_SERIAL
#define Serial DummySerial
static class {
 public:
  void begin(...) {}
  void print(...) {}
  void println(...) {}
  void printf(...) {}
} Serial;
#endif

#ifndef ENABLE_MQTT
#define PubSubClient DummyPubSubClient
class PubSubClient{
 public:
  PubSubClient(Client& client) {}
  bool connect(...) { return false; }
  bool connected(...) { return true; }
  void loop(...) {}
  void publish(...) {}
  void subscribe(...) {}
  void setServer(...) {}
};
#endif

// Data Structures
typedef struct TaskData
{
  TaskHandle_t handler;
  byte taskId;
  float hashrate;
  unsigned long shares;
  unsigned int difficulty;
  
} TaskData_t;

// Include required libraries

#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32 

#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>  //Include WDT libary

#ifdef ENABLE_OTA
#include <ArduinoOTA.h>
#endif

#ifdef ENABLE_MQTT
#include <PubSubClient.h>
#endif

// Global Definitions
#define NUMBEROFCORES 2
#define MSGDELIMITER ','
#define MSGNEWLINE '\n'

#define OLED_RESET     4 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Handles for additional threads
TaskHandle_t WiFirec;
TaskData_t TaskThreadData[NUMBEROFCORES];
TaskHandle_t MqttPublishHandle;
SemaphoreHandle_t xMutex;

// Internal Variables

uint32_t lastDrawTime;
uint32_t deauths = 0;       

int start;
int wait = 0;
int miners = 0;
float oldb = 0.0;
float userbalance;
float balance;
float ducomadesincesartdaily = 0.0;
float daily = 0;
String ducosmem;
String price;

String serverName = "https://server.duinocoin.com/users/";
String serverPrice = "https://server.duinocoin.com/api.json";
const char* root_ca= \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFLTCCBBWgAwIBAgISBLrlR4aCBlmofGjUwxvNkJ9IMA0GCSqGSIb3DQEBCwUA\n" \
"MDIxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQswCQYDVQQD\n" \
"EwJSMzAeFw0yMTA0MjAxMjAwMThaFw0yMTA3MTkxMjAwMThaMB8xHTAbBgNVBAMT\n" \
"FHNlcnZlci5kdWlub2NvaW4uY29tMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB\n" \
"CgKCAQEAv7SZV8V8uikoSOnw9KVa9M3HQsyB6l3QdinCuyZz7sRckOdSrUKtYFK/\n" \
"g+RmBqWVrwRLgnVmLtUlulHI7/H7Rbz0Exiv0beyjakd+p40O8V7Y1kuhE9z8WT6\n" \
"gtDd4grLNBMtUkd2Y8MmYFed2SP5OUoIdySdcUuo50NiS2KH5j3DhZB+ZmZn0lXw\n" \
"E91O2eNgEaryrnDoMftlJAUOSGh3uuCS27y1SXm8IC1Cxkv+OUyjXu9HnZoIX5v0\n" \
"59cTAFzrSo5tF/XtJlZ7fJU7VC98NaurfWKrF1XWgoKHJcjsFgep1K6PRIb7g5xq\n" \
"wlWUOFYTlDIYMAocGYJabw6jZrSphQIDAQABo4ICTjCCAkowDgYDVR0PAQH/BAQD\n" \
"AgWgMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjAMBgNVHRMBAf8EAjAA\n" \
"MB0GA1UdDgQWBBSGBC/8G5lAbUvFVjt4wL7kzsnzDjAfBgNVHSMEGDAWgBQULrMX\n" \
"t1hWy65QCUDmH6+dixTCxjBVBggrBgEFBQcBAQRJMEcwIQYIKwYBBQUHMAGGFWh0\n" \
"dHA6Ly9yMy5vLmxlbmNyLm9yZzAiBggrBgEFBQcwAoYWaHR0cDovL3IzLmkubGVu\n" \
"Y3Iub3JnLzAfBgNVHREEGDAWghRzZXJ2ZXIuZHVpbm9jb2luLmNvbTBMBgNVHSAE\n" \
"RTBDMAgGBmeBDAECATA3BgsrBgEEAYLfEwEBATAoMCYGCCsGAQUFBwIBFhpodHRw\n" \
"Oi8vY3BzLmxldHNlbmNyeXB0Lm9yZzCCAQMGCisGAQQB1nkCBAIEgfQEgfEA7wB2\n" \
"APZclC/RdzAiFFQYCDCUVo7jTRMZM7/fDC8gC8xO8WTjAAABeO9eAZAAAAQDAEcw\n" \
"RQIgXLdccQKNKjfPXVV1dbVW8LPmCKb80E8FOVBNw6WswN8CIQDerRKSvtgx4PxF\n" \
"1lOq8sFWwlMvchIkpiELDpmSQ7/6iAB1AFzcQ5L+5qtFRLFemtRW5hA3+9X6R9yh\n" \
"c5SyXub2xw7KAAABeO9eAYAAAAQDAEYwRAIgAkQ5SNxBduS8ckP7z0wEMMLdcNmf\n" \
"rR76s3MD8KIG/UQCIB4txuVGP/citMxxQYKFan1H0l4l2dYJuV4IAizWJ5GHMA0G\n" \
"CSqGSIb3DQEBCwUAA4IBAQCGELdxTgyUsqnCgE6bgOkRyAiVZHcSqNMg17DWw+ek\n" \
"IqfXFMbfuTFAs3+VRRCZS2BUiCttsmNzdiPIGgZLvyyv20RLDEjg7Kq22mudGg3F\n" \
"i/koB5DlGWWv9t2Ng/qnM/4+y6kAwpbJ8R1v4P3NpZgGFIccYgP+N41IOMT+OIzT\n" \
"6PxYyUH9yQx58e0t2FNTRjJSwIPZKfRUdLVIYb4sjXHqvLA9s76SkrUebDlEt16O\n" \
"vjDNTcK0q6jXGQjjRwjzPPmze36jvzhhGBypU3tXJifciRSl/4766tlvLwszMaOe\n" \
"yx+m+83OOchAq+N7/QxbXkNhAXrEzIOXOoYmA6QpfndQ\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIEZTCCA02gAwIBAgIQQAF1BIMUpMghjISpDBbN3zANBgkqhkiG9w0BAQsFADA/\n" \
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \
"DkRTVCBSb290IENBIFgzMB4XDTIwMTAwNzE5MjE0MFoXDTIxMDkyOTE5MjE0MFow\n" \
"MjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxCzAJBgNVBAMT\n" \
"AlIzMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuwIVKMz2oJTTDxLs\n" \
"jVWSw/iC8ZmmekKIp10mqrUrucVMsa+Oa/l1yKPXD0eUFFU1V4yeqKI5GfWCPEKp\n" \
"Tm71O8Mu243AsFzzWTjn7c9p8FoLG77AlCQlh/o3cbMT5xys4Zvv2+Q7RVJFlqnB\n" \
"U840yFLuta7tj95gcOKlVKu2bQ6XpUA0ayvTvGbrZjR8+muLj1cpmfgwF126cm/7\n" \
"gcWt0oZYPRfH5wm78Sv3htzB2nFd1EbjzK0lwYi8YGd1ZrPxGPeiXOZT/zqItkel\n" \
"/xMY6pgJdz+dU/nPAeX1pnAXFK9jpP+Zs5Od3FOnBv5IhR2haa4ldbsTzFID9e1R\n" \
"oYvbFQIDAQABo4IBaDCCAWQwEgYDVR0TAQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8E\n" \
"BAMCAYYwSwYIKwYBBQUHAQEEPzA9MDsGCCsGAQUFBzAChi9odHRwOi8vYXBwcy5p\n" \
"ZGVudHJ1c3QuY29tL3Jvb3RzL2RzdHJvb3RjYXgzLnA3YzAfBgNVHSMEGDAWgBTE\n" \
"p7Gkeyxx+tvhS5B1/8QVYIWJEDBUBgNVHSAETTBLMAgGBmeBDAECATA/BgsrBgEE\n" \
"AYLfEwEBATAwMC4GCCsGAQUFBwIBFiJodHRwOi8vY3BzLnJvb3QteDEubGV0c2Vu\n" \
"Y3J5cHQub3JnMDwGA1UdHwQ1MDMwMaAvoC2GK2h0dHA6Ly9jcmwuaWRlbnRydXN0\n" \
"LmNvbS9EU1RST09UQ0FYM0NSTC5jcmwwHQYDVR0OBBYEFBQusxe3WFbLrlAJQOYf\n" \
"r52LFMLGMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjANBgkqhkiG9w0B\n" \
"AQsFAAOCAQEA2UzgyfWEiDcx27sT4rP8i2tiEmxYt0l+PAK3qB8oYevO4C5z70kH\n" \
"ejWEHx2taPDY/laBL21/WKZuNTYQHHPD5b1tXgHXbnL7KqC401dk5VvCadTQsvd8\n" \
"S8MXjohyc9z9/G2948kLjmE6Flh9dDYrVYA9x2O+hEPGOaEOa1eePynBgPayvUfL\n" \
"qjBstzLhWVQLGAkXXmNs+5ZnPBxzDJOLxhF2JIbeQAcH5H0tZrUlo5ZYyOqA7s9p\n" \
"O5b85o3AM/OJ+CktFBQtfvBhcJVd9wvlwPsk+uyOy2HI7mNxKKgsBTt375teA2Tw\n" \
"UdHkhVNcsAKX1H7GNNLOEADksd86wuoXvg==\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/\n" \
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \
"DkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow\n" \
"PzEkMCIGA1UEChMbRGlnaXRhbCBTaWduYXR1cmUgVHJ1c3QgQ28uMRcwFQYDVQQD\n" \
"Ew5EU1QgUm9vdCBDQSBYMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEB\n" \
"AN+v6ZdQCINXtMxiZfaQguzH0yxrMMpb7NnDfcdAwRgUi+DoM3ZJKuM/IUmTrE4O\n" \
"rz5Iy2Xu/NMhD2XSKtkyj4zl93ewEnu1lcCJo6m67XMuegwGMoOifooUMM0RoOEq\n" \
"OLl5CjH9UL2AZd+3UWODyOKIYepLYYHsUmu5ouJLGiifSKOeDNoJjj4XLh7dIN9b\n" \
"xiqKqy69cK3FCxolkHRyxXtqqzTWMIn/5WgTe1QLyNau7Fqckh49ZLOMxt+/yUFw\n" \
"7BZy1SbsOFU5Q9D8/RhcQPGX69Wam40dutolucbY38EVAjqr2m7xPi71XAicPNaD\n" \
"aeQQmxkqtilX4+U9m5/wAl0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\n" \
"HQ8BAf8EBAMCAQYwHQYDVR0OBBYEFMSnsaR7LHH62+FLkHX/xBVghYkQMA0GCSqG\n" \
"SIb3DQEBBQUAA4IBAQCjGiybFwBcqR7uKGY3Or+Dxz9LwwmglSBd49lZRNI+DT69\n" \
"ikugdB/OEIKcdBodfpga3csTS7MgROSR6cz8faXbauX+5v3gTt23ADq1cEmv8uXr\n" \
"AvHRAosZy5Q6XkjEGB5YGV8eAlrwDPGxrancWYaLbumR9YbK+rlmM6pZW87ipxZz\n" \
"R8srzJmwN0jP41ZL9c8PDHIyh8bwRLtTcm1D9SZImlJnt1ir/md2cXjbDaJWFBM5\n" \
"JDGFoqgCWjBH4d1QB7wCCZAA62RjYJsWvIjJEubSfZGL+T0yjWW06XyxV3bqxbYo\n" \
"Ob8VZRzI9neWagqNdwvYkQsEjgfbKbYK7p2CNTUQ\n" \
"-----END CERTIFICATE-----\n";


const char *get_pool_api[] = {"https://server.duinocoin.com/getPool"};
const char *miner_version = "Official ESP32 Miner 2.76";
String pool_name = "";
String host = "";
int port = 0;
int walletid = 0;
volatile int wifi_state = 0;
volatile int wifi_prev_state = WL_CONNECTED;
volatile bool ota_state = false;
volatile char chip_id[23];  // DUCO MCU ID
char rigname_auto[23]; // SPACE TO STORE RIG NAME
int mqttUpdateTrigger = 0;
String mqttRigTopic = "";

// Wifi and Mqtt Clients
WiFiClient wifiMqttClient;
PubSubClient mqttClient(wifiMqttClient);

// Util Functions
void blink(uint8_t count, uint8_t pin = LED_BUILTIN) {
  uint8_t state = LOW;

  for (int x = 0; x < (count << 1); ++x) {
    digitalWrite(pin, state ^= HIGH);
    delay(75);
  }
}

// Communication Functions
void UpdatePool() {
  String input = "";
  int waitTime = 1;
  int poolIndex = 0;
  int poolSize = sizeof(get_pool_api) / sizeof(char*);
  
  while (input == "") {
    Serial.println("Fetching pool (" + String(get_pool_api[poolIndex]) + ")... ");
    input = httpGetString(get_pool_api[poolIndex]);
    poolIndex += 1;

    // Check if pool index needs to roll over
    if( poolIndex >= poolSize ){
      Serial.println("Retrying pool list in: " + String(waitTime) + "s");
      poolIndex %= poolSize;
      delay(waitTime * 1000);

      // Increase wait time till a maximum of 16 seconds (addresses: Limit connection requests on failure in ESP boards #1041)
      waitTime *= 2;
      if( waitTime > 16 )
        waitTime = 16;
    }
  }

  // Setup pool with new input
  UpdateHostPort(input);
}

void UpdateHostPort(String input) {
  // Thanks @ricaun for the code
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, input);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  const char *name = doc["name"];
  const char *h = doc["ip"];
  int p = doc["port"];

  host = h;
  port = p;

  // Send to MQTT
  mqttClient.publish((mqttRigTopic + "pool_name").c_str(), name);
  mqttClient.publish((mqttRigTopic + "pool_ip").c_str(), h);
  mqttClient.publish((mqttRigTopic + "pool_port").c_str(), String(p).c_str());

  // Send to Serial
  Serial.println("Fetched pool: " + String(name) + " - " + String(host) + ":" + String(port));
}

String httpGetString(String URL) {
  String payload = "";
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  if (http.begin(client, URL)) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      payload = http.getString();
    } else {
      Serial.printf("[HTTP] GET... failed, error: %s\n",
                    http.errorToString(httpCode).c_str());
    }
    http.end();
  }
  return payload;
}

void HandleMqttConnection()
{
  // Check Connection
  if (!mqttClient.connected()) {
    // Setup MQTT Client
    Serial.println("Connecting to mqtt server: " + String(mqtt_server) + " on port: " + String(mqtt_port));
    mqttClient.setServer(mqtt_server, mqtt_port);

    // Setup Rig Topic
    mqttRigTopic = "duinocoin/" + String(rig_identifier) + "/";

    // Try to connect
    if (mqttClient.connect(rig_identifier, (mqttRigTopic + "state").c_str(), 0, true, String(0).c_str())) {
      // Connection Succesfull
      Serial.println("Succesfully connected to mqtt server");

      // Output connection info
      mqttClient.publish((mqttRigTopic + "ip").c_str(), WiFi.localIP().toString().c_str());
      mqttClient.publish((mqttRigTopic + "name").c_str(), String(rig_identifier).c_str());
    }
    else{
      // Connection Failed
      Serial.println("Failed to connect to mqtt server");
    }
  }
  
  // Default MQTT Loop
  mqttClient.loop();
}

void WiFireconnect(void *pvParameters) {
  int n = 0;
  unsigned long previousMillis = 0;
  const long interval = 500;
  esp_task_wdt_add(NULL);
  for (;;) {
    wifi_state = WiFi.status();

#ifdef ENABLE_OTA
    ArduinoOTA.handle();
#endif

    if (ota_state)  // If OTA is working, reset the watchdog
      esp_task_wdt_reset();

    // check if WiFi status has changed.
    if ((wifi_state == WL_CONNECTED) && (wifi_prev_state != WL_CONNECTED)) {
      esp_task_wdt_reset();  // Reset watchdog timer

      // Connect to MQTT (will do nothing if MQTT is disabled)
      HandleMqttConnection();

      // Write Data to Serial
      Serial.println(F("\nConnected to WiFi!"));
      Serial.println("    IP address: " + WiFi.localIP().toString());
      Serial.println("      Rig name: " + String(rig_identifier));
      Serial.println();

      // Notify Setup Complete
      blink(BLINK_SETUP_COMPLETE);// Sucessfull connection with wifi network
      
      // Update Pool and wait a bit
      UpdatePool();
      yield();
      delay(100);
    }

    else if ((wifi_state != WL_CONNECTED) &&
             (wifi_prev_state == WL_CONNECTED)) {
      esp_task_wdt_reset();  // Reset watchdog timer
      Serial.println(F("\nWiFi disconnected!"));
      WiFi.disconnect();

      Serial.println(F("Scanning for WiFi networks"));
      n = WiFi.scanNetworks(false, true);
      Serial.println(F("Scan done"));

      if (n == 0) {
        Serial.println(F("No networks found. Resetting ESP32."));
        blink(BLINK_RESET_DEVICE);
        esp_restart();
      }
      else {
        Serial.print(n);
        Serial.println(F(" networks found"));
        for (int i = 0; i < n; ++i) {
          // Print wifi_ssid and RSSI for each network found
          Serial.print(i + 1);
          Serial.print(F(": "));
          Serial.print(WiFi.SSID(i));
          Serial.print(F(" ("));
          Serial.print(WiFi.RSSI(i));
          Serial.print(F(")"));
          Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " "
                                                                    : "*");
          delay(10);
        }
      }

      esp_task_wdt_reset();  // Reset watchdog timer
      Serial.println();
      Serial.println(
          F("Please, check if your WiFi network is on the list and check if "
            "it's strong enough (greater than -90)."));
      Serial.println("ESP32 will reset itself after " + String(WDT_TIMEOUT) +
                     " seconds if can't connect to the network");

      Serial.print("Connecting to: " + String(wifi_ssid));
      WiFi.reconnect();
    }

    else if ((wifi_state == WL_CONNECTED) &&
             (wifi_prev_state == WL_CONNECTED)) {
      esp_task_wdt_reset();  // Reset watchdog timer
      delay(1000);
    }

    else {
      // Don't reset watchdog timer
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.print(F("."));
      }
    }
    wifi_prev_state = wifi_state;
  }
}

// Miner Code
void TaskMining(void *pvParameters) {
  // Setup Thread
  esp_task_wdt_add(NULL); // Disable watchdogtimer for this thread

  // Setup thread data
  String taskCoreName = "CORE" + String(xPortGetCoreID());
  int taskId = xPortGetCoreID();

  // Start main thread loop
  for ( ;; ) {
    // If OTA needs to be preformed reset the task watchdog
    if (ota_state)  
      esp_task_wdt_reset();

    // Wait for a valid network connection
    while (wifi_state != WL_CONNECTED){
      delay(1000);
      esp_task_wdt_reset();
    }

    // Wait for server to get pool information
    while( port == 0 ){
      Serial.println(String("MinerThread on " + taskCoreName + " waiting for pool"));
      delay(1000);
      esp_task_wdt_reset();
    }

    // Setup WiFi Client and connection details
    Serial.println(String("MinerThread on " + taskCoreName + " connecting to Duino-Coin server..."));
    WiFiClient jobClient;
    jobClient.setTimeout(1);
    jobClient.flush();
    yield();

    // Start connection to Duino-Coin server
    if (!jobClient.connect(host.c_str(), port)) {
      Serial.println(String("MinerThread on " + taskCoreName + " failed to connect"));
      delay(500);
      continue;
    }

    // Wait for server connection
    Serial.println(String("MinerThread on " + taskCoreName + " is connected"));
    while (!jobClient.available()) {
      yield();
      if (!jobClient.connected()) break;
      delay(10);
    }

    // Server sends SERVER_VERSION after connecting
    String SERVER_VER = jobClient.readString();
    Serial.println(String("MinerThread on " + taskCoreName + " received server version: " + SERVER_VER));
    blink(BLINK_CLIENT_CONNECT); // Sucessfull connection with the server

    // Define job loop variables
    int jobClientBufferSize = 0;

    // Start Job loop
    while (jobClient.connected()) {
      // Reset watchdog timer before each job
      esp_task_wdt_reset();
      
      // We are connected and are able to request a job
      Serial.println(String("MinerThread on " + taskCoreName + " asking for a new job for user: " + username));
      jobClient.flush();
      jobClient.print("JOB," + String(username) + ",ESP32");
      while (!jobClient.available()) {
        if (!jobClient.connected()) break;
        delay(10);
      }
      yield();

      // Check buffer size is larget than 10
      jobClientBufferSize = jobClient.available();
      if (jobClientBufferSize <= 10) {
        Serial.println(String("MinerThread on " + taskCoreName + " buffer size is: " + jobClientBufferSize + " (FAILED, requesting new job...)"));
        continue;
      }
      else{
        Serial.println(String("MinerThread on " + taskCoreName + " buffer size is: " + jobClientBufferSize + " (OK)"));
      }

      // Read hash, expected hash and difficulty from job description
      String previousHash = jobClient.readStringUntil(MSGDELIMITER);
      String expectedHash = jobClient.readStringUntil(MSGDELIMITER);
      TaskThreadData[taskId].difficulty = jobClient.readStringUntil(MSGNEWLINE).toInt() * 100;
      jobClient.flush();
   
      // Global Definitions
      unsigned int job_size_task_one = 100;
      unsigned char *expectedHashBytes = (unsigned char *)malloc(job_size_task_one * sizeof(unsigned char));
      
      // Clear expectedHashBytes
      memset(expectedHashBytes, 0, job_size_task_one);
      size_t expectedHashLength = expectedHash.length() / 2;

      // Convert expected hash to byte array (for easy comparison)
      const char *cExpectedHash = expectedHash.c_str();
      for (size_t i = 0, j = 0; j < expectedHashLength; i += 2, j++)
        expectedHashBytes[j] = (cExpectedHash[i] % 32 + 9) % 25 * 16 + (cExpectedHash [i + 1] % 32 + 9) % 25;

      // Start measurement
      unsigned long startTime = micros();
      byte shaResult[20];
      String hashUnderTest;
      unsigned int hashUnderTestLength;
      bool ignoreHashrate = false;

      // Try to find the nonce which creates the expected hash
      for (unsigned long nonceCalc = 0; nonceCalc <= TaskThreadData[taskId].difficulty; nonceCalc++) {
        // Define hash under Test
        hashUnderTest = previousHash + String(nonceCalc);
        hashUnderTestLength = hashUnderTest.length();
        
        // Wait for hash module lock
        while( xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE );

        // We are allowed to perform our hash
        esp_sha(SHA1, (const unsigned char *)hashUnderTest.c_str(), hashUnderTestLength, shaResult);

        // Release hash module lock
        xSemaphoreGive(xMutex);

        // Check if we have found the nonce for the expected hash
        if( memcmp( shaResult, expectedHashBytes, sizeof(shaResult) ) == 0 ){
          // Found the nonce submit it to the server
          Serial.println(String("MinerThread on " + taskCoreName + " found hash with nonce: " + nonceCalc ));

          // Calculate mining time
          float elapsedTime = (micros() - startTime) / 1000.0 / 1000.0; // Total elapsed time in seconds
          TaskThreadData[taskId].hashrate = nonceCalc / elapsedTime;

          // Validate connection
          if (!jobClient.connected()) {
            Serial.println(String("MinerThread on " + taskCoreName + " Lost connection. Trying to reconnect"));
            if (!jobClient.connect(host.c_str(), port)) {
              Serial.println(String("MinerThread on " + taskCoreName + " connection failed"));
              break;
            }
            Serial.println(String("MinerThread on " + taskCoreName + " Reconnection successful."));
          }

          // Send result to server
          jobClient.flush();
          jobClient.print(
            String(nonceCalc) + MSGDELIMITER + String(TaskThreadData[taskId].hashrate) + MSGDELIMITER +
            String(miner_version) + MSGDELIMITER + String(rig_identifier) + MSGDELIMITER +
            "DUCOID" + String((char *)chip_id) + MSGDELIMITER + String(walletid));
          jobClient.flush();

          // Wait for job result
          while (!jobClient.available()) {
            if (!jobClient.connected()) {
              Serial.println(String("MinerThread on " + taskCoreName + " Lost connection. Didn't receive feedback."));
              break;
            }
            delay(10);
            yield();
          }
          delay(50);
          yield();

          // Handle feedback
          String feedback = jobClient.readStringUntil(MSGNEWLINE);
          jobClient.flush();
          TaskThreadData[taskId].shares++;
          blink(BLINK_SHARE_FOUND);

          // Validate Hashrate
          if( TaskThreadData[taskId].hashrate < 4000 && !ignoreHashrate){
            // Hashrate is low so restart esp
            Serial.println(String("MinerThread on " + taskCoreName + " Low hashrate (" + TaskThreadData[taskId].hashrate + "), Feedback: " + feedback + ". Restarting..."));
            jobClient.flush();
            jobClient.stop();
            blink(BLINK_RESET_DEVICE);
            esp_restart();
          }
          else{
            // Print statistics
            Serial.println(String("MinerThread on " + taskCoreName + " Job Feedback: " + feedback + ", Hashrate: " + TaskThreadData[taskId].hashrate + ", shares found: #" + TaskThreadData[taskId].shares)); 
          }

          // Stop current loop and ask for a new job
          break;
        }
      }
    }

    Serial.println(String("MinerThread on " + taskCoreName + " Not connected. Restarting core"));
    jobClient.flush();
    jobClient.stop(); 
  }
}


























void setup() {
  Serial.begin(500000);  // Start serial connection

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  display.clearDisplay();
  display.display();
  
  Serial.println("\n\nDuino-Coin " + String(miner_version));

  WiFi.mode(WIFI_STA);  // Setup ESP in client mode
  btStop();
  WiFi.begin(wifi_ssid, wifi_password);  // Connect to wifi

  uint64_t chipid = ESP.getEfuseMac();  // Getting chip chip_id
  uint16_t chip =
      (uint16_t)(chipid >> 32);  // Preparing for printing a 64 bit value (it's
                                 // actually 48 bits long) into a char array
  snprintf(
      (char *)chip_id, 23, "%04X%08X", chip,
      (uint32_t)chipid);  // Storing the 48 bit chip chip_id into a char array.
  walletid = random(0, 2811);

  // Autogenerate ID if required
  if( strcmp(rig_identifier, "Auto") == 0 ){
    snprintf(rigname_auto, 23, "ESP32-%04X%08X", chip, (uint32_t)chipid);
    rig_identifier = &rigname_auto[0];
  }

  ota_state = false;

#ifdef ENABLE_OTA
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
        ota_state = true;
      })
      .onEnd([]() { Serial.println(F("\nEnd")); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        esp_task_wdt_reset();
        ota_state = true;
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println(F("Auth Failed"));
        else if (error == OTA_BEGIN_ERROR)
          Serial.println(F("Begin Failed"));
        else if (error == OTA_CONNECT_ERROR)
          Serial.println(F("Connect Failed"));
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println(F("Receive Failed"));
        else if (error == OTA_END_ERROR)
          Serial.println(F("End Failed"));
        ota_state = false;
        blink(BLINK_RESET_DEVICE);
        esp_restart();
      });

  ArduinoOTA.setHostname(rig_identifier);
  ArduinoOTA.begin();
#endif

  esp_task_wdt_init(WDT_TIMEOUT, true);  // Init Watchdog timer
  pinMode(LED_BUILTIN, OUTPUT);

  // Determine which cores to use
  int wifiCore = 0;
  int mqttCore = 0;
  if( NUMBEROFCORES >= 2 ) mqttCore = 1;

  // Create Semaphore and main Wifi Monitoring Thread
  xMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
      WiFireconnect, "WiFirec", 10000, NULL, NUMBEROFCORES + 2, &WiFirec,
      mqttCore);  // create a task with highest priority and executed on core 0
  delay(250);

  // If MQTT is enabled create a sending thread
  #ifdef ENABLE_MQTT
  Serial.println("Creating mqtt thread on core: " + String(mqttCore));
  xTaskCreatePinnedToCore(
      MqttPublishCode, "MqttPublishCode", 10000, NULL, 1, &MqttPublishHandle, 
      mqttCore); //create a task with lowest priority and executed on core 1
  delay(250);
  #endif

  // Create Mining Threads
  for( int i = 0; i < NUMBEROFCORES; i++ ){
    Serial.println("Creating mining thread on core: " + String(i));
    xTaskCreatePinnedToCore(
      TaskMining, String("Task" + String(i)).c_str(), 10000, NULL, 2 + i, &TaskThreadData[NUMBEROFCORES].handler,
      i);  // create a task with priority 2 (+ core id) and executed on a specific core
    delay(250);
  }
}

// ************************************************************
void MqttPublishCode( void * pvParameters ) {
  unsigned long lastWdtReset = 0;
  unsigned long wdtResetDelay = 30000;

  for(;;) {
    if ((millis() - lastWdtReset) > wdtResetDelay) {
      // Reset timers
      esp_task_wdt_reset();
      lastWdtReset = millis();

      // Calculate combined hashrate and average difficulty
      float avgDiff = 0.0;
      float totHash = 0.0; 
      unsigned long totShares = 0;
      for (int i=0; i<NUMBEROFCORES; i++) {
        avgDiff += TaskThreadData[i].difficulty;
        totHash += TaskThreadData[i].hashrate;
        totShares += TaskThreadData[i].shares;
      }
      avgDiff /= NUMBEROFCORES;
      
      // Update States
      mqttClient.publish((mqttRigTopic + "state").c_str(), String(1).c_str());
      mqttClient.publish((mqttRigTopic + "hashrate").c_str(), String(totHash).c_str());
      mqttClient.publish((mqttRigTopic + "avgdiff").c_str(), String(avgDiff).c_str());
      mqttClient.publish((mqttRigTopic + "shares").c_str(), String(totShares).c_str());
    }

    mqttClient.loop();
    yield();
  }
}

void loop() { 
   if ((WiFi.status() == WL_CONNECTED)) {
    HTTPClient http;
    http.setTimeout(6000);
    http.begin(serverName + username, root_ca);
    int httpCode = http.GET();
    display.clearDisplay();
    while(httpCode < 0) {
      httpCode = http.GET();
    }
    if (httpCode > 0) {
      DynamicJsonDocument doc(9216);
      String jsonbalance = http.getString();
      deserializeJson(doc, jsonbalance);
      JsonObject datadoc = doc.as<JsonObject>();
      String ducos = datadoc["result"]["balance"]["balance"];
      ducosmem = ducos;
      userbalance = ducos.toFloat();
      //Serial.println("Ducos: " + String(ducos));

      // Miners
      miners = 0;
      for (JsonObject elem : datadoc["result"]["miners"].as<JsonArray>()) {
        miners++;
      }
      //Serial.println("Miners: " + String(miners));
      doc.clear();
    } else {
      ducosmem = "-";
      Serial.println("Error http get: " + String(httpCode));
    }
    http.end();
     
    if(wait == 0){
        oldb = userbalance;
    } if(wait == 5){
        wait = 0;
        calcule_daily();
    } else{
        wait++;
    }
    
    display.clearDisplay();
    display.setTextSize(1);            
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(6,0);            
    display.print("Ducos:");
    display.println(ducosmem);
    
    display.setCursor(6,10);            
    display.print("Miners:");
    display.println(miners);

    display.setCursor(6,20);            
    display.print("Daily:");
    display.println(daily);
  
    display.display();
  } else {
    display.clearDisplay();

    display.setTextSize(1);            
    display.setTextColor(SSD1306_WHITE);
    for (int i=0; i<3 ; i++){  
for (int a=95; a<99; a++){  
    display.setCursor(a,16);            
    display.print(" . ");
    delay(500);
    }
}

    display.setCursor(29,16);            
    display.print("CONNECTING");
    
    display.display();
    
    //WiFi.begin(wifi_ssid, wifi_password); 
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
  }
  delay(500);
}

void calcule_daily() {
  float ducomadein = userbalance - oldb;
  float dayduco = ducomadein * 960;
  daily = dayduco * 100 / 100;
  int ducomadesincestart = userbalance - balance;
  int secondssincestart = millis() - start;
  start = millis();
  ducomadesincesartdaily = ((86400/secondssincestart)*ducomadesincestart)*10 / 10;
}
