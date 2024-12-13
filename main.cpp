#include <Arduino.h>
#include <WiFi.h>
#include <PID_v1.h>
#include <Wifimanager.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>

#define VCC 3.264       // Voltage of ESP32
#define ThermistorPIN 4 // Thermistor is connected to GPIO 4
#define HeaterPIN 5     // Heater is connected to GPIO 5
#define stepPin 6       // Stepper motor is connected to GPIO 6
#define limit1 1
#define limit2 2
#define MSG_BUFFER_SIZE (50)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

const char *mqtt_server = "172.203.134.8";

float ResistorValue = 4700; // 4k7 Ohm
float beta = 3950;          // Beta factor
float T0 = 298.15;          // 25°C in Kelvin
float R0 = 100000;          // 100k Ohm at 25°C

double setpoint = 220;
double input, output;

double Kp = 26, Ki = 0, Kd = 1;

WiFiClient espClient;
PubSubClient client(espClient);
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
Adafruit_ADS1115 ads;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// PWM configuration
const int pwmChannel = 0;
const int pwmFrequency = 400;
const int pwmResolution = 8;

// PubSubClient client(espClient);
unsigned long lastMsg = 0;
unsigned long previousMillis = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
unsigned long lastPrintRunningTime = 0;
bool heatingComplete = false;
const long interval = 1000;
int16_t adcValue;
float voltage;
int motorSpeed = 600;

const unsigned char WiFi_Icon[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x01, 0xc1, 0xc0, 0x00,
    0x03, 0x00, 0x60, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0xc1, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00,
    0x00, 0x3e, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const unsigned char Temp_Icon[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00,
    0x03, 0x18, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x02,
    0x08, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x03, 0x08,
    0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00,
    0x00, 0x00, 0x03, 0x08, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00,
    0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x02, 0xe8, 0x00, 0x00, 0x00, 0x02, 0xe8, 0x00, 0x00, 0x00,
    0x02, 0xe8, 0x00, 0x00, 0x00, 0x02, 0xe8, 0x00, 0x00, 0x00, 0x02, 0xe8, 0x00, 0x00, 0x00, 0x02,
    0xe8, 0x00, 0x00, 0x00, 0x06, 0xe4, 0x00, 0x00, 0x00, 0x0d, 0xf2, 0x00, 0x00, 0x00, 0x0b, 0xfa,
    0x00, 0x00, 0x00, 0x0b, 0xfa, 0x00, 0x00, 0x00, 0x0b, 0xfb, 0x00, 0x00, 0x00, 0x0b, 0xfa, 0x00,
    0x00, 0x00, 0x09, 0xfa, 0x00, 0x00, 0x00, 0x04, 0xe4, 0x00, 0x00, 0x00, 0x02, 0x0c, 0x00, 0x00,
    0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("Connected");
      // Once connected, publish an announcement...
      client.publish("iotfrontier/mqtt", "iotfrontier");
      // ... and resubscribe
      client.subscribe("testking");
      client.subscribe("petamen/motorspeed");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String messageBuffer;
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  Serial.print("Message: ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);

    messageBuffer += (char)payload[i];
  }
  if (String(topic) == "testking")
  {
    setpoint = messageBuffer.toFloat();
    Serial.print("Setpoint: ");
    Serial.println(setpoint);
  }

  else if (String(topic) == "petamen/motorspeed")
  {
    motorSpeed = messageBuffer.toInt();
    Serial.print("Motor speed: ");
    Serial.println(motorSpeed);
  }

  Serial.println();
}

int termNom = 100000; // Thermistor reference resistance

int refTemp = 25;

float current;

float readTemp()
{
  adcValue = ads.readADC_SingleEnded(0);
  voltage = ads.computeVolts(adcValue);

  float resistance = ResistorValue * (VCC / voltage - 1);

  float tempK = 1 / ((log(resistance / R0) / beta) + 1 / T0);

  float tempCelsius = tempK - 273.15;

  return tempCelsius;
}

void wifiManager()
{

  WiFiManager wifiManager;

  // wifiManager.resetSettings();

  if (!wifiManager.autoConnect("PETAMEN ESP32", "password"))
  {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void temp_display()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    Serial.print("ADC Value:");
    Serial.println(adcValue);

    Serial.print("Voltage: ");
    Serial.println(voltage, 5);

    Serial.print("Temperature: ");
    Serial.print(input + 2, 0);
    Serial.println(" °C");

    Serial.print("PID Output: ");
    Serial.println(output);

    char tempStr[6];
    dtostrf(input + 2, 4, 0, tempStr);
    client.publish("petamen/temp", tempStr);
  }
}

void stepper()
{
  for (int i = 0; i < 200; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorSpeed); // Adjust delay for speed control
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorSpeed); // Adjust delay for speed control
  }
}

void oledDisplay()
{
  if (millis() % 1000 == 0)
  {
    display.clearDisplay();

    // Draw icons
    display.drawBitmap(0, 0, WiFi_Icon, 25, 22, SSD1306_WHITE);
    display.drawBitmap(2, 18, Temp_Icon, 35, 35, SSD1306_WHITE);
    display.drawCircle(88, 40, 2, SSD1306_WHITE); // Draw a circle for the Degree icon

    // Text buat indikator WiFi
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(25, 3);
    display.print("ON");

    // Text buat nama Project
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(75, 3);
    display.print("PETAMEN");

    // Text buat menampilkan suhu
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(45, 20); // Set cursor to the top-left corner
    display.print("Temp: ");

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(45, 38); // Set cursor to the top-left corner
    display.print(random(20, 250));
    display.setCursor(81, 37);
    display.print(" C");

    // Text untuk menampilkan status Limit Switch
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 56);
    display.print("S1: O");

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(90, 56);
    display.print("S2: O");

    // Buat menampilkan display diatas
    display.display();
  }
}

void runningTime()
{
  if (input >= setpoint && !heatingComplete)
  {
    heatingComplete = true;
    startTime = millis(); // Record the start time
  }

  // Calculate elapsed time if heating is complete
  if (heatingComplete)
  {
    elapsedTime = millis() - startTime;

    if (millis() - lastPrintRunningTime >= 1000)
    {
      Serial.print("Elapsed Time: ");
      Serial.print(elapsedTime / 1000); // Convert to seconds
      Serial.println(" seconds");
      lastPrintRunningTime = millis();

      char timeStr[10];
      snprintf(timeStr, sizeof(timeStr), "%lu", elapsedTime / 1000); // Convert to seconds
      client.publish("petamen/elapsedTime", timeStr);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(ThermistorPIN, INPUT);
  pinMode(HeaterPIN, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limit1, INPUT);

  myPID.SetMode(AUTOMATIC);
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(HeaterPIN, pwmChannel);

  // Wire.begin(8, 9);

  if (!ads.begin())
  {
    Serial.println("ADS1115 tidak terdeteksi!");
    while (1)
      ;
  }
  Serial.println("ADS1115 berhasil diinisialisasi.");

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ; // Don't proceed, loop forever
  }
  // Clear the display buffer
  display.clearDisplay();
  wifiManager();

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  reconnect();
}

void loop()
{

  if (!client.connected())
  {
    reconnect();
  }

  client.loop();
  // int limitOne = digitalRead(limit1);
  // Serial.print(limitOne);

  // if (limitOne == 0)
  // {
  //   Serial.print("Limit Switch 1 pressed ");
  //   stepper();
  // }
  // else
  // {
  //   Serial.print("Limit Switch 1 not pressed ");
  // }

  input = readTemp();

  myPID.Compute();

  if (output < 20)
  {
    output = 20;
  }

  ledcWrite(pwmChannel, output);
  stepper();
  temp_display();
  oledDisplay();
  runningTime();
  // if (millis() % 1000 == 0)
  // {
  // displayData();
  // }
}