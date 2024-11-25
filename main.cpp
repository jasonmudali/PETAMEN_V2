#include <Arduino.h>
#include <WiFi.h>
#include <PID_v1.h>
// #include <Wifimanager.h>
// #include <PubSubClient.h>

#define VCC 3.3         // Voltage of ESP32
#define ThermistorPIN 4 // Thermistor is connected to GPIO 4
#define HeaterPIN 5     // Heater is connected to GPIO 5
#define MSG_BUFFER_SIZE (50)

const char *mqtt_server = "172.203.134.8";

float ResistorValue = 100000; // 100k Ohm
float beta = 3950;            // Beta factor
float T0 = 298.15;            // 25°C in Kelvin
float R0 = 100000;            // 100k Ohm at 25°C

double setpoint = 200;
double input, output, pidError;
double integral = 0;
int constrainedOutput;

double Kp = 1, Ki = 0, Kd = 1;

unsigned long currentMillis, currentTime, lastTime;
// PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// PWM configuration
const int pwmChannel = 0;
const int pwmFrequency = 5000;
const int pwmResolution = 8;

WiFiClient espClient;
// PubSubClient client(espClient);
unsigned long lastMsg = 0;
unsigned long previousMillis = 0;
const long interval = 1000;

// void reconnect() {
//   // Loop until we're reconnected
//   while (!client.connected()) {
//     Serial.print("Attempting MQTT connection...");
//     // Create a random client ID
//     String clientId = "ESP8266Client-";
//     clientId += String(random(0xffff), HEX);
//     // Attempt to connect
//     if (client.connect(clientId.c_str())) {
//       Serial.println("Connected");
//       // Once connected, publish an announcement...
//       client.publish("iotfrontier/mqtt", "iotfrontier");
//       // ... and resubscribe
//       client.subscribe("iotfrontier/mqtt");
//     } else {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       // Wait 5 seconds before retrying
//       delay(5000);
//     }
//   }
// }

int termNom = 100000; // Thermistor reference resistance

int refTemp = 25;

float current;

float read_temp2()
{
  // measuring the voltage on the thermistor

  current = analogRead(ThermistorPIN);

  // Convert the measured value to the thermistor resistance

  current = 4095 / current - 1;

  current = ResistorValue / current;

  // Calculation of temperature according to the relation for the beta factor

  float temperature;

  temperature = current / termNom; // (R/Ro)

  temperature = log(temperature); // ln(R/Ro)

  temperature /= beta; // 1/B * ln(R/Ro)

  temperature += 1.0 / (refTemp + 273.15); // + (1/To)

  temperature = 1.0 / temperature; // The inverted value

  temperature -= 273.15; // Convert from Kelvin to degrees Celsius

  return temperature;
}

float readTemp()
{
  int adcValue = analogRead(ThermistorPIN);
  float voltage = (adcValue * VCC) / 4095;

  float resistance = ResistorValue * (VCC / voltage - 1);

  float tempK = 1 / ((log(resistance / R0) / beta) + 1 / T0);

  float tempCelsius = tempK - 273.15;

  return tempCelsius;
}

void pidController()
{
  input = readTemp();
  double pidError = setpoint - input;

  currentTime = millis();
  double deltaTime = (currentTime - lastTime) / 1000;

  // Proportional
  double pid_p = Kp * pidError;

  // Integral
  integral += pidError * deltaTime;
  double pid_i = Ki * integral;

  // Derivative
  double derivative = (pidError - pidError) / deltaTime;
  double pid_d = Kd * derivative;

  // Output
  output = pid_p + pid_i + pid_d;

  constrainedOutput = constrain(output, 0, 255);
}

// void wifiManager(){

//   WiFiManager wifiManager;

//   // wifiManager.resetSettings();

//   if(!wifiManager.autoConnect("PETAMEN ESP32", "password")) {
//     Serial.println("failed to connect and hit timeout");
//     delay(3000);
//     ESP.restart();
//   }

//   Serial.println("Connected to WiFi!");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
// }

void temp_display()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    Serial.print("ADC Value:");
    Serial.println(analogRead(ThermistorPIN));

    float tempCelsius = readTemp();
    Serial.print("Temperature: ");
    Serial.print(tempCelsius, 0);
    Serial.println(" °C");

    Serial.print("PID Output: ");
    Serial.println(output);
  }

  // char tempStr[6];
  // dtostrf(tempCelsius, 4, 0, tempStr);
  // client.publish("petamen/temp", tempStr);
}

void setup()
{
  Serial.begin(115200);
  pinMode(ThermistorPIN, INPUT);
  pinMode(HeaterPIN, OUTPUT);

  // myPID.SetMode(AUTOMATIC);
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(HeaterPIN, pwmChannel);

  // wifiManager();

  // client.setServer(mqtt_server, 1883);
}

void loop()
{

  // if (!client.connected()) {
  //   reconnect();
  // }

  // client.loop();
  pidController();

  // if (output < 20)
  // {
  //   output = 20;
  // }
  // else if (output > 192)
  // {
  //   output = 192;
  // }

  ledcWrite(pwmChannel, constrainedOutput);

  temp_display();
}