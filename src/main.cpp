#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <NewPing.h>
#include <WiFi.h>
#include <Wire.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

// Declare display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WiFi credentials
const char *ssid = "VIVOFIBRA-B190";
const char *password = "9A93A32AA7";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

float setpoint = 20.0; // Default setpoint

#define TRIGGER_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 4
#define SDA_PIN 21
#define SCL_PIN 22

#define MAX_DISTANCE 50

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myServo;

// Definições para a média móvel exponencial
float alpha = 0.1; // Fator de suavização para a leitura da distância
float beta = 0.1;  // Fator de suavização para a saída do PID
float filteredDistance = 0.0;
float filteredAngle = 113.0;
float filteredDerivative = 0.0;

// PID
float kp = 0.7;
float ki = 0.04;
float kd = 0.6;
float integral_sum = 0.0;
float error_before = 0.0;

float pid(float error) {
    float dt = 0.01;
    float p = kp * error;
    integral_sum += ki * error * dt;

    // Calcula o erro derivativo filtrado
    float derivative = (error - error_before) / dt;
    filteredDerivative = 0.1 * derivative + 0.9 * filteredDerivative;

    float d = kd * filteredDerivative;
    float pid_value = p + integral_sum + d;
    error_before = error;
    return pid_value;
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Initialize Servo
    myServo.attach(SERVO_PIN);

    // Initialize OLED display
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.display();

    // Initialize WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());

    // Initialize web server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html",
                      "<form action=\"/setpoint\" method=\"post\"><input "
                      "type=\"number\" name=\"value\"><input type=\"submit\" "
                      "value=\"Set Setpoint\"></form>");
    });

    server.on("/setpoint", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value", true)) {
            setpoint = request->getParam("value", true)->value().toFloat();
        }
        request->send(200, "text/html",
                      "Setpoint updated. <a href=\"/\">Go back</a>");
    });

    server.begin();
}

void loop() {
    unsigned int distance_raw = sonar.ping_cm();

    // Aplicando o filtro passa-baixa na leitura da distância
    filteredDistance = alpha * distance_raw + (1 - alpha) * filteredDistance;

    float error = filteredDistance - setpoint;

    float pid_output = pid(error);

    // Aplicando o filtro passa-baixa na saída do PID
    // filteredAngle = beta * pid_output + (1 - beta) * filteredAngle;

    float angle = 113.0 + pid_output;

    if (millis() % 300 == 0) {
        Serial.print("Error: ");
        Serial.print(error);
        Serial.print(", Angle: ");
        Serial.print(angle);
        Serial.print(", Sensor: ");
        Serial.println(filteredDistance);

        // Display values on OLED
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.printf("Error: %.2f\n", error);
        display.printf("Setpoint: %.2f\n", setpoint);
        display.printf("Distance: %.2f\n", filteredDistance);
        display.printf("Angle: %.2f\n", angle);
        display.printf("Kp: %.2f, Ki: %.3f, Kd: %.2f\n", kp, ki, kd);
        display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
        display.display();
    }

    if (angle > 170) {
        angle = 170;
    }
    if (angle < 30) {
        angle = 30;
    }
    myServo.write(angle);
}
