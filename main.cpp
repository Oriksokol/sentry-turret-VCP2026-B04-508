#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <esp_camera.h>
#include <Adafruit_PWMServoDriver.h>

// ============ STEPPER MOTOR ============

#define STEPPER_STEP_PIN 12
#define STEPPER_DIR_PIN  13

const int STEPPER_STEP_DELAY_US = 1000;

volatile bool stepperRunning = false;
volatile bool stepperDirection = false;

// ================= WIFI =================

const char* ssid = "Romster";
const char* password = "0404_07Ro";

// ============ CAMERA: AI-THINKER ============

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============ PCA9685 CHANNELS ============

#define I2C_SDA 15
#define I2C_SCL 14

Adafruit_PWMServoDriver pca(0x40, Wire);

// Сервоприводы на каналах 0 и 1
const int SERVO_1 = 0;
const int SERVO_2 = 1;

// Моторы через MOSFET подключены к каналам 2 и 3 PCA9685
const int MOTOR_1_CH = 2;
const int MOTOR_2_CH = 3;

WebServer server(80);
WiFiServer streamServer(81);

bool cameraReady = false;
bool pcaReady = false;

// Сервоприводы
const int CENTER_ANGLE = 90;
const int STEP_DEG = 30;
const int MAX_OFFSET = 150;
int servoOffset = 0;

// Таймер работы моторов
const unsigned long MOTOR_RUN_TIME_MS = 2000; // 2 секунды
unsigned long motorStopTime = 0;
bool motorsActive = false;

// ============ DC MOTOR CONTROL (ЧЕРЕЗ PCA9685) ============

void startMotors() {
    if (!pcaReady) return;
    // Включаем каналы 2 и 3 на 100% мощности
    pca.setPin(MOTOR_1_CH, 4095, false);
    pca.setPin(MOTOR_2_CH, 4095, false);
    motorStopTime = millis() + MOTOR_RUN_TIME_MS;
    motorsActive = true;
    Serial.println("Моторы MOT1 и MOT2 включены через PCA9685");
}

void stopMotors() {
    if (!pcaReady) return;
    // Полностью выключаем каналы
    pca.setPin(MOTOR_1_CH, 0, false);
    pca.setPin(MOTOR_2_CH, 0, false);
    motorsActive = false;
    Serial.println("Моторы MOT1 и MOT2 выключены");
}

// ============ CAMERA INIT ============

bool initCamera() {
    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Ошибка камеры: 0x%x\n", err);
        return false;
    }
    return true;
}

// ============ SERVO CONTROL ============

uint16_t angleToPulse(int angle) {
    angle = constrain(angle, 0, 180);
    return (uint16_t)map(angle, 0, 180, 500, 2500);
}

void updateServos() {
    if (!pcaReady) return;

    int angle1 = CENTER_ANGLE + servoOffset;
    int angle2 = CENTER_ANGLE - servoOffset;

    pca.writeMicroseconds(SERVO_1, angleToPulse(angle1));
    pca.writeMicroseconds(SERVO_2, angleToPulse(180 - angle2));

    Serial.printf("Servo 1: %d deg; Servo 2: %d deg\n", angle1, angle2);
}

void moveLeft() {
    if (servoOffset > -MAX_OFFSET) servoOffset -= STEP_DEG;
    servoOffset = constrain(servoOffset, -MAX_OFFSET, MAX_OFFSET);
    updateServos();
}

void moveRight() {
    if (servoOffset < MAX_OFFSET) servoOffset += STEP_DEG;
    servoOffset = constrain(servoOffset, -MAX_OFFSET, MAX_OFFSET);
    updateServos();
}

// ============ STEPPER CONTROL ============

void startStepper(bool direction) {
    stepperDirection = direction;
    digitalWrite(STEPPER_DIR_PIN, direction ? HIGH : LOW);
    delayMicroseconds(10);
    stepperRunning = true;
}

void stopStepper() {
    stepperRunning = false;
    digitalWrite(STEPPER_STEP_PIN, LOW);
}

void stepperTask(void* parameter) {
    for (;;) {
        if (stepperRunning) {
            digitalWrite(STEPPER_STEP_PIN, HIGH);
            delayMicroseconds(5);
            digitalWrite(STEPPER_STEP_PIN, LOW);
            delayMicroseconds(STEPPER_STEP_DELAY_US);
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ============ WEB PAGE ============

void handleRoot() {
    String page = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ESP32-CAM Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            background: #202124;
            color: white;
            margin: 20px;
        }
        img {
            width: 100%;
            max-width: 640px;
            border-radius: 8px;
        }
        button {
            font-size: 20px;
            padding: 15px 24px;
            margin: 8px;
            border: 0;
            border-radius: 8px;
            cursor: pointer;
            background-color: #3c4043;
            color: white;
        }
        button:active {
            transform: scale(0.96);
        }
        .btn-motor {
            background-color: #d93025;
            font-weight: bold;
        }
        #status {
            margin: 16px;
            min-height: 24px;
        }
    </style>
</head>
<body>
    <h2>ESP32-CAM</h2>

    <h3>Видеопоток</h3>
    <img id="video" alt="Ожидание видео...">

    <h3>Управление коллекторными моторами</h3>
    <button class="btn-motor" onclick="sendCommand('motors/start')">
        ⚡ Запустить моторы (2 сек)
    </button>

    <h3>Управление сервоприводами</h3>
    <button onclick="sendCommand('left')">&#8592; Влево</button>
    <button onclick="sendCommand('right')">Вправо &#8594;</button>

    <h3>Управление шаговым двигателем</h3>
    <button onclick="sendCommand('stepper/left')">&#8592; Вращать влево</button>
    <button onclick="sendCommand('stepper/right')">Вращать вправо &#8594;</button>
    <button onclick="sendCommand('stepper/stop')">Стоп</button>

    <div id="status">Готово к работе</div>

    <script>
        document.getElementById("video").src =
            "http://" + location.hostname + ":81/stream";

        async function sendCommand(command) {
            const status = document.getElementById("status");
            try {
                const response = await fetch("/" + command, {
                    method: "GET",
                    cache: "no-store"
                });
                const message = await response.text();
                status.textContent = message;
            } catch (error) {
                status.textContent = "Не удалось связаться с ESP32-CAM";
            }
        }
    </script>
</body>
</html>
)rawliteral";

    server.send(200, "text/html; charset=utf-8", page);
}

// ============ STREAM TASK ============

void streamTask(void* parameter) {
    for (;;) {
        WiFiClient client = streamServer.available();
        if (!client) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        client.setTimeout(1000);
        String request = client.readStringUntil('\n');

        while (client.connected()) {
            String line = client.readStringUntil('\n');
            if (line.length() == 0 || line == "\r") break;
        }

        if (request.indexOf("GET /stream") < 0) {
            client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
            client.stop();
            continue;
        }

        if (!cameraReady) {
            client.print("HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\nCamera unavailable");
            client.stop();
            continue;
        }

        client.print(
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Cache-Control: no-cache\r\n"
            "Access-Control-Allow-Origin: *\r\n\r\n"
        );

        while (client.connected()) {
            camera_fb_t* fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Не удалось получить кадр");
                break;
            }

            client.print("--frame\r\n");
            client.print("Content-Type: image/jpeg\r\n");
            client.printf("Content-Length: %u\r\n\r\n", (unsigned int)fb->len);

            size_t sent = client.write(fb->buf, fb->len);
            client.print("\r\n");
            esp_camera_fb_return(fb);

            if (sent == 0) break;
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        client.stop();
        Serial.println("Видеоклиент отключился");
    }
}

// ============ SETUP ============

void setup() {
    Serial.begin(115200);

    // Настройка пинов шагового мотора
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    digitalWrite(STEPPER_DIR_PIN, LOW);

    delay(1000);

    Serial.println();
    Serial.println("Запуск ESP32-CAM...");

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    pcaReady = pca.begin();
    if (pcaReady) {
        pca.setPWMFreq(50);
        delay(10);
        
        // Начальное положение сервоприводов
        servoOffset = 0;
        updateServos();

        // Выключаем моторы на старте
        stopMotors();

        Serial.println("PCA9685 обнаружена!");
    } else {
        Serial.println("PCA9685 не обнаружена. Проверь SDA, SCL и питание.");
    }

    cameraReady = initCamera();
    if (cameraReady) {
        Serial.println("Камера запущена!");
    } else {
        Serial.println("Камера не запустилась. Проверь шлейф и настройки.");
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    Serial.print("Подключение к Wi-Fi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Не удалось подключиться к Wi-Fi!");
        return;
    }

    Serial.println("Wi-Fi подключен!");
    Serial.print("Открой браузер: http://");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, handleRoot);

    server.on("/motors/start", HTTP_GET, []() {
        if (!pcaReady) {
            server.send(503, "text/plain; charset=utf-8", "PCA9685 не доступна");
            return;
        }
        startMotors();
        server.send(200, "text/plain; charset=utf-8", "Моторы запущены на 2 сек");
    });

    server.on("/left", HTTP_GET, []() {
        if (!pcaReady) {
            server.send(503, "text/plain; charset=utf-8", "PCA9685 не обнаружена.");
            return;
        }
        moveLeft();
        String message = "Влево: servo 1 = " + String(CENTER_ANGLE + servoOffset) + "°, servo 2 = " + String(CENTER_ANGLE - servoOffset) + "°";
        server.send(200, "text/plain; charset=utf-8", message);
    });

    server.on("/right", HTTP_GET, []() {
        if (!pcaReady) {
            server.send(503, "text/plain; charset=utf-8", "PCA9685 не обнаружена.");
            return;
        }
        moveRight();
        String message = "Вправо: servo 1 = " + String(CENTER_ANGLE + servoOffset) + "°, servo 2 = " + String(CENTER_ANGLE - servoOffset) + "°";
        server.send(200, "text/plain; charset=utf-8", message);
    });

    server.on("/stepper/left", HTTP_GET, []() {
        startStepper(false);
        server.send(200, "text/plain; charset=utf-8", "Шаговый двигатель вращается влево");
    });

    server.on("/stepper/right", HTTP_GET, []() {
        startStepper(true);
        server.send(200, "text/plain; charset=utf-8", "Шаговый двигатель вращается вправо");
    });

    server.on("/stepper/stop", HTTP_GET, []() {
        stopStepper();
        server.send(200, "text/plain; charset=utf-8", "Шаговый двигатель остановлен");
    });

    server.begin();
    streamServer.begin();

    xTaskCreatePinnedToCore(streamTask, "CameraStream", 8192, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(stepperTask, "StepperTask", 2048, nullptr, 1, nullptr, 1);

    Serial.println("Веб-сервер запущен на порту 80.");
}

// ============ LOOP ============

void loop() {
    server.handleClient();

    // Неблокирующее отключение через 2 секунды
    if (motorsActive && millis() >= motorStopTime) {
        stopMotors();
    }

    delay(2);
}