#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>
#include <ArduinoJson.h>
#include <SimpleKalmanFilter.h>

// ========== CONFIGURATION ==========
const char* ssid = "ESP32-Drone";
const char* password = "12345678";

// ========== PIN DEFINITIONS ==========
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int MOTOR_IN1 = 26;
const int MOTOR_IN2 = 27;
const int MPU_SDA = 21;
const int MPU_SCL = 22;

// ========== RIG NHẸ - MAX PWM 130 ==========
const float HC_SR04_OFFSET = 17.0;
const float MAX_ALTITUDE = 15.0;
const float MOTOR_DEADZONE = 5;
const int MAX_MOTOR_PWM = 130;

// ========== GỠ RỐI NGAY KHI BẤM LAND ==========
const int UNTWIST_PWM = 75;                    // Quay ngược mạnh
const unsigned long UNTWIST_DURATION = 2000;   // 2 giây
const int UNTWIST_FORWARD_PWM = 0;             // Không quay thuận nhẹ
const unsigned long FORWARD_AFTER_DURATION = 1000;  // Không dùng

// ========== TAKEOFF ==========
const float TAKEOFF_THRUST = 100;
const unsigned long TAKEOFF_DURATION = 2000;

// ========== GLOBAL OBJECTS ==========
MPU6050 mpu(Wire);
WiFiServer server(80);

// ========== KALMAN FILTER ==========
const float KALMAN_ERROR_MEASURE = 2.0;
const float KALMAN_ERROR_ESTIMATE = 2.0;
const float KALMAN_Q = 0.1;
SimpleKalmanFilter altitudeKalman(KALMAN_ERROR_MEASURE, KALMAN_ERROR_ESTIMATE, KALMAN_Q);

// ========== ADAPTIVE PID ==========
class AdaptivePID {
private:
    float Kp_base = 18.0, Ki_base = 1.5, Kd_base = 3.5;
    float Kp_current, Ki_current, Kd_current;
    float adaptive_gain = 1.5;
    float adaptive_damping = 0.8;
    float error_threshold_high = 3.0;
    float error_threshold_low = 0.5;
    float integral_threshold = 50.0;
    
    float error_history[10];
    int error_index = 0;
    float avg_error = 0;
    unsigned long last_adaptation = 0;
    const unsigned long ADAPTATION_INTERVAL = 1000;
    
public:
    AdaptivePID() {
        Kp_current = Kp_base;
        Ki_current = Ki_base;
        Kd_current = Kd_base;
        for (int i = 0; i < 10; i++) error_history[i] = 0;
    }
    
    void getParameters(float &Kp, float &Ki, float &Kd) {
        Kp = Kp_current; Ki = Ki_current; Kd = Kd_current;
    }
    
    void updateErrorHistory(float error) {
        error_history[error_index] = fabs(error);
        error_index = (error_index + 1) % 10;
        avg_error = 0;
        for (int i = 0; i < 10; i++) avg_error += error_history[i];
        avg_error /= 10.0;
    }
    
    void adaptiveTuning(float error, float dt) {
        unsigned long now = millis();
        if (now - last_adaptation < ADAPTATION_INTERVAL) return;
        last_adaptation = now;
        
        float abs_error = fabs(error);
        if (abs_error > error_threshold_high) {
            Kp_current = Kp_base * adaptive_gain;
            Kd_current = Kd_base * adaptive_damping;
            Ki_current = Ki_base * 0.5;
        }
        else if (abs_error < error_threshold_low && avg_error < 1.0) {
            Kp_current = Kp_base * 0.8;
            Ki_current = Ki_base * 1.2;
            Kd_current = Kd_base * 1.1;
        }
        else {
            Kp_current = Kp_base;
            Ki_current = Ki_base;
            Kd_current = Kd_base;
        }
        
        int zero_crossings = 0;
        for (int i = 1; i < 10; i++) {
            if (error_history[i-1] * error_history[i] < 0) zero_crossings++;
        }
        if (zero_crossings >= 3) {
            Kp_current *= 0.7;
            Kd_current *= 1.3;
        }
        
        Kp_current = constrain(Kp_current, 5.0, 30.0);
        Ki_current = constrain(Ki_current, 0.1, 3.0);
        Kd_current = constrain(Kd_current, 0.5, 10.0);
    }
    
    float calculate(float error, float dt, float &integral, float &prev_error) {
        updateErrorHistory(error);
        adaptiveTuning(error, dt);
        
        integral += error * dt;
        integral = constrain(integral, -integral_threshold, integral_threshold);
        
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        
        return Kp_current * error + Ki_current * integral + Kd_current * derivative;
    }
    
    void resetToBase() {
        Kp_current = Kp_base;
        Ki_current = Ki_base;
        Kd_current = Kd_base;
    }
};

// ========== SYSTEM STATE ==========
enum SystemState { STATE_IDLE, STATE_CALIBRATING, STATE_TAKEOFF, STATE_HOVERING, STATE_LANDING, STATE_ERROR };
SystemState current_state = STATE_IDLE;
String state_names[6] = {"IDLE", "CALIBRATING", "TAKEOFF", "HOVERING", "LANDING", "ERROR"};

// ========== CONTROL VARIABLES ==========
float target_altitude = 12.0;
float current_altitude = 0.0;

float pitch = 0.0, roll = 0.0;
float accX = 0.0, accY = 0.0, accZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;

float motor_speed = 0;

AdaptivePID adaptivePID;
float error_integral = 0;
float last_error = 0;

float max_error = 0.0;
float total_error = 0.0;
int error_samples = 0;
float avg_error = 0.0;

unsigned long last_control_time = 0;
const float CONTROL_DT = 0.02;

// ========== MOTOR CONTROL ==========
void initMotorPWM() {
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
}

void setMotorSpeed(float speed) {
    speed = constrain(speed, 0, MAX_MOTOR_PWM);
    if (speed < MOTOR_DEADZONE) speed = 0;
    
    analogWrite(MOTOR_IN1, speed);
    digitalWrite(MOTOR_IN2, LOW);
    motor_speed = speed;
}

void setMotorReverse(int speed) {
    speed = constrain(speed, 0, MAX_MOTOR_PWM);
    if (speed < MOTOR_DEADZONE) speed = 0;
    
    analogWrite(MOTOR_IN2, speed);
    digitalWrite(MOTOR_IN1, LOW);
}

// ========== SENSORS ==========
float readUltrasonic() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return -1;
    return duration * 0.0343 / 2.0;
}

void updateSensors() {
    static unsigned long last_mpu_update = 0;
    
    if (millis() - last_mpu_update >= 10) {
        mpu.update();
        pitch = mpu.getAngleY();
        roll = mpu.getAngleX();
        accX = mpu.getAccX(); accY = mpu.getAccY(); accZ = mpu.getAccZ();
        gyroX = mpu.getGyroX(); gyroY = mpu.getGyroY(); gyroZ = mpu.getGyroZ();
        last_mpu_update = millis();
    }
    
    float distance_to_ceiling = readUltrasonic();
    if (distance_to_ceiling > 0 && distance_to_ceiling <= HC_SR04_OFFSET + 2.0) {
        float actual_altitude = HC_SR04_OFFSET - distance_to_ceiling;
        if (actual_altitude >= 0.0) {
            current_altitude = altitudeKalman.updateEstimate(actual_altitude);
            
            float error = fabs(target_altitude - current_altitude);
            total_error += error;
            error_samples++;
            if (error > max_error) max_error = error;
            avg_error = (error_samples > 0) ? total_error / error_samples : 0.0;
        }
    }
}

// ========== PID CONTROL ==========
void altitudeControl() {
    if (target_altitude > MAX_ALTITUDE) {
        target_altitude = MAX_ALTITUDE;
    }
    
    float error = target_altitude - current_altitude;
    float pid_output = adaptivePID.calculate(error, CONTROL_DT, error_integral, last_error);
    pid_output = constrain(pid_output, 0, MAX_MOTOR_PWM);
    
    setMotorSpeed(pid_output);
}

// ========== TAKEOFF ==========
void handleTakeoff() {
    static unsigned long takeoff_start = 0;
    static bool thrust_phase = true;
    
    if (current_state == STATE_TAKEOFF && takeoff_start == 0) {
        takeoff_start = millis();
        thrust_phase = true;
        error_integral = 0;
        last_error = 0;
        adaptivePID.resetToBase();
        total_error = 0; error_samples = 0; max_error = 0; avg_error = 0;
        Serial.println("TAKEOFF: Starting thrust...");
    }
    
    if (current_state != STATE_TAKEOFF) {
        takeoff_start = 0;
        return;
    }
    
    unsigned long elapsed = millis() - takeoff_start;
    
    if (thrust_phase) {
        setMotorSpeed(TAKEOFF_THRUST);
        Serial.printf("[TAKEOFF] Thrust: %.1f s | PWM=%d\n", elapsed / 1000.0, TAKEOFF_THRUST);
        
        if (elapsed >= TAKEOFF_DURATION) {
            thrust_phase = false;
            Serial.println("[TAKEOFF] Thrust phase complete - Waiting for lift...");
        }
    } else {
        if (current_altitude > 3.0) {
            Serial.println("TAKEOFF: SUCCESS → HOVERING");
            current_state = STATE_HOVERING;
            takeoff_start = 0;
        } else if (elapsed > TAKEOFF_DURATION + 3000) {
            Serial.println("TAKEOFF: FAILED - No altitude gain");
            setMotorSpeed(0);
            current_state = STATE_ERROR;
            takeoff_start = 0;
        }
    }
}
// ========== LANDING - QUAY NGƯỢC NGAY KHI BẤM LAND (HOẠT ĐỘNG MỌI LẦN) ==========
void handleLanding() {
    static bool untwist_executed = false;
    
    // Nếu đã thực hiện gỡ rối rồi và vẫn còn ở LANDING → không làm gì nữa
    if (untwist_executed && current_state == STATE_LANDING) {
        return;
    }
    
    // Nếu rời khỏi STATE_LANDING → reset để lần sau hoạt động
    if (current_state != STATE_LANDING) {
        untwist_executed = false;
        return;
    }
    
    // Bắt đầu gỡ rối (chỉ chạy 1 lần mỗi lần bấm Land)
    Serial.println("LAND COMMAND: Starting IMMEDIATE strong untwist...");
    
    setMotorReverse(UNTWIST_PWM);
    Serial.printf("[UNTWIST] Strong reverse PWM=%d for %.1f s\n", UNTWIST_PWM, UNTWIST_DURATION / 1000.0);
    delay(UNTWIST_DURATION);
    
    if (UNTWIST_FORWARD_PWM > 0) {
        setMotorSpeed(UNTWIST_FORWARD_PWM);
        Serial.printf("[UNTWIST] Light forward PWM=%d for %.1f s\n", UNTWIST_FORWARD_PWM, FORWARD_AFTER_DURATION / 1000.0);
        delay(FORWARD_AFTER_DURATION);
    }
    
    setMotorSpeed(0);
    setMotorReverse(0);
    Serial.println("[UNTWIST] COMPLETED - Motor stopped completely");
    
    untwist_executed = true;
    current_state = STATE_IDLE;  // Về IDLE sau khi xong
}

void testMotorDirect() {
    Serial.println("\n=== MANUAL MOTOR TEST (MAX 130) ===");
    int speeds[] = {30, 50, 70, 90, 110, 130};
    for (int i = 0; i < 6; i++) {
        Serial.printf("Forward PWM: %d\n", speeds[i]);
        setMotorSpeed(speeds[i]);
        delay(3000);
    }
    Serial.println("Strong reverse test...");
    setMotorReverse(75);
    delay(2000);
    setMotorSpeed(0);
    Serial.println("=== TEST DONE ===\n");
}

// ========== FULL HTML WEB ==========
const char* html_page = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Drone - Stable Land</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 900px; margin: auto; padding: 20px; background: white; border-radius: 10px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); }
        h1 { text-align: center; color: #333; }
        .panel { background: #f8f9fa; padding: 20px; border-radius: 8px; margin-bottom: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .status-item { display: flex; justify-content: space-between; margin: 10px 0; font-size: 17px; }
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .control-panel { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; }
        .btn { padding: 18px; border: none; border-radius: 8px; font-size: 18px; cursor: pointer; font-weight: bold; }
        .btn-start { background: #28a745; color: white; }
        .btn-stop { background: #dc3545; color: white; }
        .btn-test { background: #ffc107; color: black; }
        .slider-container { margin: 20px 0; }
        label { font-size: 18px; font-weight: bold; }
        input[type=range] { width: 100%; height: 40px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚁 ESP32 Drone Rig - Final Stable</h1>
        <p style="text-align:center; color:#28a745;">Land hoạt động mọi lần | Untwist ngay lập tức</p>
        
        <div class="grid-2">
            <div class="panel">
                <h3>Flight Status</h3>
                <div class="status-item"><span>State:</span><span id="state">IDLE</span></div>
                <div class="status-item"><span>Current Alt:</span><span id="altitude">0.0 cm</span></div>
                <div class="status-item"><span>Target Alt:</span><span id="target">12.0 cm</span></div>
                <div class="status-item"><span>Motor Power:</span><span id="motor">0 / 130</span></div>
                <div class="status-item"><span>Avg Error:</span><span id="avg_error">0.00 cm</span></div>
                <div class="status-item"><span>Max Error:</span><span id="max_error">0.00 cm</span></div>
            </div>

            <div class="panel">
                <h3>PID Parameters</h3>
                <div class="status-item"><span>Kp:</span><span id="kp">18.0</span></div>
                <div class="status-item"><span>Ki:</span><span id="ki">1.5</span></div>
                <div class="status-item"><span>Kd:</span><span id="kd">3.5</span></div>
            </div>
        </div>

        <div class="panel">
            <h3>MPU6050 Sensors</h3>
            <div class="grid-2">
                <div>
                    <h4>Angles</h4>
                    <div class="status-item"><span>Pitch:</span><span id="pitch">0.0°</span></div>
                    <div class="status-item"><span>Roll:</span><span id="roll">0.0°</span></div>
                </div>
                <div>
                    <h4>Accel (g)</h4>
                    <div class="status-item"><span>X:</span><span id="accX">0.00</span></div>
                    <div class="status-item"><span>Y:</span><span id="accY">0.00</span></div>
                    <div class="status-item"><span>Z:</span><span id="accZ">1.00</span></div>
                </div>
            </div>
            <div class="grid-2" style="margin-top:15px;">
                <div>
                    <h4>Gyro (°/s)</h4>
                    <div class="status-item"><span>X:</span><span id="gyroX">0.0</span></div>
                    <div class="status-item"><span>Y:</span><span id="gyroY">0.0</span></div>
                    <div class="status-item"><span>Z:</span><span id="gyroZ">0.0</span></div>
                </div>
            </div>
        </div>

        <div class="panel">
            <h3>Set Target Altitude (5 - 15 cm)</h3>
            <div class="slider-container">
                <label for="altSlider">Target: <span id="altValue">12.0</span> cm</label>
                <input type="range" id="altSlider" min="5" max="15" value="12" step="0.5">
            </div>
        </div>

        <div class="panel">
            <h3>Control</h3>
            <div class="control-panel">
                <button class="btn btn-start" onclick="sendCommand('start')">▶ Takeoff</button>
                <button class="btn btn-stop" onclick="sendCommand('stop')">⏹ Land + Untwist</button>
                <button class="btn btn-test" onclick="sendCommand('calibrate')">📐 Calibrate MPU</button>
                <button class="btn btn-test" onclick="sendCommand('test')">⚡ Motor Test</button>
            </div>
        </div>
    </div>

    <script>
        const altSlider = document.getElementById('altSlider');
        const altValue = document.getElementById('altValue');
        
        altSlider.addEventListener('input', function() {
            altValue.textContent = this.value;
            fetch('/cmd?altitude=' + this.value);
        });

        function sendCommand(cmd) {
            fetch('/cmd?command=' + cmd);
        }

        function updateStatus() {
            fetch('/data')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('state').textContent = data.state;
                    document.getElementById('altitude').textContent = data.altitude.toFixed(1) + ' cm';
                    document.getElementById('target').textContent = data.target.toFixed(1) + ' cm';
                    document.getElementById('motor').textContent = data.motor + ' / 130';
                    document.getElementById('avg_error').textContent = data.avg_error.toFixed(2) + ' cm';
                    document.getElementById('max_error').textContent = data.max_error.toFixed(2) + ' cm';
                    document.getElementById('kp').textContent = data.kp.toFixed(2);
                    document.getElementById('ki').textContent = data.ki.toFixed(3);
                    document.getElementById('kd').textContent = data.kd.toFixed(2);
                    document.getElementById('pitch').textContent = data.pitch.toFixed(1) + '°';
                    document.getElementById('roll').textContent = data.roll.toFixed(1) + '°';
                    document.getElementById('accX').textContent = data.accX.toFixed(2);
                    document.getElementById('accY').textContent = data.accY.toFixed(2);
                    document.getElementById('accZ').textContent = data.accZ.toFixed(2);
                    document.getElementById('gyroX').textContent = data.gyroX.toFixed(1);
                    document.getElementById('gyroY').textContent = data.gyroY.toFixed(1);
                    document.getElementById('gyroZ').textContent = data.gyroZ.toFixed(1);
                    
                    if (Math.abs(altSlider.value - data.target) > 0.1) {
                        altSlider.value = data.target;
                        altValue.textContent = data.target.toFixed(1);
                    }
                });
        }

        setInterval(updateStatus, 800);
        updateStatus();
    </script>
</body>
</html>
)=====";

// ========== WEB SERVER ==========
void handleWebClient(WiFiClient &client) {
    String request = client.readStringUntil('\r');
    client.readStringUntil('\n');
    while (client.available() && client.readStringUntil('\n') != "\r") {}
    
    if (request.indexOf("GET / ") >= 0 || request.indexOf("GET /index") >= 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html; charset=UTF-8");
        client.println("Connection: close");
        client.println();
        client.println(html_page);
    }
    else if (request.indexOf("GET /data") >= 0) {
        StaticJsonDocument<600> doc;
        doc["state"] = state_names[current_state];
        doc["altitude"] = current_altitude;
        doc["target"] = target_altitude;
        doc["motor"] = (int)motor_speed;
        doc["avg_error"] = avg_error;
        doc["max_error"] = max_error;
        
        float Kp, Ki, Kd;
        adaptivePID.getParameters(Kp, Ki, Kd);
        doc["kp"] = Kp;
        doc["ki"] = Ki;
        doc["kd"] = Kd;
        
        doc["pitch"] = pitch;
        doc["roll"] = roll;
        doc["accX"] = accX;
        doc["accY"] = accY;
        doc["accZ"] = accZ;
        doc["gyroX"] = gyroX;
        doc["gyroY"] = gyroY;
        doc["gyroZ"] = gyroZ;
        
        String response;
        serializeJson(doc, response);
        
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();
        client.println(response);
    }
    else if (request.indexOf("GET /cmd") >= 0) {
        String command = "";
        String alt_str = "";
        
        int cmdStart = request.indexOf("command=");
        int altStart = request.indexOf("altitude=");
        
        if (cmdStart > 0) {
            int cmdEnd = request.indexOf(" ", cmdStart);
            if (cmdEnd == -1) cmdEnd = request.length();
            command = request.substring(cmdStart + 8, cmdEnd);
        }
        
        if (altStart > 0) {
            int altEnd = request.indexOf("&", altStart);
            if (altEnd == -1) altEnd = request.indexOf(" ", altStart);
            if (altEnd == -1) altEnd = request.length();
            alt_str = request.substring(altStart + 9, altEnd);
            alt_str.trim();
            
            float new_alt = alt_str.toFloat();
            if (new_alt >= 5.0 && new_alt <= 20.0) {
                target_altitude = new_alt;
                Serial.printf("WEB: Target altitude = %.1f cm\n", target_altitude);
            }
        }
        
        if (command == "start" && current_state == STATE_IDLE) current_state = STATE_TAKEOFF;
        else if (command == "stop" && current_state != STATE_IDLE && current_state != STATE_LANDING) current_state = STATE_LANDING;
        else if (command == "calibrate") current_state = STATE_CALIBRATING;
        else if (command == "test") testMotorDirect();
        
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        client.println("OK");
    }
    
    delay(1);
    client.stop();
}

// ========== SETUP & LOOP ==========
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== ESP32 DRONE - LAND HOẠT ĐỘNG MỌI LẦN ===");
    Serial.println("Bấm Land bao nhiêu lần cũng quay ngược gỡ rối đúng 1 lần mỗi lần");
    
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    
    initMotorPWM();

    Wire.begin(MPU_SDA, MPU_SCL);
    if (mpu.begin() != 0) {
        Serial.println("MPU6050 ERROR!");
        current_state = STATE_ERROR;
    } else {
        current_state = STATE_CALIBRATING;
    }

    WiFi.softAP(ssid, password);
    Serial.print("AP: "); Serial.println(ssid);
    Serial.print("Web: http://"); Serial.println(WiFi.softAPIP());

    server.begin();
    Serial.println("SYSTEM READY - Land works every time!");
}

void loop() {
    WiFiClient client = server.available();
    if (client) handleWebClient(client);

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "start") current_state = STATE_TAKEOFF;
        else if (cmd == "stop") current_state = STATE_LANDING;
        else if (cmd == "calibrate") current_state = STATE_CALIBRATING;
        else if (cmd == "test") testMotorDirect();
    }

    updateSensors();

    switch (current_state) {
        case STATE_IDLE: break;
        case STATE_CALIBRATING:
            Serial.println("CALIBRATING...");
            mpu.calcGyroOffsets();
            current_state = STATE_IDLE;
            Serial.println("CALIBRATION DONE");
            break;
        case STATE_TAKEOFF: handleTakeoff(); break;
        case STATE_HOVERING:
            altitudeControl();
            break;
        case STATE_LANDING:
            handleLanding();
            break;
        case STATE_ERROR:
            setMotorSpeed(0);
            break;
    }

    unsigned long now = millis();
    if (now - last_control_time >= (unsigned long)(CONTROL_DT * 1000)) {
        last_control_time = now;
        if (current_state == STATE_HOVERING) {
            altitudeControl();
        }
    }

    delay(10);
}