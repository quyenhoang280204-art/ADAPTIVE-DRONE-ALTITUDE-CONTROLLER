// Code kiểm tra tín hiệu từ ESP32 đến L298N
#define IN1 26
#define IN2 27

void setup() {
  Serial.begin(115200);
  delay(2000);  // Chờ Serial Monitor mở
  
  Serial.println("\n=== KIEM TRA KET NOI ESP32 -> L298N ===");
  Serial.println("Hay do dien ap tuong ung:");
  Serial.println("1. Do giua IN1 va GND");
  Serial.println("2. Do giua IN2 va GND");
  Serial.println("========================================");
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Đảm bảo bắt đầu ở trạng thái LOW
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void loop() {
  // TEST 1: IN1 HIGH, IN2 LOW
  Serial.println("\n[TEST 1] IN1=HIGH (3.3V), IN2=LOW (0V)");
  Serial.println("  Do: IN1-GND = ~3.3V, IN2-GND = 0V");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(5000);  // Đo trong 5 giây
  
  // TEST 2: IN1 LOW, IN2 HIGH
  Serial.println("\n[TEST 2] IN1=LOW (0V), IN2=HIGH (3.3V)");
  Serial.println("  Do: IN1-GND = 0V, IN2-GND = ~3.3V");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(5000);
  
  // TEST 3: Cả hai LOW
  Serial.println("\n[TEST 3] IN1=LOW, IN2=LOW");
  Serial.println("  Do: IN1-GND = 0V, IN2-GND = 0V");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(3000);
  
  // TEST 4: Cả hai HIGH
  Serial.println("\n[TEST 4] IN1=HIGH, IN2=HIGH");
  Serial.println("  Do: IN1-GND = 3.3V, IN2-GND = 3.3V");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  delay(3000);
  
  // TEST 5: Nhấp nháy để dễ đo
  Serial.println("\n[TEST 5] Nhap nhay IN1 (500ms)");
  for(int i=0; i<6; i++) {
    digitalWrite(IN1, !digitalRead(IN1));
    Serial.printf("  IN1: %s\n", digitalRead(IN1) ? "HIGH" : "LOW");
    delay(500);
  }
  
  Serial.println("\n[TEST 6] Nhap nhay IN2 (500ms)");
  for(int i=0; i<6; i++) {
    digitalWrite(IN2, !digitalRead(IN2));
    Serial.printf("  IN2: %s\n", digitalRead(IN2) ? "HIGH" : "LOW");
    delay(500);
  }
  
  Serial.println("\n=== KET THUC 1 CHU KY ===");
  delay(3000);
}