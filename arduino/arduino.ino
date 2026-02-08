// --- CODE ARDUINO UNO / NANO (Motor Controller + Line Follower + AI Override) ---


#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 10
#define IN3 8
#define IN4 9


// ===================== CHÂN LINE SENSOR =====================
#define LINE_L 3   // Sensor trái (digital)
#define LINE_M 12  // Sensor giữa
#define LINE_R 2   // Sensor phải


// ===================== TỐC ĐỘ =====================
const int BASE_SPEED = 255;       // tốc độ tiến/lùi manual
const int LINE_FOLLOW_SPEED = 170; // tốc độ tiến trong chế độ line_follow
const int TURN_SPEED = 255;       // tốc độ rẽ gắt (spin)


// ================== BIẾN TOÀN CỤC =====================
char aiCommand = 'F';  // Lệnh từ AI/ESP32 (S=dừng, F=chạy)
bool lineFollowMode = true;  // true = chạy theo line, false = nhận lệnh trực tiếp
char lastMotorState = '\0';  // Lưu trạng thái motor để tránh spam log


void setup() {
  Serial.begin(115200);


  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  // Line sensors
  pinMode(LINE_L, INPUT);
  pinMode(LINE_M, INPUT);
  pinMode(LINE_R, INPUT);


  stopCar();
  Serial.println("🚗 Arduino Line Follower + AI Ready");
  delay(1000);
}


void loop() {
  // ========== ĐỌC LỆNH TỪ ESP32 ==========
  // Đọc HẾT tất cả lệnh trong buffer để không bỏ lỡ
  while (Serial.available()) {
    char cmd = Serial.read();
    Serial.print(" Received Command: ");
    Serial.println(cmd);


    // ========== XỬ LÝ LỆNH ==========
    // Lệnh 'S' → LINE_FOLLOW mode (dừng vì đèn đỏ/STOP)
    if (cmd == 'S') {
      aiCommand = 'S';
      lineFollowMode = true;
      Serial.println(" LINE_FOLLOW: AI STOP");
      // Không return ở đây để code tiếp tục xử lý stopCar() bên dưới
    }
   
    // Lệnh 'A' → Bật lại LINE_FOLLOW mode (cho phép chạy)
    else if (cmd == 'A') {
      lineFollowMode = true;
      aiCommand = 'A';  // Đổi từ 'F' sang 'A' để phân biệt rõ
      Serial.println(" LINE FOLLOW: AI ALLOW (đèn xanh/không có gì)");
    }
   
    // Lệnh F/B/L/R → AUTO/MANUAL mode (điều khiển trực tiếp motor)
    else if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R') {
      lineFollowMode = false;
      aiCommand = cmd;
      executeCommand(cmd);
      Serial.print(" AUTO/MANUAL: ");
      Serial.println(cmd);
    }
  }


  // ========== XỬ LÝ THEO CHẾ ĐỘ ==========
  if (lineFollowMode) {
    // Chế độ LINE FOLLOW: dùng line sensor
    if (aiCommand == 'S') {
      // AI yêu cầu dừng (đèn đỏ/STOP)
      stopCar();
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 1000) {
        Serial.println(" LINE_FOLLOW: Dừng (đèn đỏ)");
        lastPrint = millis();
      }
    } else {
      // AI cho phép chạy (aiCommand == 'A' hoặc khác 'S')
      // → Chạy theo line sensor
      followLine();
    }
  }
  // Chế độ AUTO/MANUAL: đã xử lý trong executeCommand()
 
  delay(20);  // 50Hz loop
}


// ================== ĐỌC LINE SENSOR ==================
void readLineSensors(bool &left, bool &mid, bool &right) {
  // Đọc giá trị digital
  // HIGH (1) = đen (line), LOW (0) = trắng (nền)
  // KHÔNG đảo vì sensor của bạn: HIGH = thấy đen
  left = digitalRead(LINE_L);
  mid = digitalRead(LINE_M);
  right = digitalRead(LINE_R);
}


// ================== CHẠY THEO LINE ĐEN ==================
void followLine() {
  bool L, M, R;
  readLineSensors(L, M, R);
 
  // Debug (in mỗi 500ms)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    Serial.print("LINE: ");
    Serial.print(L ? "⚫" : "⚪");
    Serial.print(M ? "⚫" : "⚪");
    Serial.println(R ? "⚫" : "⚪");
    lastDebug = millis();
  }
 
  // ========== LOGIC LINE FOLLOWING ==========
  if (!L && !M && !R) {
        stopCar();
  }
  else if (!L && !M && R) {
        turnRightSpin();
  }
  else if (!L && M && !R) {
      moveForward();
  }
  else if (!L && M && R) {
        turnRightSpin();
  }
  else if (L && !M && !R) {
        turnLeftSpin();
  }
  else if (L && !M && R) {
   
    moveForward();
  }
  else if (L && M && !R) {
        turnLeftSpin();
  }
  else {
       moveForward();
  }
}


// ================== XỬ LÝ LỆNH MANUAL ==================
void executeCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeftSpin(); break;
    case 'R': turnRightSpin(); break;
    case 'S': stopCar(); break;
  }
}


// ================== HÀM XE ==================
void moveForward() {
  // Sử dụng tốc độ cao hơn cho chế độ line_follow
  int speed = lineFollowMode ? LINE_FOLLOW_SPEED : BASE_SPEED;
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);


  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);


  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  if (lastMotorState != 'F') {
    Serial.println(" Tiến");
    lastMotorState = 'F';
  }
}


void moveBackward() {
  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);


  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);


  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  if (lastMotorState != 'B') {
    Serial.println(" Lùi");
    lastMotorState = 'B';
  }
}


// ================== RẼ GẮT (SPIN TURN) ==================
void turnLeftSpin() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);


  // Bánh phải tiến
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);


  // Bánh trái lùi
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  if (lastMotorState != 'L') {
    Serial.println(" Rẽ trái GẮT");
    lastMotorState = 'L';
  }
}


void turnRightSpin() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);


  // Bánh phải lùi
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);


  // Bánh trái tiến
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  if (lastMotorState != 'R') {
    Serial.println(" Rẽ phải GẮT");
    lastMotorState = 'R';
  }
}


void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);


  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);


  if (lastMotorState != 'S') {
    Serial.println(" Dừng");
    lastMotorState = 'S';
  }
}







