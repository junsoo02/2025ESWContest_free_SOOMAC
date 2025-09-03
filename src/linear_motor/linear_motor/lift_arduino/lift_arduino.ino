int DIR1 = 4;
int PWM1 = 3;

void setup() {
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  Serial.begin(115200);
  Serial.println("Ready for commands: 33 / 44");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    int val = cmd.toInt();  // 문자열 → 정수 변환

    if (val == 44) {
      // 아래로 이동
      digitalWrite(DIR1, HIGH);
      analogWrite(PWM1, 150);
      delay(3000);  // 이동 시간 (Python 노드와 동일)
      analogWrite(PWM1, 0);
      Serial.println("Moved down (44)");

    } else if (val == 33) {
      // 위로 이동
      digitalWrite(DIR1, LOW);
      analogWrite(PWM1, 130);
      delay(4000);  // 이동 시간
      analogWrite(PWM1, 0);
      Serial.println("Moved up (33)");

    } else {
      Serial.println("Unknown command");
    }
  }
}