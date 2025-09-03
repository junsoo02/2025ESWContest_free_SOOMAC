#define NUM_SENSORS 6

int trigPins[NUM_SENSORS] = {2, 4, 6, 8, 10, 12};
int echoPins[NUM_SENSORS] = {3, 5, 7, 9, 11, 13};

String drinks[NUM_SENSORS] = {"Cider", "Coke", "Cass", "Pocari", "Samdasoo", "ZeroCoke"};

unsigned long lastPrintTime = 0;
const unsigned long cooldown = 10000; // 10초
int confirmCount[NUM_SENSORS] = {0};  // 각 센서별 카운트

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 최대 30ms
  if (duration == 0) return -1;
  return duration * 0.034 / 2; // cm
}

void loop() {
  // 쿨다운 확인
  if (millis() - lastPrintTime < cooldown) {
    delay(100);
    return;
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    long dist = readDistance(trigPins[i], echoPins[i]);

    if (dist >= 23 && dist <= 30) {
      confirmCount[i]++;  // 조건 만족 → 카운트 증가
      if (confirmCount[i] >= 3) {  // 3번 연속 확인 시 출력
        Serial.println(i + 1);
        lastPrintTime = millis(); // 쿨다운 시작
        confirmCount[i] = 0;      // 카운트 리셋
        break; // 한 번 출력 후 10초 대기
      }
    } else {
      confirmCount[i] = 0;  // 조건 안 맞으면 리셋
      Serial.print("센서 ");
      Serial.print(i + 1);
      Serial.print(" Error! 거리: ");
      if (dist == -1) Serial.println("NA");
      else {
        Serial.print(dist);
        Serial.println(" cm");
      }
    }

    delay(200); // 센서 간 간섭 방지 (조금 줄임)
  }
}
