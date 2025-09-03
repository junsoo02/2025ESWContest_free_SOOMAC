#define ROS2_SERIAL Serial       // GUI/ROS2와 Serial 통신

bool stop_request = false;
bool stop_process = false;
bool robot_running = true;
int stock[6] = {0,0,0,0,0,0};

void setup() {
  ROS2_SERIAL.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

// 재고 감소
void decreaseStock(int index) {
  if(index>=0 && index<6){
    if(stock[index]>0) stock[index]--;
    ROS2_SERIAL.print("UPDATE:");
    for(int i=0;i<6;i++){
      ROS2_SERIAL.print(stock[i]);
      if(i<5) ROS2_SERIAL.print(",");
    }
    ROS2_SERIAL.println(":END");
  }
}

void loop() {
  if(ROS2_SERIAL.available()){
    String msg = ROS2_SERIAL.readStringUntil('\n'); 
    msg.trim();

    int code = msg.toInt();

    // 수행 슬롯 인덱스 (0~5)
    if(code >=0 && code <=5){
      if(!stop_process){
        decreaseStock(code);
        ROS2_SERIAL.print("drink_code=");
        ROS2_SERIAL.print(code);
        ROS2_SERIAL.println(" → 재고 감소 완료");
      } else {
        ROS2_SERIAL.println("ALERT: 음료수를 떨어뜨렸습니다! 재고 감소 안 함");
      }
    }

    // 테스크 사이클 종료 신호
    else if(code == 14){
      ROS2_SERIAL.println("drink_code=14 → 테스크 사이클 종료, 로봇 정지");
    }

    // 낙하 감지
    else if(msg=="stop_process"){
      stop_process=true;
      ROS2_SERIAL.println("ALERT: 음료수를 떨어뜨렸습니다!");
    }

    // IN / OUT 버튼
    else if(msg=="IN"){ 
      stop_request=true; 
      robot_running=false; 
      ROS2_SERIAL.println("MSG:현재 로봇이 실행 중입니다. 잠시만 기다려주세요."); // 레이아웃1
      ROS2_SERIAL.println("MSG:로봇을 정지합니다. 다시 실행할 경우 반드시 OUT 버튼을 눌러주세요."); // 레이아웃2
    }
    else if(msg=="OUT"){ 
      stop_request=false; 
      robot_running=true; 
      stop_process=false; 
      ROS2_SERIAL.println("MSG:OUT 버튼 수신 → 로봇 다시 실행"); 
    }

    // 재고 초기화
    else if(msg.startsWith("STOCK:")){
      msg.remove(0,6); 
      int idx=0;
      while(msg.length()>0 && idx<6){
        int commaIndex = msg.indexOf(','); 
        String token;
        if(commaIndex==-1){ token=msg; msg=""; }
        else{ token=msg.substring(0,commaIndex); msg=msg.substring(commaIndex+1); }
        stock[idx]=token.toInt(); 
        idx++;
      }
      ROS2_SERIAL.println("MSG:재고 초기화 완료");
    }
  }

  // 로봇 상태 LED 표시
  digitalWrite(LED_BUILTIN, robot_running?HIGH:LOW);
  delay(50);
}
