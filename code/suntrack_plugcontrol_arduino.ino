/*
  blue test: 
  http://www.kccistc.net/
  작성일 : 2024.03.22
  작성자 : IoT 임베디드 KSH
*/
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define DEBUG
#define RELAY1_PIN 7
#define RELAY2_PIN 8
#define LED_BUILTIN_PIN 13

#define ARR_CNT 10
#define CMD_SIZE 60

char lcdLine1[17] = " SOLAR.P SYSTEM ";
char lcdLine2[17] = "";

char sendBuf[CMD_SIZE];
char recvId[10] = "LT_SQL";  // SQL 저장 클라이이언트 ID

bool plug1State = false;  // plug1의 현재 상태 (on/off)
bool plug2State = false;  // plug2의 현재 상태 (on/off)

int pos = 0;        // pos 변수 (정수형)
float solar = 0.0;  // solar 변수 (실수형)

unsigned long previousMillis = 0;
const long interval = 2000;  // 2초 간격

bool timerIsrFlag = false;
int getSensorTime;
bool updatTimeFlag = false;

SoftwareSerial BTSerial(10, 11);  // RX ==>BT:TXD, TX ==> BT:RXD

void lcdDisplay(int x, int y, char* str);

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("setup() start!");
#endif

  lcd.init();
  lcd.backlight();
  lcdDisplay(0, 0, lcdLine1);
  lcdDisplay(0, 1, lcdLine2);

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);  // 릴레이 기본 상태 OFF
  digitalWrite(RELAY2_PIN, LOW);  // 릴레이 기본 상태 OFF

  BTSerial.begin(9600);  // set the data rate for the SoftwareSerial port

  sprintf(sendBuf, "[LT_SQL]SETDB@PLUG1@OFF@RGB_AND\n");
  BTSerial.write(sendBuf);
  delay(1000);
  sprintf(sendBuf, "[LT_SQL]SETDB@PLUG2@OFF@RGB_AND\n");
  BTSerial.write(sendBuf);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // pos 요청
    sprintf(sendBuf, "[LT_SQL]GETDB@KCCI\n");
    BTSerial.write(sendBuf);
  }

  if (BTSerial.available())
    bluetoothEvent();

  // LCD에 릴레이 상태 출력
  sprintf(lcdLine2, "P1:%s P2:%s", plug1State ? "ON " : "OFF", plug2State ? "ON " : "OFF");
  lcdDisplay(0, 1, lcdLine2);
}

void bluetoothEvent() {
  int i = 0;
  char* pToken;
  char* pArray[ARR_CNT] = { 0 };
  char recvBuf[CMD_SIZE] = { 0 };
  int len = 0;

#ifdef DEBUG
  Serial.print("Received raw BT data: ");
  Serial.println(recvBuf);
#endif
  if (BTSerial.available()) {
    len = BTSerial.readBytesUntil('\n', recvBuf, sizeof(recvBuf) - 1);
    recvBuf[len] = '\0';
    pToken = strtok(recvBuf, "[@]");
    while (pToken != NULL) {
      pArray[i] = pToken;
      if (++i >= ARR_CNT)
        break;
      pToken = strtok(NULL, "[@]");
    }
  }
  char posStr[10];
  char solStr[10];
  // 응답 예시: [LT_SQL]@KCCI@Pos@BL@Solar@4.350
  if (i >= 4 && strcmp(pArray[1], "GETDB") == 0) {
    strncpy(posStr, pArray[4], sizeof(posStr) - 1);
    posStr[sizeof(posStr) - 1] = '\0';
    strncpy(solStr, pArray[6], sizeof(solStr) - 1);

    sprintf(lcdLine1, "POS:%s SOL:%s", posStr, solStr);
    lcdDisplay(0, 0, lcdLine1);
  }

  //recvBuf : [XXX_BTM]LED@ON
  //pArray[0] = "XXX_LIN"   : 송신자 ID
  //pArray[1] = "LED"
  //pArray[2] = "ON"
  //pArray[3] = 0x0
  /*if ((strlen(pArray[1]) + strlen(pArray[2])) < 16) {
    sprintf(lcdLine2, "%s %s", pArray[1], pArray[2]);
    lcdDisplay(0, 1, lcdLine2);
  }*/

  if (!strcmp(pArray[0], "LT_ARD")) {
    return;
  }

 if (!strcmp(pArray[1], "PLUG1")) {
    Serial.println("PLUG1 command recognized");
    if (!strcmp(pArray[2], "ON")) {
      digitalWrite(RELAY1_PIN, HIGH);
      plug1State = true;
    }
    else if (!strcmp(pArray[2], "OFF")) {
      digitalWrite(RELAY1_PIN, LOW);
      plug1State = false;
    }
    sprintf(sendBuf, "[LT_SQL]SETDB@%s@%s@%s\n", pArray[1], plug1State ? "ON" : "OFF", pArray[0]);
    BTSerial.write(sendBuf);

  } else if (!strcmp(pArray[1], "PLUG2")) {
    Serial.println("PLUG2 command recognized");
    if (!strcmp(pArray[2], "ON")) {
      digitalWrite(RELAY2_PIN, HIGH);
      plug2State = true;

    } else if (!strcmp(pArray[2], "OFF")) {
      digitalWrite(RELAY2_PIN, LOW);
      plug2State = false;
    }
    sprintf(sendBuf, "[LT_SQL]SETDB@%s@%s@%s\n", pArray[1], plug2State ? "ON" : "OFF", pArray[0]);
    BTSerial.write(sendBuf);
  }
/*
  else if (!strcmp(pArray[1], "GETSTATE")) {
    if (pArray[2] == NULL) {
      getSensorTime = 0;
    } else {
      getSensorTime = atoi(pArray[2]);
      strcpy(recvId, pArray[0]);
    }
  }
  */
}

void lcdDisplay(int x, int y, char* str) {
  int len = 16 - strlen(str);
  lcd.setCursor(x, y);
  lcd.print(str);
  for (int i = len; i > 0; i--)
    lcd.write(' ');
}
