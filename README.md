# 빛 추적 스마트 전력 제어 시스템  
<!-- <img width="1840" height="777" alt="Image" src="https://github.com/user-attachments/assets/3232f946-62a0-4f54-866c-43c3ea1adb5a" /> -->
<!-- <img width="2308" height="685" alt="Image" src="https://github.com/user-attachments/assets/335bddba-67d5-42a7-bbe8-9c78f891ad86" /> -->

## 1. 프로젝트 개요
빛 추적 스마트 전력 제어 시스템은 빛 추적 태양패널로 발전 효율을 높이고 스마트 콘센트로 전력을 제어하는 IoT 기반 에너지 관리 프로젝트입니다.

기존 고정형 태양광 패널은 태양의 이동이나 주변 장애물에 따른 조도 변화에 대응하지 못해 발전 효율이 떨어집니다.  
이를 해결하기 위해 CDS 센서 기반 태양광 추적 기능을 구현하는 한편, 효과적으로 에너지를 관리할 수 있도록 패널 관련 정보 실시간 서버 저장, 스마트 콘센트 제어, LCD 시각화 UI 기능을 포함하는 통합 에너지 관리 시스템을 구축했습니다.


### 전체 시스템 구성
<!-- <img width="960" height="540" alt="Image" src="https://github.com/user-attachments/assets/43fa549c-5ee2-49ef-9705-b3aec7b91486" /> -->

- #### **STM32 Nucleo-F411RE**
  - 8개의 CDS 센서 입력
  - 태양광 패널 자동 회전(스텝 모터)
  - 태양광 패널 전압 측정
  - Wi-Fi(ESP 모듈)로 Raspberry Pi 서버에 데이터 송신

- #### **Raspberry Pi 5 (서버)**
  - 소켓 서버(iot_server.c)
  - MariaDB 데이터 저장
  - Bluetooth로 Arduino와 데이터 송수신

- #### **Arduino UNO**
  - 패널 방향 / 전압 / 콘센트 상태 LCD 출력
  - 사용자 콘센트 ON/OFF 제어(릴레이)
  - Bluetooth로 명령 수신


### 🔗 통신 구조
- STM32 ↔ **Wi-Fi** ↔ Raspberry Pi  
- Arduino ↔ **Bluetooth** ↔ Raspberry Pi

---

## 2. 주요 기능

### 1) STM32 — 태양광 패널 자동 추적 및 센서 데이터 송신
- 8개 CDS 센서로 빛의 강도 측정
- 가장 밝은 방향으로 패널 자동 회전
- 태양광 패널 전압 측정
- Wi-Fi(ESP 모듈)로 Raspberry Pi에 실시간 정보 송신
<!-- <img width="960" height="540" alt="Image" src="https://github.com/user-attachments/assets/78693961-5826-4d20-9175-7829c63f153f" /> -->

### 2) Raspberry Pi 5 — IoT 서버 + 데이터베이스 저장
- **iot_server.c**
  - 멀티 클라이언트 소켓 연결
  - ID/PW 기반 인증 처리
  - 메시지 라우팅 (ALLMSG, 특정 ID 전송)

- **sql_client.c**
  - “[LT_STM_SQL]SENSOR@…” 패킷 파싱
  - MariaDB(sensor 테이블)에 실시간 INSERT

- **bt_client.c**
  - Bluetooth HC-06 모듈 사용
  - Arduino에 명령 전달
  - Arduino 상태 재수신하여 서버로 전달
<!-- <img width="960" height="540" alt="Image" src="https://github.com/user-attachments/assets/df6e9a35-0a65-4247-82fc-e0819f64ae56" /> -->
<!-- <img width="960" height="540" alt="Image" src="https://github.com/user-attachments/assets/64098ebe-fb36-40ef-9fb6-705e0fad388c" /> -->

### 3) Arduino UNO — LCD UI + 스마트 콘센트 제어
- Raspberry Pi → Bluetooth로 데이터 수신
- LCD에 패널 방향, 전압, 콘센트 상태 출력
- 사용자 버튼 입력으로 콘센트 ON/OFF
- 릴레이 제어 + 상태를 Raspberry Pi로 다시 송신
<!-- <img width="960" height="540" alt="Image" src="https://github.com/user-attachments/assets/0c44df98-3c01-47f0-9a35-7bc1c6404dd3" /> -->

---

## 3. 담당 역할

### Raspberry Pi **IoT 서버 설계 및 구현 (iot_server 계열)**

#### ① STM32 ↔ ESP8266 Wi-Fi 통신 모듈 연동 개발
- STM32와 ESP8266 간 UART 통신 제어
- 서버(Raspberry Pi)와 실시간 센서 데이터 송신 구조 구현
- AT Command 기반 Wi-Fi 설정

#### ② 센서 데이터 패킷 설계 및 송신 로직 구현
- CDS 센서 값 / 태양광 패널 전압 데이터 패킷 포맷 정의
- 서버 수신을 고려한 문자열 기반 패킷 구성
- 데이터 손실 방지를 위한 송신 타이밍 및 포맷 안정화

#### ③ 서버 연동 테스트 및 통신 검증
- Raspberry Pi 소켓 서버와 ESP8266 TCP 통신 테스트
- 패킷 수신/DB 저장 여부 검증

---

## 4. 트러블슈팅
### 1) **데이터 송신 중 스텝 모터 끊김 현상**  
- 원인: Wi-Fi 송신(ESP) 루틴이 블로킹으로 동작  
- 해결: **TIM4 기반 논블로킹(non-blocking)방식 스텝 모터 제어 구조 구현**  
- 결과: 모터 회전 안정성 확보  

### 2) **스텝 모터 회전 시 전압 강하 → CDS 값 일시적 하락**  
- 원인: 모터 구동 시 순간적인 전류 피크 → ADC 레일 변동  
- 해결: **모터 전원과 MCU/센서 전원을 완전 분리**  
- 결과: CDS 값 일시적 하락 현상 해결  

### 3) **LCD 잔상(ghosting) 발생** 
- 원인: 문자열 길이 변화로 이전 텍스트 일부가 남음  
- 해결:  
  - `lcd.clear()` 반복 호출 지양  
  - partial update 구조 도입 (변경된 라인만 업데이트)  
  - 공백 패딩으로 잔상 제거  
- 결과: LCD 화면이 깨끗하게 유지되고 출력 속도 향상  

---
