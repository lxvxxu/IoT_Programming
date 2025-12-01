import serial
import serial.tools.list_ports
import requests
import random, time
import threading
from collections import deque

# -------------------- [자동 테스트 모드 데이터 정의] --------------------
RECIPES = [
    {"mood": "Fresh", "recipe": [1.0, 1.0, 1.0, 4.0]},
    {"mood": "Calm", "recipe": [4.0, 2.0, 1.0, 1.0]},
    {"mood": "Confident", "recipe": [1.0, 3.0, 1.0, 2.0]},
    {"mood": "Romantic", "recipe": [1.0, 1.0, 3.0, 2.0]},
    {"mood": "Energetic", "recipe": [0.5, 0.5, 0.5, 5.0]},
    {"mood": "Cozy", "recipe": [1.0, 1.5, 4.0, 0.5]},
    {"mood": "Deep/Mystic", "recipe": [0.5, 5.0, 1.0, 0.5]},
]
SENSOR_NAMES = ["Lavender", "Cedarwood", "Vanilla", "Bergamot"]

UI_STATES = ["START", "BLENDING_L", "BLENDING_C", "BLENDING_V", "BLENDING_B", "FINISH"]

STATE_DURATION = 2  # 각 상세 상태 유지 시간 (초)
current_test_state_index = 0 # 0부터 5까지 순환
current_mood_index = 0
last_transition_time = time.time()
# ------------------------------------------------------------------

ports = serial.tools.list_ports.comports()

print("=== Available COM Ports ===")
for port in ports:
    print(f"Port: {port.device}")
    print(f"Description: {port.description}")
    print(f"HWID: {port.hwid}")
    print("---------------------------")

flag = False
ser = None

# 데이터 버퍼와 락
data_buffer = deque()
buffer_lock = threading.Lock()
MAX_BUFFER_SIZE = 10 

def send_buffered_data():
    """별도 스레드에서 버퍼링된 (잔량 센서) 데이터를 전송"""
    global data_buffer
    
    while True:
        time.sleep(0.1) 
        
        with buffer_lock:
            if not data_buffer:
                continue
            
            # 버퍼에서 데이터 꺼내기
            name, value = data_buffer.popleft()
        
        # 락 해제 후 네트워크 요청 (논블로킹)
        try:
            # 잔량 센서는 setProx 엔드포인트로 전송 (name, value)
            data = {'name': name, 'value': int(value)}
            response = requests.post("http://localhost:8000/sensor/setProx", json=data, timeout=2)
            # Optionally inspect response.status_code / .json()
        except Exception as e:
            print(f"[ERROR] setProx Web Server error: {e}")
            # 전송 실패 시 버퍼에 다시 넣기
            with buffer_lock:
                # 맨 앞에 다시 넣어 재시도
                data_buffer.appendleft((name, value))

# 백그라운드 전송 스레드 시작
sender_thread = threading.Thread(target=send_buffered_data, daemon=True)
sender_thread.start()
time.sleep(0.1)

portName = input("Enter the port name (or 'test' for simulation): ")
if portName.lower() == "test":
    flag = True
    print("[INFO] Starting in TEST MODE: Detailed logic simulation enabled.")
else:
    try:
        ser = serial.Serial(
            port=portName,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5 
        )
        print(f"Connected to {portName}")
    except Exception as e:
        print(f"Invalid Value or cannot open port: {e}")
        ser = None

# 메인 루프: 테스트 모드와 시리얼 모드 분리
while True:
    try:
        if flag:
            # ------------- TEST MODE -------------
            current_time = time.time()
            
            # 1. 상태 전환 및 STATUS 메시지 전송 로직
            if current_time - last_transition_time >= STATE_DURATION:
                
                # 다음 상태로 전환 (0 -> 1 -> ... -> 5 -> 0)
                current_test_state_index = (current_test_state_index + 1) % len(UI_STATES)
                
                # FINISH 상태에서 START 상태로 전환될 때 (인덱스 0) 기분 사이클 전환
                if current_test_state_index == 0:
                    current_mood_index = (current_mood_index + 1) % len(RECIPES)
                
                mood_data = RECIPES[current_mood_index]
                L, C, V, B = mood_data["recipe"]
                ui_state = UI_STATES[current_test_state_index]
                
                # START 상태에서는 레시피를 0.0으로 리셋
                if ui_state == "START":
                    L, C, V, B = [0.0, 0.0, 0.0, 0.0]

                print(f"[TEST STATUS] Transition to: {ui_state} (Mood: {mood_data['mood']})")

                # 전송할 STATUS 데이터 구성
                status_data = {
                    'ui_state': ui_state,
                    'mood_name': mood_data["mood"],
                    'L': round(L, 1), 'C': round(C, 1), 'V': round(V, 1), 'B': round(B, 1),
                    'lora_state': 'IDLE' # 테스트에서는 'IDLE'로 고정
                }
                
                # STATUS 데이터 웹 서버로 전송 (setStatus 엔드포인트 사용)
                try:
                    response = requests.post("http://localhost:8000/sensor/setStatus", json=status_data, timeout=2)
                    # print("[TEST] setStatus result:", response.status_code, response.text)
                except Exception as e:
                    print(f"[ERROR] setStatus Web Server error: {e}")
                
                last_transition_time = current_time # 시간 갱신

                # 2. 잔량 센서 데이터 전송 (제조 가능 상태로 0.5초마다 전송)
                for name in SENSOR_NAMES:
                    # 제조 가능 = 0 (수위 정상)
                    value = 0
                    with buffer_lock:
                        if len(data_buffer) < MAX_BUFFER_SIZE:
                            data_buffer.append((name, value))
                        else:
                            data_buffer.popleft()
                            data_buffer.append((name, value))
            
                time.sleep(0.5) 
                continue
            # -------------------------------------
        else:
            # ----------------- SERIAL MODE -----------------
            if ser is None:
                print("[ERROR] Serial port not open")
                time.sleep(0.5)
                continue

            # 시리얼로부터 한 줄 읽기
            raw = ser.readline()
            if not raw:
                # 읽을 데이터가 없으면 잠깐 대기
                time.sleep(0.05)
                continue

            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                line = str(raw).strip()

            if not line:
                continue

            print(f"[RAW] {repr(line)}")

            # ===============================================
            # STATUS 메시지 처리 로직
            if line.startswith("STATUS:"):
                try:
                    data_str = line.replace("STATUS:", "", 1).strip()
                    parts = data_str.split(',')
                        
                    # 딕셔너리 형태로 파싱하여 POST 요청 데이터 준비
                    new_status = {}
                    for part in parts:
                        if '=' in part:
                            key, value = part.split('=', 1)
                            new_status[key.strip()] = value.strip()
                            
                    print(f"[STATUS] Parsed: {new_status}")
                        
                    # 새로운 엔드포인트로 전송
                    response = requests.post("http://localhost:8000/sensor/setStatus", json=new_status, timeout=2)
                    try:
                        print(f"[SENT STATUS] -> {response.status_code} {response.json()}")
                    except Exception:
                        print(f"[SENT STATUS] -> {response.status_code} (no json)")
                except Exception as e:
                    print(f"[ERROR] Status parse/send error: {e}")
                    
                continue # STATUS 메시지 처리 후 다음 루프

            # -----------------------------------------------
            # 잔량 센서 데이터 처리 로직 (예: "Lavender : 1")
            if " : " not in line:
                print(f"[ERROR] Invalid format: {line}")
                continue
            
            msg = line.split(" : ")
            if len(msg) != 2:
                print(f"[ERROR] Parse failed: expected 2 parts, got {len(msg)}")
                continue

            name = msg[0].strip()
            value_str = msg[1].strip()

            # 유효성 검사
            if value_str not in ['0', '1']:
                print(f"[ERROR] Invalid value: {value_str}")
                continue

            value = int(value_str)
            print(f"[PARSED] name={name}, value={value}")

            # 버퍼에 데이터 추가 (논블로킹)
            with buffer_lock:
                if len(data_buffer) < MAX_BUFFER_SIZE:
                    data_buffer.append((name, value))
                    print(f"[BUFFERED] Added to buffer (size: {len(data_buffer)}/{MAX_BUFFER_SIZE})")
                else:
                    print(f"[WARNING] Buffer full, dropping oldest data")
                    data_buffer.popleft()
                    data_buffer.append((name, value))

            # ------------------------------------------------
    except Exception as e:
        print(f"[ERROR] Main loop error: {e}")
        time.sleep(0.1)
