import serial
import serial.tools.list_ports
import requests
import random, time
import threading
from collections import deque

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
MAX_BUFFER_SIZE = 10  # 최대 10개까지 버퍼링

def send_buffered_data():
    """별도 스레드에서 버퍼링된 데이터를 전송"""
    global data_buffer
    
    while True:
        time.sleep(0.1)  # 100ms마다 확인
        
        with buffer_lock:
            if not data_buffer:
                continue
            
            # 버퍼에서 데이터 꺼내기
            name, value = data_buffer.popleft()
        
        # 락 해제 후 네트워크 요청 (논블로킹)
        try:
            data = {'name': name, 'value': value}
            response = requests.post("http://localhost:8000/sensor/setProx", 
                                   data=data, timeout=2)
            print(f"[SENT] {data} -> {response.json()}")
        except Exception as e:
            print(f"[ERROR] Web Server error: {e}")
            # 전송 실패 시 버퍼에 다시 넣기
            with buffer_lock:
                data_buffer.appendleft((name, value))

# 백그라운드 전송 스레드 시작
sender_thread = threading.Thread(target=send_buffered_data, daemon=True)
sender_thread.start()
time.sleep(0.1)
portName = input("Enter the port name:")
if portName == "test":
    flag = True
try:
    ser = serial.Serial(
        port=portName,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.5  # 500ms 타임아웃 (더 빠른 응답)
    )
    print(f"Connected to {portName}")
except Exception as e:
    print(f"Invalid Value: {e}")

while True:
    # ------------ TEST MODE -------------
    if flag:
        try:
            name = 'Lavender'
            value = random.randint(0, 1)
            with buffer_lock:
                if len(data_buffer) < MAX_BUFFER_SIZE:
                    data_buffer.append((name, value))
                    print(f"[TEST] Buffered: name={name}, value={value} (buffer size: {len(data_buffer)})")
        except Exception as e:
            print(f"[ERROR] Buffer error: {e}")
        time.sleep(0.5)
        continue

    # ------------ SENSOR MODE (실제 센서 데이터 읽기) -------------
    try:
        # readline()은 timeout 설정에 따라 대기 (블로킹)
        if ser and ser.is_open:
            line = ser.readline().decode().strip()
            
            # 빈 줄 무시
            if not line:
                continue
            
            print(f"[RAW] {repr(line)}")

            # ===============================================
            # [새로 추가] 1. STATUS 메시지 처리 로직
            # 예상 포맷: "STATUS:ui_state=BLENDING,mood_name=Calm,L=8.0,C=4.0,V=2.0,B=2.0,lora_state=TX"
            if line.startswith("STATUS:"):
                try:
                    data_str = line.replace("STATUS:", "").strip()
                    parts = data_str.split(',')
                        
                    # 딕셔너리 형태로 파싱하여 POST 요청 데이터 준비
                    new_status = {}
                    for part in parts:
                        if '=' in part:
                            key, value = part.split('=', 1)
                            new_status[key.strip()] = value.strip()
                            
                    print(f"[STATUS] Parsed: {new_status}")
                        
                    # 새로운 엔드포인트로 전송 (HTTP POST)
                     # http://localhost:8000/sensor/setStatus 로 전송
                    response = requests.post("http://localhost:8000/sensor/setStatus", 
                                                 data=new_status, timeout=2)
                    print(f"[SENT STATUS] -> {response.status_code} {response.json()}")

                except Exception as e:
                    print(f"[ERROR] Status parse/send error: {e}")
                    
                continue # ⭐️ STATUS 메시지를 처리했으므로, 아래 기존 로직은 건너뜀
                
                # ===============================================

                # [기존 로직] 2. 잔량 센서 데이터 처리 로직 (STATUS가 아닐 경우만 실행됨)
            
            # Expected format: "Lavender : 1" 또는 "Lavender : 0"
            if " : " not in line:
                print(f"[ERROR] Invalid format: {line}")
                continue
            
            msg = line.split(" : ")
            if len(msg) != 2:
                print(f"[ERROR] Parse failed: expected 2 parts, got {len(msg)}")
                continue

            name = msg[0].strip()      # Lavender
            value = msg[1].strip()     # 1 or 0

            # 유효성 검사
            if value not in ['0', '1']:
                print(f"[ERROR] Invalid value: {value}")
                continue

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
        else:
            print("[ERROR] Serial port not open")
            
    except Exception as e:
        print(f"[ERROR] Serial read error: {e}")
        time.sleep(0.1)
