# app/routers/devices.py
from fastapi import APIRouter
from pydantic import BaseModel
import serial
import time

router = APIRouter(prefix="/device", tags=["Device"])

# 🔌 아두이노 시리얼 연결 (전역에서 1번만 연결 유지)
try:
    arduino = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

    time.sleep(2)  # 아두이노 리셋 대기
except Exception as e:
    print("⚠ 아두이노 연결 실패:", e)
    arduino = None

class ServoCommand(BaseModel):
    target: bool  # true = ON, false = OFF

@router.post("/servo")
def control_servo(cmd: ServoCommand):
    if not arduino:
        return {"status": "error", "msg": "Arduino not connected"}

    # 아두이노로 보낼 값 결정
    command = "1\n" if cmd.target else "0\n"

    # USB로 전송
    arduino.write(command.encode())

    return {"status": "ok", "servo_state": cmd.target}
