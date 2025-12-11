# app/routers/devices.py
from fastapi import APIRouter
from pydantic import BaseModel

router = APIRouter(prefix="/device", tags=["Device"])

class ServoCommand(BaseModel):
    target: str  # 예: "on" / "off"

@router.post("/servo")
def control_servo(cmd: ServoCommand):
    # TODO: 여기서 실제 아두이노 직렬 통신 등으로 서보 제어
    # 예시: /dev/ttyUSB0 로 write 등
    # 현재는 더미 응답
    return {"status": "ok", "servo_state": cmd.target}
