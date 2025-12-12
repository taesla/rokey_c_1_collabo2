# backend/app/routers/access.py
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.database import SessionLocal
from app import crud
from app.schemas import AccessLogResponse

router = APIRouter(prefix="/access", tags=["Access Logs"])

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# 🔹 전체 로그 조회
@router.get("/logs", response_model=list[AccessLogResponse])
def logs(db: Session = Depends(get_db)):
    return crud.get_logs(db)


# 🔹 특정 군번 로그 조회 (프론트에서 요청하는 URL)
@router.get("/logs/{serial}", response_model=list[AccessLogResponse])
def logs_by_serial(serial: str, db: Session = Depends(get_db)):
    return crud.get_logs_by_serial(db, serial)


# 🔹 체크인
@router.post("/{serial}/entry", response_model=AccessLogResponse)
def entry(serial: str, db: Session = Depends(get_db)):
    log, err = crud.mark_entry(db, serial)

    if err == "invalid":
        raise HTTPException(404, "Person not found")

    if err == "already_inside":
        raise HTTPException(409, "User already entered")

    return log


# 🔹 체크아웃
@router.post("/{serial}/exit", response_model=AccessLogResponse)
def exit(serial: str, db: Session = Depends(get_db)):
    log, err = crud.mark_exit(db, serial)

    if err == "not_inside":
        raise HTTPException(409, "User has no active entry")

    return log
