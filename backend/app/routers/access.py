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


@router.get("/logs", response_model=list[AccessLogResponse])
def logs(db: Session = Depends(get_db)):
    return crud.get_logs(db)


@router.post("/{serial}/entry", response_model=AccessLogResponse)
def entry(serial: str, db: Session = Depends(get_db)):
    log, err = crud.mark_entry(db, serial)

    if err == "invalid":
        raise HTTPException(404, "Person not found")

    if err == "already_inside":
        raise HTTPException(409, "User already entered")

    return log


@router.post("/{serial}/exit", response_model=AccessLogResponse)
def exit(serial: str, db: Session = Depends(get_db)):
    log, err = crud.mark_exit(db, serial)

    if err == "not_inside":
        raise HTTPException(409, "User has no active entry")

    return log
