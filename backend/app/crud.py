# backend/app/crud.py

from sqlalchemy.orm import Session
from datetime import datetime, timedelta, timezone

from app.models import People, AccessLog
from app.schemas import PeopleCreate


def get_people(db: Session):
    return db.query(People).all()


def create_person(db: Session, data):
    person = People(
        military_serial=data["military_serial"],
        name=data["name"],
        department=data["department"],
        rank=data.get("rank"),
        picture=data.get("picture")  # Base64 or bytes
    )
    db.add(person)
    db.commit()
    db.refresh(person)
    return person


def get_logs(db: Session):
    return db.query(AccessLog).all()


def get_logs_by_serial(db: Session, serial: str):
    return db.query(AccessLog).filter(
        AccessLog.military_serial == serial
    ).order_by(AccessLog.in_time.desc()).all()


def is_inside(db: Session, serial: str):
    last = db.query(AccessLog).filter(
        AccessLog.military_serial == serial
    ).order_by(AccessLog.in_time.desc()).first()

    return bool(last and last.out_time is None)


def mark_entry(db: Session, serial: str):
    # validate person exists
    person = db.query(People).filter(People.military_serial == serial).first()
    if not person:
        return None, "invalid"

    # prevent double entry
    if is_inside(db, serial):
        return None, "already_inside"
    KST = timezone(timedelta(hours=9))
    log = AccessLog(military_serial=serial, in_time=datetime.now(KST))
    db.add(log)
    db.commit()
    db.refresh(log)
    return log, None


def mark_exit(db: Session, serial: str):
    last = db.query(AccessLog).filter(
        AccessLog.military_serial == serial,
        AccessLog.out_time == None
    ).order_by(AccessLog.in_time.desc()).first()

    if not last:
        return None, "not_inside"
    KST = timezone(timedelta(hours=9))
    last.out_time = datetime.now(KST)
    db.commit()
    db.refresh(last)
    return last, None

def delete_person(db: Session, serial: str):
    person = db.query(People).filter(People.military_serial == serial).first()
    if not person:
        return None
    
    db.delete(person)
    db.commit()
    return True

def get_person_by_serial(db: Session, serial: str):
    serial = serial.strip()
    return db.query(People).filter(People.military_serial == serial).first()
