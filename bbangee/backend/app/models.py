from sqlalchemy import Column, Integer, String, DateTime, ForeignKey, LargeBinary
from datetime import datetime
from app.database import Base

class People(Base):
    __tablename__ = "people"

    id = Column(Integer, primary_key=True, index=True)
    military_serial = Column(String, unique=True, index=True)
    name = Column(String)
    department = Column(String)
    rank = Column(String, nullable=True)
    picture = Column(LargeBinary, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)


class AccessLog(Base):
    __tablename__ = "access_logs"

    id = Column(Integer, primary_key=True)
    military_serial = Column(String, ForeignKey("people.military_serial"))
    in_time = Column(DateTime, default=datetime.utcnow)
    out_time = Column(DateTime, nullable=True)
