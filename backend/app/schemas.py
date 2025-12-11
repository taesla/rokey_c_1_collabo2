from pydantic import BaseModel
from datetime import datetime
from typing import Optional

class PeopleCreate(BaseModel):
    military_serial: str
    name: str
    department: str
    rank: Optional[str] = None

class PeopleResponse(BaseModel):
    id: int
    military_serial: str
    name: str
    department: str
    rank: Optional[str]
    created_at: datetime
    picture: Optional[str] = None
    class Config:
        from_attributes = True


class AccessLogResponse(BaseModel):
    id: int
    military_serial: str
    in_time: datetime
    out_time: Optional[datetime]

    class Config:
        from_attributes = True
