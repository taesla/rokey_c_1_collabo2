# app/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.routers import people, access
from app.database import engine, Base

app = FastAPI(title="Security System API")

# DB 생성
Base.metadata.create_all(bind=engine)

# ✅ CORS 설정 (React Vite: 5173 포트 기준)
origins = [
    "http://localhost:5173",
    "http://127.0.0.1:5173",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 라우터 등록
app.include_router(people.router)
app.include_router(access.router)

# 나중에 추가할 장비 제어 라우터
from app.routers import devices
app.include_router(devices.router)

@app.get("/")
def hello():
    return {"msg": "Backend running!"}
