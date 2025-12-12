// src/pages/LoginPage.tsx
import React, { useState } from "react";
import { useNavigate } from "react-router-dom";

type Props = {
  onLoginSuccess: () => void;
};

const VALID_ID = "guard";
const VALID_PW = "1234";

export default function LoginPage({ onLoginSuccess }: Props) {
  const [id, setId] = useState("");
  const [pw, setPw] = useState("");
  const [error, setError] = useState("");
  const navigate = useNavigate();

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    if (id === VALID_ID && pw === VALID_PW) {
      onLoginSuccess();
      navigate("/main");
    } else {
      setError("인증 실패: 아이디 또는 비밀번호가 올바르지 않습니다.");
    }
  };

  return (
    <div className="auth-root">
      <div className="auth-panel">
        <h1 className="auth-title">GATE SECURITY CONSOLE</h1>
        <p className="auth-subtitle">AUTHORIZED PERSONNEL ONLY</p>

        <form onSubmit={handleSubmit} className="auth-form">
          <label className="auth-label">
            ID
            <input
              className="auth-input"
              value={id}
              onChange={(e) => setId(e.target.value)}
              autoComplete="username"
            />
          </label>

          <label className="auth-label">
            PASSWORD
            <input
              className="auth-input"
              type="password"
              value={pw}
              onChange={(e) => setPw(e.target.value)}
              autoComplete="current-password"
            />
          </label>

          {error && <div className="auth-error">{error}</div>}

          <button className="btn-primary" type="submit">
            ENTER CHECKPOINT
          </button>
        </form>

        <div className="auth-footer">LOGGED EVENTS ARE MONITORED IN REAL TIME</div>
      </div>
    </div>
  );
}
