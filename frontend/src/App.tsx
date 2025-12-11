// src/App.tsx
import React, { useState } from "react";
import { Routes, Route, Navigate, useNavigate } from "react-router-dom";
import LoginPage from "./pages/LoginPage";
import MainPage from "./pages/MainPage";
import AdminPage from "./pages/AdminPage";

export default function App() {
  const [isAuthed, setIsAuthed] = useState<boolean>(
    !!localStorage.getItem("auth_token")
  );

  const handleLogin = () => {
    localStorage.setItem("auth_token", "dummy");
    setIsAuthed(true);
  };

  const handleLogout = () => {
    localStorage.removeItem("auth_token");
    setIsAuthed(false);
  };

  return (
    <Routes>
      <Route
        path="/login"
        element={
          isAuthed ? (
            <Navigate to="/main" replace />
          ) : (
            <LoginPage onLoginSuccess={handleLogin} />
          )
        }
      />
      <Route
        path="/main"
        element={
          isAuthed ? (
            <MainPage onLogout={handleLogout} />
          ) : (
            <Navigate to="/login" replace />
          )
        }
      />
      <Route
        path="/admin"
        element={
          isAuthed ? (
            <AdminPage onLogout={handleLogout} />
          ) : (
            <Navigate to="/login" replace />
          )
        }
      />
      <Route path="*" element={<Navigate to="/login" replace />} />
    </Routes>
  );
}
