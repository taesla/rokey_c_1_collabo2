// src/components/ServoControl.tsx
import React, { useState } from "react";
import { api } from "../api/client";

export default function ServoControl() {
  const [state, setState] = useState<"idle" | "on" | "off">("idle");
  const [loading, setLoading] = useState(false);

  const sendCommand = async (target: "on" | "off") => {
    setLoading(true);
    try {
      await api.post("/device/servo", { target });
      setState(target);
    } catch (e) {
      console.error(e);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="panel servo-panel">
      <div className="panel-header">
        <span className="panel-title">SERVO CONTROL</span>
        <span className="panel-tag">WEAPON MOUNT</span>
      </div>
      <div className="panel-body servo-body">
        <div className="servo-status">
          현재 상태:{" "}
          <span className={`servo-state servo-state-${state}`}>
            {state.toUpperCase()}
          </span>
        </div>
        <div className="servo-buttons">
          <button
            className="btn-primary"
            disabled={loading}
            onClick={() => sendCommand("on")}
          >
            ARM - SERVO ON
          </button>
          <button
            className="btn-danger"
            disabled={loading}
            onClick={() => sendCommand("off")}
          >
            SAFE - SERVO OFF
          </button>
        </div>
      
      </div>
    </div>
  );
}
