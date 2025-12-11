import React, { useEffect, useState } from "react";
import { fetchPeople, searchPerson } from "../api/people";
import type { Person } from "../api/people";
import { fetchLogs, recordIn, recordOut } from "../api/access";
import { fetchLogsBySerial } from "../api/access";
import type { AccessLog } from "../api/access";


export default function PersonnelPanel() {
  const [latestPerson, setLatestPerson] = useState<Person | null>(null);
  const [latestLog, setLatestLog] = useState<AccessLog | null>(null);
  const [events, setEvents] = useState<string[]>([]);
  const [photoModalOpen, setPhotoModalOpen] = useState(false);
  const [searchSerial, setSearchSerial] = useState("");

  /** 📌 최신 People + Logs 불러오기 */
  const load = async () => {
    const people = await fetchPeople();
    if (people.length > 0) setLatestPerson(people[people.length - 1]);

    const logs = await fetchLogs();
    if (logs.length > 0) setLatestLog(logs[logs.length - 1]);
  };

  /** 🔁 최초 로드 + 2초 refresh */
  useEffect(() => {
    load();
    const refresh = setInterval(load, 2000);
    return () => clearInterval(refresh);
  }, []);

  /** 📌 inside 상태 계산 */
  const isInside = latestLog && latestLog.out_time === null;

  /** 📌 이벤트 push helper */
  const pushEvent = (text: string) => {
    setEvents((prev) => {
      const next = [`[${new Date().toLocaleTimeString()}] ${text}`, ...prev];
      return next.slice(0, 10);
    });
  };

  /** 🔍 수동 검색 */
 const handleSearch = async () => {
  if (!searchSerial.trim()) return;
  try {
    const person = await searchPerson(searchSerial.trim());
    setLatestPerson(person);

    const logs = await fetchLogsBySerial(person.military_serial);
    if (logs.length > 0) setLatestLog(logs[logs.length - 1]);
    else setLatestLog(null);

    pushEvent(`✔ ${person.name} 검색됨`);
  } catch (err) {
    pushEvent("❌ 해당 군번의 사용자가 없습니다.");
  }
};
  /** 🚪 CHECK-IN */
  const handleCheckIn = async () => {
    if (!latestPerson) return;
    if (isInside) return pushEvent("⚠ 이미 입실 상태입니다. 퇴실 먼저 하세요.");

    await recordIn(latestPerson.military_serial);
    await load();
    pushEvent("✔ CHECK-IN 승인");
  };

  /** 🚪 CHECK-OUT */
  const handleCheckOut = async () => {
    if (!latestPerson) return;
    if (!isInside) return pushEvent("⚠ 입실 기록이 없어 퇴실할 수 없습니다.");

    await recordOut(latestPerson.military_serial);
    await load();
    pushEvent("✔ CHECK-OUT 처리");
  };

  return (
    <div className="panel personnel-panel">
      <div className="panel-header">
        <span className="panel-title">PERSONNEL STATUS</span>
        <span className="panel-tag">ID CHECK · ACCESS</span>
      </div>

      <div className="panel-body personnel-body">
        {/* ───── 검색 UI ───── */}
        <div className="manual-search">
          <input
            placeholder="군번 검색 (예: 19-76034359)"
            value={searchSerial}
            onChange={(e) => setSearchSerial(e.target.value)}
          />
          <button className="btn-primary" onClick={handleSearch}>SEARCH</button>
        </div>

        {latestPerson ? (
          <>
            {/* ───── 사진 + 정보 ───── */}
            <div className="person-main">
              <div className="person-photo-holder">
                {latestPerson.picture ? (
                  <img
                    className="person-photo"
                    src={`data:image/jpeg;base64,${latestPerson.picture}`}
                    alt="face"
                    onClick={() => setPhotoModalOpen(true)}
                  />
                ) : (
                  <div className="no-photo">NO PHOTO</div>
                )}
              </div>

              <div className="person-info">
                <div className="person-name">
                  {latestPerson.name}
                  <span className="person-rank">{latestPerson.rank}</span>
                </div>
                <div className="person-meta">
                  <div>군번: {latestPerson.military_serial}</div>
                  <div>소속: {latestPerson.department}</div>
                </div>
              </div>
            </div>

            {/* ───── 최근 출입 로그 ───── */}
            <div className="person-log">
              <h3>최근 출입 로그</h3>
              {latestLog ? (
                <ul>
                  <li>IN: {new Date(latestLog.in_time).toLocaleString()}</li>
                  <li>OUT: {latestLog.out_time ? new Date(latestLog.out_time).toLocaleString() : "미기록"}</li>
                </ul>
              ) : (
                <p>로그 데이터 없음</p>
              )}
            </div>

            {/* ───── ACTION 버튼 ───── */}
            <div className="person-actions">
              <button className="btn-primary" disabled={isInside} onClick={handleCheckIn}>CHECK IN</button>
              <button className="btn-danger" disabled={!isInside} onClick={handleCheckOut}>CHECK OUT</button>
            </div>

            {/* ───── EVENT FEED 스크롤 ───── */}
            <div className="person-events">
              <h3>EVENT FEED</h3>
              <div className="event-scroll">
                {events.length === 0 ? (
                  <p>대기 중...</p>
                ) : (
                  <ul>{events.map((e, i) => <li key={i}>{e}</li>)}</ul>
                )}
              </div>
            </div>

            {/* ───── 사진 확대 팝업 ───── */}
            {photoModalOpen && latestPerson.picture && (
              <div className="photo-modal-bg" onClick={() => setPhotoModalOpen(false)}>
                <img
                  className="photo-modal-img"
                  src={`data:image/jpeg;base64,${latestPerson.picture}`}
                  alt="face large"
                />
              </div>
            )}
          </>
        ) : (
          <div className="person-empty">
            등록된 인원이 없습니다.
            <br /> Admin Panel에서 인원 등록 후 사용하세요.
          </div>
        )}
      </div>
    </div>
  );
}
