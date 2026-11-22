"""SQLite store; the repository API is isolated so the storage engine can change."""
from __future__ import annotations
import json, sqlite3
from pathlib import Path
from datetime import datetime, timezone
from typing import Protocol
import threading
import os

DB_PATH = Path(os.environ.get("DROVER_DB_PATH", Path(__file__).resolve().parents[2] / "drover.sqlite3"))

class Repository(Protocol):
    def create_run(self, run_id, name, configuration, metadata): ...
    def record(self, run_id, row): ...
    def finish(self, run_id, status): ...
    def fail(self, run_id, reason): ...
    def list_runs(self): ...
    def rows(self, run_id): ...

class Store:
    def __init__(self, path=DB_PATH):
        self.path=Path(path)
        self._buffers={}
        self._buffer_lock=threading.Lock()
        self.path.parent.mkdir(parents=True,exist_ok=True)
        with self.connect() as db:
            db.executescript("""
            PRAGMA foreign_keys=ON;
            CREATE TABLE IF NOT EXISTS runs(id TEXT PRIMARY KEY,name TEXT,configuration TEXT NOT NULL,metadata TEXT NOT NULL,started_utc TEXT NOT NULL,ended_utc TEXT,status TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS telemetry(run_id TEXT NOT NULL REFERENCES runs(id),simulation_time_s REAL NOT NULL,payload TEXT NOT NULL,PRIMARY KEY(run_id,simulation_time_s));
            CREATE TABLE IF NOT EXISTS analyses(id INTEGER PRIMARY KEY,run_id TEXT NOT NULL REFERENCES runs(id),result TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS physical_measurements(id TEXT PRIMARY KEY,run_id TEXT NOT NULL REFERENCES runs(id),field TEXT NOT NULL,units TEXT,records TEXT NOT NULL,comparison TEXT NOT NULL);
            """)
    def connect(self):
        db=sqlite3.connect(self.path)
        db.execute("PRAGMA foreign_keys=ON")
        return db
    def create_run(self, run_id, name, configuration, metadata):
        with self._buffer_lock: self._buffers[run_id]=[]
        with self.connect() as db:
            db.execute("INSERT INTO runs VALUES(?,?,?,?,?,?,?)",(run_id,name,json.dumps(configuration),json.dumps(metadata),datetime.now(timezone.utc).isoformat(),None,"running"))
    def record(self, run_id, row):
        row={**row,"experiment_id":run_id}
        flush=None
        with self._buffer_lock:
            buf=self._buffers.setdefault(run_id,[]); buf.append((run_id,row["simulation_time_s"],json.dumps(row)))
            if len(buf)>=100: flush=buf[:]; buf.clear()
        if flush: self._write_rows(flush)
    def _write_rows(self, rows):
        with self.connect() as db: db.executemany("INSERT OR REPLACE INTO telemetry VALUES(?,?,?)",rows)
    def finish(self, run_id, status):
        with self._buffer_lock: pending=self._buffers.pop(run_id,[])
        if pending: self._write_rows(pending)
        with self.connect() as db: db.execute("UPDATE runs SET status=?,ended_utc=? WHERE id=?",(status,datetime.now(timezone.utc).isoformat(),run_id))
    def fail(self, run_id, reason):
        run=self.run(run_id); meta=run["metadata"]; meta["failure_reason"]=str(reason)
        with self.connect() as db: db.execute("UPDATE runs SET metadata=? WHERE id=?",(json.dumps(meta),run_id))
        self.finish(run_id,"failed")
    def list_runs(self):
        with self.connect() as db: return [dict(id=r[0],name=r[1],metadata=json.loads(r[2]),started_utc=r[3],ended_utc=r[4],status=r[5]) for r in db.execute("SELECT id,name,metadata,started_utc,ended_utc,status FROM runs ORDER BY started_utc DESC")]
    def rows(self, run_id):
        with self.connect() as db:
            if not db.execute("SELECT 1 FROM runs WHERE id=?",(run_id,)).fetchone(): raise KeyError(run_id)
            return [json.loads(r[0]) for r in db.execute("SELECT payload FROM telemetry WHERE run_id=? ORDER BY simulation_time_s",(run_id,))]
    def run(self, run_id):
        with self.connect() as db:
            r=db.execute("SELECT id,name,configuration,metadata,started_utc,ended_utc,status FROM runs WHERE id=?",(run_id,)).fetchone()
            if not r: raise KeyError(run_id)
            return dict(id=r[0],name=r[1],configuration=json.loads(r[2]),metadata=json.loads(r[3]),started_utc=r[4],ended_utc=r[5],status=r[6])
    def save_measurement_comparison(self, measurement_id, run_id, field, units, records, comparison):
        with self.connect() as db:
            db.execute("INSERT INTO physical_measurements VALUES(?,?,?,?,?,?)",(measurement_id,run_id,field,units,json.dumps(records),json.dumps(comparison)))
            db.execute("INSERT INTO analyses(run_id,result) VALUES(?,?)",(run_id,json.dumps(comparison)))
    def save_analysis(self, run_id, result):
        with self.connect() as db: db.execute("INSERT INTO analyses(run_id,result) VALUES(?,?)",(run_id,json.dumps(result)))
    def measurements(self, run_id):
        with self.connect() as db:
            if not db.execute("SELECT 1 FROM runs WHERE id=?",(run_id,)).fetchone(): raise KeyError(run_id)
            return [dict(id=r[0],field=r[1],units=r[2],records=json.loads(r[3]),comparison=json.loads(r[4])) for r in db.execute("SELECT id,field,units,records,comparison FROM physical_measurements WHERE run_id=?",(run_id,))]
