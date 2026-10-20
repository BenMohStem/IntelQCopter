"""Materialize the factual JSON ledger into a searchable SQLite database."""
from pathlib import Path
import json
import sqlite3
root=Path(__file__).resolve().parents[1]
ledger=json.loads((root/"knowledge/project.json").read_text(encoding="utf8"))
with sqlite3.connect(root/"knowledge/project.sqlite") as db:
    db.execute("CREATE TABLE IF NOT EXISTS metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL)")
    db.execute("CREATE TABLE IF NOT EXISTS facts(id TEXT PRIMARY KEY,status TEXT NOT NULL,topic TEXT NOT NULL,statement TEXT NOT NULL,source TEXT NOT NULL)")
    db.execute("DELETE FROM metadata")
    db.execute("DELETE FROM facts")
    db.executemany("INSERT INTO metadata VALUES(?,?)",[(k,str(v)) for k,v in ledger.items() if k!="facts"])
    db.executemany("INSERT INTO facts VALUES(:id,:status,:topic,:statement,:source)",ledger["facts"])
    db.execute("CREATE INDEX IF NOT EXISTS facts_topic ON facts(topic)")
print(f"Updated project.sqlite: {len(ledger['facts'])} factual entries; JSON remains authoritative.")
