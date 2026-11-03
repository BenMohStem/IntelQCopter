"""Exclude superseded pose attempts, preserving original CSV evidence.

For one capture session (including _part02 checkpoints), the largest
phase_attempt for each (pose, mag_stage) supersedes previous attempts.
Old CSVs without phase_attempt are backward compatible and use attempt 0.
"""
from pathlib import Path
import re


def session_key(path):
    p=Path(path)
    return str(p.with_name(re.sub(r'_part\d+$','',p.stem)))


def attempt_key(row):
    return (session_key(row.get('_source_file','')),row.get('pose',''),row.get('mag_stage','STILL') or 'STILL')


def latest_attempts(rows):
    last={}
    for row in rows:
        if row.get('kind') not in (None,'D','L'):
            continue
        try: n=int(row.get('phase_attempt') or 0)
        except ValueError: n=0
        key=attempt_key(row)
        if key not in last or n>last[key]: last[key]=n
    return last


def only_latest(rows):
    rows=list(rows)
    last=latest_attempts(rows)
    kept=[]
    for row in rows:
        try: n=int(row.get('phase_attempt') or 0)
        except ValueError: n=0
        if n==last.get(attempt_key(row),0): kept.append(row)
    return kept
