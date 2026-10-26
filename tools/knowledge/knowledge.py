from contextlib import closing
"""Git-backed project facts and transactional, local multi-agent reservations."""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess
import uuid

ROOT = Path(__file__).resolve().parents[2]
KB = ROOT / 'knowledge'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def records():
    return {p.stem: read(p) for p in (KB / 'tasks').glob('*.json')}


def conflicts(a, b):
    a = a.replace('\\', '/').strip('/').casefold()
    b = b.replace('\\', '/').strip('/').casefold()
    return a == b or a.startswith(b + '/') or b.startswith(a + '/')


def database():
    common = subprocess.check_output(['git', 'rev-parse', '--git-common-dir'], cwd=ROOT, text=True).strip()
    path = (ROOT / common).resolve() / 'intelqcopter-coordination.sqlite3'
    connection = sqlite3.connect(path, timeout=10)
    connection.execute('CREATE TABLE IF NOT EXISTS claims (scope TEXT PRIMARY KEY, task TEXT, owner TEXT, token TEXT, created TEXT, review_after TEXT)')
    return connection


def reserve(connection, task, owner, scopes, minutes):
    if not owner.strip() or minutes < 1 or not scopes:
        raise ValueError('Require owner, scopes and positive review interval')
    now = dt.datetime.now(dt.timezone.utc)
    token = str(uuid.uuid4())
    connection.execute('BEGIN IMMEDIATE')
    try:
        for scope, existing_task, existing_owner in connection.execute('SELECT scope, task, owner FROM claims'):
            if any(conflicts(scope, wanted) for wanted in scopes):
                raise ValueError(f'Conflict: {scope} owned by {existing_owner} for {existing_task}; even overdue claims require explicit release')
        for scope in scopes:
            connection.execute('INSERT INTO claims VALUES (?,?,?,?,?,?)',
                               (scope, task, owner, token, now.isoformat(), (now + dt.timedelta(minutes=minutes)).isoformat()))
        connection.commit()
    except Exception:
        connection.rollback()
        raise
    return token


def release(connection, owner, token):
    with connection:
        changed = connection.execute('DELETE FROM claims WHERE owner=? AND token=?', (owner, token)).rowcount
    if not changed:
        raise ValueError('No matching owner/token; nothing released')
    return changed


def check():
    problems = []
    state = read(KB / 'state.json')
    for fingerprint in state['fingerprints']:
        path = (ROOT / fingerprint['path']).resolve()
        if not path.is_relative_to(ROOT.resolve()) or not path.is_file():
            problems.append('Missing or unsafe fingerprint path: ' + fingerprint['path'])
        elif hashlib.sha256(path.read_bytes()).hexdigest() != fingerprint['sha256']:
            problems.append('STALE evidence snapshot: ' + fingerprint['path'])
    evidence = read(KB / 'evidence.json')
    ids = set()
    for item in evidence:
        if item['id'] in ids:
            problems.append('Duplicate evidence id: ' + item['id'])
        ids.add(item['id'])
        path = (ROOT / item['report']).resolve()
        if not path.is_relative_to(ROOT.resolve()) or not path.is_file():
            problems.append('Missing/unsafe evidence link: ' + item['report'])
    tasks = records()
    visiting = set()
    visited = set()
    def visit(key):
        if key in visiting:
            raise ValueError('Task dependency cycle: ' + key)
        if key in visited:
            return
        if key not in tasks:
            raise ValueError('Unknown dependency: ' + key)
        visiting.add(key)
        for dependency in tasks[key]['depends_on']:
            visit(dependency)
        visiting.remove(key)
        visited.add(key)
    for key, task in tasks.items():
        if task['id'] != key or not task['scopes'] or not task['acceptance']:
            problems.append('Malformed task: ' + key)
        try:
            visit(key)
        except ValueError as error:
            problems.append(str(error))
        for reference in task['evidence']:
            if reference not in ids:
                problems.append('Unknown evidence reference: ' + reference)
    if problems:
        raise ValueError('\n'.join(problems))
    print(f'OK: {len(tasks)} tasks, {len(evidence)} evidence records; fingerprints and dependency graph valid.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'check', 'search', 'claim', 'release', 'claims'])
    parser.add_argument('query', nargs='?')
    parser.add_argument('--owner')
    parser.add_argument('--token')
    parser.add_argument('--minutes', type=int, default=30)
    args = parser.parse_args()
    if args.action == 'check':
        check()
    elif args.action == 'status':
        state = read(KB / 'state.json')
        print(json.dumps(state, indent=2))
        for key, task in records().items():
            print(key, task['status'], '-', task['title'])
    elif args.action == 'search':
        if not args.query:
            parser.error('Supply a search phrase')
        for path in sorted(KB.rglob('*')):
            if path.suffix in ['.md', '.json']:
                for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
                    if args.query.casefold() in line.casefold():
                        print(f'{path.relative_to(ROOT)}:{number}: {line.strip()}')
    else:
        if args.action == 'claim':
            tasks = records()
            if args.query not in tasks or not args.owner:
                parser.error('claim needs a known task ID and --owner')
            task = tasks[args.query]
            if task['status'] != 'ready' or any(tasks[d]['status'] != 'done' for d in task['depends_on']):
                raise ValueError('Task not ready or dependency unresolved; claims cannot bypass acceptance gates')
            with closing(database()) as connection:
                token = reserve(connection, args.query, args.owner, task['scopes'], args.minutes)
            print(json.dumps({'owner': args.owner, 'task': args.query, 'token': token, 'scopes': task['scopes']}))
        elif args.action == 'release':
            if not args.owner or not args.token:
                parser.error('release needs --owner and --token from the claim')
            with closing(database()) as connection:
                print('Released scopes:', release(connection, args.owner, args.token))
        else:
            with closing(database()) as connection:
                for row in connection.execute('SELECT scope,task,owner,created,review_after FROM claims ORDER BY scope'):
                    print(json.dumps(dict(zip(['scope','task','owner','created','review_after'], row))))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, sqlite3.Error) as error:
        raise SystemExit(str(error))
