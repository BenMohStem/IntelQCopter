from contextlib import closing
"""Coordination tests use a temporary database, never project reservations."""
import concurrent.futures
import sqlite3
import tempfile
import unittest
from pathlib import Path
from knowledge import conflicts, reserve, release


class CoordinationTests(unittest.TestCase):
    def test_paths_conflict_by_component_and_case(self):
        self.assertTrue(conflicts('firmware', 'FIRMWARE/main.cpp'))
        self.assertFalse(conflicts('tools/a', 'tools/abc'))
        self.assertTrue(conflicts('device:COM3', 'device:com3'))

    def test_concurrent_claim_has_exactly_one_winner(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'claims.db'
            with closing(sqlite3.connect(path)) as con:
                con.execute('CREATE TABLE claims(scope TEXT PRIMARY KEY,task TEXT,owner TEXT,token TEXT,created TEXT,review_after TEXT)')
            def claim(owner):
                with closing(sqlite3.connect(path, timeout=10)) as con:
                    try:
                        return owner, reserve(con, 'test', owner, ['firmware'], 1)
                    except ValueError:
                        return None
            with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
                winners = [value for value in pool.map(claim, map(str, range(8))) if value]
            self.assertEqual(len(winners), 1)
            owner, token = winners[0]
            with closing(sqlite3.connect(path)) as con:
                with self.assertRaises(ValueError):
                    release(con, 'wrong-owner', token)
                con.execute("UPDATE claims SET review_after='2000-01-01'")
                con.commit()
                with self.assertRaises(ValueError):
                    reserve(con, 'other', 'other', ['firmware/main.cpp'], 1)
                self.assertEqual(release(con, owner, token), 1)
                reserve(con, 'other', 'other', ['firmware/main.cpp'], 1)


if __name__ == '__main__':
    unittest.main()
