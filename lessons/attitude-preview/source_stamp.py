"""Embed the exact main.cpp SHA in every PlatformIO build, without a header."""
from pathlib import Path
import hashlib

Import("env")
source = Path(env.subst("$PROJECT_DIR")) / "main.cpp"
digest = hashlib.sha256(source.read_bytes()).hexdigest()
env.Append(CPPDEFINES=[("BENCH_SOURCE_SHA", '\\"' + digest + '\\"')])
