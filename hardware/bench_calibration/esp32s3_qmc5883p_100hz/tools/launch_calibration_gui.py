#!/usr/bin/env python3
"""Start the GUI with Tcl/Tk runtime libraries from the installed Python."""
from __future__ import annotations

import os
from pathlib import Path
import re
import runpy
import sys
import tempfile
from zipfile import ZipFile


def prepare_tk_runtime() -> None:
    import tkinter

    base = Path(sys.base_prefix)
    tcl_root = base / "tcl"
    if (tcl_root / "tcl9.0" / "init.tcl").is_file():
        return

    tcl_archives = sorted(tcl_root.glob("libtcl*.zip"))
    tk_archives = sorted(tcl_root.glob("libtk*.zip"))
    if not tcl_archives or not tk_archives:
        return

    runtime = Path(tempfile.gettempdir()) / (
        f"qcopter_tk_{sys.version_info.major}{sys.version_info.minor}"
    )
    tcl_dest = runtime / "tcl"
    tk_dest = runtime / "tk"
    if not (tcl_dest / "tcl_library" / "init.tcl").is_file():
        tcl_dest.mkdir(parents=True, exist_ok=True)
        with ZipFile(tcl_archives[-1]) as archive:
            archive.extractall(tcl_dest)
    if not (tk_dest / "tk_library" / "tk.tcl").is_file():
        tk_dest.mkdir(parents=True, exist_ok=True)
        with ZipFile(tk_archives[-1]) as archive:
            archive.extractall(tk_dest)

    version_match = re.search(r"libtk([0-9.]+)\.zip$", tk_archives[-1].name)
    version = version_match.group(1) if version_match else str(tkinter.TkVersion)
    dll_version = "".join(version.split(".")[:2])
    dll = base / "DLLs" / f"tcl9tk{dll_version}.dll"
    if not dll.is_file():
        matches = list((base / "DLLs").glob("tcl*tk*.dll"))
        if not matches:
            return
        dll = matches[0]

    package_dir = runtime / "package"
    package_dir.mkdir(parents=True, exist_ok=True)
    package_dir.joinpath("pkgIndex.tcl").write_text(
        f"package ifneeded tk {version} [list load [file normalize {{{dll.as_posix()}}}]]\n"
        f"package ifneeded Tk {version} [list package require -exact tk {version}]\n",
        encoding="utf-8",
    )
    os.environ["TCL_LIBRARY"] = str(tcl_dest / "tcl_library")
    os.environ["TK_LIBRARY"] = str(tk_dest / "tk_library")
    os.environ["TCLLIBPATH"] = "{" + package_dir.as_posix() + "}"
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(base / "DLLs"))


def main() -> None:
    prepare_tk_runtime()
    script = Path(__file__).with_name("live_calibration_gui.py")
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
