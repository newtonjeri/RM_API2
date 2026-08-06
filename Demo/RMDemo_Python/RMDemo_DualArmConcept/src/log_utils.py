#!/usr/bin/env python3
"""
log_utils.py
------------
Shared logging helper for the RMDemo_LiftBenchmark test suite.

When a test script is run directly (``__name__ == '__main__'``), calling
``setup_log(__file__)`` installs a stdout/stderr Tee that simultaneously
writes all output to a timestamped log file in the same directory as the
script.

Log file naming:  ``<script_stem>.log`` (append mode, timestamped banner per run)

Usage in a test script::

    if __name__ == "__main__":
        from log_utils import setup_log
        setup_log(__file__)
        sys.exit(main())
"""

import sys
import pathlib
import datetime


class _Tee:
    """Mirror all writes to both the original stream and a log file."""

    def __init__(self, original, log_file):
        self._orig = original
        self._log  = log_file

    def write(self, data):
        self._orig.write(data)
        self._log.write(data)
        self._orig.flush()
        self._log.flush()

    def flush(self):
        self._orig.flush()
        self._log.flush()

    def isatty(self):
        return getattr(self._orig, "isatty", lambda: False)()

    def __getattr__(self, name):
        return getattr(self._orig, name)


def setup_log(script_path: str) -> pathlib.Path:
    """Install a stdout/stderr Tee and return the log file path.

    Parameters
    ----------
    script_path:
        Pass ``__file__`` from the calling script.

    Returns
    -------
    pathlib.Path
        Absolute path of the created log file.
    """
    script  = pathlib.Path(script_path).resolve()
    logpath = script.parent / f"{script.stem}.log"
    logfile = logpath.open("a", encoding="utf-8", errors="replace")

    sys.stdout = _Tee(sys.stdout, logfile)
    sys.stderr = _Tee(sys.stderr, logfile)

    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    header = (
        f"\n{'='*72}\n"
        f"  Script : {script.name}\n"
        f"  Log    : {logpath.name}\n"
        f"  Started: {ts}\n"
        f"{'='*72}\n"
    )
    sys.stdout.write(header)
    return logpath
