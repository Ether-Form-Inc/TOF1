# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Checks that the Python packages the tools need are installed, and says how to install them if not."""
import importlib.util
import os
import sys

REQUIRED = {"dronecan": "dronecan", "serial": "pyserial"}   # import name -> package name


def check():
    missing = [pkg for mod, pkg in REQUIRED.items() if importlib.util.find_spec(mod) is None]
    if not missing:
        return
    here = os.path.dirname(os.path.abspath(__file__))
    sys.exit("This tool needs the Python package(s): %s (not installed for %s).\n"
             "The system Python usually refuses 'pip install', so use a virtual environment (once):\n"
             "    cd %s\n"
             "    python3 -m venv .venv && .venv/bin/pip install -r tools/requirements.txt\n"
             "and then run the tool with that Python, for example:\n"
             "    .venv/bin/python tools/%s ..." % (
                 ", ".join(missing), sys.executable, os.path.dirname(here), os.path.basename(sys.argv[0]) or "<tool>.py"))
