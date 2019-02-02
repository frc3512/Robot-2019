#!/usr/bin/env python3

import os
import subprocess
import sys

files = [
    os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn if f.endswith(".py")
]

subprocess.run([sys.executable, "-m", "black", "-q", "format.py"])
for f in files:
    subprocess.run([sys.executable, "-m", "black", "-q", f])
