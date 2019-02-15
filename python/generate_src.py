#!/usr/bin/env python3

import subprocess
import sys

# Generate messages
subprocess.run([sys.executable, "messages.py", "messages.mq"])
