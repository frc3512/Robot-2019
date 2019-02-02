#!/usr/bin/env python3

import os
import subprocess
import sys

subprocess.run([sys.executable, "elevator.py", "--noninteractive"])
os.rename("ElevatorCoeffs.hpp", "../src/main/include/control/ElevatorCoeffs.hpp")
os.rename("ElevatorCoeffs.cpp", "../src/main/cpp/control/ElevatorCoeffs.cpp")
