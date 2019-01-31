#!/usr/bin/env python3

import os
import subprocess
import sys

# Generate coeffs
subprocess.run([sys.executable, "generate_coeffs.py", "--noninteractive"])
subprocess.run([sys.executable, "elevator.py", "--noninteractive"])
subprocess.run([sys.executable, "four_bar_lift.py", "--noninteractive"])
os.rename("ElevatorCoeffs.hpp", "../src/main/include/control/ElevatorCoeffs.hpp")
os.rename("ElevatorCoeffs.cpp", "../src/main/cpp/control/ElevatorCoeffs.cpp")
os.rename("FourBarLiftCoeffs.hpp", "../src/main/include/control/FourBarLiftCoeffs.hpp")
os.rename("FourBarLiftCoeffs.cpp", "../src/main/cpp/control/FourBarLiftCoeffs.cpp")
# Generate messages
subprocess.run([sys.executable, "messages.py", "messages.mq"])
