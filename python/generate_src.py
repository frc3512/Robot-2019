#!/usr/bin/env python3

import os
import subprocess
import sys

DEST = "../build/generated"

# Generate coeffs
subprocess.run([sys.executable, "elevator.py", "--noninteractive"])
subprocess.run([sys.executable, "four_bar_lift.py", "--noninteractive"])
subprocess.run([sys.executable, "climber.py", "--noninteractive"])

# Move .cpp files
if not os.path.exists(f"{DEST}/cpp/control"):
    os.makedirs(f"{DEST}/cpp/control")
os.rename("ElevatorCoeffs.cpp", f"{DEST}/cpp/control/ElevatorCoeffs.cpp")
os.rename("FourBarLiftCoeffs.cpp", f"{DEST}/cpp/control/FourBarLiftCoeffs.cpp")
os.rename("ElevatorClimbCoeffs.cpp", f"{DEST}/cpp/control/ElevatorClimbCoeffs.cpp")
os.rename("ClimberCoeffs.cpp", f"{DEST}/cpp/control/ClimberCoeffs.cpp")

# Move .hpp files
if not os.path.exists(f"{DEST}/include/control"):
    os.makedirs(f"{DEST}/include/control")
os.rename("ElevatorCoeffs.hpp", f"{DEST}/include/control/ElevatorCoeffs.hpp")
os.rename("FourBarLiftCoeffs.hpp", f"{DEST}/include/control/FourBarLiftCoeffs.hpp")
os.rename("ElevatorClimbCoeffs.hpp", f"{DEST}/include/control/ElevatorClimbCoeffs.hpp")
os.rename("ClimberCoeffs.hpp", f"{DEST}/include/control/ClimberCoeffs.hpp")

# Generate messages
subprocess.run([sys.executable, "generate_messages.py", "messages.mq"])
