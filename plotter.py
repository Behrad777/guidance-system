import re
from pathlib import Path
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------
LOG_FILE = Path("output.txt")
DT       = 0.01            # seconds between samples (100 Hz)
DECIMALS = 4               # number of digits after the decimal in the log

# Build the individual regexes
num4  = rf"[-+]?\d+\.\d{{{DECIMALS}}}"    # e.g. -0.1165
rx_roll  = re.compile(rf"Roll:\s*({num4})\s*deg",  re.I)
rx_pitch = re.compile(rf"Pitch:\s*({num4})\s*deg", re.I)
rx_yaw   = re.compile(rf"Yaw:\s*({num4})\s*deg",   re.I)

# If you also need the quaternion line, compile this too:
rx_quat = re.compile(
    rf"Quaternion:\s*\[\s*({num4})\s*,\s*({num4})\s*,\s*({num4})\s*,\s*({num4})\s*\]",
    re.I
)

# ----------------------------------------------------------------------
# Parse the file
# ----------------------------------------------------------------------
roll, pitch, yaw = [], [], []
quaternions      = []        # optional

with LOG_FILE.open(encoding='utf-16') as f:
    for line in f:
        # match the attitude line
        m_roll  = rx_roll .search(line)
        m_pitch = rx_pitch.search(line)
        m_yaw   = rx_yaw  .search(line)
        if m_roll and m_pitch and m_yaw:
            roll .append(float(m_roll .group(1)))
            pitch.append(float(m_pitch.group(1)))
            yaw  .append(float(m_yaw  .group(1)))

        # match the quaternion line (optional)
        m_quat = rx_quat.search(line)
        if m_quat:
            quaternions.append(tuple(map(float, m_quat.groups())))

print(f"Parsed {len(roll)} Roll/Pitch/Yaw samples "
      f"and {len(quaternions)} quaternions from {LOG_FILE}")

if not roll:
    raise RuntimeError("No attitude samples found — check the log format or DECIMALS")

# ----------------------------------------------------------------------
# Build the time axis and plot
# ----------------------------------------------------------------------
time = [i * DT for i in range(len(roll))]

plt.figure(figsize=(12, 6))
plt.plot(time, roll,  label="Roll  (deg)")
plt.plot(time, pitch, label="Pitch (deg)")
plt.plot(time, yaw,   label="Yaw   (deg)")

plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("EKF Orientation Over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()