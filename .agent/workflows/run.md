---
description: Run the robot dashboard (with AI Vision) and beacon receiver
---

# Run Robot Dashboard and Beacon

This workflow starts the main dashboard server (integrated with AI Vision) and beacon receiver.

## Steps

// turbo-all

1. Install/Verify Dependencies:
```bash
pip install --break-system-packages opencv-python-headless numpy
```

2. Start the main dashboard (runs on https://10.86.157.230:5000):
```bash
cd /home/mono/Documents/robot && python robot_dashboard.py
```

3. Start the beacon receiver (listens on UDP port 5005):
```bash
cd /home/mono/Documents/robot && python beacon_receiver.py
```

## Access Points

- **Dashboard**: https://10.86.157.230:5000
- **Video Feed**: https://10.86.157.230:5000/video_feed
- **Beacon Page**: https://10.86.157.230:5000/beacon
