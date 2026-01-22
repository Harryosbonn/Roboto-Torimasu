import cv2
import os

print(f"OpenCV Version: {cv2.__version__}")
print(f"Build Info: {cv2.getBuildInformation()}")

indices = [0, 1, 2, 21]

for idx in indices:
    print(f"\n--- Testing Index {idx} ---")
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        print(f"FAILED to open index {idx}")
        # Try forcing V4L2
        print(f"Retrying index {idx} with CAP_V4L2...")
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"FAILED to open index {idx} with CAP_V4L2")
        else:
            print(f"SUCCESS with CAP_V4L2 on index {idx}")
            cap.release()
    else:
        print(f"SUCCESS opening index {idx}")
        ret, frame = cap.read()
        if ret:
            print(f"SUCCESS reading frame: {frame.shape}")
        else:
            print("FAILED reading frame")
        cap.release()

print("\nListing /dev/video*:")
os.system("ls -l /dev/video*")
