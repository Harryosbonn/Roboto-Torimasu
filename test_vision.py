import time
from vision_driver import VisionDriver
import cv2

def main():
    print("Testing Vision Driver...")
    v = VisionDriver()
    v.start()
    
    # Give it time to warm up
    time.sleep(2)
    
    print("Capturing 100 frames...")
    for i in range(100):
        det = v.get_latest_detection()
        if det:
             print(f"Frame {i}: Person detected at Bearing {det['bearing']:.1f}")
        else:
             print(f"Frame {i}: No detection")
        
        # Test JPEG encoding
        jpg = v.get_frame_jpeg()
        if jpg and len(jpg) > 0:
            pass # good
        else:
            print("Warning: Empty JPEG")
            
        time.sleep(0.1)

    print("Stopping...")
    v.stop()
    print("Done.")

if __name__ == "__main__":
    main()
