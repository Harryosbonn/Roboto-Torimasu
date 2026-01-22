import cv2
import numpy as np
import time
import threading

class VisionDriver:
    def __init__(self, camera_index=0, width=320, height=240):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None
        self.running = False
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.last_detection = None # (bearing, distance_estimate, timestamp)
        self.lidar_distance = None # Injected from dashboard
        
        # Initialize HSV ranges for Light Green
        # Tuning: Tightened range to reduce sensitivity/noise
        # Hue: 40-85 (Less Yellow/Teal bleed)
        # Sat: > 60 (Must be at least somewhat green, not gray)
        self.lower_green = np.array([40, 60, 60])
        self.upper_green = np.array([85, 255, 255])

    def start(self):
        if self.running:
            return
        
        # Try to find a working camera
        # Try to find a working camera
        camera_found = False
        print("VISION: Starting camera scan (indices 0-20)...")
        for index in range(21): # Try indices 0-20
            try:
                cap = cv2.VideoCapture(index)
                if cap.isOpened():
                    # Try to read a frame to confirm it actually works
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap
                        self.camera_index = index
                        # Set resolution
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        print(f"VISION: Success! Using camera index {index}")
                        camera_found = True
                        break
                    else:
                        print(f"VISION: Index {index} opened but returned no frame (read failed).")
                        cap.release()
                else:
                    # Provide feedback on why it failed if possible (usually standard out/err captures it)
                    print(f"VISION: Index {index} failed to open (isOpened=False).")
            except Exception as e:
                print(f"VISION: Error probing camera {index}: {e}")
        
        if not camera_found:
            print("VISION: CRITICAL - No working camera found!")
            return

        self.running = True
        self.thread = threading.Thread(target=self._run_loop)
        self.thread.daemon = True
        self.thread.start()
        print("VISION: Started (Green Object Mode)")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()

    def _run_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            # Resize if needed
            if frame.shape[1] != self.width:
                frame = cv2.resize(frame, (self.width, self.height))

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create Green Mask
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

            # Morphological operations to clean up noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            # VISUALIZATION: Grayscale + Green only
            # 1. Convert original to Grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 2. Convert back to BGR so we can draw colored rectangles/text later
            gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            # 3. Create display frame: background is grayscale
            display_frame = gray_bgr
            # 4. Where mask is green, copy original colored pixels
            # mask is single channel, frame is 3 channel
            # Use numpy boolean indexing or bitwise
            display_frame = cv2.bitwise_and(display_frame, display_frame, mask=cv2.bitwise_not(mask))
            green_part = cv2.bitwise_and(frame, frame, mask=mask)
            display_frame = cv2.add(display_frame, green_part)
            
            # Use this for drawing debug info
            frame = display_frame

            # Find Contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            best_object = None # (x, y, w, h)
            max_area = 0

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Filter small noise
                if area > 500: 
                    x, y, w, h = cv2.boundingRect(cnt)
                    if area > max_area:
                        max_area = area
                        best_object = (x, y, w, h)
                    
                    # Draw candidate
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 100, 255), 1)

            if best_object:
                (x, y, w, h) = best_object
                # Draw best
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                x_center = x + w / 2
                center_offset = (x_center - (self.width / 2)) / (self.width / 2)
                bearing = center_offset * 30.0 # Approx 60 deg fov
                
                # Draw distance overlay if available
                if self.lidar_distance is not None:
                    dist_text = f"{self.lidar_distance:.2f}m"
                    cv2.putText(frame, dist_text, (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                self.last_detection = {
                    "bearing": bearing,
                    "bbox": (x, y, w, h),
                    "distance": self.lidar_distance,
                    "ts": time.time()
                }
            
            with self.frame_lock:
                self.current_frame = frame

            # Don't burn CPU
            time.sleep(0.05) 

    def get_frame_jpeg(self):
        """Returns the latest frame encoded as JPEG bytes"""
        with self.frame_lock:
            if self.current_frame is None:
                return None
            ret, buffer = cv2.imencode('.jpg', self.current_frame)
            return buffer.tobytes()

    def get_latest_detection(self):
        """Returns dictionary of latest detection info"""
        return self.last_detection

    def set_lidar_distance(self, distance):
        """Set the lidar distance to be overlaid on video"""
        self.lidar_distance = distance
