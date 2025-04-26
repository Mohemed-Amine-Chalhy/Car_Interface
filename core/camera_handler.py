import cv2
import threading
import time
from queue import Queue
import queue
class CameraHandler:
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = None
        self.frame_queue = Queue(maxsize=10)
        self.is_running = False
        self.read_thread = None

    def start_capture(self):
        try:
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) # Try with DSHOW for Windows
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.camera_index) # Try without DSHOW
                if not self.cap.isOpened():
                    print(f"Error: Could not open camera with index {self.camera_index}")
                    return False
            self.is_running = True
            self.read_thread = threading.Thread(target=self._read_frames, daemon=True)
            self.read_thread.start()
            return True
        except Exception as e:
            print(f"Error initializing camera {self.camera_index}: {e}")
            return False

    def stop_capture(self):
        self.is_running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join()
        if self.cap and self.cap.isOpened():
            self.cap.release()
            print("Camera capture stopped.")
            self.cap = None

    def _read_frames(self):
        while self.is_running:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                continue
                
            try:
                # Try to put the frame in the queue with a timeout
                if not self.frame_queue.full():
                    self.frame_queue.put(frame, timeout=0.1)
                else:
                    # If queue is full, discard oldest frame and add new one
                    try:
                        self.frame_queue.get_nowait()  # Remove oldest frame
                        self.frame_queue.put(frame, block=False)  # Add new frame
                    except queue.Empty:
                        pass  # Queue was emptied by another thread
            except Exception as e:
                print(f"Error in frame processing: {e}")

    def get_frame(self):
        try:
            return self.frame_queue.get(timeout=0.1)
        except queue.Empty:
            return None

    def __del__(self):
        self.stop_capture()