import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import math
import cv2
import numpy as np
from ultralytics import YOLO

class CameraView(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, style="Dark.TFrame")
        ttk.Label(self, text="Camera Feed", style="Title.TLabel").pack(pady=5)
        self.canvas = tk.Canvas(self, bg="black", width=640, height=360)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.image_tk = None
        
        # YOLO model initialization
        try:
            self.model = YOLO('yolo11n.pt')
            self.model_loaded = True
            print("YOLO model loaded successfully")
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            self.model_loaded = False
            
        # Parameters for distance calculation
        self.focal_length = 720  # Focal length in pixels, may need adjustment
        self.actual_object_sizes = {
            'person': 50,   # Width in cm
            'car': 180,     # Width in cm
            'bus': 250,     # Width in cm
            'bottle': 6.5   # Width in cm
            # Add more objects as needed
        }
        
    def update_frame(self, frame):
        if frame is not None:
            try:
                # Process the frame with YOLO if model is loaded
                processed_frame = self.process_frame(frame) if self.model_loaded else frame
                
                # Convert and display the frame
                frame_rgb = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                img = img.resize((640, 360), Image.LANCZOS)
                self.image_tk = ImageTk.PhotoImage(image=img)
                self.canvas.create_image(0, 0, anchor="nw", image=self.image_tk)
                self.canvas.image = self.image_tk  # Keep a reference
            except Exception as e:
                print(f"Error updating camera feed: {e}")
                self.canvas.delete("all")
                self.canvas.config(bg="black")
        else:
            self.canvas.delete("all")
            self.canvas.config(bg="black")
    
    def process_frame(self, frame):
        """Process frame with YOLO and add object detection and distance estimation"""
        if not self.model_loaded:
            return frame
            
        try:
            # Make a copy to avoid modifying the original
            processed_frame = frame.copy()
            
            # Run YOLO detection
            results = self.model(processed_frame, verbose=False)[0]
            
            # Extract detection information
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            
            # Process each detection
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = map(int, box)
                class_id = int(classes[i])
                label = self.model.names[class_id]
                conf = confidences[i]
                
                # Calculate object width in pixels
                bbox_width_pixels = x2 - x1
                
                # Get real-world width of the object
                actual_width = self.actual_object_sizes.get(label, 100)  # Default if not in dictionary
                
                # Calculate distance using the formula: distance = (actual_width * focal_length) / width_in_pixels
                if bbox_width_pixels != 0:
                    distance = (actual_width * self.focal_length) / bbox_width_pixels
                    distance_text = f"{distance:.1f} cm"
                else:
                    distance = float('inf')
                    distance_text = "Unknown"
                
                # Draw bounding box
                cv2.rectangle(processed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Add label and confidence
                text = f"{label} ({conf:.2f})"
                cv2.putText(processed_frame, text, (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Add distance
                cv2.putText(processed_frame, distance_text, (x1, y2+20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Add warning if object is too close
                if distance < 150:  # Threshold in cm
                    cv2.putText(processed_frame, "WARNING: Object Close!", 
                                (x1, y1 + (y2 - y1) // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            return processed_frame
            
        except Exception as e:
            print(f"Error in YOLO processing: {e}")
            return frame


class LidarView(ttk.Frame):
    def __init__(self, parent, point_size=2, max_distance=5000, **kwargs):
        super().__init__(parent, style="Dark.TFrame", **kwargs)
        self.point_size = point_size
        self.max_distance = max_distance
        self.scale_factor = 300 / self.max_distance
        self.center_x = 150
        self.center_y = 150
        self._create_widgets()

    def _create_widgets(self):
        ttk.Label(self, text="LIDAR Visualization", style="Title.TLabel").pack(pady=5)
        self.canvas = tk.Canvas(self, bg="black", width=300, height=300)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self._draw_safe_zone() # Initialize safe zone

    def _draw_safe_zone(self, inner_radius_scaled=50, outer_radius_scaled=150, color="#444"):
        """Draws visual cues for safe zones."""
        self.canvas.create_oval(self.center_x - inner_radius_scaled,
                                self.center_y - inner_radius_scaled,
                                self.center_x + inner_radius_scaled,
                                self.center_y + inner_radius_scaled,
                                outline=color, dash=(2, 2), tags="safe_zone")
        self.canvas.create_oval(self.center_x - outer_radius_scaled,
                                self.center_y - outer_radius_scaled,
                                self.center_x + outer_radius_scaled,
                                self.center_y + outer_radius_scaled,
                                outline=color, dash=(2, 2), tags="safe_zone")

    def update_display(self, lidar_data):
        self.canvas.delete("lidar_point") # Use a specific tag for points
        if not lidar_data:
            return

        points_to_draw = []

        for angle_deg, distance_mm in lidar_data:
            if distance_mm > self.max_distance or distance_mm < 0:
                continue  # Skip out-of-range or invalid data

            angle_rad = math.radians(angle_deg)
            scaled_distance = distance_mm * self.scale_factor
            end_x = self.center_x + scaled_distance * math.cos(angle_rad)
            end_y = self.center_y - scaled_distance * math.sin(angle_rad)

            color = "#00FF00"  # Default green
            if distance_mm < 500:
                color = "#FF0000"  # Red for very close
            elif distance_mm < 1500:
                color = "#FFFF00"  # Yellow for close

            points_to_draw.append((end_x, end_y, color))

        for x, y, color in points_to_draw:
            self.canvas.create_oval(x - self.point_size, y - self.point_size,
                                    x + self.point_size, y + self.point_size,
                                    fill=color, outline=color, tags="lidar_point")