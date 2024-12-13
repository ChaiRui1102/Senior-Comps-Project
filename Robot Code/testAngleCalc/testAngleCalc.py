#!/usr/bin/env python3
import cv2
import numpy as np
import math
import time

class RobotAngleDetector:
    def __init__(self, camera_id=0, tolerance_width=20):
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Initialize camera with adjusted settings
        self.cap = cv2.VideoCapture(camera_id)
        # Adjust camera settings for better detection
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -2)  # Lower exposure
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 40)  # Adjust brightness
        self.cap.set(cv2.CAP_PROP_CONTRAST, 60)   # Increase contrast
        
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.tolerance_width = tolerance_width
        
        # Get user input for all markers
        print("\nEnter the IDs of your markers:")
        self.LEFT_MARKER_ID = int(input("Left marker ID (180°): "))
        self.TOP_MARKER_ID = int(input("Top marker ID (90°): "))
        self.RIGHT_MARKER_ID = int(input("Right marker ID (0°): "))
        self.CURRENT_MARKER_ID = int(input("Current position marker ID: "))
        self.TARGET_MARKER_ID = int(input("Target position marker ID: "))
        
        # Storage for parameters
        self.saved_center = None
        self.saved_radius = None
        self.is_calibrated = False

    def preprocess_frame(self, frame):
        """Enhanced image preprocessing for better marker detection"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply adaptive histogram equalization
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        # Apply adaptive thresholding
        gray = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11,
            2
        )
        
        # Optional: Add slight blur to reduce noise
        gray = cv2.GaussianBlur(gray, (3,3), 0)
        
        return gray

    def detect_markers(self, frame):
        """Detect ArUco markers with enhanced preprocessing"""
        processed = self.preprocess_frame(frame)
        corners, ids, _ = self.detector.detectMarkers(processed)
        return corners, ids

    def get_marker_center(self, corner):
        """Calculate center point of a marker"""
        return np.mean(corner[0], axis=0)

    def is_point_on_semicircle(self, point):
        """Check if a point lies within the curved strip of the semicircle"""
        if not self.is_calibrated:
            return False
            
        # Calculate distance from point to center
        distance = np.linalg.norm([point[0] - self.saved_center[0], 
                                 point[1] - self.saved_center[1]])
        
        # Check if point is within the tolerance band of the circle
        within_radius = abs(distance - self.saved_radius) < self.tolerance_width
        
        # Check if point is in upper half of circle (y less than center)
        above_center = point[1] <= self.saved_center[1]
        
        return within_radius and above_center

    def calculate_circle_parameters(self, left_pt, top_pt, right_pt):
        """Calculate circle parameters that pass through reference markers"""
        # Convert points to numpy arrays
        p1 = np.array([left_pt[0], left_pt[1]])
        p2 = np.array([top_pt[0], top_pt[1]])
        p3 = np.array([right_pt[0], right_pt[1]])
        
        # Find center using perpendicular bisectors
        m1 = (p1 + p2) / 2
        m2 = (p2 + p3) / 2
        
        d1 = np.array([-(p2[1] - p1[1]), p2[0] - p1[0]])
        d2 = np.array([-(p3[1] - p2[1]), p3[0] - p2[0]])
        
        d1 = d1 / np.linalg.norm(d1)
        d2 = d2 / np.linalg.norm(d2)
        
        try:
            A = np.column_stack([d1, -d2])
            b = m2 - m1
            t, s = np.linalg.solve(A, b)
            center = m1 + t * d1
        except np.linalg.LinAlgError:
            center = np.array([
                (left_pt[0] + right_pt[0]) / 2,
                top_pt[1]
            ])
        
        # Calculate radius
        radius = np.mean([
            np.linalg.norm(center - p1),
            np.linalg.norm(center - p2),
            np.linalg.norm(center - p3)
        ])
        
        return tuple(center), radius

    def save_current_configuration(self):
        """Save current marker configuration when all markers are visible"""
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to get camera frame")
            return False

        corners, ids = self.detect_markers(frame)
        if ids is None:
            print("No markers detected")
            return False

        # Check if all reference markers are visible
        ids = ids.flatten()
        required_ids = [self.LEFT_MARKER_ID, self.TOP_MARKER_ID, self.RIGHT_MARKER_ID]
        if not all(marker_id in ids for marker_id in required_ids):
            print("Not all reference markers are visible")
            return False

        # Save positions
        left_idx = np.where(ids == self.LEFT_MARKER_ID)[0][0]
        top_idx = np.where(ids == self.TOP_MARKER_ID)[0][0]
        right_idx = np.where(ids == self.RIGHT_MARKER_ID)[0][0]

        left_pt = self.get_marker_center(corners[left_idx])
        top_pt = self.get_marker_center(corners[top_idx])
        right_pt = self.get_marker_center(corners[right_idx])

        self.saved_center, self.saved_radius = self.calculate_circle_parameters(left_pt, top_pt, right_pt)
        self.is_calibrated = True
        print("Configuration saved successfully!")
        return True
    
    def calculate_angle(self, point, center, right_point):
        """Calculate angle between object and rightmost point"""
        vector = np.array([point[0] - center[0], 
                        point[1] - center[1]])
        
        angle = math.degrees(math.atan2(vector[1], vector[0]))
        angle = -angle  # Convert to clockwise
        angle = angle % 360  # Normalize to 0-360
        
        if angle > 180:
            angle = 360 - angle
            
        return angle

    def get_visible_and_saved_positions(self, corners, ids):
        """Get marker positions, only using currently visible markers"""
        positions = {}
        current_ids = ids.flatten() if ids is not None else []
        
        # Get all currently visible markers
        if ids is not None:
            for i, marker_id in enumerate(current_ids):
                center = self.get_marker_center(corners[i])
                if marker_id in [self.CURRENT_MARKER_ID, self.TARGET_MARKER_ID]:
                    if self.is_point_on_semicircle(center):
                        positions[marker_id] = center
                else:
                    positions[marker_id] = center
        
        return positions

    def draw_visualization(self, frame, positions):
        """Draw visualization with all markers and semicircle"""
        try:
            if not self.is_calibrated:
                return frame

            center = self.saved_center
            radius = self.saved_radius
            center = tuple(map(int, center))
            radius = int(radius)
            
            # Draw reference points and semicircle
            for marker_id in [self.LEFT_MARKER_ID, self.TOP_MARKER_ID, self.RIGHT_MARKER_ID]:
                if marker_id in positions:
                    pt = tuple(map(int, positions[marker_id]))
                    cv2.circle(frame, pt, 3, (255, 0, 0), -1)
                    # Draw label for reference markers
                    label = "180°" if marker_id == self.LEFT_MARKER_ID else "90°" if marker_id == self.TOP_MARKER_ID else "0°"
                    cv2.putText(frame, label, (pt[0]-20, pt[1]-10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Draw semicircle strip
            for r in range(radius - self.tolerance_width//2, radius + self.tolerance_width//2 + 1, 2):
                cv2.ellipse(frame, center, (r, r), 0, 180, 360, (0, 255, 0), 1)
            
            right_pt = positions.get(self.RIGHT_MARKER_ID)
            if right_pt is not None:
                # Draw current position if detected
                if self.CURRENT_MARKER_ID in positions:
                    current_pt = positions[self.CURRENT_MARKER_ID]
                    current_pt_int = tuple(map(int, current_pt))
                    cv2.circle(frame, current_pt_int, 5, (0, 0, 255), -1)
                    current_angle = self.calculate_angle(current_pt, center, right_pt)
                    cv2.putText(frame, f"Current: {round(current_angle)}°", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.line(frame, center, current_pt_int, (0, 0, 255), 1)
                
                # Draw target position if detected
                if self.TARGET_MARKER_ID in positions:
                    target_pt = positions[self.TARGET_MARKER_ID]
                    target_pt_int = tuple(map(int, target_pt))
                    cv2.circle(frame, target_pt_int, 5, (255, 0, 0), -1)
                    target_angle = self.calculate_angle(target_pt, center, right_pt)
                    cv2.putText(frame, f"Target: {round(target_angle)}°", (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    cv2.line(frame, center, target_pt_int, (255, 0, 0), 1)

            # Draw calibration status
            status = "Calibrated" if self.is_calibrated else "Not Calibrated"
            cv2.putText(frame, f"Status: {status}", (10, frame.shape[0]-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if self.is_calibrated else (0, 0, 255), 2)

        except Exception as e:
            cv2.putText(frame, f"Visualization error: {str(e)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return frame

    def run(self):
        """Run continuous angle detection with visualization"""
        cv2.namedWindow('Angle Detection', cv2.WINDOW_NORMAL)
        print("\nPress 'c' to calibrate/save current marker configuration")
        print("Press 'q' to quit")
        
        last_current_angle = None
        last_target_angle = None
        min_angle_change = 5  # Minimum angle change to trigger update
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            corners, ids = self.detect_markers(frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                if self.save_current_configuration():
                    print("Configuration saved. System will now track marker positions.")
                else:
                    print("Calibration failed. Please ensure all reference markers are visible.")
            
            # Get positions and draw visualization
            positions = self.get_visible_and_saved_positions(corners, ids)
            if self.is_calibrated:
                frame = self.draw_visualization(frame, positions)
                
                # Write angles to file only if both markers are currently visible
                if (self.CURRENT_MARKER_ID in positions and 
                    self.TARGET_MARKER_ID in positions and 
                    self.RIGHT_MARKER_ID in positions):
                    
                    current_pt = positions[self.CURRENT_MARKER_ID]
                    target_pt = positions[self.TARGET_MARKER_ID]
                    right_pt = positions[self.RIGHT_MARKER_ID]
                    
                    # Calculate new angles
                    current_angle = round(self.calculate_angle(current_pt, self.saved_center, right_pt))
                    target_angle = round(self.calculate_angle(target_pt, self.saved_center, right_pt))
                    
                    # Check if angles have changed significantly
                    angles_changed = (last_current_angle is None or 
                                    last_target_angle is None or 
                                    abs(current_angle - last_current_angle) > min_angle_change or 
                                    abs(target_angle - last_target_angle) > min_angle_change)
                    
                    if angles_changed:
                        print(f"New angles detected: Current={current_angle}°, Target={target_angle}°")
                        with open('robot_angles.txt', 'w') as f:
                            f.write(f"{current_angle},{target_angle}")
                        with open('new_positions.trigger', 'w') as f:
                            f.write('1')
                        last_current_angle = current_angle
                        last_target_angle = target_angle
            
            cv2.imshow('Angle Detection', frame)
        
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = RobotAngleDetector(camera_id=0)
    detector.run()