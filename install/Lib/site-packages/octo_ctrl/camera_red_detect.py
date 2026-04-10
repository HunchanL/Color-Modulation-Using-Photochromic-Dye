import cv2
import numpy as np
from collections import deque
import time

def trigger_blue_flash():
    """Placeholder for real actuator / LED control."""
    print("Trigger action.")

class RedDetector:
    """Red object detection with clean start/stop control."""

    def __init__(self, camera_index=1, show_camera=True):
        self.camera_index = camera_index
        self.show_camera = show_camera
        self._running = False

    def stop(self):
        """Request a clean stop."""
        self._running = False

    def run(self):
        """Main red object detection loop."""
        # Camera setup
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)

        if not cap.isOpened():
            raise IOError("Cannot open Logitech camera")

        print("'q' to quit'")

        # Red detection parameters
        lower_red1 = np.array([0, 130, 90])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 130, 90])
        upper_red2 = np.array([180, 255, 255])
        kernel = np.ones((5, 5), np.uint8)

        # Tracking variables
        area_history = deque(maxlen=10)
        trigger_threshold = 0.03  # fraction of frame that must be red
        triggered = False
        cooldown = 5.0  # seconds before allowing retrigger
        last_trigger_time = 0.0

        self._running = True
        while self._running:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Red mask
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            # Red coverage
            red_fraction = np.sum(mask > 0) / (mask.shape[0] * mask.shape[1])
            area_history.append(red_fraction)
            avg_red = np.mean(area_history)

            # Trigger logic
            now = time.time()
            if avg_red > trigger_threshold and not triggered and (now - last_trigger_time > cooldown):
                triggered = True
                last_trigger_time = now
                print("Red detected.")
                trigger_blue_flash()  # Replace with action function here..

            elif avg_red < trigger_threshold / 2:
                triggered = False  # Reset when red disappears

            # Visualization
            if self.show_camera:
                cv2.putText(frame, f"Red fraction: {avg_red*100:.2f}%", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)
                cv2.putText(frame, f"Trigger: {'ON' if triggered else 'OFF'}", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.2,
                            (0, 0, 255) if triggered else (255, 255, 255), 1)

                cv2.imshow("Danger detection", cv2.resize(frame, (320, 180)))

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    # Revise the code block here depending on how you want to close the camera.
                    self.stop()

        cap.release()
        cv2.destroyAllWindows()
        print("Camera released and closed cleanly.")

if __name__ == "__main__":
    detector = RedDetector(camera_index=1, show_camera=True)
    try:
        detector.run()  # Start detection
    except KeyboardInterrupt:
        print("Exit.")
        detector.stop()
