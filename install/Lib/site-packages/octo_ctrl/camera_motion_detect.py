import cv2
import numpy as np
import time
import rclpy
from threading import Thread
from . import color_gen_lib as color_gen
from rclpy.node import Node
from std_srvs.srv import Trigger


class MotionDetector(Node):
    def __init__(self):
        super().__init__('motion_detector')
        self.camera_index = 1
        self.show_camera = True
        self.last_hex = '#FFFFFF'
        self.closest_hex = '#FFFFFF'
        self._running = False
        self.last_trigger_time = 0

        # Environment classifier
        self.environment_label = "Unknown"

        # Define region of interest for the tank
        cx, cy = 320, 180
        L = 320
        self.x0 = cx - L // 2
        self.x1 = cx + L // 2
        self.y0 = cy - L // 2
        self.y1 = cy + L // 2


        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)

        if not self.cap.isOpened():
            raise IOError("Cannot open camera")

        ret, self.prev = self.cap.read()
        if not ret:
            raise IOError("Failed to read initial frame")

        # 1) Crop the camera view to the region of interest (tank)
        self.prev = self.prev[self.y0:self.y1, self.x0:self.x1]
        self.prev_gray = cv2.cvtColor(self.prev, cv2.COLOR_BGR2GRAY)
        self.prev_gray = cv2.GaussianBlur(self.prev_gray, (9, 9), 0)

        # Parameters
        self.motion_threshold = 0.02
        self.cooldown = 1
        self.triggered = False
        self.N_cols = 5
        self.color_patch_bgr = (255, 255, 255)
        self.hex_color = "#FFFFFF"
        self.new_hex = "#FFFFFF"
        self.max_col = None

        # Create a timer to periodically check for camera input
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.srv = self.create_service(Trigger, 'color_detection', self.trigger_camera_callback)

    def trigger_camera_callback(self, request, response):
        self.get_logger().info('Camera triggered via service call.')
        response.success = True
        response.message = self.new_hex
        #add this later.
        #self.last_hex = self.new_hex
        return response

    @staticmethod
    def rgb_to_hex(rgb):
        return '#%02x%02x%02x' % rgb

    @staticmethod
    def hex_to_rgb(hex_str):
        hex_str = hex_str.lstrip('#')
        return tuple(int(hex_str[i:i+2], 16) for i in (0, 2, 4))

    @staticmethod
    def brighten_rgb(rgb, gain=1.3):
        arr = np.clip(np.array(rgb, dtype=float) * gain, 0, 255)
        return tuple(int(x) for x in arr)

    # add to ros2 topic
    # # LED thread to find the closest RGB
    # def _led_thread(self, prev_hex, new_hex):
    #     _, _, plan, steps, closest_hex = color_gen.get_led_seq_from_rgb(prev_hex, new_hex, print_output=True)
    #     print(f"\nTransition from {prev_hex} → {new_hex}")
    #     print(f"Closest achievable dye color: {closest_hex}")
    #     print(f"LED Plan: {plan}")
    #     for s in steps:
    #         print(" ", s)
    #     self.last_hex = new_hex
    #     self.closest_hex = closest_hex


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from camera.")
            return

        frame = cv2.flip(frame, 1)
        frame = frame[self.y0:self.y1, self.x0:self.x1]  # crop every frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)     

        diff = cv2.absdiff(self.prev_gray, gray)
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        motion_fraction = np.mean(thresh > 0)
        self.prev_gray = gray

        now = time.time()
        if motion_fraction > self.motion_threshold and not self.triggered and (now - self.last_trigger_time > self.cooldown):
            self.triggered = True
            self.last_trigger_time = now
            print("\nMotion detected.")

            frame_height, frame_width = thresh.shape
            patch_width = max(1, frame_width // self.N_cols)

            motion_per_col = []
            for i in range(self.N_cols):
                x_start = i * patch_width
                x_end = x_start + patch_width
                patch = thresh[:, x_start:x_end]
                motion_level = np.mean(patch > 0)
                motion_per_col.append(motion_level)

            motion_per_col = np.array(motion_per_col)
            self.max_col = np.argmax(motion_per_col)
            max_motion = motion_per_col[self.max_col]

            if max_motion > self.motion_threshold:
                x_start = self.max_col * patch_width
                x_end = x_start + patch_width
                color_patch = frame[:, x_start:x_end]

                # Apply color bias to correct for the output hues:
                hsv = cv2.cvtColor(color_patch, cv2.COLOR_BGR2HSV)
                median_hue = np.median(hsv[:,:,0])
                lo = max(0, int(median_hue) - 10)
                hi = min(179, int(median_hue) + 10)
                mask = cv2.inRange(hsv, (lo, 50, 50), (hi, 255, 255))
                avg_bgr = cv2.mean(color_patch, mask)[0:3]
                avg_rgb = (
                    int(avg_bgr[2] * 0.7),  
                    int(avg_bgr[1] * 1.1), 
                    int(avg_bgr[0] * 1.4)   
                )
                avg_rgb = np.clip(avg_rgb, 0, 255)
                avg_rgb = tuple(int(x) for x in avg_rgb)
                avg_rgb = self.brighten_rgb(avg_rgb, gain=0.9)

                self.new_hex = self.rgb_to_hex(avg_rgb)

                rgb_prev = np.array(self.hex_to_rgb(self.last_hex))
                rgb_new = np.array(self.hex_to_rgb(self.new_hex))
                color_diff = np.linalg.norm(rgb_new - rgb_prev)

                if color_diff > 40:
                    print(f"Detected dominant color in motion region: {self.new_hex}")
                    self.color_patch_bgr = (int(avg_bgr[0]), int(avg_bgr[1]), int(avg_bgr[2]))
                    self.last_hex = self.new_hex
                    self.hex_color = self.new_hex

        elif motion_fraction < self.motion_threshold / 2:
            self.triggered = False
            self.max_col = None

        # 3) Visualization
        if self.show_camera:
            frame_height, frame_width = frame.shape[:2]
            patch_width = frame_width // self.N_cols

            # Draw grid lines and highlight motion column
            for i in range(self.N_cols):
                x_start = i * patch_width
                cv2.line(frame, (x_start, 0), (x_start, frame_height), (80, 80, 80), 1)
            if self.triggered and self.max_col is not None:
                x_start = self.max_col * patch_width
                x_end = x_start + patch_width
                cv2.rectangle(frame, (x_start, 0), (x_end, frame_height), (0, 255, 0), 2)

            # Dual swatch layout (Prev, Curr, Closest)
            w, h = 35, 15
            x0, y0 = frame_width - (w * 3 + 45), 15

            prev_bgr = tuple(int(c) for c in np.array(self.hex_to_rgb(self.last_hex))[::-1])
            curr_bgr = self.color_patch_bgr
            closest_bgr = tuple(int(c) for c in np.array(self.hex_to_rgb(self.closest_hex))[::-1])

            cv2.rectangle(frame, (x0, y0), (x0 + w, y0 + h), prev_bgr, -1)
            cv2.putText(frame, "Prev", (x0 + 5, y0 + h + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255,255,255), 1)

            x1 = x0 + w + 10
            cv2.rectangle(frame, (x1, y0), (x1 + w, y0 + h), curr_bgr, -1)
            cv2.putText(frame, "Curr", (x1 + 5, y0 + h + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255,255,255), 1)

            x2 = x1 + w + 10
            cv2.rectangle(frame, (x2, y0), (x2 + w, y0 + h), closest_bgr, -1)
            cv2.putText(frame, "Closest", (x2 + 2, y0 + h + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255,255,255), 1)

            cv2.putText(frame, f"{self.hex_color}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(frame, f"Closest: {self.closest_hex}", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(frame, f"Motion: {motion_fraction*100:.1f}%", (20, frame_height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            cv2.imshow("Motion + Color Detection (Cropped ROI)", frame)
            key = cv2.waitKey(1) & 0xFF

    def stop(self):
        """Stop camera and release OpenCV resources."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Camera stopped cleanly.")

def main(args=None):
    try:
        rclpy.init(args=args)
        camera = MotionDetector()
        rclpy.spin(camera)
    except KeyboardInterrupt:
        camera.stop()
        pass
    finally:
        camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()

