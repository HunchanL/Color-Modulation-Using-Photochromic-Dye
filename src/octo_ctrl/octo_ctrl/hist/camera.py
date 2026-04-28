import numpy as np
import rclpy
from rclpy.node import Node
import cv2

from std_srvs.srv import Trigger

class Camera(Node):
    def __init__(self):
        super().__init__('octopus_control')
        self.camera_index = 1
        self.show_camera = True
        self._running = False
        self.target_color = "#FFFFFF"  

        # Define region of interest to fit tank
        cx, cy = 320, 180   # center of tank (half of frame width and height)
        L = 150            # side length of the cropped square
        self.x0 = cx - L // 2
        self.x1 = cx + L // 2
        self.y0 = cy - L // 2
        self.y1 = cy + L // 2

        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

        if not self.cap.isOpened():
            raise IOError("Cannot open camera")

        # Create a timer to periodically check for camera input
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.srv = self.create_service(Trigger, 'color_detection', self.trigger_camera_callback)

    def trigger_camera_callback(self, request, response):
        self.get_logger().info('Camera triggered via service call.')
        response.success = True
        response.message = self.target_color
        return response

    @staticmethod
    def rgb_to_hex(rgb):
        return '#%02x%02x%02x' % rgb
    
    @staticmethod
    def brighten_rgb(rgb, gain=1.3):
        arr = np.clip(np.array(rgb, dtype=float) * gain, 0, 255)
        return tuple(int(x) for x in arr)

    def get_dominant_rgb(self, frame):
        """Return dominant RGB color of cropped frame."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pixels = rgb.reshape(-1, 3)

        # Remove overly dark or bright pixels to avoid glare/background
        mask = np.all((pixels > 30) & (pixels < 240), axis=1)
        pixels = pixels[mask]

        if len(pixels) > 0:
            pixels = np.float32(pixels)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
            _, _, centers = cv2.kmeans(pixels, 1, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
            dominant_rgb = tuple(map(int, centers[0]))
        else:
            dominant_rgb = (255, 255, 255)

        return dominant_rgb
    
    def stop(self):
        """Stop camera and release OpenCV resources."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Camera stopped cleanly.")
    
    def timer_callback(self):
        dominant_rgb = None
        ret, frame = self.cap.read()
        if not ret:
            raise IOError("Failed to read frame")

        frame = cv2.flip(frame, 1)

        # Crop to the square ROI
        roi = frame[self.y0:self.y1, self.x0:self.x1]

        # Define region of interest to fit tank
        cx, cy = 320, 180   # center of tank (half of frame width and height)
        L = 300            # side length of the cropped square
        sqr_region = frame[cy - L // 2:cy + L // 2, cx - L // 2:cx + L // 2]
        dominant_rgb = self.get_dominant_rgb(roi)
        dominant_rgb = self.brighten_rgb(dominant_rgb, gain=1)
        dominant_hex = self.rgb_to_hex(dominant_rgb)
        self.target_color = dominant_hex
        print(f"Detected dominant color: {dominant_hex}")

        # Visualization
        if self.show_camera:
            frame_height, frame_width = frame.shape[:2]

            # Draw the ROI rectangle on full frameq
            cv2.rectangle(frame, (self.x0, self.y0), (self.x1, self.y1), (255, 255, 255), 1)

            # Draw the color patch and text
            bgr_patch = tuple(int(c) for c in dominant_rgb[::-1])
            cv2.rectangle(frame, (cx - L // 2, cy - L // 2), ((cx - L // 2) + 50, (cy - L // 2) + 50), bgr_patch, -1)
            cv2.putText(frame, dominant_hex, (cx - L // 2, cy - L // 2 + 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("Dominant Color Detection (Cropped ROI)", sqr_region)
            cv2.waitKey(1)


    
def main(args=None):
    try:
        rclpy.init(args=args)
        camera = Camera()
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
