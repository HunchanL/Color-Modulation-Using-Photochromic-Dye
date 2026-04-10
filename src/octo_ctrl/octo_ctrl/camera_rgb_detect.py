import cv2
import numpy as np

class DominantColorDetector:
    def __init__(self, camera_index=1, show_camera=True):
        self.camera_index = camera_index
        self.show_camera = show_camera
        self._running = False

        # Define region of interest to fit tank
        cx, cy = 320, 180   # center of tank (half of frame width and height)
        L = 150            # side length of the cropped square
        self.x0 = cx - L // 2
        self.x1 = cx + L // 2
        self.y0 = cy - L // 2
        self.y1 = cy + L // 2

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

    def run(self):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        cap.set(cv2.CAP_PROP_FPS, 15)
        # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

        if not cap.isOpened():
            raise IOError("Cannot open camera")

        self._running = True
        while self._running:
            ret, frame = cap.read()
            if not ret:
                break

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

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._running = False

        cap.release()
        cv2.destroyAllWindows()
        print("Stopped cleanly.")

if __name__ == "__main__":
    detector = DominantColorDetector(camera_index=1, show_camera=True)
    detector.run()
