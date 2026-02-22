# stream_camera_thermal_threaded.py

from picamera2 import Picamera2
import cv2
import numpy as np
from flask import Flask, Response
import board
import busio
import adafruit_mlx90640
import threading
import time

app = Flask(__name__)

# -----------------------------
# Initialize Camera (30 FPS)
# -----------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480)},
    controls={"FrameRate": 30}
)
picam2.configure(config)
picam2.start()

# -----------------------------
# Initialize MLX90640 (8 FPS)
# -----------------------------
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ

thermal_raw = np.zeros((24 * 32,))
latest_thermal_img = np.zeros((480, 320, 3), dtype=np.uint8)
thermal_lock = threading.Lock()

# -----------------------------
# Thermal Thread (8 Hz)
# -----------------------------
def thermal_worker():
    global latest_thermal_img
    while True:
        try:
            mlx.getFrame(thermal_raw)
            data = np.reshape(thermal_raw, (24, 32))

            min_temp = np.min(data)
            max_temp = np.max(data)

            if max_temp - min_temp == 0:
                continue

            norm = (data - min_temp) / (max_temp - min_temp)
            norm = (norm * 255).astype(np.uint8)

            resized = cv2.resize(norm, (320, 480), interpolation=cv2.INTER_CUBIC)
            heatmap = cv2.applyColorMap(resized, cv2.COLORMAP_JET)

            with thermal_lock:
                latest_thermal_img = heatmap.copy()

        except Exception as e:
            print("Thermal error:", e)

        time.sleep(0.125)  # 8 FPS

# Start thermal thread
threading.Thread(target=thermal_worker, daemon=True).start()

# -----------------------------
# Frame Generator (30 FPS)
# -----------------------------
def generate_frames():
    while True:
        rgb_frame = picam2.capture_array()

        with thermal_lock:
            thermal_img = latest_thermal_img.copy()

        combined = np.hstack((rgb_frame, thermal_img))

        ret, buffer = cv2.imencode('.jpg', combined, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# -----------------------------
# Flask Route
# -----------------------------
@app.route('/video')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='100.70.10.57', port=5000, threaded=True)