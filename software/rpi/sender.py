# stream_camera_thermal.py

from picamera2 import Picamera2
import cv2
import numpy as np
from flask import Flask, Response
import board
import busio
import adafruit_mlx90640

app = Flask(__name__)

# -----------------------------
# Initialize Picamera2
# -----------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# -----------------------------
# Initialize MLX90640
# -----------------------------
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ

thermal_frame = np.zeros((24 * 32,))

def get_thermal_image():
    try:
        mlx.getFrame(thermal_frame)
        data = np.reshape(thermal_frame, (24, 32))

        # Normalize temperature range
        min_temp = np.min(data)
        max_temp = np.max(data)
        norm = (data - min_temp) / (max_temp - min_temp)
        norm = (norm * 255).astype(np.uint8)

        # Resize to match camera height
        resized = cv2.resize(norm, (320, 480), interpolation=cv2.INTER_CUBIC)

        # Apply heatmap
        heatmap = cv2.applyColorMap(resized, cv2.COLORMAP_JET)

        return heatmap

    except Exception as e:
        print("Thermal read error:", e)
        return np.zeros((480, 320, 3), dtype=np.uint8)

# -----------------------------
# Frame Generator
# -----------------------------
def generate_frames():
    while True:
        rgb_frame = picam2.capture_array()

        thermal_img = get_thermal_image()

        # Combine side-by-side
        combined = np.hstack((rgb_frame, thermal_img))

        ret, buffer = cv2.imencode('.jpg', combined)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='192.168.1.200', port=5000, threaded=True)