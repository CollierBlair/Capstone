# thermal_server.py
import board
import busio
import adafruit_mlx90640
import cv2
import numpy as np
from flask import Flask, Response
import time

app = Flask(__name__)

# Initialize I2C and MLX90640
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ

thermal_raw = np.zeros((24*32,))
latest_thermal_img = np.zeros((240, 320, 3), dtype=np.uint8)

def generate_frames():
    global latest_thermal_img
    while True:
        try:
            mlx.getFrame(thermal_raw)
            data = np.reshape(thermal_raw, (24, 32))

            # Normalize safely
            min_temp = np.min(data)
            max_temp = np.max(data)
            if max_temp - min_temp < 1e-3:
                norm = np.zeros_like(data, dtype=np.uint8)
            else:
                norm = ((data - min_temp) / (max_temp - min_temp) * 255).astype(np.uint8)

            # Resize and apply colormap
            resized = cv2.resize(norm, (320, 240), interpolation=cv2.INTER_CUBIC)
            heatmap = cv2.applyColorMap(resized, cv2.COLORMAP_JET)
            latest_thermal_img = heatmap.copy()

            ret, buffer = cv2.imencode('.jpg', latest_thermal_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            print("Thermal read error:", e)
            time.sleep(0.1)


@app.route('/thermal')
def thermal_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='100.70.10.57', port=5001, threaded=True)