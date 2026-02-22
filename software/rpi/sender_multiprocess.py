from picamera2 import Picamera2
import cv2
import numpy as np
from flask import Flask, Response
import multiprocessing
import time
import board
import busio
import adafruit_mlx90640

# -----------------------------
# Thermal Process Function
# -----------------------------
def thermal_process(queue):
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    mlx = adafruit_mlx90640.MLX90640(i2c)
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ

    thermal_raw = np.zeros((24*32,))
    while True:
        try:
            mlx.getFrame(thermal_raw)
            data = np.reshape(thermal_raw, (24, 32))

            # Normalize to 0-255 safely
            min_temp = np.min(data)
            max_temp = np.max(data)
            if max_temp - min_temp < 1e-3:
                norm = np.zeros_like(data, dtype=np.uint8)
            else:
                norm = ((data - min_temp) / (max_temp - min_temp) * 255).astype(np.uint8)

            # Resize and apply colormap
            resized = cv2.resize(norm, (320, 480), interpolation=cv2.INTER_CUBIC)
            heatmap = cv2.applyColorMap(resized, cv2.COLORMAP_JET)

            # Put latest frame in queue (overwrite if needed)
            if not queue.empty():
                try:
                    queue.get_nowait()
                except:
                    pass
            queue.put(heatmap)
        except Exception as e:
            print("Thermal error:", e)

        time.sleep(0.125)  # 8 Hz

# -----------------------------
# Flask / RGB Process
# -----------------------------
app = Flask(__name__)
thermal_queue = multiprocessing.Queue()

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

def generate_frames():
    last_thermal = np.zeros((480, 320, 3), dtype=np.uint8)
    while True:
        rgb_frame = picam2.capture_array()
        if rgb_frame.shape[2] == 4:  # RGBA -> BGR
            rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

        # Get latest thermal frame if available
        while not thermal_queue.empty():
            last_thermal = thermal_queue.get()

        # Combine RGB and thermal side by side
        combined = np.hstack((rgb_frame, last_thermal))

        ret, buffer = cv2.imencode('.jpg', combined, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# -----------------------------
# Main entry point
# -----------------------------
if __name__ == "__main__":
    # Start thermal process
    p = multiprocessing.Process(target=thermal_process, args=(thermal_queue,), daemon=True)
    p.start()

    # Start Flask server
    app.run(host='100.70.10.57', port=5000, threaded=True)