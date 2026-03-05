import cv2
import urllib.request
import numpy as np

# URLs of the two streams
RGB_URL = "http://100.70.10.57:5000/video"
THERMAL_URL = "http://100.70.10.57:5001/thermal"

def mjpeg_frame_reader(url):
    """Generator that yields frames from an MJPEG stream"""
    stream = urllib.request.urlopen(url)
    bytes_data = b''
    while True:
        bytes_data += stream.read(1024)
        a = bytes_data.find(b'\xff\xd8')
        b = bytes_data.find(b'\xff\xd9')
        if a != -1 and b != -1 and b > a:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                yield frame

# Create frame generators
rgb_frames = mjpeg_frame_reader(RGB_URL)
thermal_frames = mjpeg_frame_reader(THERMAL_URL)

print("Starting receiver. Press 'q' to quit.")

while True:
    try:
        rgb_frame = next(rgb_frames)
        thermal_frame = next(thermal_frames)

        # Resize thermal frame to match RGB height
        if thermal_frame.shape[0] != rgb_frame.shape[0]:
            thermal_frame = cv2.resize(thermal_frame,
                                       (int(thermal_frame.shape[1] * rgb_frame.shape[0] / thermal_frame.shape[0]),
                                        rgb_frame.shape[0]),
                                       interpolation=cv2.INTER_CUBIC)

        # Combine side by side
        combined = np.hstack((rgb_frame, thermal_frame))

        cv2.imshow("RGB + Thermal", combined)
    except Exception as e:
        print("Stream error:", e)
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()