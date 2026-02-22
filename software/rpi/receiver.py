import cv2
import urllib.request
import numpy as np

# URL of Pi stream
url = "http://100.70.10.57:5000/video"

stream = urllib.request.urlopen(url)
bytes_data = b''

while True:
    bytes_data += stream.read(4096)

    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')

    if a != -1 and b != -1 and b > a:
        jpg = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        if frame is not None:
            h, w, _ = frame.shape

            # Split frame (left = RGB, right = thermal)
            rgb_frame = frame[:, :w//2]
            thermal_frame = frame[:, w//2:]

            cv2.imshow('RGB Camera', rgb_frame)
            cv2.imshow('Thermal Camera', thermal_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()