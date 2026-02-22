import cv2
import urllib.request
import numpy as np

# URL of Pi stream
url = "http://100.70.10.57:5000/video"

# Open connection to the MJPEG stream
stream = urllib.request.urlopen(url)
bytes_data = b''

# Known widths of the sender frames
RGB_WIDTH = 640
THERMAL_WIDTH = 320

while True:
    # Read a chunk of the stream
    bytes_data += stream.read(4096)

    # Look for JPEG start and end markers
    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')

    if a != -1 and b != -1 and b > a:
        jpg = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        if frame is not None:
            h, w, _ = frame.shape

            # Split frame into RGB and thermal using correct widths
            rgb_frame = frame[:, :RGB_WIDTH]
            thermal_frame = frame[:, RGB_WIDTH:RGB_WIDTH + THERMAL_WIDTH]

            cv2.imshow('RGB Camera', rgb_frame)
            cv2.imshow('Thermal Camera', thermal_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()