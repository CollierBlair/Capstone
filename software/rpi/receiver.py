import cv2
import urllib.request
import numpy as np

# URL of Pi camera stream
url = "http://192.168.4.1:5000/video"

# Open a connection to the MJPEG stream
stream = urllib.request.urlopen(url)
bytes_data = b''

while True:
    # Read a chunk of the stream
    bytes_data += stream.read(1024)
    
    # Look for JPEG start and end markers
    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')
    
    if a != -1 and b != -1 and b > a:
        jpg = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        
        if frame is not None:
            cv2.imshow('Pi Camera Feed', frame)
        
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()