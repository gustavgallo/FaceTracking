import cv2
import serial
import mediapipe as mp

capture = cv2.VideoCapture(1)

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.8)

def send_data_arduino(x, y, w, h):
    coords = f"{x},{y}\r"
    ser.write(coords.encode())
    print(f"X{x}Y{y}\n")

while True:
    isTrue, frame = capture.read()

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_detection.process(frame_rgb)

    if results.detections:
        h, w, _ = frame.shape
        maior_area = 0
        melhor_bbox = None

        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            x = int(bbox.xmin * w)
            y = int(bbox.ymin * h)
            bw = int(bbox.width * w)
            bh = int(bbox.height * h)
            area = bw * bh

            if area > maior_area:
                maior_area = area
                melhor_bbox = (x, y, bw, bh)

        if melhor_bbox:
            x, y, bw, bh = melhor_bbox
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 5)
            send_data_arduino(x, y, bw, bh)

    cv2.imshow('Video', frame)

    if cv2.waitKey(20) & 0xFF == ord('d'):
        break

capture.release()
cv2.destroyAllWindows()
