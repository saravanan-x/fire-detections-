from ultralytics import YOLO
import cv2
import serial
import time

# Load YOLO fire model
model = YOLO("/home/saravanan/Documents/VS-CODE/fire_cv/fire_smoke_yolov8/runs/detect/train4/weights/best.pt")   # fire + smoke trained model

# Serial to ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

cap = cv2.VideoCapture(0)

last_status = "SAFE"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.4)

    status = "SAFE"

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])

            label = model.names[cls]

            if label in ["fire", "smoke"]:
                status = label.upper()

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,0,255), 2)
                cv2.putText(frame, f"{label} {conf:.2f}",
                            (x1,y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0,0,255), 2)

    # Send only when status changes
    if status != last_status:
        ser.write((status + "\n").encode())
        last_status = status

    cv2.putText(frame, status, (20,50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                (0,0,255) if status!="SAFE" else (0,255,0), 3)

    cv2.imshow("YOLO Fire & Smoke Detection", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
