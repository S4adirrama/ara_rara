from ultralytics import YOLO
import mss
import cv2
import numpy as np

model = YOLO('drone_sheep.pt')
sheep_count = 0

with mss.mss() as sct:
    region = {'top': 30, 'left': 0, 'width': 800, 'height': 600}
    
    while True:
        screenshot = sct.grab(region)
        img = np.array(screenshot)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        results = model(img)

        for result in results:
            for bbox in result.boxes:
                class_id = int(bbox.cls)
                label = model.names[class_id]
                x1, y1, x2, y2 = map(int, bbox.xyxy[0].tolist())

                if class_id == 0:
                    sheep_count += 1
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 0), 2)  # Cyan box

    
        cv2.putText(img, f'Sheep Count: {sheep_count}', (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)

        cv2.imshow('YOLO Detection', img)
        sheep_count = 0
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
