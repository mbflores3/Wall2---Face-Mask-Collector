import serial
import time
import cv2
import numpy as np
import imutils
from imutils.video import FPS, WebcamVideoStream

from utils import hsv_masking, hsv_detection


#####PARÁMETROS PARA LA SEGMENTACIÓN #####
# upperThresh = np.array([115, 255, 255])
# lowerThresh = np.array([95, 35, 120])

upperThresh = np.array([111, 255, 255])
lowerThresh = np.array([91, 62, 61])


cannyLowerThresh = 50



debounce = 0

try:
    print("----------- INICIANDO WALL2 MARK5 ------------")


    print("----------- INICIANDO CAMARA ------------")

    height = 300
    stream = WebcamVideoStream(0)
    stream.start()
    # tracker = Tracker()
    fps = FPS().start()
    time.sleep(2)

    print("----------- COMENZANDO LECTURAS ------------")
    time.sleep(1)

    while True:

        init_loop = time.time()

        frame = stream.read()
        frame = imutils.resize(frame, height=height)
        fps.stop()

        # Detect Object
        hsv_mask, res1 = hsv_masking(frame, lowerThresh, upperThresh)

        # Track Object
        track_position = hsv_detection(hsv_mask)

        if(len(track_position) > 0):
        # Calculate node and draw finger
            cv2.circle(res1, tuple(track_position[0]), 6, (255, 255, 255), -1)
            cv2.putText(
                res1,
                "(%d, %d)" % (*track_position[0],),
                (abs(track_position[0][0] - 25), abs(track_position[0][1] - 25)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
            

         # Update the frames
        cv2.imshow("Live Feed", frame)
        cv2.imshow("Segmentation", res1)

        # Keyboard OP
        k = cv2.waitKey(5) & 0xFF
        if k == 27 or k == ord("q"):  # Esc
            break
        
        fps.update()

       




except KeyboardInterrupt:
    print("\nInterrupcion por teclado")
except ValueError as ve:
    print(ve)
    print("Otra interrupcion")
finally:
    pass
