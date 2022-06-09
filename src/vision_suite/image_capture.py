#!/usr/bin/env python3
import cv2

if __name__ == "__main__":
    vid = cv2.VideoCapture(2)
    
    while True:
        ret, frame = vid.read()
        dim = (480, 300)
        frame_resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        cv2.imshow("hello", frame_resized)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite("pic1.png", frame_resized)
            break

    vid.release()