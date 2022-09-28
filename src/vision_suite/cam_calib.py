#!/usr/bin/env python
import cv2
import numpy as np
import glob
import rospkg

rp = rospkg.RosPack()
dir_path = rp.get_path("mav_companion_api")
print(dir_path)

boardSize = (7, 7)
frameSize = (480, 300)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((boardSize[0]*boardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:boardSize[0], 0:boardSize[1]].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(dir_path+"/data/checker_board_images/*.png")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, boardSize, None)

    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        cv2.drawChessboardCorners(img, boardSize, corners2, ret)
        cv2.imshow(fname, img)
        cv2.waitKey(500)

# while not (cv2.waitKey(1) & 0xFF == ord('q')):
#     pass

cv2.destroyAllWindows()

################# Calibration #################
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("camera matrix:\n", mtx)
print("distortion:\n", dist)



