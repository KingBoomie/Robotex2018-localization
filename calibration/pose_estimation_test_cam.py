import cv2.aruco as aruco
import numpy
import yaml
import cv2

MARKER_SIZE_PHONE = 0.041
MARKER_SIZE_CHARUCO = 0.0109611111
MARKER_SIZE_HUGE = 0.195
MARKER_SIZE_CALCULATOR = 0.0555
MARKER_SIZE_REAL_FIELD = 0.25

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_PARAMETERS.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

# ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# output:
# POSITION [X]     [Y]          [Z]            [dunno]      [yaw]      [dunno]
# [[ 0.07525625 -0.06655711  0.56860975]] [[ 1.84563977  1.16536322 -0.85372057]]
# X - väiksem number = vasakul pool ekraani, suurem paremal, keskpunktiks ekraani keskpunkt
# Y - väiksem number üleval pool, väiksem allpool, keskpunktiks on ekraani alumine serv
# Z - väiksem number lähemal, suurem number kaugemal, nullpunktiks on kaamera lääts

with open("calibration/calibration-live.yml", 'r') as stream:
    try:
        calibration = yaml.load(stream)
        cam = numpy.array(calibration['camera_matrix']['data'])
        dc = numpy.array(calibration['distortion_coefficients']['data'])
        print("Calibration parameters:", calibration)
        print("Cam:", cam)
        print("DisCo:", dc)

        cap = cv2.VideoCapture(0)

        while True:
            check, img = cap.read()
            grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(grayscale, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
            rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_REAL_FIELD, cam, dc)

            # img = aruco.drawDetectedMarkers(img, corners, ids, borderColor=(0, 255, 0))
            # retval, rvec, tvec = cv2.solvePnP(corners)

            if rvec is not None:
                for i in range(len(rvec)):
                    img = aruco.drawAxis(img, cam, dc, rvec[i], tvec[i], 0.1)
                    # print(rvec)
                    # rvec, jacobian = cv2.Rodrigues(rvec)
                    # rv, tv = inversePerspective(rvec, tvec)
                    # print(ids[i], tvec[i], rvec[i])

                    const = 1  # 180/3.14159265358979323
                    img = cv2.putText(img, str(rvec[i][0][0]*const), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 3)
                    img = cv2.putText(img, str(rvec[i][0][1]*const), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 3)
                    img = cv2.putText(img, str(rvec[i][0][2]*const), (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 3)

                    # rvec_matrix = cv2.Rodrigues(rvec)
                    # proj_matrix = numpy.hstack((rvec_matrix, tvec))
                    # eulerAngles = -cv2.decomposeProjectionMatrix(proj_matrix)[6]

                    # dst, jacobian = cv2.Rodrigues(rvec[0])
                    # retval, eigenvalues, eigenvectors = cv2.eigen(dst)
                    # print(eigenvalues)

            cv2.imshow("Output", img)
            if cv2.waitKey(1) != -1:
                break
        cv2.imwrite("field/center.jpg", img)
        cv2.destroyAllWindows()
    except yaml.YAMLError as exc:
        print(exc)