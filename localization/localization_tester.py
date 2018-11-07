import localizer
import map
import cv2

cap = cv2.VideoCapture(0)
localiser = localizer.Localizer("../calibration/calibration-laptop.yml")

while True:
    # read the image in and detect the marker & position
    check, img = cap.read()
    field_map = localiser.estimateLocation(img, True)
    field_map.showWindow()
    cv2.imshow("Original", img)

    if cv2.waitKey(1) != -1:
        break
cv2.destroyAllWindows()