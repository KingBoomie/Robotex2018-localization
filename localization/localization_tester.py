import localizer
import map
import cv2

cap = cv2.VideoCapture(0)
localiser = localizer.Localizer("../calibration/calibration-laptop.yml")
field_map = map.BasketballFieldMap()

while True:
    # read the image in and detect the marker & position
    check, img = cap.read()
    position = localiser.estimateLocation(img, True)
    field_map.setRobotPosition(position)
    field_map.showWindow()

    if cv2.waitKey(1) != -1:
        break
cv2.destroyAllWindows()