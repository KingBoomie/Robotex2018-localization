import localizer
import cv2


localiser = localizer.Localizer("../calibration/calibration-laptop.yml")
if 1 == 0:
    # For testing with static images
    img = cv2.imread("pilt3.png")
    localiser.estimateLocation(img, True)
    x = 123
    localiser.updateMapWithMotorData(x, x, x, x)
    field_map = localiser.getLatestMap()
    field_map.showWindow()
    print(field_map.robot_position)
    cv2.imshow("Changed", img)
    cv2.waitKey(0)
else:
    # For testing with the live feed
    cap = cv2.VideoCapture(0)
    while True:
        # read the image in and detect the marker & position
        check, img = cap.read()

        localiser.estimateLocation(img, True)
        x = 3
        localiser.updateMapWithMotorData(x, x, x, x)
        field_map = localiser.getLatestMap()
        field_map.showWindow()
        cv2.imshow("Changed", img)

        if cv2.waitKey(1) != -1:
            check, img = cap.read()
            cv2.imwrite('pilt1.png', img)
            break
cv2.destroyAllWindows()