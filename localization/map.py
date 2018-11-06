import cv2


def convertRealCoordsToFieldImg(x, y, round_to_integer=True):
    pixelsPerMeter = (630-121)/4.6  # 630-121 pixels correspond to 4.6 meters

    x *= pixelsPerMeter
    y *= -pixelsPerMeter

    x += 376
    y += 262

    if round_to_integer:
        return int(x), int(y)
    else:
        return x, y


def getFieldWithPoint(x, y):
    field_background = cv2.imread("field.png")
    center = convertRealCoordsToFieldImg(x, y)
    cv2.circle(field_background, center, 3, (255, 0, 0), 3)
    return field_background


cv2.imshow("Field", getFieldWithPoint(0, 0))
cv2.waitKey(0)
cv2.destroyAllWindows()