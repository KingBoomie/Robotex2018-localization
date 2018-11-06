import cv2


class BasketballFieldMap:

    """ Contains the latest known robot position, (None, None) means the position is unknown """
    robot_position = (None, None)

    """ Creates a new field map object """
    def __init__(self):
        pass

    def setRobotPosition(self, new_position):
        self.robot_position = new_position

    @staticmethod
    def convertRealCoordsToFieldImg(coordinates, round_to_integer=True):
        """
        Converts meters to pixels on the field image. The axis zero point is in the center of
        the field with X-axis going to the right and the Y-axis going up

        :param coordinates A tuple of the x and y coordinates that should be converted
        :param round_to_integer if set to True, the output coordinates will always be integers;
               if set to false, the output coordinates will be floating point values
        :returns the corresponding pixel value for the input coordinates
        """

        pixelsPerMeter = (630 - 121) / 4.6  # 630-121 pixels correspond to 4.6 meters

        output_x = coordinates[0] * pixelsPerMeter
        output_y = coordinates[1] * -pixelsPerMeter

        output_x += 376
        output_y += 262

        if round_to_integer:
            return int(output_x), int(output_y)
        else:
            return output_x, output_y

    def getFieldAsImage(self):
        field_background = cv2.imread("field.png")
        if self.robot_position[0] is not None and self.robot_position[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position)
            cv2.circle(field_background, robot_location, 3, (50, 255, 50), 3)
        return field_background

    def showWindow(self, window_name="Basketball field"):
        cv2.imshow(window_name, self.getFieldAsImage())