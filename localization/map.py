import math
import cv2


class BasketballFieldMap:

    def __init__(self, robot_position=None, robot_position_m11=None, robot_position_m12=None,
                 robot_position_m21=None, robot_position_m22=None, robot_position_m1=None, robot_position_m2=None):
        """
        Creates a new field map object with optional parameters.
        :param robot_position The estimated robot position
        :param robot_position_m11 Position estimated from marker 11
        :param robot_position_m12 Position estimated from marker 12
        :param robot_position_m21 Position estimated from marker 21
        :param robot_position_m22 Position estimated from marker 22
        :param robot_position_m1 Position estimated from markers 11 and 12
        :param robot_position_m2 Position estimated from markers 21 and 22
        """
        if robot_position is None:
            self.robot_position = (None, None)
        else:
            self.robot_position = robot_position

        if robot_position_m11 is None:
            self.robot_position_m11 = (None, None)
        else:
            self.robot_position_m11 = robot_position_m11

        if robot_position_m12 is None:
            self.robot_position_m12 = (None, None)
        else:
            self.robot_position_m12 = robot_position_m12

        if robot_position_m21 is None:
            self.robot_position_m21 = (None, None)
        else:
            self.robot_position_m21 = robot_position_m21

        if robot_position_m22 is None:
            self.robot_position_m22 = (None, None)
        else:
            self.robot_position_m22 = robot_position_m22

        if robot_position_m1 is None:
            self.robot_position_m1 = (None, None)
        else:
            self.robot_position_m1 = robot_position_m1

        if robot_position_m2 is None:
            self.robot_position_m2 = (None, None)
        else:
            self.robot_position_m2 = robot_position_m2

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
            if math.isnan(output_x) or math.isnan(output_y):
                return None, None
            else:
                return int(output_x), int(output_y)
        else:
            return output_x, output_y

    def getFieldAsImage(self):
        field_background = cv2.imread("field.png")

        font_size = 0.5
        color = (25, 25, 150)
        # Draw the estimations from single markers
        if self.robot_position_m11[0] is not None and self.robot_position_m11[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position_m11)
            if not any(map(lambda x: x is None, robot_location)):
                cv2.circle(field_background, robot_location, 3, color, 3)
                cv2.putText(field_background, "m11", robot_location, cv2.FONT_HERSHEY_COMPLEX, font_size, (0, 0, 0))

        if self.robot_position_m12[0] is not None and self.robot_position_m12[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position_m12)
            if not any(map(lambda x: x is None, robot_location)):
                cv2.circle(field_background, robot_location, 3, color, 3)
                cv2.putText(field_background, "m12", robot_location, cv2.FONT_HERSHEY_COMPLEX, font_size, (0, 0, 0))

        if self.robot_position_m21[0] is not None and self.robot_position_m21[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position_m21)
            if not any(map(lambda x: x is None, robot_location)):
                cv2.circle(field_background, robot_location, 3, color, 3)
                cv2.putText(field_background, "m21", robot_location, cv2.FONT_HERSHEY_COMPLEX, font_size, (0, 0, 0))

        if self.robot_position_m22[0] is not None and self.robot_position_m22[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position_m22)
            if not any(map(lambda x: x is None, robot_location)):
                cv2.circle(field_background, robot_location, 3, color, 3)
                cv2.putText(field_background, "m22", robot_location, cv2.FONT_HERSHEY_COMPLEX, font_size, (0, 0, 0))

        # Draw the estimations from multiple markers
        color = (150, 25, 25)
        if self.robot_position_m1[0] is not None and self.robot_position_m1[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position_m1)
            if not any(map(lambda x: x is None, robot_location)):
                cv2.circle(field_background, robot_location, 3, color, 3)
                cv2.putText(field_background, "m1x", robot_location, cv2.FONT_HERSHEY_COMPLEX, font_size, (0, 0, 0))

        if self.robot_position_m2[0] is not None and self.robot_position_m2[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position_m2)
            if not any(map(lambda x: x is None, robot_location)):
                cv2.circle(field_background, robot_location, 3, color, 3)
                cv2.putText(field_background, "m2x", robot_location, cv2.FONT_HERSHEY_COMPLEX, font_size, (0, 0, 0))

        # Draw the final estimated robot position
        if self.robot_position[0] is not None and self.robot_position[1] is not None:
            robot_location = self.convertRealCoordsToFieldImg(self.robot_position)
            if not any(map(lambda x: x is None, robot_location)):
                cv2.circle(field_background, robot_location, 5, (50, 255, 50), 3)

        return field_background

    def showWindow(self, window_name="Basketball field"):
        cv2.imshow(window_name, self.getFieldAsImage())