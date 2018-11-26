import numpy
import yaml
import cv2
import map


class Localizer:
    MARKER_SIZE_PHONE = 0.041
    MARKER_SIZE_CHARUCO = 0.0109611111
    MARKER_SIZE_HUGE = 0.195
    MARKER_SIZE_CALCULATOR = 0.0555
    MARKER_SIZE_REAL_FIELD = 0.16

    MARKER_SIZE = MARKER_SIZE_REAL_FIELD

    marker_ids_to_detect = [11, 12, 21, 22]
    markerdata = {
        11: {"loc_x": -2.3, "loc_y": -0.23, "angle": numpy.pi},
        12: {"loc_x": -2.3, "loc_y": +0.23, "angle": numpy.pi},
        21: {"loc_x": +2.3, "loc_y": +0.23, "angle": 0},
        22: {"loc_x": +2.3, "loc_y": -0.23, "angle": 0}
    }

    def __init__(self, camera_parameters_file_location):
        with open(camera_parameters_file_location, 'r') as stream:
            try:
                calibration = yaml.load(stream)
                self.camera_matrix = numpy.array(calibration['camera_matrix']['data'])
                self.distortion_coefficients = numpy.array(calibration['distortion_coefficients']['data'])

                self.ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()
                self.ARUCO_PARAMETERS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

                self.ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
            except yaml.YAMLError as exc:
                print(exc)

    def estimateLocation(self, frame, draw_axis_to_frame=False):
        # Initialize the output variables
        out_position_two_markers_1x = None
        out_position_two_markers_2x = None
        out_position_11 = None
        out_position_12 = None
        out_position_21 = None
        out_position_22 = None
        angle_to_blue = None
        angle_to_pink = None

        # Convert the frame to grayscale so that ArUco detection would work
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers and try to do pose estimation
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
            grayscale, self.ARUCO_DICT, parameters=self.ARUCO_PARAMETERS)
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.MARKER_SIZE, self.camera_matrix, self.distortion_coefficients)

        # If any markers were detected ...
        if rvec is not None:
            detected_markers_with_corrent_ids = []
            for i in range(len(rvec)):
                if ids[i] in self.marker_ids_to_detect:
                    detected_markers_with_corrent_ids.append(tvec)
                    if draw_axis_to_frame:
                        frame = cv2.aruco.drawAxis(frame, self.camera_matrix,
                                                   self.distortion_coefficients, rvec[i], tvec[i], 0.1)
                        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                    # Read in some metadata about the detected marker
                    marker_id = int(ids[i])
                    markerdata = self.markerdata[marker_id]
                    marker_absolute_position = (markerdata['loc_x'], markerdata['loc_y'])

                    # Calculate the marker's distance and angle from camera (position)
                    marker_distance_from_camera = numpy.sqrt(tvec[i][0][0] ** 2 + tvec[i][0][2] ** 2)
                    marker_angle_from_camera = numpy.tan(tvec[i][0][0] / tvec[i][0][2])
                    #print(marker_angle_from_camera*180/3.14159265358979323)

                    # Calculate the marker's attitude/orientation (rotation)
                    rotM = numpy.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec[i - 1], rotM, jacobian=0)
                    ypr = cv2.RQDecomp3x3(rotM)
                    angle_between_marker_normal_and_camera = numpy.arccos(ypr[4][2][2])

                    sign_calculation = rvec[i][0][2]
                    if rvec[i][0][0] < 0:
                        sign_calculation = sign_calculation * -1

                    if sign_calculation < 0:
                        angle = -angle_between_marker_normal_and_camera
                    else:
                        angle = angle_between_marker_normal_and_camera

                    # Do some orientation measurements and saving first
                    angle_to_basket = markerdata['angle'] - angle # angle_between_marker_normal_and_camera
                    if markerdata['angle'] == 0:
                        # This is the blue basket
                        angle_to_blue = angle_to_basket
                    else:
                        # This is the pink basket
                        angle_to_pink = angle_to_basket

                    print(angle_to_basket * 180 / 3.14159265358979323)

                    # Combine the markers location angle and orientation angle to get the real angle
                    angle += marker_angle_from_camera
                    angle += markerdata['angle']

                    # Knowing the angle and distance, calculate the location of the camera
                    robot_x = marker_absolute_position[0] - marker_distance_from_camera * numpy.cos(angle)
                    robot_y = marker_absolute_position[1] + marker_distance_from_camera * numpy.sin(angle)

                    if marker_id == 11:
                        out_position_11 = robot_x, robot_y
                    if marker_id == 12:
                        out_position_12 = robot_x, robot_y
                    if marker_id == 21:
                        out_position_21 = robot_x, robot_y
                    if marker_id == 22:
                        out_position_22 = robot_x, robot_y

            if len(detected_markers_with_corrent_ids) == 2:
                dist = [
                    numpy.sqrt(tvec[0][0][0] ** 2 + tvec[0][0][2] ** 2),
                    numpy.sqrt(tvec[1][0][0] ** 2 + tvec[1][0][2] ** 2)
                ]
                print(dist)

                if ids[0] == 11 or ids[0] == 12:
                    dist11 = -1
                    dist12 = -1
                    if ids[0] == 11 and ids[1] == 12:
                        dist11 = dist[0]
                        dist12 = dist[1]
                    elif ids[0] == 12 and ids[1] == 11:
                        dist12 = dist[0]
                        dist11 = dist[1]

                    if dist11 != -1 and dist12 != -1:

                        # dist between the markers is 0.46 meters
                        dist_between_markers = 0.46

                        # Apply the law of cosines
                        cos_b = (dist_between_markers**2 + dist12**2 - dist11**2)/(2*dist12*dist_between_markers)
                        angle = numpy.arccos(cos_b)

                        out_x = self.markerdata[12]['loc_x'] + numpy.sin(angle) * dist12
                        out_y = self.markerdata[12]['loc_y'] - numpy.cos(angle) * dist12
                        out_position_two_markers_1x = out_x, out_y

                if ids[0] == 21 or ids[0] == 22:
                    dist21 = -1
                    dist22 = -1
                    if ids[0] == 21 and ids[1] == 22:
                        dist21 = dist[0]
                        dist22 = dist[1]
                    elif ids[0] == 22 and ids[1] == 21:
                        dist22 = dist[0]
                        dist21 = dist[1]

                    if dist21 != -1 and dist22 != -1:
                        # dist between the markers is 0.46 meters
                        dist_between_markers = 0.46

                        # Apply the law of cosines
                        cos_b = (dist_between_markers ** 2 + dist22 ** 2 - dist21 ** 2) / (
                                    2 * dist22 * dist_between_markers)
                        angle = numpy.arccos(cos_b)

                        out_x = self.markerdata[22]['loc_x'] + numpy.sin(angle) * dist22
                        out_y = self.markerdata[22]['loc_y'] - numpy.cos(angle) * dist22
                        out_position_two_markers_2x = out_x, out_y

            elif len(detected_markers_with_corrent_ids) > 2:
                print("Too many markers found!")

        out_map = map.BasketballFieldMap(None, out_position_11, out_position_12, out_position_21, out_position_22,
                                         out_position_two_markers_1x, out_position_two_markers_2x)

        out_map.angle_to_pink = angle_to_pink
        out_map.angle_to_blue = angle_to_blue

        if angle_to_pink is not None or angle_to_blue is not None:
            if angle_to_pink is not None:
                out_map.robot_angle = angle_to_pink
            else:
                out_map.robot_angle = angle_to_blue

        #if out_position_two_markers_1x is not None:
        #    out_map.robot_position = out_position_two_markers_1x
        #el
        if out_position_11 is not None:
            out_map.robot_position = out_position_11
        elif out_position_12 is not None:
            out_map.robot_position = out_position_12
        elif out_position_21 is not None:
            out_map.robot_position = out_position_21
        elif out_position_22 is not None:
            out_map.robot_position = out_position_22
        else:
            out_map.robot_position = (None, None)

        return out_map
