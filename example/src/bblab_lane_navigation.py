import cv2
import datetime
import logging
import numpy as np
import math
import sys
import matplotlib.pyplot as plt


_PRESENT_IMAGE = True
imagesGrid = plt.figure(figsize=(6,10))

class BBLabLaneNavigation(object):

    def __init__(self):
        logging.info('Creating a BBLabLaneFollower class ... ')
        self.current_steering_angle = 90
        
        
    def navigate_lane(self, img):
        #present_image('original_image', img, 1)

        lines_of_lane, lane_image = lane_detector(img)
        final_result_img, current_navigating_angle = self.navigating(img, lines_of_lane)

        return final_result_img, current_navigating_angle


    def navigating(self, img, lines_of_lane):
        logging.debug('Navigating...')
        if len(lines_of_lane) == 0:
            logging.error('Nothing...')
            return img, 90
        det_new_steering_angle = calculating_steering_angle(img, lines_of_lane)
        self.current_steering_angle = keep_steering_angle_stable_value(self.current_steering_angle, det_new_steering_angle, len(lines_of_lane))

        current_navigating_img = draw_navigating_line(img, self.current_steering_angle)
        present_image('navigating image', current_navigating_img, 7)
        
        return current_navigating_img, self.current_steering_angle

# functions
def lane_detector(img):
    try:
        logging.debug('Trying to find lane lines...')
        
        detected_edges_img = edges_detector(img)
        present_image('edges image', detected_edges_img, 2)
        
        cropped_edges_img = crop_interest_region(detected_edges_img)
        present_image('edges cropped image', cropped_edges_img, 4)

        segments_of_line = detector_for_segments_of_line(cropped_edges_img)
        segments_line_img = draw_lines(img, segments_of_line)
        present_image('segments line image', segments_line_img, 5)

        lines_of_lane = line_parameters_mean(img, segments_of_line)
        lines_lane_img = draw_lines(img, lines_of_lane)
        present_image('lines lane image', lines_lane_img, 6)
        
                
        return lines_of_lane, lines_lane_img
    
    except Exception as ex:
        print("Error here: ", ex)
        

def edges_detector(img):
    try:
        # Gray image
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # GaussianBlur
        blur_img = cv2.GaussianBlur(gray_img, (5,5), 0)
        
        # Detect edges
        edges_img = cv2.Canny(blur_img, 200, 400)
        
        return edges_img
    
    except Exception as ex:
        print("Error here: ", ex)
        
def crop_interest_region(canny_img):
    height, width = canny_img.shape
    mask_img = np.zeros_like(canny_img)
    
    polygon_area = np.array([[
        (width * 1/4, height * 1/2),
        (width * 3/4, height * 1/2),
        (width * 8/8, height),
        (width * 0/8, height),
    ]], np.int32)
    
    cv2.fillPoly(mask_img, polygon_area, 255)
    present_image('mask_imgage', mask_img, 3)
    det_masked_img = cv2.bitwise_and(canny_img, mask_img)
    
    return det_masked_img
    

def present_image(title, img, pos, isPresent = _PRESENT_IMAGE):
    if isPresent:
        resize_img = cv2.resize(img, (480, 240))
        cv2.imshow(title + ' ' + str(pos), resize_img)
##        imagesGrid.add_subplot(3,3, pos)
##        plt.imshow(img)
##        #plt.show()
##        plt.axis('off')
##        plt.title(title)
        
        

def create_points(img, line_paras):
    h, w, _ = img.shape
    a, b = line_paras
    
    y1 = h
    y2 = int(y1 / 2)
    
    x1 = max(0, min(w, int((y1 - b) / a)))
    x2 = max(0, min(w, int((y2 - b) / a)))
    
    return [[x1, y1, x2, y2]]
    


def Euclidean_length_of_line(line_pos):
    x1, y1, x2, y2 = line_pos
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def draw_navigating_line(img, det_steering_angle, lineColour=(0, 0, 255), lineWidth=5, ):
    navigating_img = np.zeros_like(img)
    h, w, _ = img.shape
    
    det_steering_angle_radian = det_steering_angle / 180.0 * math.pi
    x1 = int(w/2)
    y1 = h
    x2 = int(x1 - h * 1/2 / math.tan(det_steering_angle_radian))
    y2 = int(h/2)
    
    cv2.line(navigating_img, (x1, y1), (x2, y2), lineColour, lineWidth)
    navigating_img = cv2.addWeighted(img, 0.8, navigating_img, 1, 1)
    
    return navigating_img


def draw_lines(img, line_points, lineColour=(0, 255, 0), lineWidth=10):
    drawlines_img = np.zeros_like(img)
    if line_points is not None:
        for line_point in line_points:
            for x1, y1, x2, y2 in line_point:
                cv2.line(drawlines_img, (x1, y1), (x2, y2), lineColour, lineWidth)
    drawlines_img = cv2.addWeighted(img, 0.8, drawlines_img, 1, 1)
    
    return drawlines_img


def keep_steering_angle_stable_value(current_steering_angle, det_new_steering_angle, number_lane_lines, maximum_variation_angle_two_lines = 3, maximum_variation_angle_one_line = 1):
    if number_lane_lines == 2:
        maximum_variation_angle = maximum_variation_angle_two_lines
    else:
        maximum_variation_angle = maximum_variation_angle_one_line
        
    variation_angle = det_new_steering_angle - current_steering_angle
    if abs(variation_angle) > maximum_variation_angle:
        steering_angle_stable_value = int(current_steering_angle + maximum_variation_angle * variation_angle / abs(variation_angle))
    else:
        steering_angle_stable_value = det_new_steering_angle
    
    return steering_angle_stable_value

def calculating_steering_angle(img, lines_of_lane):
    
    if len(lines_of_lane) == 0:
        logging.info('Did not detect any lane, nothing to do')
        return -90
    
    h, w, _ = img.shape
    
    if len(lines_of_lane) == 1:
        logging.debug('Just detected one line of lane merely, so use it for navigating: %s' % lines_of_lane[0])
        x1, _, x2, _ = lines_of_lane[0][0]
        offset_x = x2 - x1
    else:
        _, _, x2_left_side, _ = lines_of_lane[0][0]
        _, _, x2_right_side, _ = lines_of_lane[1][0]
        offset_percent_of_camera_mid_pos = 0.00
        x_middle = int(w/2 * (1+offset_percent_of_camera_mid_pos))
        offset_x = (x2_left_side + x2_right_side) / 2 - x_middle
        
    offset_y = int(h/2)
    
    steering_angle_in_radian_unit = math.atan(offset_x / offset_y)
    steering_angle_in_degree_unit = int(steering_angle_in_radian_unit * 180.0 / math.pi)
    
    return steering_angle_in_degree_unit + 90



def detector_for_segments_of_line(cropped_edges_img):
    min_precision = 1 # uint: pixel
    one_degree_in_radian_unit = np.pi / 180
    minimum_threshold = 10
    segments_of_line = cv2.HoughLinesP(cropped_edges_img, min_precision, one_degree_in_radian_unit,
                                       minimum_threshold, np.array([]), minLineLength=8, maxLineGap=4)

    if segments_of_line is not None:
        for segment_of_line in segments_of_line:
            pass

    return segments_of_line

    
    

def line_parameters_mean(img, segments_detected_line):

    lines_of_lane = []

    if segments_detected_line is None:
        logging.info('Not have any segments of line detected')
        return lines_of_lane

    h, w, _ = img.shape
    fit_left_side = []
    fit_right_side = []

    boundary = 1/2
    boundary_left_area = w * (1 - boundary)
    boundary_right_area = w * boundary

    for segment_detected_line in segments_detected_line:
        for x1, y1, x2, y2 in segment_detected_line:
            if x1 == x2:
                continue
            fit_paras = np.polyfit((x1, x2), (y1, y2), 1)
            a = fit_paras[0]
            b = fit_paras[1]
            if (abs(a) < 0.5):
                continue
            else:
                if a < 0:
                    if x1 < boundary_left_area and x2 < boundary_left_area:
                        fit_left_side.append((a, b))
                else:
                    if x1 > boundary_right_area and x2 > boundary_right_area:
                        fit_right_side.append((a, b))

    mean_value_of_fit_left = np.average(fit_left_side, axis=0)
    if len(fit_left_side) > 0:
        lines_of_lane.append(create_points(img, mean_value_of_fit_left))

    mean_value_of_fit_right = np.average(fit_right_side, axis=0)
    if len(fit_right_side) > 0:
        lines_of_lane.append(create_points(img, mean_value_of_fit_right))
                    
    return lines_of_lane    

def image_test(file_path):
    bblab_lane_navigating = BBLabLaneNavigation()
    img = cv2.imread(file_path)
    result_img, _ = bblab_lane_navigating.navigate_lane(img)
    present_image('result image', result_img, 8)
    cv2.waitKey(0)
    cv2.destroyAllWindow()

def video_test(video_path):
    bblab_lane_navigating = BBLabLaneNavigation()
    capture = cv2.VideoCapture(video_path + '.avi')

    for i in range(3):
        _, img = capture.read()

    h, w, _ = img.shape

    output_video_type = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter("%s_output_result_1.mp4" % (video_path), output_video_type, 30.0, (w, h))
    try:
        #i = 0
        while capture.isOpened():
            _, img = capture.read()
            result_img, _ = bblab_lane_navigating.navigate_lane(img)
            video_writer.write(result_img)
            present_image('result image', result_img, 8)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as ex:
        print("Error here: ", ex)
        
    finally:
        capture.release()
        video_writer.release()
        cv2.destroyAllWindows()
        
    
if __name__ == '__main__':
    image_test('/home/tiendungnguyen257/catkin_ws/src/Simulator/example/src/image0/frame2705.jpg')
    #video_test('output_1')
    
    















