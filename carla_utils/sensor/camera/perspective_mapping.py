import carla_utils as cu

import numpy as np
import cv2


def linear_interpolation(pose1, pose2, min_dist):
    '''
    Args:
        pose1, pose2: numpy.ndarray (6×1)
    '''
    distance = np.linalg.norm(pose1[:3,0] - pose2[:3,0])
    total = int(distance / min_dist)
    if total <= 0:
        total = 1
    t_array = np.arange(total) / total
    interpolated_pose_array = (1-t_array) * pose1 + (t_array) * pose2
    interpolated_pose_array[3:,:] = cu.basic_tools.pi2pi(interpolated_pose_array[3:,:])
    return interpolated_pose_array


def data_augmentation(pose_array):
    number = pose_array.shape[1]
    augmented = None
    for i in range(number-1):
        if i/number < 0.4:
            min_dist = 0.02
        elif i/number < 0.6:
            min_dist = 0.04
        else:
            min_dist = 0.06
        
        p1 = pose_array[:,i][:,np.newaxis]
        p2 = pose_array[:,i+1][:,np.newaxis]
        interpolated = linear_interpolation(p1, p2, min_dist)
        if augmented is None:
            augmented = interpolated
        else:
            augmented = np.hstack((augmented, interpolated))
    return augmented



class PerspectiveMapping(object):
    def __init__(self, param):
        vehicle_width = param.vehicle_width
        self.vehicle_half_width = vehicle_width / 2
        self.lateral_step_factor = param.lateral_step_factor
        self.image_width = param.image_width
        self.image_height = param.image_height
        self.max_pixel = np.array([self.image_height, self.image_width]).reshape(2,1)
        self.min_pixel = np.array([0, 0]).reshape(2,1)
    
    def get(self, image, pose_array, T):
        '''
            image: numpy.ndarray (height, width, 3)
            pose_array: world coordinate
            T: transformation matrix from word to image
        '''
        augmented = data_augmentation(pose_array)

        yaw_array = augmented[-1,:][np.newaxis,:] + np.pi/2
        direction_array = np.vstack((np.cos(yaw_array), np.sin(yaw_array), np.zeros((1,augmented.shape[1]))))

        one_row = np.ones((1,augmented.shape[1]))
        point1 = np.vstack((augmented[:3,:] - self.vehicle_half_width * direction_array, one_row))
        point2 = np.vstack((augmented[:3,:] + self.vehicle_half_width * direction_array, one_row))
        pixel1, pixel2 = np.dot(T, point1), np.dot(T, point2)
        pixel_array1, pixel_array2 = pixel1[:2,:] / pixel1[-1,:], pixel2[:2,:] / pixel2[-1,:]

        image = cv2.resize(image, (self.image_width, self.image_height), interpolation=cv2.INTER_CUBIC)
        for i in range(augmented.shape[1]):
            pixel1, pixel2 = pixel_array1[:,i][:,np.newaxis], pixel_array2[:,i][:,np.newaxis]
            self.draw_line(image, pixel1, pixel2)

        return self.postprocess(image)
    

    def draw_line(self, image, pixel1, pixel2):
        pixel1, pixel2 = pixel1[::-1,:], pixel2[::-1,:]
        flag1 = (pixel1 >= self.min_pixel).all() and (pixel1 < self.max_pixel).all()
        flag2 = (pixel2 >= self.min_pixel).all() and (pixel2 < self.max_pixel).all()
        if not flag1 and not flag2: return

        length = np.linalg.norm(pixel2 - pixel1)
        direction = (pixel2 - pixel1) / length
        lateral_sample_number = round(length / self.lateral_step_factor) + 1
        distance_array = np.linspace(0, length, lateral_sample_number)
        
        pixel_vec = pixel1 + distance_array * direction

        x_pixel = pixel_vec.astype(int)[0]
        y_pixel = pixel_vec.astype(int)[1]

        mask = np.where((x_pixel >= 0)&(x_pixel < self.image_height))[0]
        x_pixel = x_pixel[mask]
        y_pixel = y_pixel[mask]
        
        mask = np.where((y_pixel >= 0)&(y_pixel < self.image_width))[0]
        x_pixel = x_pixel[mask]
        y_pixel = y_pixel[mask]

        '''white'''
        # image[x_pixel, y_pixel] = 255
        '''gold'''
        image[x_pixel, y_pixel, 2] = 255
        image[x_pixel, y_pixel, 1] = 215


    def postprocess(self, image):
        kernel = np.ones((5,5),np.uint8)
        image = cv2.dilate(image ,kernel, iterations = 1)
        image = cv2.erode(image, kernel, iterations = 1)
        return image