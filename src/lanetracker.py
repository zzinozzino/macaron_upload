#!/usr/bin/env python
#-*-coding:utf-8-*_

## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
##   rostopic echo color
## this demo shows some of the more advanced APIs in rospy.
import sys, time
from cv_bridge import CvBridge, CvBridgeError
# from scipy.ndimage import filters

import cv2
import numpy as np
import roslib

roslib.load_manifest('macaron')

import rospy

from matplotlib import pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension

from macaron.msg import Floats
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

flaots = Floats()




def plot_peak_hist(warped_gray, is_plot=True):
    warped_gray[:,(warped_gray.shape[1]//7)*3 -1:(warped_gray.shape[1]//7)*4 -1] = 0
    histogram = np.sum(warped_gray[warped_gray.shape[0] // 2:, :], axis=0)
    if is_plot: plt.plot(histogram)
    midpoint = np.int(histogram.shape[0] / 2)

    if max(histogram[:midpoint]) <= 10 and max(histogram[midpoint:]) <= 10:
        error = 1
        print('error = 1, plot_peak_hist')
        return [], error
    error = 0
    return histogram, error


def find_left_right_via_histogram(binary_warped, is_debug=False, is_plot=False, add_debug_image=False):
    debug_images = []

    histogram, error = plot_peak_hist(binary_warped, is_plot=False)
    if error == 1: return [], [], [], error, False, False
    error = 0

    if add_debug_image: debug_images.append(histogram)
    # if add_debug_image: debug_images.append(combined_binary)

    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    if np.max(histogram[:midpoint]) <= 10:
        leftx_base = 0
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        following_left_lane = False
        following_right_lane = True
    elif np.max(histogram[midpoint:]) <= 10:
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = 0
        following_left_lane = True
        following_right_lane = False
    else:
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        following_left_lane = False
        following_right_lane = False

    if is_debug:
        print("leftx_base:" + str(leftx_base))
        print("rightx_base:" + str(rightx_base))
        print("midpoint:" + str(midpoint))

    return leftx_base, rightx_base, debug_images, error, following_left_lane, following_right_lane


def find_ploy_fit_window_search(binary_warped, leftx_base, rightx_base, following_left_lane, following_right_lane, road_width, PIXEL_CONVERSION_RATE, nwindows=9, is_debug=False, margin=100, minpix=50, is_plot=True, add_debug_image=True):
    debug_images = []

    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    # Set height of windows
    window_height = np.int(binary_warped.shape[0] / nwindows)
    if is_debug:
        print("window_height:" + str(window_height))

    # get x, y points for non zero pixels.
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated for each window
    # this is the max position for left and right
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Set the width of the windows +/- margin
    # Set minimum number of pixels found to recenter window

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    if following_left_lane == False and following_right_lane == False:
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 3)
            cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), (int(win_xright_high), int(win_y_high)), (0, 255, 0), 3)

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]
            # good_left_inds = list(good_left_inds)
            # good_right_inds = good_right_inds

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # # # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # print(out_img.shape , '-', out_img.dtype , '-', np.max(out_img), '-', np.min(out_img))
        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [255, 0, 0]

    if following_left_lane == True and following_right_lane == False:
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            # win_xright_low = rightx_current - margin
            # win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 3)
            # cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), (int(win_xright_high), int(win_y_high)), (0, 255, 0), 3)

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]
            # good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
            #            nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            # right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            # if len(good_right_inds) > minpix:
            #    rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        # right_lane_inds = np.concatenate(right_lane_inds)

        # # # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = leftx + round(int(road_width / PIXEL_CONVERSION_RATE))
        righty = nonzeroy[left_lane_inds]

        # print(out_img.shape , '-', out_img.dtype , '-', np.max(out_img), '-', np.min(out_img))
        out_img[lefty, leftx] = [255, 0, 0]
        # out_img[righty, rightx] = [255, 0, 0]

    if following_left_lane == False and following_right_lane == True:
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            # win_xleft_low = leftx_current - margin
            # win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            # cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 3)
            cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), (int(win_xright_high), int(win_y_high)), (0, 255, 0), 3)

            # Identify the nonzero pixels in x and y within the window
            # good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
            #            nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            # left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            # if len(good_left_inds) > minpix:
            #    leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        # left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # # # Extract left and right line pixel positions
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        leftx = rightx - round(int(road_width / PIXEL_CONVERSION_RATE))
        lefty = nonzeroy[right_lane_inds]

        # print(out_img.shape , '-', out_img.dtype , '-', np.max(out_img), '-', np.min(out_img))
        # out_img[lefty, leftx] = [0, 0, 255]
        out_img[righty, rightx] = [0, 0, 255]

    left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, rightx, righty)
    if add_debug_image:
        out_img = plot_detected_poly(out_img, left_fit, right_fit, is_plot=is_plot)
        debug_images.append(out_img)

    return left_fit, right_fit, debug_images


def fit_ploy_2_degree(leftx, lefty, rightx, righty):
    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    return left_fit, right_fit


def plot_detected_poly(out_img, left_fit, right_fit, is_plot=True):
    # Generate x and y values for plotting
    ploty = np.linspace(0, out_img.shape[0] - 1, out_img.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    for pts, index in enumerate(left_fitx):
        cv2.circle(out_img, ((int)(left_fitx[pts]), (int)(ploty[pts])), 3, (255, 255, 255), -1)

    for pts, index in enumerate(right_fitx):
        cv2.circle(out_img, ((int)(right_fitx[pts]), (int)(ploty[pts])), 3, (255, 255, 255), -1)

    #     plt.plot(left_fitx, ploty, color='yellow')
    #     plt.plot(right_fitx, ploty, color='yellow')

    if is_plot:
        plt.imshow(out_img)
        # print(out_img.shape, '-', out_img.dtype, '-', np.min(out_img), '-', np.max(out_img))
        # out_img_uint32 = out_img.astype(np.uint8)
        # cv2.imshow('out_img_uint32',out_img_uint32)
        # plt.xlim(0, out_img.shape[1])
        # plt.ylim(out_img.shape[0], 0)
    return out_img


def next_frame_find_poly_already_fitted(binary_warped_left, binary_warped_right, left_fit, right_fit, lower_offset, upper_offset, following_right_lane, following_left_lane, road_width, PIXEL_CONVERSION_RATE, margin=50, is_plot=True, add_debug_image=True):
    # Assume you now have a new warped binary image
    # from the next frame of video (also called "binary_warped")
    # It's now much easier to find line pixels!

    debug_images = []

    nonzero_left = binary_warped_left.nonzero()
    nonzero_right = binary_warped_right.nonzero()
    nonzeroy_left = np.array(nonzero_left[0])
    nonzerox_left = np.array(nonzero_left[1])
    nonzeroy_right = np.array(nonzero_right[0])
    nonzerox_right = np.array(nonzero_right[1])

    margin2 = margin + 20
    left_lane_inds = ((nonzerox_left > (left_fit[0] * (nonzeroy_left ** 2) + left_fit[1] * nonzeroy_left + left_fit[2] - margin))
                      & (nonzerox_left < (left_fit[0] * (nonzeroy_left ** 2) + left_fit[1] * nonzeroy_left + left_fit[2] + margin)))
    right_lane_inds = ((nonzerox_right > (right_fit[0] * (nonzeroy_right ** 2) + right_fit[1] * nonzeroy_right + right_fit[2] - margin))
                       & (nonzerox_right < (right_fit[0] * (nonzeroy_right ** 2) + right_fit[1] * nonzeroy_right + right_fit[2] + margin)))

    # Again, extract left and right line pixel positions
    leftx = nonzerox_left[left_lane_inds]
    lefty = nonzeroy_left[left_lane_inds]
    rightx = nonzerox_right[right_lane_inds]
    righty = nonzeroy_right[right_lane_inds]

    # Fit a second order polynomial to each
    if len(leftx) == 0 and len(rightx) == 0:
        print('error = 2, next_frame_find_poly_already_fitted, cant_find_line - leftx : ', len(leftx), ', rightx : ', len(rightx))
        error = 2
        return [], [], 0, 0, 0, [], error, False, False

    if following_left_lane:
        if len(leftx) == 0:
            print('error = 2, next_frame_find_poly_already_fitted, cant_find_line :: leftx : ', len(leftx), ', rightx : ', len(rightx))
            error = 2
            return [], [], 0, 0, 0, [], error, False, False
        left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, leftx, lefty)
        right_fit[2] = left_fit[2] + 3.5 / PIXEL_CONVERSION_RATE
    elif following_right_lane:
        if len(rightx) == 0:
            print('error = 2, next_frame_find_poly_already_fitted, cant_find_line :: leftx : ', len(leftx), ', rightx : ', len(rightx))
            error = 2
            return [], [], 0, 0, 0, [], error, False, False

        left_fit, right_fit = fit_ploy_2_degree(rightx, righty, rightx, righty)
        left_fit[2] = right_fit[2] - 3.5 / PIXEL_CONVERSION_RATE
    else:
        if len(leftx) == 0 or len(rightx) == 0:
            print('error = 2, next_frame_find_poly_already_fitted, cant_find_line :: leftx : ', len(leftx), ', rightx : ', len(rightx))
            error = 2
            return [], [], 0, 0, 0, [], error, False, False
        left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, rightx, righty)

    left_fitx = left_fit[0] * (binary_warped_left.shape[0] - 1) ** 2 + left_fit[1] * (binary_warped_left.shape[0] - 1) + left_fit[2]
    right_fitx = right_fit[0] * (binary_warped_right.shape[0] - 1) ** 2 + right_fit[1] * (binary_warped_right.shape[0] - 1) + right_fit[2]
    left_fitx_top = left_fit[2]
    right_fitx_top = right_fit[2]
    left_fitx_mid = left_fit[0] * (binary_warped_left.shape[0]//2 - 1) ** 2 + left_fit[1] * (binary_warped_left.shape[0]//2 - 1) + left_fit[2]
    right_fitx_mid = right_fit[0] * (binary_warped_right.shape[0]//2 - 1) ** 2 + right_fit[1] * (binary_warped_right.shape[0]//2 - 1) + right_fit[2]

    if right_fitx - left_fitx <= lower_offset or right_fitx - left_fitx >= upper_offset:  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 3, next_frame_find_poly_already_fitted, _bot_line_overlaped or line_breakaway :: right_fitx - left_fitx = ', right_fitx - left_fitx, 'left_fitx = ', left_fitx, 'right_fitx = ', right_fitx)
        error = 3
        return [], [], 0, 0, 0, [], error, False, False
    if right_fitx_top - left_fitx_top <= lower_offset or right_fitx_top - left_fitx_top >= upper_offset+400:  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 4, next_frame_find_poly_already_fitted, _top_line_overlaped or line_breakaway :: right_fitx_top - left_fitx_top = ', right_fitx_top - left_fitx_top, 'left_fitx_top = ', left_fitx_top, 'right_fitx_top = ', right_fitx_top)
        error = 4
        return [], [], 0, 0, 0, [], error, False, False
    if right_fitx_mid - left_fitx_mid <= lower_offset or right_fitx_mid - left_fitx_mid >= upper_offset:  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 5, next_frame_find_poly_already_fitted, _mid_line_overlaped or line_breakaway :: right_fitx_mid - left_fitx_mid = ', right_fitx_mid - left_fitx_mid, 'left_fitx_mid = ', left_fitx_mid, 'right_fitx_mid = ', right_fitx_mid)
        error = 5
        return [], [], 0, 0, 0, [], error, False, False
    if left_fitx > (binary_warped_left.shape[1] * 2) // 3 or (right_fitx < binary_warped_right.shape[1]) // 3:  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 6, next_frame_find_poly_already_fitted, driving_on_lane(deviation>(road_width/2) :: deviation = ', binary_warped_right.shape[1] // 2 - (right_fitx + left_fitx) / 2, '  road_width/2 = ', road_width / 2)
        error = 6
        return [], [], 0, 0, 0, [], error, False, False

    mid_fit = np.add(left_fit, right_fit) / 2
    mid_fit_grad_sum = 0
    for i in range(23):
        mid_fit_grad_sum = mid_fit_grad_sum + 2 * mid_fit[0] * (viewsize[1] // 23) * i + mid_fit[1]
        print(np.rad2deg(np.arctan(2 * mid_fit[0] * (viewsize[1] // 23) * i + mid_fit[1])))
    mid_fit_grad_ave = mid_fit_grad_sum / 23
    mid_fit_deg_ave = np.rad2deg(np.arctan(mid_fit_grad_ave))



    error = 0

    # Create an image to draw on and an image to show the selection window
    out_img_left = np.dstack((binary_warped_left, binary_warped_left, binary_warped_left)) * 255
    out_img_right = np.dstack((binary_warped_right, binary_warped_right, binary_warped_right)) * 255
    # Color in left and right line pixels
    out_img_left[lefty, leftx] = [255, 0, 0]
    out_img_right[righty, rightx] = [0, 0, 255]

    out_img_half_sum = np.zeros_like(out_img_left)
    out_img_half_sum[:, : out_img_half_sum.shape[1] // 2, :] = out_img_left[:, : out_img_left.shape[1] // 2, :]
    out_img_half_sum[:, out_img_half_sum.shape[1] // 2:, :] = out_img_right[:, out_img_right.shape[1] // 2:, :]

    result = None
    if add_debug_image:
        # debug_images.append(out_img_left)
        result, window_img_left, window_img_right, window_img_left_submargin, window_img_right_submargin = get_already_fit_image_plot(out_img_half_sum, left_fit, right_fit, mid_fit, road_width, PIXEL_CONVERSION_RATE, is_plot=is_plot, margin=margin)
        debug_images.append(result)
        debug_images.append(window_img_left)
        debug_images.append(window_img_right)
        debug_images.append(window_img_left_submargin)
        debug_images.append(window_img_right_submargin)

    cv2.imshow('before_error_6', result)


    if abs(mid_fit_deg_ave) >= 30:
        print('error = 7, next_frame_find_poly_already_fitted, lane_deg_ave is over than 30 :: mid_fit_deg_ave = ', mid_fit_deg_ave)
        error = 7
        return [], [], 0, 0, 0, [], error, False, False


    left_curverad, right_curverad = get_radius_of_curvature(left_fit, right_fit, out_img_left)
    deviation = get_vehicle_deviation(out_img_left, left_fit, right_fit)
    return left_fit, right_fit, left_curverad, right_curverad, deviation, debug_images, error, following_left_lane, following_right_lane


def get_radius_of_curvature(left_fit_cr, right_fit_cr, image_for_size):
    plot_yyy = np.linspace(0, image_for_size.shape[0] - 1, image_for_size.shape[0])

    # Define conversions in x and y from pixels space to meters
    # ym_per_pix = 30 / 720  # meters per pixel in y dimension
    # xm_per_pix = 3.7 / 700  # meters per pixel in x dimension
    y_eval = np.max(plot_yyy)

    # Fit new polynomials to x,y in world space
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

    # Now our radius of curvature is in meters
    return left_curverad, right_curverad


def get_vehicle_deviation(img, left_fit, right_fit):
    image_bottom_pixel = img.shape[0] - 1

    bottom_x_position_left_lane = left_fit[0] * (image_bottom_pixel ** 2) + left_fit[1] * (image_bottom_pixel) \
                                  + left_fit[2]
    bottom_x_position_right_lane = right_fit[0] * (image_bottom_pixel ** 2) + right_fit[1] * (image_bottom_pixel) \
                                   + right_fit[2]
    vehicle_offset = (bottom_x_position_left_lane + bottom_x_position_right_lane) / 2.0 - img.shape[1] / 2

    # convert pixels to real space
    return vehicle_offset * PIXEL_CONVERSION_RATE


def get_already_fit_image_plot(out_img_half_sum, left_fit, right_fit, mid_fit, road_width, PIXEL_CONVERSION_RATE, margin=50, is_plot=True):
    # Generate x and y values for plotting

    ploty = np.linspace(0, out_img_half_sum.shape[0] - 1, out_img_half_sum.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    sub_left_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2] - road_width / PIXEL_CONVERSION_RATE
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    sub_right_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2] + road_width / PIXEL_CONVERSION_RATE
    mid_fitx = mid_fit[0] * ploty ** 2 + mid_fit[1] * ploty + mid_fit[2]

    # # Generate a polygon to illustrate the search window area
    # # And recast the x and y points into usable format for cv2.fillPoly()
    # stack -> v(row), h(column), d(depth)
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx - margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    margin2 = margin + 20
    left_line_window1_submargin = np.array([np.transpose(np.vstack([left_fitx - margin2, ploty]))])
    left_line_window2_submargin = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin2, ploty])))])
    left_line_pts_submargin = np.hstack((left_line_window1_submargin, left_line_window2_submargin))
    right_line_window1_submargin = np.array([np.transpose(np.vstack([right_fitx - margin2, ploty]))])
    right_line_window2_submargin = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin2, ploty])))])
    right_line_pts_submargin = np.hstack((right_line_window1_submargin, right_line_window2_submargin))

    # # Draw the lane onto the warped blank image
    window_img_full = np.zeros_like(out_img_half_sum)
    window_img_left = np.zeros_like(out_img_half_sum)
    window_img_right = np.zeros_like(out_img_half_sum)
    window_img_left_submargin = np.zeros_like(out_img_half_sum)
    window_img_right_submargin = np.zeros_like(out_img_half_sum)
    cv2.fillPoly(window_img_full, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img_full, np.int_([right_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img_left, np.int_([left_line_pts]), (255, 255, 255))
    cv2.fillPoly(window_img_right, np.int_([right_line_pts]), (255, 255, 255))
    cv2.fillPoly(window_img_left_submargin, np.int_([left_line_pts_submargin]), (255, 255, 255))
    cv2.fillPoly(window_img_right_submargin, np.int_([right_line_pts_submargin]), (255, 255, 255))  # 필요하면(간소화 등) 특정 경우에만 코드가 돌도록 수정

    result = cv2.addWeighted(out_img_half_sum, 1, window_img_full, 0.3, 0)

    for pts, index in enumerate(left_fitx):
        cv2.circle(result, ((int)(left_fitx[pts]), (int)(ploty[pts])), 3, (255, 0, 255), -1)
        cv2.circle(result, ((int)(sub_right_fitx[pts]), (int)(ploty[pts])), 3, (0, 255, 255), -1)

    for pts, index in enumerate(right_fitx):
        cv2.circle(result, ((int)(right_fitx[pts]), (int)(ploty[pts])), 3, (255, 0, 255), -1)

    for pts, index in enumerate(mid_fitx):
        cv2.circle(result, ((int)(mid_fitx[pts]), (int)(ploty[pts])), 1, (0, 155, 255), -1)


    if is_plot:
        plt.xlim(0, 1280)
        plt.ylim(720, 0)
        plt.imshow(result)

    return result, window_img_left, window_img_right, window_img_left_submargin, window_img_right_submargin


def plot_lanes_unwrap(image_for_size, left_poly, right_poly, left_trun, right_trun, left_yellow_lane, right_yellow_lane, undistorted_in_rgb, M_INV, is_plot=False, add_debug_image=True):
    debug_images = []

    # Create an image to draw the lines on
    warp_zero = np.zeros_like(image_for_size).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    mid_foly = np.add(left_fit, right_fit)/2

    plot_yyy = np.linspace(0, image_for_size.shape[0] - 1, image_for_size.shape[0])

    #     pdb.set_trace()
    left_fitxxx = left_poly[0] * plot_yyy ** 2 + left_poly[1] * plot_yyy + left_poly[2]
    right_fitxxx = right_poly[0] * plot_yyy ** 2 + right_poly[1] * plot_yyy + right_poly[2]
    mid_fitxxx = mid_foly[0] * plot_yyy ** 2 + mid_foly[1] * plot_yyy + mid_foly[2]

    right_endxxx = np.copy(plot_yyy)
    left_endxxx = np.copy(plot_yyy)
    right_endxxx[:] = image_for_size.shape[1] - 1
    left_endxxx[:] = 0

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitxxx, plot_yyy]))])
    pts_left2 = np.array([np.flipud(np.transpose(np.vstack([left_fitxxx, plot_yyy])))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitxxx, plot_yyy])))])
    pts = np.hstack((pts_left, pts_right))
    pts_left_end = np.array([np.transpose(np.vstack([left_endxxx, plot_yyy]))])
    pts_right_end = np.array([np.transpose(np.vstack([right_endxxx, plot_yyy]))])
    pts_left_end_area = np.hstack((pts_left_end, pts_left2))
    pts_right_end_area = np.hstack((pts_right, pts_right_end))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    if left_trun:
        cv2.fillPoly(color_warp, np.int_([pts_left_end_area]), (255, 0, 0))
    if right_trun:
        cv2.fillPoly(color_warp, np.int_([pts_right_end_area]), (255, 0, 0))
    if left_yellow_lane:
        cv2.fillPoly(color_warp, np.int_([pts_left_end_area]), (0, 0, 255))
    if right_yellow_lane:
        cv2.fillPoly(color_warp, np.int_([pts_right_end_area]), (0, 0, 255))
    for pts, index in enumerate(mid_fitxxx):
        cv2.circle(color_warp, ((int)(mid_fitxxx[pts]), (int)(plot_yyy[pts])), 2, (0, 155, 255), -1)
    dot = mid_foly[0] * (image_for_size.shape[0]-15) ** 2 + mid_foly[1] * (image_for_size.shape[0]-15) + mid_foly[2]
    cv2.circle(color_warp, ((int)(dot), (int)(image_for_size.shape[0]-15)), 2, (255, 0, 0), -1)

    if add_debug_image: debug_images.append(color_warp)

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, M_INV, (undistorted_in_rgb.shape[1], undistorted_in_rgb.shape[0]))
    # cv2.imshow('newwarp',newwarp)
    # cv2.imshow('color_warp', color_warp)
    if add_debug_image: debug_images.append(newwarp)

    # Combine the result with the original image
    result = cv2.addWeighted(undistorted_in_rgb, 1, newwarp, 0.3, 0)

    #dot = mid_fitxxx[image_for_size.shape[0] - 1]
    dot_diff = dot - image_for_size.shape[1]//2
    dot_diff_inv = (dot_diff+1)/(dst_pts[3,0]-dst_pts[0,0] + 20)*undistorted_in_rgb.shape[1]
    cv2.line(result, (int(undistorted_in_rgb.shape[1] // 2 - 1), int((undistorted_in_rgb.shape[0]/2 + (undistorted_in_rgb.shape[0]/2 // 9) * 7))), (int(undistorted_in_rgb.shape[1] // 2 +dot_diff_inv), int(undistorted_in_rgb.shape[0]/2 + (undistorted_in_rgb.shape[0]/2 // 9) * 7)), [0, 0, 255], 3)
    cv2.line(result, (int(undistorted_in_rgb.shape[1] // 2 - 1), int((undistorted_in_rgb.shape[0]/2 + (undistorted_in_rgb.shape[0]/2 // 9) * 7))), (int(undistorted_in_rgb.shape[1] // 2 - 1), int(undistorted_in_rgb.shape[0]-1)), [0, 0, 255], 3)


    # if is_plot:
    # plot_two_images(color_warp, result, plot_diff=False)
    # plot_two_images(newwarp, result, plot_diff=False)

    if add_debug_image: debug_images.append(result)

    return color_warp, result, debug_images


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 좌표추출 함수 선언 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def mouse_callback(event, x, y, flags, param):  # 화면에서 마우스 클릭시 호출되는 함수
    global selected_pts, video_copy1
    if event == cv2.EVENT_LBUTTONUP:
        selected_pts.append([x, y])
        # cv2.circle(video_copy1, (x,y),10,(0,255,0),3)


def select_points(image, points_num):  # 로드 되는 창에서 변환할 좌표 4 곳을 클릭
    global selected_pts
    selected_pts = []
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)
    while True:
        if len(selected_pts) >= 1:  # 점에 원 표시
            cv2.circle(image, tuple(selected_pts[0]), 10, (0, 0, 255), 3)
            if len(selected_pts) >= 2:
                cv2.circle(image, tuple(selected_pts[1]), 10, (0, 127, 255), 3)
                if len(selected_pts) >= 3:
                    cv2.circle(image, tuple(selected_pts[2]), 10, (0, 255, 255), 3)
        cv2.imshow('image', image)
        k = cv2.waitKey(1)
        if k == 27 or len(selected_pts) == points_num:
            break
    cv2.destroyAllWindows()
    return np.array(selected_pts, dtype=np.float32)


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 좌표추출 함수 선언 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def PTtrans_and_Circle(video_copy1, video_copy2, src_pts_yfix, dst_pts, viewsize):
    cv2.circle(video_copy2, tuple(src_pts_yfix[0]), 10, (0, 0, 255), 3)  # 변환하는 위치를 원본 이미지에 표시
    cv2.circle(video_copy2, tuple(src_pts_yfix[1]), 10, (0, 127, 255), 3)  # 빨주노초, 왼쪽 아래부터 시계방향
    cv2.circle(video_copy2, tuple(src_pts_yfix[2]), 10, (0, 255, 255), 3)
    cv2.circle(video_copy2, tuple(src_pts_yfix[3]), 10, (0, 255, 0), 3)


    perspective_m = cv2.getPerspectiveTransform(src_pts_yfix, dst_pts)
    video_copy1_PTtrans = cv2.warpPerspective(video_copy1, perspective_m, (viewsize[0], viewsize[1]), flags=cv2.INTER_LINEAR)  # 변환된 hsv 영상 출력 영역 (dst_pts 랑 똑같이 맞춰주면 됨)
    # video_copy1_PTtrans = cv2.warpPerspective(video_copy2, perspective_m, (viewsize[0],viewsize[1]))  # circle 포함
    # video_copy1_PTtrans = np.copy(video_copy1)                                        # 투상변환 적용 안하기

    # cv2.imshow('video_copy1_PTtrans', video_copy1_PTtrans)  #변환 이미지 출력
    return video_copy1_PTtrans


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ역투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def PTtrans_Reverse(video_window, src_pts_yfix, dst_pts, originalsize):
    perspective_m_resvers = cv2.getPerspectiveTransform(dst_pts, src_pts_yfix)
    video_window_PTtrans_reverse = cv2.warpPerspective(video_window, perspective_m_resvers, originalsize, flags=cv2.INTER_LINEAR)
    return video_window_PTtrans_reverse


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ역투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ블러필터, 정규화 적용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Blur_and_Normalize(video_copy1_PTtrans):
    video_copy1_PTtrans_blur = cv2.GaussianBlur(video_copy1_PTtrans, ksize=(3, 3), sigmaX=0.0)  # 투상변환 + 블러

    hsv = cv2.cvtColor(video_copy1_PTtrans_blur, cv2.COLOR_BGR2HSV)  # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡhsv
    h, s, v = cv2.split(hsv)
    v2 = np.copy(v)
    s2 = np.copy(s)
    cv2.normalize(v, v2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    cv2.normalize(s, s2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    # hsv2 = cv2.merge([h, s2, v2])
    hsv2 = cv2.merge([h, s, v2])

    video_copy1_PTtrans_blur_normalize4hsv = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)  # 투상변환 + 블러 + hsv정규화

    # yCrCv = cv2.cvtColor(video_copy1_PTtrans_blur, cv2.COLOR_BGR2YCrCb)    #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡyCrCv
    # y, Cr, Cv = cv2.split(yCrCv)
    # y2 = np.copy(y)
    # cv2.normalize(y, y2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    # yCrCv2 = cv2.merge([y2, Cr, Cv])

    # video_copy1_PTtrans_blur_normalize4yCrCv = cv2.cvtColor(yCrCv2, cv2.COLOR_YCrCb2BGR) # 투상변환 + 블러 + yCrCb정규화

    # gray = cv2.cvtColor(video_copy1_PTtrans_blur, cv2.COLOR_BGR2GRAY)
    # gray2 = np.copy(gray)
    # cv2.normalize(gray, gray2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    # video_copy1_PTtrans_blur_normalize4gray = cv2.cvtColor(gray2, cv2.COLOR_GRAY2BGR) #투상변환 + 블러 + gray정규화       #야간 도로에서 테스트 해보자 (그레이스케일 변환 후 정규화 해서 이진화하기)
    return video_copy1_PTtrans_blur_normalize4hsv


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ블러필터, 정규화 적용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV Blue Yellow 추출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_img, upper_img):
    video_adapt = np.copy(video_copy1_PTtrans_blur_normalize4hsv)  # hsv정규화 영상 적용
    ##video_adapt = np.copy(video_copy1_PTtrans_blur_normalize4yCrCv) #yCrCv정규화 영상 적용
    hsv_change = cv2.cvtColor(video_adapt, cv2.COLOR_BGR2HSV)

    img_mask = cv2.inRange(hsv_change, lower_img, upper_img)
    # print(img_mask.dtype,'-',img_mask.shape, '-', np.max(img_mask), '-', np.min(img_mask))
    kernel_img = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    opening_img = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, kernel_img, iterations=2)
    closing_img = cv2.morphologyEx(opening_img, cv2.MORPH_CLOSE, kernel_img, iterations=2)
    img_mask_area_detect_morph_rgb = cv2.cvtColor(closing_img, cv2.COLOR_GRAY2BGR)
    img_detect_morph_brg = cv2.bitwise_and(video_adapt, video_adapt, mask=closing_img)
    return img_detect_morph_brg, img_mask_area_detect_morph_rgb
    # blue_mask = cv2.inRange(hsv_change, lower_blue, upper_blue)
    # blue_mask_area_detect = cv2.bitwise_and(white, white, mask=blue_mask)
    # kernel_blue = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    # opening_blue = cv2.morphologyEx(blue_mask_area_detect, cv2.MORPH_OPEN, kernel_blue, iterations=2)
    # closing_blue = cv2.morphologyEx(opening_blue, cv2.MORPH_CLOSE, kernel_blue, iterations=2)
    # blue_mask_area_detect_morph = np.copy(closing_blue) #-간소화 가능
    # blue_mask_area_detect_morph_hsv = cv2.cvtColor(closing_blue, cv2.COLOR_BGR2HSV)
    # blue_mask_morph = cv2.inRange(blue_mask_area_detect_morph_hsv, (0,0,212), (131,255,255))
    # blue_detect_morph_brg = cv2.bitwise_and(video_adapt, video_adapt, mask=blue_mask_morph)

    # cv2.imshow('yellow_detect_morph_brg', yellow_detect_morph_brg)
    # color_brg = cv2.bitwise_or(yellow_detect_morph_brg, blue_detect_morph_brg)
    # color_brg_hsv = cv2.cvtColor(color_brg, cv2.COLOR_BGR2HSV)


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV Blue Yellow 추출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ이진화ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh):
    # ㅡㅡㅡㅡㅡㅡㅡtest용ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    # video_binary_check = np.copy(gray2) # gray 정규화 영상 적용(gray)
    # ret_b, binary_detect_check = cv2.threshold(video_binary_check, 205, 255, cv2.THRESH_BINARY)
    # kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    # closing_binary_check = cv2.morphologyEx(binary_detect_check, cv2.MORPH_CLOSE, kernel, iterations=2)
    # opening_binary_check = cv2.morphologyEx(closing_binary_check, cv2.MORPH_OPEN, kernel, iterations=2)
    # ㅡㅡㅡㅡㅡㅡㅡtest용ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ #야간 도로에서 테스트 해보자

    # video_binary = np.copy(video_copy1_PTtrans_blur_normalize4yCrCv) # yCrCv 정규화 영상 적용
    video_binary = np.copy(video_copy1_PTtrans_blur_normalize4hsv)  # hsv 정규화 영상 적용
    # video_binary = np.copy(gray2) # gray 정규화 영상 적용(GRAY)
    gray_change = cv2.cvtColor(video_binary, cv2.COLOR_BGR2GRAY)
    ret_b, binary_detect = cv2.threshold(gray_change, white_tresh[0], white_tresh[1], cv2.THRESH_BINARY)
    # ret_b, binary_detect = cv2.threshold(video_binary, 205, 255, cv2.THRESH_BINARY) # gray 정규화 영상 적용할 때
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    closing_binary = cv2.morphologyEx(binary_detect, cv2.MORPH_CLOSE, kernel, iterations=0)
    opening_binary = cv2.morphologyEx(closing_binary, cv2.MORPH_OPEN, kernel, iterations=0)

    # binary_detect_morph_gray = np.copy(opening_binary) #비활성화 해도 됨
    binary_detect_morph_brg = cv2.cvtColor(opening_binary, cv2.COLOR_GRAY2BGR)
    return binary_detect_morph_brg


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ이진화ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ최종 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Final_Bynary_Mix(color_brg, binary_detect_morph_brg, blue_mask_area_detect_morph, yellow_mask_area_detect_morph):
    binary_over = np.zeros_like(binary_detect_morph_brg)
    binary_over[((blue_mask_area_detect_morph == 255) | (yellow_mask_area_detect_morph == 255)) & (binary_detect_morph_brg == 255)] = 255
    binary_seperate = cv2.absdiff(binary_over, binary_detect_morph_brg)
    final_line_brg = cv2.bitwise_or(color_brg, binary_seperate)
    return final_line_brg


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ최종 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV lower upper 값 추적용 히스토그램ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def HSV_Tresh_Trace(input_brg_image):
    input_hsv_image = cv2.cvtColor(input_brg_image, cv2.COLOR_BGR2HSV)

    plt.clf()
    histColor = ('b', 'g', 'r')
    binX = np.arange(32) * 8
    plt.ylim(0, 4000)
    for i in range(3):
        hist = cv2.calcHist(images=[input_hsv_image], channels=[i], mask=None, histSize=[256 / 8], ranges=[0, 256])
        plt.plot(binX, hist, color=histColor[i])
    plt.show()


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV lower upper 값 추적용 히스토그램ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ벼열 변환 및 dtype 변환ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def List_to_Array_and_Type(out_img2, binary_detect_morph_brg):
    out_img2_array = np.array(out_img2)
    # print(out_img2_array.shape,'-',out_img2_array.dtype, '-', np.min(out_img2_array) , '-' , np.max(out_img2_array))
    out_img2_shape = out_img2_array.reshape(binary_detect_morph_brg.shape)
    # print(out_img2_shape.shape, '-', out_img2_shape.dtype, '-', np.min(out_img2_shape), '-', np.max(out_img2_shape))
    out_img2_shape_uint8 = out_img2_shape.astype(np.uint8)
    return out_img2_shape_uint8


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ벼열 변환 및 dtype 변환ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ원본 영상, 윈도우 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize):
    video_window = np.copy(bynary_and_red_blue_and_poly_area)
    video_window_PTtrans_reverse = PTtrans_Reverse(video_window, src_pts_yfix, dst_pts, originalsize)
    # cv2.imshow('video_window_PTtrans_reverse',video_window_PTtrans_reverse)
    video_window_PTtrans_reverse_gray = cv2.cvtColor(video_window_PTtrans_reverse, cv2.COLOR_BGR2GRAY)
    video_window_PTtrans_reverse_vinary = np.zeros_like(video_window_PTtrans_reverse_gray)
    video_window_PTtrans_reverse_vinary[(video_window_PTtrans_reverse_gray >= 1)] = 255
    video_reverse_diff = np.copy(video_copy1)
    video_reverse_diff[(video_window_PTtrans_reverse_vinary == 255)] = 0
    video_original_window_mix = cv2.add(video_reverse_diff, video_window_PTtrans_reverse)
    # cv2.imshow('video_original_window_mix', video_original_window_mix)
    return video_window_PTtrans_reverse, video_original_window_mix


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ원본 영상, 윈도우 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


# ㅡㅡ영상루프 전 setupㅡㅡㅡ
def find_proper_transper_point(proper_size, sight_height, target_distance, road_width, margin, view_scale, focal_length_horizontal, focal_length_vertical, view_length1, view_length2, PIXEL_CONVERSION_RATE):
    angle_of_view_vertical_center = np.arctan(view_length2 / focal_length_vertical)
    angle_of_view_horizontal_center = np.arctan(view_length1 / focal_length_horizontal)
    angle_of_view_horizontal_bottom = np.arctan(np.cos(angle_of_view_vertical_center) * view_length1 / focal_length_horizontal)
    print('angle_of_view_horizontal_center = ', np.rad2deg(angle_of_view_horizontal_center), '  angle_of_view_vertical_center = ', np.rad2deg(angle_of_view_vertical_center), '   angle_of_view_horizontal_bottom = ', np.rad2deg(angle_of_view_horizontal_bottom))

    view_angle = np.arctan(target_distance / sight_height)
    print('view_angle = ', np.rad2deg(view_angle))
    loss_distance = sight_height * np.tan(view_angle - angle_of_view_vertical_center)
    print('loss_distance = ', loss_distance)
    Hypotenuse_distance = sight_height / np.cos(view_angle - angle_of_view_vertical_center)
    straight_distance = sight_height / np.cos(view_angle)

    window_face_bottom_length = Hypotenuse_distance * np.tan(angle_of_view_horizontal_bottom)
    print('window_face_bottom_length*2 = ', window_face_bottom_length * 2)
    ratio_window_target_road = road_width / (straight_distance * np.tan(angle_of_view_horizontal_center))
    ratio_window_target_margin = (road_width + margin) / (2 * straight_distance * np.tan(angle_of_view_horizontal_center))

    # window_face_target_road_width_length = ratio_window_target_road * window_face_bottom_length
    PT_view_width = np.int(round((road_width + margin) * 100 * view_scale))  # 현재 1미터당 100픽셀로 설정되어있다. 비율을 줄여서 연산 속도를 향상할 수도 있으나 그만큼 세밀함이 저하될 수 있다.
    PT_view_height = np.int(round((target_distance - loss_distance) * 100 * view_scale))
    print('PT_view_width = ', PT_view_width, '   PT_view_height = ', PT_view_height)

    road_margin_length_pixel = round(proper_size.shape[1] * ratio_window_target_margin)
    print('center_view_horizontal_center_length = ', (straight_distance * np.tan(angle_of_view_horizontal_center)))
    target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)
    #target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1-50], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1-50], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)
    # target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1 - 70], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 90], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 90], [proper_size.shape[1] - 1, proper_size.shape[0] - 1 - 70]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #video_good.mp4
    # target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 55], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 55], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #video_v1.mp4
    # target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 205], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 205], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #shool_road1.mp4
    dst_bot_pixel = np.int(round(2 * window_face_bottom_length / (road_width + margin) * PT_view_width))
    target_road_margin_dst = np.array([[(PT_view_width - dst_bot_pixel) / 2 - 1, PT_view_height - 1], [0, 0], [PT_view_width - 1, 0], [(PT_view_width + dst_bot_pixel) / 2 - 1, PT_view_height - 1]], dtype=np.float32)

    start_focus_distance = (road_width / 2) / np.tan(angle_of_view_horizontal_center)
    start_zet_distance = (straight_distance - start_focus_distance) * sight_height / target_distance
    start_distance = sight_height * np.tan(view_angle - np.arctan(start_zet_distance / start_focus_distance))
    print('start_distance = ', start_distance)

    vertical_lane = np.array([[0, round((target_distance - start_distance) / PIXEL_CONVERSION_RATE) - 1], [PT_view_width - 1, round((target_distance - start_distance) / PIXEL_CONVERSION_RATE) - 1]], dtype=np.float32)
    straght_lane_left = np.array([[round(PT_view_width / 2 - road_width / 2 / PIXEL_CONVERSION_RATE - 1), PT_view_height - 1], [round(PT_view_width / 2 - road_width / 2 / PIXEL_CONVERSION_RATE - 1), 0]])
    straght_lane_right = np.array([[round(PT_view_width / 2 + road_width / 2 / PIXEL_CONVERSION_RATE - 1), PT_view_height - 1], [round(PT_view_width / 2 + road_width / 2 / PIXEL_CONVERSION_RATE - 1), 0]])
    return target_road_margin_srt, target_road_margin_dst, PT_view_width, PT_view_height, vertical_lane, straght_lane_left, straght_lane_right




# cv brigde sub,pub
class image_converter:

    def __init__(self):

        self.image_pub = rospy.Publisher("lanetraker", Image, queue_size=1000)
        self.prediction_pub = rospy.Publisher("lane", Floats, queue_size=100)

        self.bridge = CvBridge()
        self.lanetracker()

        # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    def get_image(self):
        try:
            data = rospy.wait_for_message("usb_cam/image_raw", Image)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        #        (rows,cols,channels) = cv_image.shape
        #
        #        try:
        #            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,desired_encodeing="passthrough"))
        #        except CvBridgeError as e:
        return cv_image

    def lanetracker(self):
        global flaots, error, total_error_left, total_error_right, total_error_half_sum, error_pixel_percent_left, error_pixel_percent_right, error_pixel_percent_half_sum, white_tresh_lower_left, white_tresh_lower_right, white_tresh_lower_half_sum, lower_offset, upper_offset, lc, rc, deviation, left_fit, right_fit, PIXEL_CONVERSION_RATE, left_trun, right_trun, left_yellow_lane, right_yellow_lane, yellow_s_tresh_lower, following_right_lane, following_left_lane, proper_size, final_error

        rospy.init_node('image_converter', anonymous=True)
        video = self.get_image()



        # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡros 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
        loop = 0
        originalsize = (video.shape[1], video.shape[0])  # h, w, d = video.shape
        print(originalsize)

        bynary_left_pixel_percent = 0
        bynary_right_pixel_percent = 0
        target_left_pixel_percent = 0.01
        bynary_half_sum_pixel_percent = 0
        target_half_sum_pixel_percent = 0
        # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡros 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


        flaots.testset.layout.dim.append(MultiArrayDimension())
        flaots.testset.layout.dim.append(MultiArrayDimension())
        flaots.testset.layout.dim[0].label = "height"
        flaots.testset.layout.dim[1].label = "width"
        flaots.testset.layout.dim[0].size = 2
        flaots.testset.layout.dim[1].size = 23
        flaots.testset.layout.dim[0].stride = 2 * 23
        flaots.testset.layout.dim[1].stride = 23
        flaots.testset.layout.data_offset = 0
        flaots.testset.data = np.zeros((46))

        dstride0 = flaots.testset.layout.dim[0].stride
        dstride1 = flaots.testset.layout.dim[1].stride
        offset = flaots.testset.layout.data_offset

        # ㅡㅡ영상루프 전 setupㅡㅡㅡ

        while True:  # @@@@@@@@@@@@@@@@@@@@영상출력루프 시작@@@@@@@@@@@@@@@@@@@@@@
            video = self.get_image()

# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ1차 로스 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            video_copy1 = np.copy(video)  # 편집용 video copy1
            video_copy2 = np.copy(video)  # 편집용 video copy2 circle 넣을 거
            loop = loop + 1
            print('loop count : ', loop)

            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ참고용 코드(astype, dtype, shape 등)ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            # binary_all_gray = binary_all_gray.astype(bool)
            # print(binary_all_gray.shape,'-',binary_all_gray.dtype, '-', np.min(binary_all_gray) , '-' , np.max(binary_all_gray))
            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ참고용 코드(astype, dtype, shape 등)ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

            video_copy1_PTtrans = PTtrans_and_Circle(video_copy1, video_copy2, src_pts_yfix, dst_pts, viewsize)
            video_copy1_PTtrans_blur_normalize4hsv = Blur_and_Normalize(video_copy1_PTtrans)  # 영상 블러처리 및 정규화
            lower_yellow = (7, yellow_s_tresh_lower, 90)  # HSV 컬러 영억 검출 임계값 영역 (색상, 채도, 조도)
            upper_yellow = (21, 255, 255)
            lower_blue = (80, 50, 180)
            upper_blue = (115, 255, 255)
            yellow_detect_morph_brg, yellow_mask_area_detect_morph = HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_yellow, upper_yellow)  # HSV 컬러 영역 이진화 + 모폴로지 연산
            blue_detect_morph_brg, blue_mask_area_detect_morph = HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_blue, upper_blue)
            color_brg = cv2.bitwise_or(yellow_detect_morph_brg, blue_detect_morph_brg)  # 노랑 파랑 검출된 영역 합치기
            color_bynary = cv2.bitwise_or(yellow_mask_area_detect_morph, blue_mask_area_detect_morph)
            white_tresh_left = [white_tresh_lower_left, 255]  # 흰색 차선 검출 임계값 영역(검출되는 영역이 너무 적으면 최솟값을 더 낮추면 됨)
            # white_tresh_left = [135, 255]  # 흰색 차선 검출 임계값 영역(검출되는 영역이 너무 적으면 최솟값을 더 낮추면 됨)
            white_tresh_right = [white_tresh_lower_right, 255]
            # white_tresh_right = [135, 255]
            # white_tresh_full = [white_tresh_lower_full, 255]
            binary_detect_morph_brg_left = White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh_left)
            binary_detect_morph_brg_right = White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh_right)
            # binary_detect_morph_brg_full = White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh_full)
            binary_detect_morph_brg_half_sum = np.zeros_like(video_copy1_PTtrans_blur_normalize4hsv)
            binary_detect_morph_brg_half_sum[:, : binary_detect_morph_brg_half_sum.shape[1] // 2, :] = binary_detect_morph_brg_left[:, : binary_detect_morph_brg_left.shape[1] // 2, :]
            binary_detect_morph_brg_half_sum[:, binary_detect_morph_brg_half_sum.shape[1] // 2:, :] = binary_detect_morph_brg_right[:, binary_detect_morph_brg_right.shape[1] // 2:, :]

            # print('binary_detect_morph_brg_half_sum.shape = ',binary_detect_morph_brg_half_sum.shape)
            final_line_color = Final_Bynary_Mix(color_brg, binary_detect_morph_brg_half_sum, blue_mask_area_detect_morph, yellow_mask_area_detect_morph)  # 컬러랑 흰색 검출 영역 합치기 #추세선 탐지에 사용한는 영상의 이진화를 하기 전단계
            # final_line_bynary_left = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_left)
            # final_line_bynary_right = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_right)
            final_line_bynary_half_sum = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_half_sum)  # 추세선 탐지에 사용되는 이진화 영상
            # final_line_bynary_full = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_full)
            # print('sum bynary pixel = ', np.sum(final_line_bynary/255)/3/(final_line_bynary.shape[0]*final_line_bynary.shape[0]),'--',np.sum(final_line_bynary/255)/3)
            if error != 0:
                bynary_half_sum_pixel_percent = np.sum(binary_detect_morph_brg_half_sum / 255) / 3 / (binary_detect_morph_brg_half_sum.shape[0] * binary_detect_morph_brg_half_sum.shape[1])
                target_half_sum_pixel_percent = target_left_pixel_percent * 3
                # white_tresh_lower_half_sum = white_tresh_lower_half * ((bynary_half_sum_pixel_percent + 0.5) / (target_half_sum_pixel_percent + 0.5))
                prev_error_pixel_percent_half_sum = error_pixel_percent_half_sum
                error_pixel_percent_half_sum = bynary_half_sum_pixel_percent - target_half_sum_pixel_percent
                total_error_half_sum = total_error_half_sum + error_pixel_percent_half_sum
                tresh_factor_half_sum = kp * error_pixel_percent_half_sum + ki * total_error_half_sum + kd * (error_pixel_percent_half_sum - prev_error_pixel_percent_half_sum)
                white_tresh_lower_half_sum = white_tresh_lower_half_sum * pow(2, tresh_factor_half_sum)
                if white_tresh_lower_half_sum >= 255: white_tresh_lower_half_sum = 255
                if white_tresh_lower_half_sum <= 0: white_tresh_lower_half_sum = 0

                white_tresh_lower_left = white_tresh_lower_half_sum
                white_tresh_lower_right = white_tresh_lower_half_sum

                print('white_tresh_lower_half_sum = ', white_tresh_lower_half_sum, '   bynary_half_sum_pixel_percent = ', bynary_half_sum_pixel_percent)

            # HSV_Tresh_Trace(input_brg_image) #노랑, 파랑 임계값 추적용 코드

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ차선 좌표 검출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

            # binary_all_left = np.zeros_like(binary_detect_morph_brg_left)
            # binary_all_right = np.zeros_like(binary_detect_morph_brg_right)
            # binary_all_half_sum = np.zeros_like(binary_detect_morph_brg_half_sum)
            # binary_all_left[(color_bynary == 255) | (binary_detect_morph_brg_left == 255)] = 1
            # binary_all_right[(color_bynary == 255) | (binary_detect_morph_brg_right == 255)] = 1
            # binary_all_half_sum[(color_bynary == 255) | (binary_detect_morph_brg_half_sum == 255)] = 1
            # binary_all_left_gray = cv2.cvtColor(binary_all_left, cv2.COLOR_BGR2GRAY)  # 1채널의 0, 1 boolean data_type
            # binary_all_right_gray = cv2.cvtColor(binary_all_right, cv2.COLOR_BGR2GRAY)
            # binary_all_half_sum_gray = cv2.cvtColor(binary_all_half_sum, cv2.COLOR_BGR2GRAY)

            binary_all_left_gray, _, _ = cv2.split(np.uint8(cv2.bitwise_or(color_bynary, binary_detect_morph_brg_left) / 255))
            binary_all_right_gray, _, _ = cv2.split(np.uint8(cv2.bitwise_or(color_bynary, binary_detect_morph_brg_right) / 255))
            binary_all_half_sum_gray, _, _ = cv2.split(np.uint8(cv2.bitwise_or(color_bynary, binary_detect_morph_brg_half_sum) / 255))

            # plt.clf() #plot초기화
            add_debug_image1 = True
            add_debug_image2 = True
            add_debug_image2_a = False
            add_debug_image2_b = False
            add_debug_image3 = True

            is_debug = True
            debug_images_all = []
            # plt.clf()
            # plot_peak_hist(binary_all_gray, is_plot=True)
            # plt.show()

            if len(left_fit) == 0:
                leftx_base, rightx_base, hist, error, following_left_lane, following_right_lane = find_left_right_via_histogram(binary_all_half_sum_gray, add_debug_image=False)  # 영상에서의 왼쪽 오른쪽 차선의 초기 위치를 뽑아내는 함수

                if error == 0:
                    left_fit, right_fit, debug_images1 = find_ploy_fit_window_search(binary_all_half_sum_gray, leftx_base, rightx_base, following_left_lane, following_right_lane, road_width, PIXEL_CONVERSION_RATE, nwindows=13, margin=round(0.65 / PIXEL_CONVERSION_RATE), is_plot=False,
                                                                                     add_debug_image=add_debug_image1)  # 사각 영역을 만들어 차선을 탐지하고 추세선을 만드는 함수
                    if add_debug_image1:
                        add_debug_image1_a = 1
                    if is_debug: print("stage 4 a.. done_1")
            if error == 0:
                left_fit, right_fit, lc, rc, deviation, debug_images2, error, following_left_lane, following_right_lane = next_frame_find_poly_already_fitted(binary_all_left_gray, binary_all_right_gray, left_fit, right_fit, lower_offset, upper_offset, following_right_lane, following_left_lane,
                                                                                                                                                              road_width, PIXEL_CONVERSION_RATE, margin=round(0.3 / PIXEL_CONVERSION_RATE), is_plot=False,
                                                                                                                                                              add_debug_image=add_debug_image2)
                if error == 0:
                    if add_debug_image2:
                        add_debug_image2_a = 1
                    if is_debug: print("stage 4 b.. done_2")
                    # window_img_left = debug_images2[1]
                    # window_img_left_4upside = debug_images2[3]
                    window_img_left_gray = cv2.cvtColor(debug_images2[1], cv2.COLOR_BGR2GRAY)
                    window_img_left_gray_4upside = cv2.cvtColor(debug_images2[3], cv2.COLOR_BGR2GRAY)
                    # window_img_right = debug_images2[2]
                    # window_img_right_4upside = debug_images2[4]
                    window_img_right_gray = cv2.cvtColor(debug_images2[2], cv2.COLOR_BGR2GRAY)
                    window_img_right_gray_4upside = cv2.cvtColor(debug_images2[4], cv2.COLOR_BGR2GRAY)

                    bynary_window_image_left_area = cv2.bitwise_and(binary_detect_morph_brg_left, binary_detect_morph_brg_left, mask=window_img_left_gray)
                    # bynary_window_image_left_area = np.zeros_like(binary_detect_morph_brg_left)
                    # bynary_window_image_left_area[(binary_detect_morph_brg_left == 255) & (window_img_left)] = 255

                    bynary_window_image_right_area_by_left = cv2.bitwise_and(binary_detect_morph_brg_left, binary_detect_morph_brg_left, mask=window_img_right_gray)
                    # bynary_window_image_right_area_by_left = np.zeros_like(binary_detect_morph_brg_left)
                    # bynary_window_image_right_area_by_left[(binary_detect_morph_brg_left == 255) &  (window_img_right)] = 255

                    bynary_window_image_right_area_by_left_4upside = cv2.bitwise_or(binary_detect_morph_brg_left, color_bynary, mask=window_img_right_gray_4upside)
                    # bynary_window_image_right_area_by_left_4upside = np.zeros_like(binary_detect_morph_brg_left[:viewsize[1]//2 - 1, :, :])
                    # bynary_window_image_right_area_by_left_4upside[(binary_detect_morph_brg_left[:viewsize[1]//2 - 1, :, :] == 255 ) | (color_bynary[:viewsize[1]//2 - 1, :, :] == 255) & (window_img_right_4upside[:viewsize[1]//2 - 1, :, :] == 255)] = 255
                    cv2.imshow('test1', bynary_window_image_right_area_by_left_4upside[:viewsize[1] // 2 - 1, :, :])
                    bynary_window_image_right_area = cv2.bitwise_and(binary_detect_morph_brg_right, binary_detect_morph_brg_right, mask=window_img_right_gray)
                    # bynary_window_image_right_area = np.zeros_like(binary_detect_morph_brg_right)
                    # bynary_window_image_right_area[(binary_detect_morph_brg_right == 255) & (window_img_right == 255)] = 255

                    bynary_window_image_left_area_by_right = cv2.bitwise_and(binary_detect_morph_brg_right, binary_detect_morph_brg_right, mask=window_img_left_gray)
                    # bynary_window_image_left_area_by_right = np.zeros_like(binary_detect_morph_brg_right)
                    # bynary_window_image_left_area_by_right[(binary_detect_morph_brg_right == 255) & (window_img_left == 255)] = 255

                    bynary_window_image_left_area_by_right_4upside = cv2.bitwise_or(binary_detect_morph_brg_right, color_bynary, mask=window_img_left_gray_4upside)
                    # bynary_window_image_left_area_by_right_4upside = np.zeros_like(binary_detect_morph_brg_right[:binary_detect_morph_brg_right.shape[0]//2 - 1, :, :])
                    # bynary_window_image_left_area_by_right_4upside[((binary_detect_morph_brg_right[:viewsize[1]//2 - 1, :, :] == 255)|(color_bynary[:viewsize[1]//2 -1,:,:]== 255)) & (window_img_left_4upside[:viewsize[1]//2 - 1, :, :] == 255)] = 255
                    cv2.imshow('test2', bynary_window_image_left_area_by_right_4upside[:viewsize[1] // 2 - 1, :, :])
                    # bynary_window_image_area = cv2.bitwise_or(bynary_window_image_left_area, bynary_window_image_right_area) #full 전용
                    bynary_left_pixel_percent = np.sum(bynary_window_image_left_area / 255) / 3 / (bynary_window_image_left_area.shape[0] * bynary_window_image_left_area.shape[1])
                    bynary_right_pixel_percent = np.sum(bynary_window_image_right_area / 255) / 3 / (bynary_window_image_right_area.shape[0] * bynary_window_image_right_area.shape[1])
                    # bynary_left_upside_pixel = np.sum(bynary_window_image_left_area[:(bynary_window_image_left_area.shape[0]/2-1),:,:])/ 3 / (bynary_window_image_left_area.shape[0] * bynary_window_image_left_area.shape[1])
                    print('white_tresh_lower_left = ', white_tresh_lower_left, '   bynary_left_pixel_percent = ', bynary_left_pixel_percent, 'white_tresh_lower_right = ', white_tresh_lower_right, '   bynary_right_pixel_percent = ', bynary_right_pixel_percent)
                    target_left_pixel_percent = 0.01
                    target_right_pixel_percent = 0.01

                    bynary_right_by_left_pixel_percent = np.sum(bynary_window_image_right_area_by_left / 255) / 3 / (bynary_window_image_right_area_by_left.shape[0] * bynary_window_image_right_area_by_left.shape[1])
                    bynary_left_by_right_pixel_percent = np.sum(bynary_window_image_left_area_by_right / 255) / 3 / (bynary_window_image_left_area_by_right.shape[0] * bynary_window_image_left_area_by_right.shape[1])
                    diff_by_left_stand = bynary_right_by_left_pixel_percent - bynary_left_pixel_percent
                    diff_by_right_stand = bynary_left_by_right_pixel_percent - bynary_right_pixel_percent
                    if (diff_by_left_stand >= target_left_pixel_percent / 4 and diff_by_right_stand < target_right_pixel_percent / 4):
                        left_trun = True
                        right_trun = False
                        print('1_left_trun, right_trun = ', left_trun, right_trun)
                    else:
                        left_trun = False
                        print('2_left_trun, right_trun = ', left_trun, right_trun)
                    if (diff_by_right_stand >= target_right_pixel_percent / 4 and diff_by_left_stand < target_left_pixel_percent / 4):
                        left_trun = False
                        right_trun = True
                        print('3_left_trun, right_trun = ', left_trun, right_trun)
                    else:
                        right_trun = False
                        print('4_left_trun, right_trun = ', left_trun, right_trun)
                    print('bynary_right_by_left_pixel_percent = ', bynary_right_by_left_pixel_percent, 'diff_by_left_stand = ', diff_by_left_stand, '  left_trun = ', left_trun)
                    print('bynary_left_by_right_pixel_percent = ', bynary_left_by_right_pixel_percent, 'diff_by_right_stand = ', diff_by_right_stand, '  right_trun = ', right_trun)

                    # white_tresh_lower_left = white_tresh_lower_left * ((bynary_left_pixel_percent + 0.15) / (target_left_pixel_percent + 0.15))
                    prev_error_pixel_percent_left = error_pixel_percent_left
                    error_pixel_percent_left = bynary_left_pixel_percent - target_left_pixel_percent
                    total_error_left = total_error_left + error_pixel_percent_left
                    tresh_factor_left = kp * error_pixel_percent_left + ki * total_error_left + kd * (error_pixel_percent_left - prev_error_pixel_percent_left)
                    white_tresh_lower_left = white_tresh_lower_left * pow(2, tresh_factor_left)
                    if white_tresh_lower_left >= 255: white_tresh_lower_left = 255
                    if white_tresh_lower_left <= 0: white_tresh_lower_left = 0

                    # white_tresh_lower_right = white_tresh_lower_right * ((bynary_right_pixel_percent + 0.15) / (target_right_pixel_percent + 0.15))
                    prev_error_pixel_percent_right = error_pixel_percent_right
                    error_pixel_percent_right = bynary_right_pixel_percent - target_right_pixel_percent
                    total_error_right = total_error_right + error_pixel_percent_right
                    tresh_factor_right = kp * error_pixel_percent_right + ki * total_error_right + kd * (error_pixel_percent_right - prev_error_pixel_percent_right)
                    white_tresh_lower_right = white_tresh_lower_right * pow(2, tresh_factor_right)
                    if white_tresh_lower_right >= 255: white_tresh_lower_right = 255
                    if white_tresh_lower_right <= 0: white_tresh_lower_right = 0

                    white_tresh_lower_half_sum = (white_tresh_lower_left + white_tresh_lower_right) / 2

                    print('error_pixel_percent_left =', error_pixel_percent_left, 'tresh_factor_left = ', tresh_factor_left, 'pow(2, tresh_factor_left) = ', pow(2, tresh_factor_left))
                    print('error_pixel_percent_right =', error_pixel_percent_right, 'tresh_factor_right = ', tresh_factor_right, 'pow(2, tresh_factor_right) = ', pow(2, tresh_factor_right))
                    add_debug_image2_b = True

                    yellow_bynary_in_left_lane_area = cv2.bitwise_and(yellow_mask_area_detect_morph, yellow_mask_area_detect_morph, mask=window_img_left_gray)
                    yellow_bynary_in_right_lane_area = cv2.bitwise_and(yellow_mask_area_detect_morph, yellow_mask_area_detect_morph, mask=window_img_right_gray)
                    yellow_bynary_in_left_lane_area_pixel_percent = np.sum(yellow_bynary_in_left_lane_area / 255) / 3 / (yellow_bynary_in_left_lane_area.shape[0] * yellow_bynary_in_left_lane_area.shape[0])
                    yellow_bynary_in_right_lane_area_pixel_percent = np.sum(yellow_bynary_in_right_lane_area / 255) / 3 / (yellow_bynary_in_right_lane_area.shape[0] * yellow_bynary_in_right_lane_area.shape[0])
                    if (yellow_bynary_in_left_lane_area_pixel_percent >= target_left_pixel_percent / 4):
                        left_yellow_lane = True
                    else:
                        left_yellow_lane = False
                    if (yellow_bynary_in_right_lane_area_pixel_percent >= target_right_pixel_percent / 4):
                        right_yellow_lane = True
                    else:
                        right_yellow_lane = False
                    print('yellow_bynary_in_left_lane_area_pixel_percent = ', yellow_bynary_in_left_lane_area_pixel_percent, 'yellow_bynary_in_right_lane_area_pixel_percent = ', yellow_bynary_in_right_lane_area_pixel_percent)

                    bynary_left_upside_pixel_percent = np.sum(bynary_window_image_left_area_by_right_4upside[:bynary_window_image_left_area.shape[0] // 2 - 1, :, :] / 255) / 3 / (bynary_window_image_left_area.shape[0] / 2 * bynary_window_image_left_area.shape[1])
                    bynary_right_upside_pixel_percent = np.sum(bynary_window_image_right_area_by_left_4upside[:bynary_window_image_left_area.shape[0] // 2 - 1, :, :] / 255) / 3 / (bynary_window_image_left_area.shape[0] / 2 * bynary_window_image_left_area.shape[1])

                    if bynary_left_upside_pixel_percent < target_left_pixel_percent / 6 and bynary_right_upside_pixel_percent > target_right_pixel_percent / 2:
                        following_right_lane = True
                        following_left_lane = False
                        print('following_right_lane = ', following_right_lane, '****', '  bynary_left_upside_pixel_percent = ', bynary_left_upside_pixel_percent)
                    elif bynary_right_upside_pixel_percent < target_right_pixel_percent / 6 and bynary_left_upside_pixel_percent > target_left_pixel_percent / 2:
                        following_right_lane = False
                        following_left_lane = True
                        print('following_left_lane = ', following_left_lane, '****', '  bynary_right_upside_pixel_percent= ', bynary_right_upside_pixel_percent)
                    else:
                        following_right_lane = False
                        following_left_lane = False

#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ2차 로스 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            if error == 0:
                undistored_image = np.copy(video_copy2)
                flaots.left_curv = lc
                flaots.right_curv = rc
                flaots.deviation = deviation
                flaots.mid_point_vector = np.add(left_fit, right_fit) / 2
                flaots.error=error
                flaots.left_trun=left_trun
                flaots.right_trun=right_trun

                for i in range(23):
                    point_pixter = viewsize[1] / 23
                    flaots.testset.data[i] = point_pixter * PIXEL_CONVERSION_RATE * i
                    m = 2 * flaots.mid_point_vector[1] * (point_pixter * i)# + flaots.mid_point_vector[1]
                    flaots.testset.data[23 + i] = m
                self.prediction_pub.publish(flaots)

                # debug_images3[0] : 투상변환에서 차선 사이 영역 확인, debug_images3[1] : [0] 역투상 , debug_images3[2] : [1]를 원본 영상 위에 겹쳐서 보여줌 = final_image
                newwarp, final_image, debug_images3 = plot_lanes_unwrap(binary_all_left_gray, left_fit, right_fit, left_trun, right_trun, left_yellow_lane, right_yellow_lane, undistored_image, M_INV, is_plot=False, add_debug_image=add_debug_image3)

            if add_debug_image3:
                if error == 0:
                    composed_image = np.copy(final_image)


                else:
                    composed_image = np.copy(video_copy2)
                # debug_images_all2 = list(itertools.chain.from_iterable(debug_images_all))
                # composed_image = compose_debug_images(debug_images_all2)
                left_pix_per_text = "lef_pix_per : " + "{:0.4f}".format(bynary_left_pixel_percent) + " in per"
                right_pix_per_text = "Right_pix_per : " + "{:0.4f}".format(bynary_right_pixel_percent) + " in per"
                target_pixel_percent = "target_pix_per : " + "{:0.4f}".format(target_left_pixel_percent) + " in per"
                deviation_text = "Deviation : " + "{:0.2f}".format(deviation) + " in m"
                error_text = "Error_type : " + "{:}".format(error) + " in type"
                # print(type(deviation),type(white_tresh_left))
                white_tresh_lower_left_text = "tresh_left : " + "{:0.2f}".format(white_tresh_lower_left) + " "
                white_tresh_lower_right_text = "tresh_right : " + "{:0.2f}".format(white_tresh_lower_right) + " "
                following_lane_text = "fallowing : " + " fair"
                flaots.fair_state=1

                if following_left_lane:
                    following_lane_text = "fallowing : " + " left"
                    flaots.fair_state=0

                if following_right_lane:
                    following_lane_text = "fallowing : " + " right"
                    flaots.fair_state=2

                loop_text = "loop : " + "{:}".format(loop) + " "
                half_sum_pixel_per_text = "half_sum_pix_per : " + "{:0.4f}".format(bynary_half_sum_pixel_percent) + " in per"
                target_half_sum_pixel_percent_text = "half_sum_target : " + "{:0.4f}".format(target_half_sum_pixel_percent) + " in per"

                fontScale = 1
                thickness = 2
                fontFace = cv2.FONT_ITALIC

                cv2.putText(composed_image, left_pix_per_text, (10, 50), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, right_pix_per_text, (10, 90), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, target_pixel_percent, (10, 130), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                if error == 0:
                    cv2.putText(composed_image, error_text, (10, 170), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                else:
                    cv2.putText(composed_image, error_text, (10, 170), fontFace, fontScale, (0, 0, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, deviation_text, (10, 210), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, loop_text, (10, 250), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, half_sum_pixel_per_text, (580, 50), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, target_half_sum_pixel_percent_text, (580, 90), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, white_tresh_lower_left_text, (580, 130), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, white_tresh_lower_right_text, (580, 170), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)
                cv2.putText(composed_image, following_lane_text, (580, 210), fontFace, fontScale, (0, 255, 255), thickness, lineType=cv2.LINE_AA)


            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ차선 좌표 검출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡcv2.imshow 이미지 쇼 컨트롤센터 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            imshow_scale = 0.5 / view_scale
            # cv2.imshow('video_copy1_PTtrans',video_copy1_PTtrans) #투상변환 영상
            # video_copy1_PTtrans_resized = cv2.resize(video_copy1_PTtrans, dsize=(round(video_copy1_PTtrans.shape[1] * imshow_scale), round(video_copy1_PTtrans.shape[0] * imshow_scale)))
            # cv2.imshow('video_copy1_PTtrans_resized', video_copy1_PTtrans_resized)

            # gray_image = cv2.cvtColor(video_copy1_PTtrans_blur_normalize4hsv, cv2.COLOR_BGR2GRAY)
            # gray_image_resized = cv2.resize(gray_image, dsize=(round(video_copy1_PTtrans.shape[1] * imshow_scale), round(video_copy1_PTtrans.shape[0] * imshow_scale)))
            # cv2.imshow('gray_image_resized',gray_image_resized)

            # cv2.imshow('left', binary_detect_morph_brg_left) #왼쪽 차선 기준 임계값 이진화
            # binary_detect_morph_brg_left_resized = cv2.resize(binary_detect_morph_brg_left, dsize=(round(binary_detect_morph_brg_left.shape[1] * imshow_scale), round(binary_detect_morph_brg_left.shape[0] * imshow_scale)))
            # cv2.imshow('left', binary_detect_morph_brg_left_resized)

            # cv2.imshow('right', binary_detect_morph_brg_right) #오른쪽 차선 기준 임계값 이진화
            # binary_detect_morph_brg_right_resized = cv2.resize(binary_detect_morph_brg_right, dsize=(round(binary_detect_morph_brg_right.shape[1] * imshow_scale), round(binary_detect_morph_brg_right.shape[0] * imshow_scale)))
            # cv2.imshow('right', binary_detect_morph_brg_right_resized)

            # cv2.imshow('binary_detect_morph_brg_half_sum', binary_detect_morph_brg_half_sum) # 각각 기준 임계값 이진화 영상을 반반 합친 영상
            binary_detect_morph_brg_half_sum_resized = cv2.resize(binary_detect_morph_brg_half_sum, dsize=(int(round(binary_detect_morph_brg_half_sum.shape[1] * imshow_scale)), int(round(binary_detect_morph_brg_half_sum.shape[0] * imshow_scale))))
            cv2.imshow('half_sum_resized', binary_detect_morph_brg_half_sum_resized)  # 필

            # cv2.imshow('final_line_color', final_line_color) #투상변환 + 블러, 정규화 + 최종 추출 영상 컬러 버전
            final_line_color_resized = cv2.resize(final_line_color, dsize=(int(round(final_line_color.shape[1] * imshow_scale)), int(round(final_line_color.shape[0] * imshow_scale))))
            cv2.imshow('color_resized', final_line_color_resized)  # 필

            # cv2.imshow('final_line_bynary_full', final_line_bynary_full) #두 차선을 구분하지 않고 임계값을 계산한 영상 (사용하지않음)
            # cv2.imshow('final_line_bynary_half_sum', final_line_bynary_half_sum)  # 투상변환 + 블러, 정규화 + 최종 추출 영상 이진화 버전 # 최종적으로 추세선 감지 함수에 사용되는 영상 # binary_detect_morph_brg_half_sum 이것과 컬러 영역 이진화 영상을 합친 것
            # final_line_bynary_half_sum_resized = cv2.resize(final_line_bynary_half_sum, dsize=(round(final_line_bynary_half_sum.shape[1] * imshow_scale), round(final_line_bynary_half_sum.shape[0] * imshow_scale))) # 최종적으로 추세선 감지 함수에 사용되는 영상
            # cv2.imshow('final_line_bynary_half_sum_resized', final_line_bynary_half_sum_resized)

            # video_copy1_PTtrans_resized = cv2.resize(video_copy1_PTtrans, dsize=(round(video_copy1_PTtrans.shape[1] * imshow_scale ), round(video_copy1_PTtrans.shape[0] * imshow_scale )))
            # cv2.imshow('video_copy1_PTtrans_resized', video_copy1_PTtrans_resized)

            video_copy1_PTtrans_copy1_to_draw_expected_lane = np.copy(video_copy1_PTtrans)
            cv2.line(video_copy1_PTtrans_copy1_to_draw_expected_lane, (int(straght_lane_left[0][0]), int(straght_lane_left[0][1])), (int(straght_lane_left[1][0]), int(straght_lane_left[1][1])), [0, 255, 0], 2)
            cv2.line(video_copy1_PTtrans_copy1_to_draw_expected_lane, (int(straght_lane_right[0][0]), int(straght_lane_right[0][1])), (int(straght_lane_right[1][0]), int(straght_lane_right[1][1])), [0, 255, 0], 2)
            cv2.line(video_copy1_PTtrans_copy1_to_draw_expected_lane, (int(vertical_lane[0][0]), int(vertical_lane[0][1])), (int(vertical_lane[1][0]), int(vertical_lane[1][1])), [0, 255, 0], 2)
            # cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane', video_copy1_PTtrans_copy1_to_draw_expected_lane)
            video_copy1_PTtrans_copy1_to_draw_expected_lane_resized = cv2.resize(video_copy1_PTtrans_copy1_to_draw_expected_lane, dsize=(int(round(final_line_bynary_half_sum.shape[1] * imshow_scale)), int(round(final_line_bynary_half_sum.shape[0] * imshow_scale))))
            cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane_resized', video_copy1_PTtrans_copy1_to_draw_expected_lane_resized)  # 필
            draw_expected_line_reverse, draw_expected_line_reverse_window_mix = Show_Original_Window_Mix(video_copy1_PTtrans_copy1_to_draw_expected_lane, video_copy1, src_pts_yfix, dst_pts, originalsize)
            draw_expected_line_reverse_window_mix_resized = cv2.resize(draw_expected_line_reverse_window_mix, dsize=(int(round(draw_expected_line_reverse_window_mix.shape[1] * imshow_scale)), int(round(draw_expected_line_reverse_window_mix.shape[0] * imshow_scale))))
            cv2.imshow('draw_expected_line_reverse_window_mix_resized', draw_expected_line_reverse_window_mix_resized)  # 필

            # video_copy1_PTtrans_copy1_to_draw_expted_lane_resized = cv2.resize(video_copy1_PTtrans_copy1_to_draw_expted_lane, dsize=(round(video_copy1_PTtrans_copy1_to_draw_expted_lane.shape[1] * imshow_scale ), round(video_copy1_PTtrans_copy1_to_draw_expted_lane.shape[0] * imshow_scale )))
            # cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane_resized', video_copy1_PTtrans_copy1_to_draw_expted_lane_resized)

            # if error == 0:
            # if add_debug_image1_a == 1:
            # sliding_window = np.copy(debug_images1[0])  # 슬라이딩 윈도우  # = debug_images[0]  # 여기서 debug_images 는 이진화영역과 좌표 추출 windows 영역과 그 안에서 검출된 픽셀들 그리고 추세선을 시각화 한 것이다.
            # cv2.imshow('sliding_window', sliding_window) # 바이너리 및 차선 영역

            if add_debug_image2_a == 1:
                bynary_and_red_blue_and_poly_area = np.copy(debug_images2[0])  # = debug_images[1] 추세선 # 0718-[0](bynary_and_red_blue)를 없애서 [0]이 추세선이됨 # 여기서 debug_images[0]은 이진화 영역과 좌표 추출 영역 안에 포함된 픽셀들을 파랑,빨강,흰색으로 보여주는 이미지이구 [1] 추세선을 시각화 한 것이다
                # cv2.imshow('bynary_and_red_blue_and_poly_area', bynary_and_red_blue_and_poly_area) # 바이너리 영역에서 추세선과 그 영역 표시 이미지
                bynary_and_red_blue_and_poly_area_resized = cv2.resize(bynary_and_red_blue_and_poly_area, dsize=(int(round(bynary_and_red_blue_and_poly_area.shape[1] * imshow_scale)), int(round(bynary_and_red_blue_and_poly_area.shape[0]) * imshow_scale)))
                cv2.imshow('bynary_and_red_blue_and_poly_area_resized', bynary_and_red_blue_and_poly_area_resized)  # 필
            # video_window_PTtrans_reverse, video_original_window_mix = Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize)

            # video_window_PTtrans_reverse, video_original_window_mix = Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize)
            # cv2.imshow('video_window_PTtrans_reverse', video_window_PTtrans_reverse)  # 차선 좌표 추세선 함수와 검출창이 포함된 이미지를 역투상변환 한 이미지
            # cv2.imshow('video_original_window_mix', video_original_window_mix)  # 차선 좌표 추세선 함수와 검출창이 포함된 이미지를 역투상변환 한 이미지와 원본 영상 모두 보이는 이미지
            # if add_debug_image2_b == True:
            # cv2.imshow('bynary_window_image_area', bynary_window_image_area)
            # bynary_window_image_area_resized = cv2.resize(bynary_window_image_area, dsize=(round(bynary_window_image_area.shape[1] * imshow_scale), round(bynary_window_image_area.shape[0] * imshow_scale)))
            # cv2.imshow('bynary_window_image_area_resized', bynary_window_image_area_resized)

            # bynary_window_image_left_area_resized = cv2.resize(bynary_window_image_left_area, dsize=(round(bynary_window_image_left_area.shape[1] * imshow_scale ), round(bynary_window_image_left_area.shape[0] * imshow_scale )))
            # bynary_window_image_right_area_resized = cv2.resize(bynary_window_image_right_area, dsize=(round(bynary_window_image_right_area.shape[1] * imshow_scale ), round(bynary_window_image_right_area.shape[0] * imshow_scale )))
            # cv2.imshow('bynary_window_image_left_area_resized', bynary_window_image_left_area_resized)  # 차선 검출 영역 안의 픽셀들만 보여주는 이미지
            # cv2.imshow('bynary_window_image_right_area_resized', bynary_window_image_right_area_resized)

            if add_debug_image3:
                #cv2.imshow('composed_image', composed_image)  # 필 # 차선을 초록색으로 덮은 이미지에 곡률과 편차를 화면에 택스트를 표시한 이미지
                composed_image_resized = cv2.resize(composed_image, dsize=(int(round(composed_image.shape[1] * imshow_scale)), int(round(composed_image.shape[0] * imshow_scale))))
                cv2.imshow('composed_image_resized', composed_image_resized)  # 필
                #out.write(composed_image)
            # out.write(video_copy1_PTtrans)

            if error == 0:
                # cv2.imshow('final_image',final_image) # 차선 영역을 초록색으로 색칠한 이미지
                warped_new = np.copy(debug_images3[0]) # 투상변환 화면에서의 초록색으로 덮은 이미지와 점선 영역
                warped_new_resized = cv2.resize(warped_new, dsize=(int(round(warped_new.shape[1] * imshow_scale)), int(round(warped_new.shape[0] * imshow_scale))))
                #cv2.imshow('warped_new', warped_new)
                cv2.imshow('warped_new_resized',warped_new_resized)

            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡcv2.imshow 이미지 쇼 컨트롤센터 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ2차 로스 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(composed_image, encoding="bgr8"))


def main():  # 메인문

    try:
        image_converter()
    except rospy.ROSInterruptException:
        #       cam.release()
        #out.release()
        cv2.destroyAllWindows()


#-----------------------ros 추가용 코드------------------------------
selected_pts = []  # tf function location array

view_scale = 1
sight_height = 1  # 카메라 높이
target_distance = 10.6 #11.25  # 목표 거리 (현재 목표 10m전방)
road_width = 3.5  # 차선 폭 (현재 3.5미터 실측 후 조정)
margin = 6  # 추가로 투상변환할 폭
focal_length_horizontal = 0.35  # sample_video(f_l_h = 0.4, f_l_v = 0.35)  C525(f_l_h = 0.515, f_l_v = 0.31(by0.1)) C930e(f_l_h = 0.35, f_l_v = 0.648)
focal_length_vertical = 0.648
view_length1 = 0.3
view_length2 = 0.3
PIXEL_CONVERSION_RATE = 0.01 / view_scale  # 100픽셀당 1미터
ym_per_pix = 0.01 / view_scale  # meters per pixel in y dimension
xm_per_pix = 0.01 / view_scale  # meters per pixel in x dimension
lower_offset = 2.5 / PIXEL_CONVERSION_RATE  # 최소 차선 감지 폭
upper_offset =4.5 / PIXEL_CONVERSION_RATE  # 최대 차선 감지 폭
kp = 2
ki = 0.1
kd = 0.7

proper_size = np.zeros([720, 1280, 3], dtype=np.uint8)
#proper_size = np.zeros([480, 640, 3], dtype=np.uint8)
#proper_size = np.copy(video)
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ좌표 자동 지정ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
src_pts_yfix, dst_pts, PT_view_width, PT_view_height, vertical_lane, straght_lane_left, straght_lane_right = find_proper_transper_point(proper_size, sight_height, target_distance, road_width, margin, view_scale, focal_length_horizontal, focal_length_vertical, view_length1, view_length2,
                                                                                                                                        PIXEL_CONVERSION_RATE)
M_INV = cv2.getPerspectiveTransform(dst_pts, src_pts_yfix)
viewsize = [PT_view_width, PT_view_height]
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ좌표 자동 지정ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ



#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ좌표 수동 지정ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
#src_pts = select_points(video_copy1, 4) #마우스로 클릭해서 좌표 뽑아내는 함수
#src_pts_yfix = np.array([src_pts[0],src_pts[1],[src_pts[2,0],src_pts[1,1]],[src_pts[3,0],src_pts[0,1]]], dtype=np.float32) #3, 4번 점의 y좌표를 각각 1, 2번 y좌표값과 같게 만듬
#print(src_pts_yfix)
#viewsize = [1000, 600] #투상변환 출력할 화면 사이즈 설정 [너비, 높이]
#weight = 300
#dst_pts = np.array([[0, viewsize[1]-1], [0, 0], [viewsize[0]-1, 0], [viewsize[0]-1,[viewsize[1]-1]]], dtype=np.float32) #변환한 영상을 출력할 화면(딱 좌표 찍은 대로 변환)
#dst_pts = np.array([[weight-1, viewsize[1]-1], [0, 0], [viewsize[0]-1, 0], [viewsize[0]-weight-1, viewsize[1]-1]], dtype=np.float32) #좀 더 넓은 시야(출력 영상 아래쪽에 영상이 비어있는 부분 발생, 전시야 새눈뷰 가능)
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ좌표 수동 지정ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


lc = 0.0
rc = 0.0
deviation = 0.0
left_fit = []
right_fit = []
white_tresh_lower_left = 160
white_tresh_lower_right = 160
white_tresh_lower_half_sum = 160
yellow_s_tresh_lower = 65
error = 0
bynary_pixel_percent = 0.02
error_pixel_percent_left = 0
total_error_left = 0
error_pixel_percent_right = 0
total_error_right = 0
error_pixel_percent_half_sum = 0
total_error_half_sum = 0
left_trun = 0
right_trun = 0
left_yellow_lane = 0
right_yellow_lane = 0
following_right_lane = False
following_left_lane = False
#-----------------------ros 추가용 코드------------------------------


if __name__ == '__main__':
    main()

 