#!/usr/bin/env python3
#computervision processing module


import cv2
import numpy as np
import time
from picamera2 import Picamera2
from config import *

class VisionProcessor:
    def __init__(self):
        self.picam2 = None
        self.setup_camera()
    
    def setup_camera(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(**CAMERA_CONFIG)
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(0.1)  # delays to allow camera to stabilize
    
    def capture_and_preprocess(self):
        #captures frame
        frame = self.picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        #applies ROI selection
        processed_frame = self._apply_roi(frame_bgr)
        
        #apply CLAHE enhancement
        enhanced_frame = self._apply_clahe_enhancement(processed_frame)
        
        #apply cropping sequence
        final_frame = self._apply_cropping_sequence(enhanced_frame)
        
        return final_frame
    
    def _apply_roi(self, frame):
        h, w = frame.shape[:2]
        roi_w = int(w * ROI_WIDTH_RATIO)
        roi_h = int(h * ROI_HEIGHT_RATIO)
        roi_x = int((w - roi_w) / 2) #dont cahnge here ofcourse, just calculate it and do it in config
        roi_y = int(h * ROI_Y_OFFSET)
        return frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
    
    def _apply_clahe_enhancement(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_GRID_SIZE)
        limg = clahe.apply(l)
        enhanced = cv2.merge([limg, a, b])
        return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
    
    def _apply_cropping_sequence(self, frame):
        #first side crop
        side_crop = int(frame.shape[1] * SIDE_CROP_RATIO)
        narrow_frame = frame[:, side_crop:-side_crop]
        
        #vertical 
        temp_frame = narrow_frame[narrow_frame.shape[0]//2:, :]
        
        #another vertical crop
        final_frame = temp_frame[int(temp_frame.shape[0] * VERTICAL_CROP_RATIO):, :]
        
        #final side crop
        final_side_crop = int(final_frame.shape[1] * FINAL_SIDE_CROP_RATIO)
        extreme_narrow_frame = final_frame[:, final_side_crop:-final_side_crop]
        
        return extreme_narrow_frame
    
    def detect_line(self, frame):
        #convert to grayscale and apply threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, GAUSSIAN_BLUR_KERNEL, 0)
        binary = cv2.adaptiveThreshold(blurred, 255,                    #tweaking magic
                                     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 
                                     ADAPTIVE_THRESH_BLOCK_SIZE, 
                                     ADAPTIVE_THRESH_C)
        
        #find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
        
        line_found = False
        line_center = 0.0
        cross_junction = False
        
        if valid_contours:
            #sort by area (big to small)
            valid_contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)
            
            #check for cross junction
            if len(valid_contours) >= CROSS_JUNCTION_MIN_CONTOURS:
                cross_junction = True
            
            #use largest contour for line tracking
            largest = valid_contours[0]
            x, y, w, h = cv2.boundingRect(largest)
            line_center = ((x + w/2) / binary.shape[1]) * 2 - 1  #normalize to -1 to 1
            line_found = True
            
            #draw the basic bounding rectangle
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        return line_found, line_center, cross_junction, binary
    
    def detect_red_objects(self, frame):
        #convert to HSV for better color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        #create masks for red color ranges
        lower_red1 = np.array(RED_HSV_LOWER1)
        upper_red1 = np.array(RED_HSV_UPPER1)
        lower_red2 = np.array(RED_HSV_LOWER2)
        upper_red2 = np.array(RED_HSV_UPPER2)
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        #apply morphological operations to clean up the mask, lets just see where this goes
        #kernel = np.ones(MORPHOLOGY_KERNEL_SIZE, np.uint8)
        #red_mask = cv2.erode(red_mask, kernel, iterations=MORPHOLOGY_ITERATIONS)
        #red_mask = cv2.dilate(red_mask, kernel, iterations=MORPHOLOGY_ITERATIONS)
        
        #find red contours
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_detected = False
        
        for cnt in red_contours:
            if cv2.contourArea(cnt) > MIN_RED_CONTOUR_AREA:
                red_detected = True
                cv2.drawContours(frame, [cnt], -1, (0, 0, 255), 2)
                break
        
        return red_detected, red_mask
    
    def add_visual_overlays(self, frame, status):
        height, width = frame.shape[:2]
        center_x = width // 2
        
        #draw center line and margins
        cv2.line(frame, (center_x, 0), (center_x, height), (0, 255, 0), 2)
        cv2.line(frame, (center_x - CENTER_LINE_MARGIN, 0), 
                (center_x - CENTER_LINE_MARGIN, height), (255, 0, 0), 1)
        cv2.line(frame, (center_x + CENTER_LINE_MARGIN, 0), 
                (center_x + CENTER_LINE_MARGIN, height), (255, 0, 0), 1)
        
        #add status text
        cv2.putText(frame, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                   0.6, (0, 255, 255), 2, cv2.LINE_AA)
    
    def cleanup(self):
        if self.picam2:
            self.picam2.stop()
        cv2.destroyAllWindows()