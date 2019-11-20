#!/usr/bin/env python
import sys
import cv2

def lane_filter(image):
    # The incoming image is BGR format, convert it to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Filter for only white pixels. Experiment with values as needed
    white_filter = cv2.inRange(hsv, (0,0,0), (180,35,255))
    cv2.imshow("White Filter", white_filter)
    cv2.imwrite("white_filter.png", white_filter)
    
    # Filter for only yellow pixels. Experiment with values as needed
    yellow_filter = cv2.inRange(hsv, (26,77,178), (31,255,255))
    cv2.imshow("Yellow Filter", yellow_filter)
    cv2.imwrite("yellow_filter.png", yellow_filter)
    
    # Create a kernel to dilate the image. 
    # Experiment with the numbers in parentheses (optional)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    
    # Dilate both the white and yellow images. 
    # No need to experiment here.
    white_dilate = cv2.dilate(white_filter, kernel)
    cv2.imshow("Dilate White", white_dilate)
    cv2.imwrite("white_dilate.png", white_dilate)
    yellow_dilate = cv2.dilate(yellow_filter, kernel)
    cv2.imshow("Dilate Yellow", yellow_dilate)
    cv2.imwrite("yellow_dilate.png", yellow_dilate)
    
    # Perform edge detection on the original image. 
    # Experiment with the first two numbers. Aperture size experimentation optional
    edges = cv2.Canny(image, 0, 300, apertureSize=3)
    cv2.imshow("Edges", edges)
    cv2.imwrite("edges.png", edges)
    
    # Use the edges to refine the lines in both white and yellow images
    # No need to experiment here 
    white_edges = cv2.bitwise_and(white_dilate, edges)
    cv2.imshow("White Edges", white_edges)
    cv2.imwrite("white_edges.png", white_edges)
    yellow_edges = cv2.bitwise_and(yellow_dilate, edges)
    cv2.imshow("Yellow Edges", yellow_edges)
    cv2.imwrite("yellow_edges.png", yellow_edges)
    
    # Wait for key press to close images
    cv2.waitKey()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s image_filename.png" % sys.argv[0])
        exit()
        
    image_filename = sys.argv[1]
    image = cv2.imread(image_filename)
    lane_filter(image)
