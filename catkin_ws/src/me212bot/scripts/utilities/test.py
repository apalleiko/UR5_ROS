import cv2
import numpy as np
from ..utilities.vision import HoughCircles, MorphOps
from ..utilities.finder import BrickFinder, LocationFinder

input_filepath = "src/utilities/test_images_input/"
output_filepath = "src/utilities/test_images_output/"
color_HSV = {"r":((np.array([0,70,0]),np.array([8,255,255])),
                        (np.array([140,35,0]),np.array([180,255,255]))), 
        "g":((np.array([85,150,0]), np.array([95,255,255])),), 
        "b":((np.array([100,50,0]),np.array([115,255,255])),), 
        "y":((np.array([12,75,0]),np.array([50,255,255])),)}

def test_hough(filename):
    img = cv2.imread(input_filepath+filename)
    img = img[:,int(img.shape[1]/2):]
    hc = HoughCircles()
    circles, output = hc.get_circles(img, True)
    cv2.imwrite(output_filepath+"hough "+filename, output)

#test_hough("test1.png")

def test_brick(filename, color):
    img = cv2.imread(input_filepath+filename)
    img = img[:,int(img.shape[1]/2):]
    bf = BrickFinder(color_HSV)
    brick, theta, output = bf.find_brick(img, color, True)
    cv2.imwrite(output_filepath+"brick "+color+" "+filename, output)

#test_brick("test1.png","b")

def test_location(filename):
    for i in range(0,13):
        img = cv2.imread(input_filepath+filename)
        img = img[:,:int(img.shape[1]/2)]
        lf = LocationFinder(color_HSV)
        lf.find_base(img)
        lf.set_placed(i)
        location, output = lf.find_location(True, img)
        cv2.imwrite(output_filepath+"base "+str(i)+" "+filename, output)

#test_location("test1.png")