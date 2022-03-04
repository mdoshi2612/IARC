from cgitb import text
import cv2
import numpy as np

text_file = np.loadtxt('output\ADL-Rundle-1.txt',delimiter=',')
print(text_file)