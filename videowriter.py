from cgitb import text
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

 

img_path = os.path.join('mot_benchmark', 'train','ADL-Rundle-1', 'img1', '000001.jpg')
img = cv2.imread(img_path)
height, width, _ = img.shape
print(width, height)

writer = cv2.VideoWriter('./yolo_model/Results/sort.mp4', 
                         cv2.VideoWriter_fourcc(*'mp4v'),
                         30, (width,height))

text_file = np.loadtxt('output\ADL-Rundle-1.txt',delimiter=',').astype(int)

max_frames = int(max(text_file[:,0]))

arr = text_file[text_file[:,0] == 1000].squeeze()[2:6]
print(arr)
for frame in range(1,max_frames+1):
    fn = os.path.join('mot_benchmark', 'train','ADL-Rundle-1', 'img1', '%06d.jpg'%(frame))
    frame_image = cv2.imread(fn)
    if frame in text_file[:,0]:
        x1,y1,w,h = text_file[text_file[:,0] == frame].squeeze()[2:6]
        #print(x1,y1,w,h)
        frame_image=cv2.rectangle(frame_image,(x1,y1),(x1+w,y1+h),(255,0,0),3)
        writer.write(frame_image)
        print(frame, True)
    else:
        print(frame, False)
        writer.write(frame_image)

writer.release()
print("Video produced")

        