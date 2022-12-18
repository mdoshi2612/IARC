#!/usr/bin/env python3
from tool.darknet2pytorch import Darknet
from tool.torch_utils import *
import cv2


def yolo(tiny = True):
    if tiny:
        board=Darknet('cfg/yolov4-tiny_box.cfg',inference=True)
        board.load_weights('backup/yolov4-tiny_box_both_last.weights')

    else:
        board=Darknet('cfg/custom-yolov4-detector.cfg',inference=True)
        board.load_weights('backup/custom-yolov4-detector_4000.weights')
    board.cuda()
    return board


def my_detect(m,cv_img):
    use_cuda=True
    img=cv2.resize(cv_img, (m.width, m.height))
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    boxes = do_detect(m, img, 0.2, 0.6, use_cuda)
    if len(boxes[0])==0:
        return [False,0,0,0,0]
    box=boxes[0][0]
    h,w,c=cv_img.shape
    x1 = int(box[0] * w)
    y1 = int(box[1] * h)
    x2 = int(box[2] * w)
    y2 = int(box[3] * h)
    return [True,x1,y1,x2,y2]
