from tool.darknet2pytorch import Darknet
from tool.torch_utils import *
import cv2
import pandas as pd

def yolo(config_file_path, weight_file_path):
    board=Darknet(config_file_path,inference=True)
    board.load_weights(weight_file_path)
    board.cuda()
    return board

def my_detect(m,cv_img):
    use_cuda=True
    img=cv2.resize(cv_img, (m.width, m.height))
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    boxes = do_detect(m, img, 0.3, 0.6, use_cuda)
    if len(boxes[0])==0:
        return [False,0,0,0,0,-1]
    box=boxes[0][0]
    h,w,c=cv_img.shape
    x1 = int(box[0] * w)
    y1 = int(box[1] * h)
    x2 = int(box[2] * w)
    y2 = int(box[3] * h)
    score = box[4]
    return np.asarray([True,x1,y1,x2,y2,score])


def draw_box(board,image):
    
    ret, x1,y1,x2,y2,score = my_detect(board, image)
    if ret == True:
        boxes = [ret, x1,y1,x2,y2,score]
        score = str(round(boxes[5],2))
        print(boxes)
        image=cv2.rectangle(image,(x1,y1),(x2,y2),(255,0,0),3)
    
        # For the text background
        # Finds space required by the text so that we can put a background with that amount of width.
        (w, h), _ = cv2.getTextSize(
                f'Mast:{score}', cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)

        # Prints the text.    
        image = cv2.rectangle(image, (x1, y1 - 20), (x1 + w, y1), (255,0,0), -1)
        image = cv2.putText(image, f'Mast:{score}', (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        # For printing text
    image=cv2.resize(image,(512,512))
    #    cv2.imshow('image',image)
#    while True:
#        if cv2.waitKey(0) & 0xFF == ord('q'):
#            break


def get_output(file_name):

  column_names = ['Frame','Detected', 'x1', 'y1', 'x2', 'y2', 'Score']
  df = pd.DataFrame(columns = column_names)
  
  cap=cv2.VideoCapture(file_name)
  size = (int(cap.get(3)),int(cap.get(4)))
  writer = cv2.VideoWriter('./Results/result_thresh_30%.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         30, size)

  n = 0
  while True :
      ret,frame=cap.read()
      if not ret:
        break
      is_detected, x1, y1, x2, y2, score=my_detect(board,frame)
      
      row = {'Frame':n, 'Detected':is_detected ,'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'Score':score}
      df = df.append(row, ignore_index = True)
      if is_detected:
          draw_box(board,frame)
      writer.write(frame)
      n+=1

  # print(n/t)
  writer.release()
  return df



if __name__ == '__main__':
    config_file_path = './custom-yolov4-detector.cfg'
    weight_file_path = './custom-yolov4-detector_4000.weights'


    board = yolo(config_file_path, weight_file_path)



    #image = cv2.imread('test2.png')
    #draw_box(image)
    results = get_output('./Video and Image Extractor/vid.mp4')
    results.to_csv('./Results/results_thresh_30%.csv', index = False)
