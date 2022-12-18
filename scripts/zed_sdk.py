import pyzed.sl as sl
from time import sleep
import cv2

class ZED_output():


    def __init__(self) -> None:
        self.zed = sl.Camera()

        self.runtime_parameters =sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL



        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD1080
        self.init_params.camera_fps = 30
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_minimum_distance = 0.15
        self.init_params.depth_maximum_distance = 20


        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(-1)

    def get_data(self):
        image = sl.Mat()
        depth_map = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns SUCCESS
            self.zed.retrieve_image(image, sl.VIEW.LEFT) # Retrieve the left image
            self.zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            self.img_bgr_left = image.get_data()
            depth_value = depth_map.get_value([1,2,3], [4,5,6])
            print(depth_value)
            return self.img_bgr_left

    def get_depth(self):
        pass

    def display(self):
        cv2.imshow('ZED left image', self.img_bgr_left)
        cv2.waitKey(0)
        cv2.destroyAllWindows()




if __name__ == '__main__':
    cam = ZED_output()
    print(cam.get_data().shape)
    sleep(5)
    cam.display()
    
