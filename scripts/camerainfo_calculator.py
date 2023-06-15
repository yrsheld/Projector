#!/usr/bin/env python
from sensor_msgs.msg import CameraInfo

class CameraInfoCalculator:
    def __init__(self, z_meters, camframe_width_meters, camframe_height_meters):
      """
      Given
      - the z level of virtual camera (could be set randomly)
      - desired camera field of view (based on measuring the actual projection in physical world)
        - camframe_width_meters
        - camframe_height_meters
        
      Calculate
      - focal length 
        - f/z = w/W (pixel/meter)
        - aspect ratio (width:height)
      """
      self.aspect_ratio = camframe_width_meters / camframe_height_meters
      self.height = int(450) # could actually be any random value
      self.width = int(round(self.height * self.aspect_ratio))

      # f/z = w/W = h/H (pixel/meters)
      self.f = z_meters * self.width / camframe_width_meters

      # set camera center
      self.cx = self.width/2
      self.cy = self.height/2

    def get_info(self):
      """
      Return camera info
      """
      data = CameraInfo()
      data.header.seq = 0
      data.header.stamp.secs = 0
      data.header.stamp.nsecs = 0
      data.header.frame_id = 'camera1'
      data.height = self.height
      data.width = self.width
      #data.distortion_model = 'plumb_bob',
      data.D = [0.0, 0.0, 0.0, 0.0, 0.0]
      data.K = [self.f, 0.0, self.cx,\
                0.0, self.f, self.cy,\
                0.0, 0.0, 1.0]
      data.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      data.P = [data.K[0], data.K[1], data.K[2], 0.0, \
                data.K[3], data.K[4], data.K[5], 0.0, \
                0.0, 0.0, 1.0, 0.0]
      data.binning_x = 0
      data.binning_y = 0
      data.roi.x_offset = 0
      data.roi.x_offset = 0
      data.roi.height = self.height
      data.roi.width = self.width
      data.roi.do_rectify = False

      
      return data
    
#if __name__=='__main__':
#    if len(sys.argv) < 4:
        # use default values
        # based on projector_sim(0.5, (-2, 0.0, -1.57), (0.5, 0.36), (16,9), 50)
#        w = 0.557211
#        h = 0.313431
#        z = 2
#    else:
#        z, w, h = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    
#    calc = CameraInfoCalculator(z, w, h)
#    calc.get_camera_info()
    