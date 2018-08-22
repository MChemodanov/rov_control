#!/usr/bin/env python


import rospy

import os
import sys
from multiprocessing.pool import ThreadPool

import cv2
import numpy as np


from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

def load_image(filename):
  img = cv2.imread(filename, -1)
  if img is None:
    #rospy.logerr("ERROR image file didn't load or doesn't exist: %s" % filename)
    exit(1)
  return img

def resize_image(img, scale):
  dimensions = (int(img.shape[1] * scale), int(img.shape[0] * scale))
  img = cv2.resize(img, dimensions, interpolation = cv2.INTER_CUBIC)
  return img

def rotate_image(img, degreesCCW, scaleFactor):
    oldY, oldX = img.shape[:2]
    M = cv2.getRotationMatrix2D(center=(oldX / 2, oldY / 2), angle=degreesCCW, scale=scaleFactor)
    newX, newY = oldX * scaleFactor, oldY * scaleFactor
    r = np.deg2rad(degreesCCW)
    newX, newY = (abs(np.sin(r) * newY) + abs(np.cos(r) * newX), abs(np.sin(r) * newX) + abs(np.cos(r) * newY))
    (tx, ty) = ((newX - oldX) / 2, (newY - oldY) / 2)
    M[0, 2] += tx
    M[1, 2] += ty

    rotatedImg = cv2.warpAffine(img, M, dsize=(int(newX), int(newY)))
    return rotatedImg

def overlay_image(fg_img, bg_img, x, y):
  for c in range(0,3):
      bg_img[y:y + fg_img.shape[0], x:x + fg_img.shape[1], c] = \
      fg_img[:, :, c] * (fg_img[:, :, 3] / 255.0) + bg_img[y:y + fg_img.shape[0], x:x + fg_img.shape[1], c] * (1.0 - fg_img[:, :, 3] / 255.0)
  return bg_img

class ImageOverlayControl:
  def __init__(self):
    self.scale_text = 0
    self.scale_percentage = 0.4
    self.heading = 0
    self.depth = 0
    self.roll = 0

    self.pool = ThreadPool(3)

    self.img = np.ones((800, 1100, 3), dtype=np.uint8) * 128 

    current_dir = os.path.dirname(os.path.abspath(__file__))

    self.roll_img = load_image('%s/images/KREN_Arrow.png' % current_dir)
    self.roll_img_bg = resize_image(load_image('%s/images/KREN_Grad.png' % current_dir), self.scale_percentage)

    self.bridge = CvBridge()

    rospy.init_node('image_overlayer')

    self.roll = 0
    self.depth = 0
    self.target_depth = 0
    self.heading = 0
    self.target_heading = 0

    self.roll_sub = rospy.Subscriber("rpy/roll", Float32, self.roll_callback)
    self.heading_sub = rospy.Subscriber("rpy/yaw", Float32, self.heading_callback)
    self.depth_sub = rospy.Subscriber("depth", Float32, self.depth_callback)
    self.target_depth_sub = rospy.Subscriber("target_depth", Float32, self.target_depth_callback)
    self.target_heading_sub = rospy.Subscriber("target_yaw", Float32, self.target_heading_callback)

    self.frame_sub = rospy.Subscriber("/cam1/compressed", CompressedImage, self.frame_callback)
    self.frame_pub = rospy.Publisher("/overlay", Image, queue_size=10)

  def roll_callback(self, data):
    self.roll = data.data

  def heading_callback(self, data):
    self.heading = data.data

  def target_heading_callback(self, data):
    self.target_heading = data.data
    if self.target_heading < 0:
      self.target_heading += 360
    rospy.logwarn(self.target_heading)

  def target_depth_callback(self, data):
    self.target_depth = data.data


  def depth_callback(self, data):
    self.depth = data.data

  def frame_callback(self, data):
    self.img = self.bridge.compressed_imgmsg_to_cv2(data)
    #self.img = self.bridge.imgmsg_to_cv2(data)
    result = self.overlay_telemetry()
    result2 = self.bridge.cv2_to_imgmsg(result,"bgr8")
    self.frame_pub.publish(result2)

  def overlay_telemetry(self):
    im_out = np.uint8(self.img.copy())
    #im_out = self.img

    def overlay_roll():
      roll_img = rotate_image(self.roll_img, self.roll, self.scale_percentage)

      x = im_out.shape[1] / 2 - roll_img.shape[1] / 2
      y = im_out.shape[0] / 2 - roll_img.shape[0] / 2
      return overlay_image(roll_img, im_out, x, y)

    def overlay_roll_bg():
      x = im_out.shape[1] / 2 - self.roll_img_bg.shape[1] / 2
      y = im_out.shape[0] / 2 - self.roll_img_bg.shape[0] / 2
      return overlay_image(self.roll_img_bg, im_out, x, y)
      
    def set_text():
      font = 2
      scale = 1
      thick = 1
      line_type = 4
      color = tuple(np.ones(im_out.shape[2]) * 255)

      loc = (int(20), int(30))
      cv2.putText(im_out, "Depth: %.2f (%.2f)" % (self.depth, self.target_depth), loc, font, scale, color, int(thick), line_type)

      loc = (im_out.shape[1] / 2 - 5, im_out.shape[0] - 15)
      cv2.putText(im_out, "%i (%i)"  % (self.heading % 360, self.target_heading), loc, font, scale, color, int(thick), line_type)

    roll_r = self.pool.apply_async(overlay_roll, ())
    roll_bg_r = self.pool.apply_async(overlay_roll_bg, ())
    text_r = self.pool.apply_async(set_text, ())

    roll_r.get()
    roll_bg_r.get()
    text_r.get()

    return im_out

if __name__ == "__main__":
  worker = ImageOverlayControl()
  rospy.spin()
