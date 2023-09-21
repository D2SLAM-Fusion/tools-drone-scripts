# Description: This script is used to upside down the fisheye camera image
#!/usr/bin/env python3
import rosbag
from os.path import exists
from cv_bridge import CvBridge
import cv2 as cv
import tqdm
import numpy as np

def generate_bagname(bag, comp=False):
  from pathlib import Path
  p = Path(bag)
  bagname = p.stem + "-upside_down.bag"
  output_bag = p.parents[0].joinpath(bagname)
  return output_bag

def upside_down(img):
  return cv.flip(img, 0)

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(description='Split quadcam images')
  parser.add_argument("-i","--input", type=str, help="input bag file")
  parser.add_argument('-v', '--show', action='store_true', help='compress the image topics')
  args = parser.parse_args()
  output_bag = generate_bagname(args.input)
  print("Output bag file:", output_bag)
  if not exists(args.input):
      print(f"Input bag file {args.input} does not exist")
      exit(1)
  
  bag = rosbag.Bag(args.input)
  num_imgs = bag.get_message_count("/arducam/image/compressed") + bag.get_message_count("/arducam/image") + \
      bag.get_message_count("/oak_ffc_4p/assemble_image/compressed") + bag.get_message_count("/oak_ffc_4p/assemble_image")
  print("Total number of images:", num_imgs)
  bridge = CvBridge()

  pbar = tqdm.tqdm(total=num_imgs, colour="green")
  upside_down_img = None
  upside_down_img_msg = None
  with rosbag.Bag(output_bag, 'w') as outbag:
      from nav_msgs.msg import Path
      path = Path()
      path_arr = []
      for topic, msg, t in bag.read_messages():
        if topic == "/arducam/image/compressed" or topic == "/arducam/image/raw" or \
            topic == "/oak_ffc_4p/assemble_image/compressed" or topic == "/oak_ffc_4p/assemble_image":
            # print("topic:", topic)
            if msg._type == "sensor_msgs/Image":
              img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
              upside_down_img =  upside_down(img)
              # print("img shape:", img.shape)
              # cv.imshow("img_ori", img)
              # cv.imshow("img", upside_down_img)
              # cv.waitKey(1)
              upside_down_img_msg = bridge.cv2_to_imgmsg(upside_down_img,encoding='bgr8')
            else:
              img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
              upside_down_img =  upside_down(img)
              upside_down_img_msg = bridge.cv2_to_compressed_imgmsg(upside_down_img,encoding='bgr8')
            outbag.write(topic, upside_down_img_msg, t)
            pbar.update(1)
        else:
            outbag.write(topic, msg, t)
                


