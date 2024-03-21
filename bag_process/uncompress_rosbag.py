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
  bagname = p.stem + "-uncompressed.bag"
  output_bag = p.parents[0].joinpath(bagname)
  return output_bag

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
    num_imgs = bag.get_message_count("/camera/infra1/image_rect_raw/compressed") + bag.get_message_count("/camera/infra2/image_rect_raw/compressed") + \
        bag.get_message_count("/oak_ffc_4p/assemble_image/compressed")
    print("Total number of images:", num_imgs)
    bridge = CvBridge()

    pbar = tqdm.tqdm(total=num_imgs, colour="green")
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in bag.read_messages():
            if topic == "/camera/infra1/image_rect_raw/compressed" or \
                topic == "/camera/infra2/image_rect_raw/compressed":
                img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                raw_image = bridge.cv2_to_imgmsg(img,encoding='passthrough', header=msg.header)
                raw_topic = topic.replace("/compressed", "")
                outbag.write(raw_topic, raw_image, t)
                pbar.update(1)
                continue
            if topic == "/oak_ffc_4p/assemble_image/compressed" :
                img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                raw_image = bridge.cv2_to_imgmsg(img,encoding='bgr8', header=msg.header)
                raw_topic = topic.replace("/compressed", "")
                outbag.write(raw_topic, raw_image, t)
                pbar.update(1)
                continue
            outbag.write(topic, msg, t)
            # print("write time stamp:", t)
    outbag.close()
    bag.close()
        
                


