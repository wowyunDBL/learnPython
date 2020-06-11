# load raw data
# load detection result


#!/usr/bin/env python

import cv2
import numpy as np
import pandas as pd

TRACKING_COLUMN_NAMES = ['frame', 'track id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top', 
               'bbox_right', 'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']

def load_image(path):
	return cv2.imread(path)

def load_pcl(path):
	return np.fromfile(path, dtype=np.float32).reshape(-1,4)

def load_detection_result(path):
	df = pd.read_csv(path, header=None, sep = ' ')
	df.columns = TRACKING_COLUMN_NAMES
	df.loc[df.type.isin(['Van', 'Truck', 'Tram']),'type'] = 'Car'
	df = df[ df.type.isin(['Car','Pedestrian', 'Cyclist']) ]
	return df
