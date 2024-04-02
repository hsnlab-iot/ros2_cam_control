import ros_sam
import cv2
import numpy as np

# ros2_sam.sam_client = ros2_sam.samclient('ros_sam')


img = cv2.imread('/home/ubuntu/AIMS50_imgproc/20210624_161404_martzi.jpg')
points = np.array([[100, 100], [200, 200], [300, 300]])
labels = [1, 1, 0]
masks, scores = ros_sam.sam_client.segment(img, points, labels)

show_mask(masks[0], plt.gca())
show_points(points, np.asarray(labels), plt.gca())