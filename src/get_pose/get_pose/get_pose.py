import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class PoseExtractorNode(Node):
    def __init__(self):
        super().__init__('pose_extractor')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',  
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            '/hough_lines',
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # turn to grayscale
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Resize the image
        resized_img = resize(gray_img, 100)
        
        # histogram equalization
        equalized_img = cv2.equalizeHist(resized_img)
        
        # Gaussian blur
        blurred_img = cv2.GaussianBlur(equalized_img, (7, 7), 0)

        # Canny edge
        canny_edge_img = cv2.Canny(blurred_img, 50, 100)

        # Apply dilation to fill in gaps between edge segments
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # Adjust kernel size as needed
        dilated_edges = cv2.dilate(canny_edge_img, kernel, iterations=1)


        # Convert OpenCV image back to ROS Image message
        resized_msg = self.bridge.cv2_to_imgmsg(resized_img, encoding='bgr8')

        # Publish the resized image
        self.publisher.publish(resized_msg)


def get_lines(canny_edge_img, original_img):
    # Apply Hough Transform to detect lines
    # lines = cv2.HoughLines(canny_edge_img, 1, np.pi/180, 200)
    lines = cv2.HoughLines(canny_edge_img, 1, np.pi/180, 150)

    # Create a copy of the source image to draw the lines
    # output_image = resized_img.copy()
    output_image = original_img.copy()

    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))

        cv2.line(output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    return output_image


def resize(img, scale=40):

    scale_percent = scale # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized_img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    return resized_img


def main(args=None):
    rclpy.init(args=args)
    pose_extractor_node = PoseExtractorNode()
    rclpy.spin(pose_extractor_node)
    pose_extractor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
