# Object width (meters)
OBJECT_WIDTH = 8.48e-2

    last_frame = None

def __init__(self):
self.bridge = CvBridge()
super().__init__('cube_detect')

self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
self.cube_image_pub = self.create_publisher(CompressedImage, "cube_image/compressed", 10)

    def print_predicted_distances(self):
    def print_predicted_distances(self, frame):
# Note: This is what we should fine tune to make our block detection better

        if self.last_frame is None:
            return

        distance, x, y, w, h = self.compute_distance(self.last_frame)
        distance, x, y, w, h = self.compute_distance(frame)

self.get_logger().info(f'the distance is {distance} the x is {x} the y is {y} the w is {w} the h is {h}\n')

@@ -77,9 +72,7 @@ def compute_distance(self, image):
def image_callback(self, msg: CompressedImage):
frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  

        self.get_logger().info("calling in here")

        self.print_predicted_distances()
        self.print_predicted_distances(frame)

cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
self.cube_image_pub.publish(cube_image_msg)
