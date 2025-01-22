from camera import get_image
from YOLO import get_block_placement
from turn import set_motor

while True:

    # Get the image
    image = get_image()

    # Pass the image into block placement
    holder = get_block_placement(image)
    if holder is not None:
        ditance, angle = holder

        # Set the motor speeds
        set_motor(angle)