from camera import get_image
from YOLO import get_block_placement
from turn import set_motor

while True:

    # Get the image
    image = get_image()

    # Pass the image into block placement
    ditance, angle = get_block_placement()

    # Set the motor speeds
    set_motor(angle)