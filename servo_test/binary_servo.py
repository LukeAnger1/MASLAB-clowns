from raven.raven import Raven

# Initialize the Raven motor controller
raven_board = Raven()

# Function to set the servo position
def set_gate(value):
    if value:
        # Rotate to +90 degrees
        raven_board.set_servo_position(Raven.ServoChannel.CH1, 90, min_us=500, max_us=2500)
        print("Gate set to +90 degrees")
    else:
        # Rotate to -90 degrees
        raven_board.set_servo_position(Raven.ServoChannel.CH1, -90, min_us=500, max_us=2500)
        print("Gate set to -90 degrees")

# Test the binary gate logic
try:
    while True:
        user_input = input("Enter '1' for True or '0' for False: ")
        if user_input == '1':
            set_gate(True)
        elif user_input == '0':
            set_gate(False)
        else:
            print("Invalid input. Enter '1' or '0'.")
except KeyboardInterrupt:
    print("\nExiting program.")
