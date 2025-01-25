import numpy as np

def convert_int32_to_location(value):

    x = value >> 16
    y = value & ((1<<16)-1)

    # Undo the shift here
    shift = 1 << 15
    shift = np.int32(shift)
    x, y = x-shift, y-shift

    # IMPORTANT NOTE: CHECHY BS
    if x < -300:
        x += 65536

    return (x, y)

def convert_location_to_int32(x, y):

    # Add this number to gaurantee they are always positive when doing the encoding
    shift = 1 << 15
    shift = np.int32(shift)
    x, y = x+shift, y+shift

    return np.int32((x << 16) + y)

def main():
    test_values = [(0, 1), (1, 0), (1, 2), (0, 100), (0, 0), (-100, -1), (-100, 4)]

    for test_value in test_values:
        print(f'test_value is {test_value} while the converted is {convert_location_to_int32(*test_value)} while going to the original is {convert_int32_to_location(convert_location_to_int32(*test_value))}')
        assert test_value == convert_int32_to_location(convert_location_to_int32(*test_value))

if __name__ == '__main__':
    main()