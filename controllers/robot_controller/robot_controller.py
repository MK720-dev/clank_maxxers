"""
Basic e-puck controller:
- Drives forward continuously
- Enables camera
- Enables distance sensors
- Uses HSV and region averaging for robust color detection
"""

from controller import Robot, DistanceSensor, Motor, Camera
import colorsys  # For RGB -> HSV conversion

# -----------------------------
# Robot & Time Step
# -----------------------------
robot = Robot()

# Get the basic time step of the current world
timestep = int(robot.getBasicTimeStep())

# -----------------------------
# Motors (Differential Drive)
# -----------------------------
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set motors to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set a constant forward speed
FORWARD_SPEED = 6.28 # Max = 6.28
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# -----------------------------
# Camera
# -----------------------------
camera = robot.getDevice('camera')
camera.enable(timestep)

# -----------------------------
# Color detection function (center region, HSV)
# -----------------------------




def detect_color():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    # Center pixel
    x = width // 2
    y = height // 2
    r = camera.imageGetRed(image, width, x, y)
    g = camera.imageGetGreen(image, width, x, y)
    b = camera.imageGetBlue(image, width, x, y)
    
    # Convert to HSV
    r_n, g_n, b_n = r/255, g/255, b/255
    h, s, v = colorsys.rgb_to_hsv(r_n, g_n, b_n)
    
    if (h < 0.05 or h > 0.95) and s > 0.4 and v > 0.2:
        return "RED"
    elif 0.25 < h < 0.42 and s > 0.4 and v > 0.2:
        return "GREEN"
    else:
        return "NONE"
        
   




# -----------------------------
# Distance Sensors
# -----------------------------
# e-puck has 8 distance sensors: ps0 → ps7
distance_sensors = []

for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(timestep)
    distance_sensors.append(sensor)

# -----------------------------
# Main Control Loop
# -----------------------------
while robot.step(timestep) != -1:
    """
    This loop runs once every timestep.
    The robot currently:
    - Keeps moving forward
    - Reads sensors and color detection
    """
    leftSpeed  = 0.5 * FORWARD_SPEED
    rightSpeed = 0.5 * FORWARD_SPEED
    
    color = detect_color()
    if color == "RED":
        print("Red detected!")
    elif color == "GREEN":
        print("Green detected!")
    else:
        print("No color detected")
        
    
    left_motor.setVelocity(leftSpeed)
    right_motor.setVelocity(rightSpeed)
    
    # Read distance sensor values
    ps_values = [sensor.getValue() for sensor in distance_sensors]

    # Example debug output (optional)

