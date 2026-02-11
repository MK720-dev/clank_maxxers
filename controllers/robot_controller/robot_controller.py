from controller import Robot
import numpy as np 
import cv2

class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.k = 0.002 # Proportional gain for turning, adjust as needed
        self.state = "SEARCH"   # SEARCH → ALIGN → CROSS
        self.cooldown_steps = 0  # Add cooldown counter
        self.cooldown_duration = 100  # ~2 seconds at 50ms timestep, adjust as needed
        

        # Initialize devices here (e.g., motors, sensors)
        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.camera = robot.getDevice("camera")

        # Enable devices
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.camera.enable(self.timestep)
        self.compass = robot.getDevice("compass")
        self.compass.enable(self.timestep)

        # Get max velocity for later use
        self.max_velocity = self.left_motor.getMaxVelocity()
        self.base_speed = 0.5 * self.max_velocity

        # State
        self.target_angle = 150  # Start by rotating to 150

    def move(self, left_speed, right_speed):
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def capture_frame(self):
        buffer_image = self.camera.getImage()
        img = np.frombuffer(buffer_image, dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4)) 
        return img[:,:,0:3]

    def red_mask(self, img): 
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower_red1 = (0, 120, 70)
        upper_red1 = (10, 255, 255)
        mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        lower_red2 = (170, 120, 70)
        upper_red2 = (179, 255, 255)
        mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        return mask1 | mask2

    def green_mask(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower_green = (40, 20, 20)   # Tighter saturation & value
        upper_green = (95, 255, 255)  # Narrower hue range
        mask = cv2.inRange(hsv_img, lower_green, upper_green)
        return mask

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)
        return cleaned_mask

    def find_centroid(self, mask):
        moments = cv2.moments(mask)
        print(f"m00: {moments['m00']}, mask sum: {np.sum(mask)}")  # DEBUG
        if moments["m00"] == 0:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        print(f"Centroid found at: ({cx}, {cy})")
        return (cx, cy)

    def EMA(self, current, previous, alpha=0.5):
        if previous is None:
            return current
        return alpha * current + (1 - alpha) * previous

    def mask_area_ratio(self, mask):
        return np.sum(mask > 0) / mask.size

    def get_heading(self):
        """Get current heading in degrees (0-360)"""
        compass_values = self.compass.getValues()
        angle = np.arctan2(compass_values[0], compass_values[1])
        return np.degrees(angle) % 360
    
    def angle_difference(self, target, current):
        """Calculate shortest angular distance"""
        diff = target - current
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff

    def run(self):
        while self.robot.step(self.timestep) != -1:
            current_heading = self.get_heading()
            angle_error = self.angle_difference(self.target_angle, current_heading)
            
            print(f"Current: {current_heading:.1f}°, Target: {self.target_angle}°, Error: {angle_error:.1f}°")
            
            # If close enough to target, switch target
            if abs(angle_error) < 5:
                if self.target_angle == 150:
                    self.target_angle = 30
                    print("→ Switching target to 30°")
                else:
                    self.target_angle = 150
                    print("→ Switching target to 150°")
            
            # Rotate toward target
            if angle_error > 0:
                # Turn left
                self.left_motor.setVelocity(-self.base_speed)
                self.right_motor.setVelocity(self.base_speed)
            else:
                # Turn right
                self.left_motor.setVelocity(self.base_speed)
                self.right_motor.setVelocity(-self.base_speed)


RobotController = Controller(Robot())
RobotController.run()