from controller import Robot
import math
import json

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_enc = robot.getDevice("left wheel sensor")
right_enc = robot.getDevice("right wheel sensor")
left_enc.enable(timestep)
right_enc.enable(timestep)

for _ in range(10):
    robot.step(timestep)

WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M  = 0.052
TURN_SPEED     = 3.0

# range of timesteps to test
STEPS_START = 20
STEPS_END   = 60
STEPS_INC   = 2

results = []

for steps in range(STEPS_START, STEPS_END + 1, STEPS_INC):
    turn_results = []

    for i in range(4):
        turned = 0.0
        prev_l = left_enc.getValue()
        prev_r = right_enc.getValue()

        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)

        for _ in range(steps):
            robot.step(timestep)
            l, r   = left_enc.getValue(), right_enc.getValue()
            dl = (l - prev_l) * WHEEL_RADIUS_M
            dr = (r - prev_r) * WHEEL_RADIUS_M
            prev_l, prev_r = l, r
            turned += abs((dr - dl) / AXLE_LENGTH_M)

        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

        turn_results.append({
            "turn": i + 1,
            "integrated_rad": round(turned, 4),
            "integrated_deg": round(math.degrees(turned), 2)
        })

        print(f"steps={steps} | turn {i+1} | integrated={turned:.4f} rad ({math.degrees(turned):.1f} deg)")

        for _ in range(20):
            robot.step(timestep)

    results.append({
        "steps": steps,
        "turns": turn_results
    })

    print(f"=== steps={steps} done, pausing to observe ===")
    for _ in range(50):
        robot.step(timestep)

with open("turn_calibration.json", "w") as f:
    json.dump(results, f, indent=2)

print("=== Done. Results saved to turn_calibration.json ===")