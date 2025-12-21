from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Color

# -----------------------------
# Robot configuration
# -----------------------------
WHEEL_RADIUS_CM = 3.175
WHEEL_DIAMETER_MM = WHEEL_RADIUS_CM * 2 * 10  # cm -> mm
AXLE_TRACK_MM = 139  # IMPORTANT: set this to your real wheel-to-wheel distance

hub = PrimeHub()

# Drive motors (A/B). Adjust Directions if your robot drives backward.
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)

# Attachment motors
motor_c = Motor(Port.C)
motor_d = Motor(Port.D)

# -----------------------------
# Shared helpers
# -----------------------------
def beep_ok():
    hub.speaker.beep(900, 80)

def beep_mission(n):
    # Beep N+1 times so kids can tell mission ran
    for _ in range(n + 1):
        hub.speaker.beep(700, 70)
        wait(80)

def emergency_stop_check():
    # CENTER button = stop anytime
    # (Bluetooth is used to START a mission; use CENTER to stop/emergency)
    if Button.CENTER in hub.buttons.pressed():
        robot.stop()
        left_motor.stop()
        right_motor.stop()
        motor_c.stop()
        motor_d.stop()
        hub.speaker.beep(200, 300)
        raise RuntimeError("Emergency stop")

def setup_drive():
    robot.straight(-10)
    robot.reset()
    # “Default-ish” settings; missions override per move
    robot.settings(straight_speed=300, straight_acceleration=300)
    beep_ok()

def gyro_turn(target_angle: float,
              mode: str = "medium",
              settle_tol: float = 2.0,
              settle_timeout_ms: int = 1500):
    """
    Hybrid turn:
      1) DriveBase robot.turn() does most of the turn (repeatable).
      2) IMU settle loop corrects final error (accurate).

    target_angle: + right, - left
    mode: "slow" | "medium" | "fast"
    settle_tol: final tolerance in degrees
    """

    # Turn speed profiles (deg/s) for DriveBase
    turn_rates = {
        "slow": 80,
        "medium": 120,
        "fast": 180,
    }
    turn_rate = turn_rates.get(mode, 120)

    # IMU nudge speeds (motor run speed)
    nudge_speed = {
        "slow": 60,
        "medium": 75,
        "fast": 90,
    }.get(mode, 75)

    # How much of the turn we trust DriveBase to do before IMU settle
    # (leave a small margin so robot.turn doesn't overshoot and fight the settle loop)
    settle_margin = {
        "slow": 6,
        "medium": 8,
        "fast": 12,
    }.get(mode, 8)

    # --- Prep ---
    emergency_stop_check()
    hub.imu.reset_heading(0)
    wait(50)

    # Configure DriveBase turning speed (this affects robot.turn)
    # Keep your straight settings unchanged; just set turn_rate.
    # If you want you can also pass turn_acceleration here.
    robot.settings(turn_rate=turn_rate)

    # --- Phase A: bulk turn with DriveBase ---
    bulk = target_angle

    # Leave a margin for IMU settling
    if abs(target_angle) > settle_margin:
        bulk = target_angle - (settle_margin if target_angle > 0 else -settle_margin)

    robot.turn(bulk)

    # --- Phase B: IMU settle (fine correction) ---
    sw = StopWatch()
    sw.reset()

    def set_nudge(sign: int):
        # sign = +1 means turn right; -1 means turn left
        if sign > 0:
            left_motor.run(nudge_speed)
            right_motor.run(-nudge_speed)
        else:
            left_motor.run(-nudge_speed)
            right_motor.run(nudge_speed)

    # Pulse-based settle is more stable than continuous run (less overshoot)
    while True:
        emergency_stop_check()

        h = hub.imu.heading()
        err = target_angle - h

        if abs(err) <= settle_tol:
            break

        if sw.time() > settle_timeout_ms:
            # give up but stop safely
            break

        set_nudge(1 if err > 0 else -1)
        wait(20)
        left_motor.stop()
        right_motor.stop()
        wait(30)

    left_motor.stop()
    right_motor.stop()


def drive_cm(distance_cm, velocity_cm_s, acceleration_cm_s2, stop=Stop.HOLD):
    """Drive a set distance in cm using DriveBase (mm units internally)."""
    distance_mm = distance_cm * 10
    speed_mm_s = velocity_cm_s * 10
    accel_mm_s2 = acceleration_cm_s2 * 10

    robot.settings(straight_speed=speed_mm_s, straight_acceleration=accel_mm_s2)
    robot.straight(distance_mm)
    #robot.stop()  # DriveBase stop uses internal behavior; motors also hold by default

def drive_cm_stall(distance_cm: float,
             velocity_cm_s: float,
             acceleration_cm_s2: float,
             stop: Stop = Stop.HOLD,
             poll_ms: int = 10) -> bool:
    """
    Drive a set distance in cm using DriveBase (mm units internally).
    Returns True if the motion completed, False if a stall was detected.
    Relies only on robot.stalled (property) and robot.done() for completion.
    """
    distance_mm = distance_cm * 10
    speed_mm_s = velocity_cm_s * 10
    accel_mm_s2 = acceleration_cm_s2 * 10

    # Configure DriveBase
    robot.settings(straight_speed=speed_mm_s, straight_acceleration=accel_mm_s2)

    # Start motion non-blocking so we can poll for stall
    robot.straight(distance_mm, wait=False)
    # print(f"initial postion : {getattr(robot, "stalled", False)}")
    # print("inside drice with stall")

    # Poll until done or stalled
    while True:
        # print("inside drice with stall while loop")
        # print(robot.stalled())
        # Stall is a property, not a callable
        stalled_attr = getattr(robot, "stalled", None)
        stalled = stalled_attr() if callable(stalled_attr) else bool(stalled_attr)
        if stalled:
            # print(robot.stalled())
            # Stop the robot immediately and report failure
            # print("motor is stalled")
            robot.stop()
            return False

        # Use done() to detect normal completion
        done_fn = getattr(robot, "done", None)
        if callable(done_fn) and robot.done():
            # Ensure final stop behavior matches requested 'stop'
            if stop is Stop.HOLD:
                robot.stop()
            elif stop is Stop.BRAKE:
                robot.stop()
            else:
                robot.stop()
            return True

        wait(100)

def run_motor_for_degrees(m: Motor, degrees: int, speed: int, accel: int = 1000, stop=Stop.HOLD):

    # Note: Motor.run_angle(speed, rotation_angle) is blocking.
    m.run_angle(speed, degrees, then=stop, wait=True)


# -----------------------------
# Mission implementations (0–5)
# Converted from your SPIKE code
# -----------------------------
def mission_0():
    setup_drive()
    # print("Mission 0")

    hub.imu.reset_heading(0)
    wait(200)

    drive_cm(15, 30, 50)
    gyro_turn(-12, mode="medium")
    wait(200)

    drive_cm(55, 100, 40)
    wait(200)
    gyro_turn(55, mode="medium")
    wait(200)
    drive_cm(10, 30, 30)

    run_motor_for_degrees(motor_d, -1000, 1000) # SPIKE: move_sidearm_mission9(port.D, -1000, 1000, 500)
    run_motor_for_degrees(motor_c, 50, 500) # SPIKE: move_sidearm_mission9(port.C, 100, 1000, 500)
    # print("Turn completed")
    gyro_turn(-35, mode="medium")
    drive_cm(-60, 100, 500)
    gyro_turn(-90, mode="fast")
    drive_cm(-60, 100, 500)

def mission_1():
    setup_drive()
    # print("Mission 1")
    
    hub.imu.reset_heading(0)
    wait(200)

    drive_cm(12, 30, 50)
    gyro_turn(-45, mode="medium")
    run_motor_for_degrees(motor_c, 150, 300)     # move_sidearm_mission9(port.C, 150, 720, 1000)
    drive_cm(39, 30, 50)

    motor_c.run_until_stalled(-100,Stop.BRAKE,50)    # move_sidearm_mission9(port.C, -120, 100, 100)
    run_motor_for_degrees(motor_d, -500, 500)   
    run_motor_for_degrees(motor_c, 180, 1000) 
    drive_cm_stall(-15, 30, 20)
    drive_cm_stall(10, 30, 10)
    run_motor_for_degrees(motor_d, 600, 1000)     # move_sidearm_mission9(port.D, 720, 360, 1000)
    drive_cm(-30, 30, 100)
def mission_2():
    setup_drive()
    # print("Mission 2")

    hub.imu.reset_heading(0)
    wait(200)

    # drive_cm(37, 30, 50)

    # # SPIKE had a complex stall-detect version; here is a simpler "repeat wiggle" version:
    # # for _ in range(3):  # repetitions=3
    # #     run_motor_for_degrees(motor_c, -180, 750)
    # #     run_motor_for_degrees(motor_c, 180, 750)

    # drive_cm(-15.5, 30, 50)

    # hub.imu.reset_heading(0)
    # wait(200)
    # gyro_turn(-45, mode="medium")
    # gyro_turn(-45, mode="medium")
    drive_cm(100, 100, 50)
    drive_cm(100, 100, 50)

def mission_3():
    # This matches your “Challenge H 90” style (your Mission 3 file)
    setup_drive()
    # print("Mission 3")

    drive_cm(41, 10, 10)
    drive_cm(-8, 30, 15)
    drive_cm(18.5, 30, 10)
    drive_cm(-4, 30, 10)
    motor_c.run_until_stalled(-200, then=Stop.BRAKE, duty_limit=35)
    drive_cm(2, 30, 10)
    motor_d.run_until_stalled(-300, then=Stop.BRAKE, duty_limit=50)
    motor_c.run_until_stalled(200, then=Stop.BRAKE, duty_limit=60)
    drive_cm(-41, 30, 10)
 
    #run_motor_for_degrees(motor_c, 600, speed: 50, accel: 100, stop=Stop.HOLD):
     
    
def mission_4():
    setup_drive()
    # print("Mission 4")


    drive_cm(69, 30, 50)
    gyro_turn(-43)

    drive_cm(21, 20, 30)

    # lift_arm(port.D , lift_arm_degrees=180)
    run_motor_for_degrees(motor_d, 180, 1000)

    # drop_arm(port.C , 150) then 200 fast
    run_motor_for_degrees(motor_c, 150, 300)
    wait(1000)
    run_motor_for_degrees(motor_c, 200, 1000)
    #motor_c.run_until_stalled(200, then=Stop.BRAKE, duty_limit=60)

    # lift_arm(port.C , -300 slow)
    run_motor_for_degrees(motor_c, -300, 500)

    drive_cm(-22, 17, 500)
    gyro_turn(-120, mode="fast")

    drive_cm(61, 30, 500)

def mission_5():
    # print("Mission 5#")
    setup_drive()
   
    hub.imu.reset_heading(0)
    wait(200)

    drive_cm(79, 30, 50)
    gyro_turn(90)
    motor_c.run_until_stalled(600, then=Stop.BRAKE, duty_limit=35)
    drive_cm(13, 30, 50)
    run_motor_for_degrees(motor_c, -400, 300)
    gyro_turn(45)
    motor_d.run_until_stalled(-600, then=Stop.BRAKE, duty_limit=35)
    drive_cm(18, 30, 50)
    motor_d.run_until_stalled(500, then=Stop.BRAKE, duty_limit=60)

    #run_motor_for_degrees(motor_d, 200, 500)
    gyro_turn(-20, mode="fast")

def mission_6():
    setup_drive()
    drive_cm(50, 100, 50)
    drive_cm(-50, 30, 50)
    
def mission_7():
    setup_drive()
    # print("Mission 7")

MISSION_COLORS = [
    Color.RED,     # Mission 0
    Color.ORANGE,  # Mission 1
    Color.YELLOW,  # Mission 2
    Color.GREEN,   # Mission 3
    Color.BLUE,    # Mission 4
    Color.BLACK,   # Mission 5
    Color.MAGENTA, # Mission 6
    Color.BROWN,    # Mission 7
]
POLL_MS = 50
def set_mission_light(index):
    hub.light.on(MISSION_COLORS[index])

# -----------------------------
# Button-to-mission launcher
# -----------------------------
MISSIONS = [
    ("Mission 0", mission_0),
    ("Mission 1", mission_1),
    ("Mission 2", mission_2),
    ("Mission 3", mission_3),
    ("Mission 4", mission_4),
    ("Mission 5", mission_5),
    ("Mission 6", mission_6),
    ("Mission 7", mission_7)
]

# -----------------------------
# Selector UI:
# LEFT/RIGHT = choose 0–5
# START = Bluetooth button
# -----------------------------
def beep_selection(index):
    for _ in range(index + 1):
        hub.speaker.beep(700, 60)
        wait(80)

def show_selection(index):
    name = MISSIONS[index][0]
    # print("Selected:", name)
    set_mission_light(index)
    #beep_selection(index)

def wait_release_all():
    while any(b in hub.buttons.pressed() for b in [Button.LEFT, Button.RIGHT, Button.CENTER]):
        wait(20)

def main():
    selected = 0
    # print("READY.")
    # print("LEFT/RIGHT = choose Mission 0–5")
    # print("BLUETOOTH = START immediately")

    show_selection(selected)

    right_hold_time = 0

    while True:
        emergency_stop_check()
        pressed = hub.buttons.pressed()

        # Bluetooth button = immediate START (press to run selected mission)
        if Button.BLUETOOTH in pressed:
            name, fn = MISSIONS[selected]
            # print("STARTING (Bluetooth):", name)
            hub.speaker.beep(1000, 200)
            hub.light.on(Color.WHITE)   # running indicator
            wait(200)

            try:
                fn()
                hub.speaker.beep(600, 150)  # done
            except Exception as e:
                # print("Error:", e)
                hub.speaker.beep(200, 500)

            # Reset after mission
            right_hold_time = 0
            set_mission_light(selected)
            show_selection(selected)

            # Wait until Bluetooth is released to avoid repeated starts
            while Button.BLUETOOTH in hub.buttons.pressed():
                wait(20)

            continue

        # ---------- RIGHT button logic(tap only) ----------
        if Button.RIGHT in pressed:
            selected = (selected + 1) % len(MISSIONS)
            show_selection(selected)
            while Button.RIGHT in hub.buttons.pressed():
                wait(20)

        # ---------- LEFT button (tap only) ----------
        if Button.LEFT in pressed:
            selected = (selected - 1) % len(MISSIONS)
            show_selection(selected)
            while Button.LEFT in hub.buttons.pressed():
                wait(20)

        wait(POLL_MS)

main()
