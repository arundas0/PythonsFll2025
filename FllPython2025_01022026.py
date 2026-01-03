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
AXLE_TRACK_MM = 140 # IMPORTANT: set this to your real wheel-to-wheel distance

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

def setup_drive(back=1):
    
    if back > 0:
       robot.straight(-10)
    
    robot.reset()
    # “Default-ish” settings; missions override per move
    robot.settings(straight_speed=300, straight_acceleration=300)
    beep_ok()

def gyro_turn_phase1(target_angle: float, mode: str = "medium", settle_tol: float = 0.25, settle_timeout_ms: int = 1500):
    """
    Improved Hybrid Turn for Pybricks 2025.
    target_angle: Degrees to turn from current heading (+ CW, - CCW).
    """
    # Speed profiles based on 2025 competition standards
    turn_rates = {"slow": 80, "medium": 150, "fast": 220}
    turn_rate = turn_rates.get(mode, 150)

    # Safety and Initialization
    emergency_stop_check()
    hub.imu.reset_heading(0)
    wait(100) # Required for IMU stabilization

    # --- Phase A: Bulk Turn ---
    # Configure DriveBase and execute the majority of the turn
    robot.settings(turn_rate=turn_rate)
    robot.turn(target_angle)

    # --- Phase B: Accurate Settle Loop ---
    sw = StopWatch()
    current_h = hub.imu.heading()
    while sw.time() < settle_timeout_ms:
        emergency_stop_check()
       
        # Calculate Error with Shortest Path Logic
        # Normalizes to range (-180, 180) to handle wrap-around issues
        current_h = hub.imu.heading()
        #err = (target_angle - current_h + 180) % 360 - 180
        err = target_angle - current_h
        # print("Target :",target_angle, "Head Angle : ",current_h, "Error :",err)
        # Exit if within tolerance and the robot has physically stopped vibrating
        if abs(err) <= settle_tol:
            print("Found the optimal result")
            break
        if abs(err) < 1 :
            # Fine-Adjustment Pulses
            # Uses lower speed and active braking for precision
            nudge_speed = 45
            direction = 1 if err > 0 else -1
           
            left_motor.run(direction * nudge_speed)
            right_motor.run(-direction * nudge_speed)
            wait(10)      # Very short pulse
            left_motor.brake()
            right_motor.brake()
            wait(40)      # Settling time for IMU reading
        else:
            robot.turn(err)
            wait(40)

    # Ensure motors are fully stopped at the end
    robot.stop()
    # print the final result
    print("Target:",target_angle, "Head Angle : ",current_h, "Error :",err)

def gyro_turn(target_angle: float,
              mode: str = "medium",
              settle_tol: float = 2.0,
              settle_timeout_ms: int = 3000):
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
        "medium": 140,
        "fast": 200,
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
        "slow": 0,
        "medium": 3,
        "fast": 5,
    }.get(mode, 3)

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
    h_after_bulk = hub.imu.heading()
    print("bulk_cmd:", bulk, "heading_after_bulk:", h_after_bulk)

    # OPTIONAL: print a suggested axle-track scale factor if bulk is very off
    # (useful for calibration; doesn't change behavior)
    if abs(bulk) >= 30 and abs(h_after_bulk) > 1:
        scale = abs(bulk) / abs(h_after_bulk)
        print("Suggested AXLE_TRACK_MM *= ~", scale)

    # --- Phase B: IMU settle (fine correction) ---
    sw = StopWatch()
    sw.reset()

    # Pulse-based settle is more stable than continuous run (less overshoot)
    while True:
        emergency_stop_check()

        h = hub.imu.heading()
        
        err = target_angle - h
        print("nudge:TargetAngle", target_angle, "heading:", h,"err:",err)

        if abs(err) <= settle_tol:
            break

        if sw.time() > settle_timeout_ms:
            # give up but stop safely
            break
        # BIG error → don't creep, just turn again
        if abs(err) > 5:
            print("robot:secondaryturn", err * 0.8) 
            robot.turn(err * 0.8)   # take most of it, leave a little margin
            continue

        # SMALL error → fine nudge
        speed = min(turn_rate, max(60, int(abs(err) * 6)))
    
        direction = 1 if err > 0 else -1
        print("nudge:speed", speed, "err:", err,"turn_rate:",turn_rate,"direction:",direction)

        left_motor.run(direction * speed)
        right_motor.run(-direction * speed)
        wait(40)                    # longer pulse
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
    setup_drive(0)
    # print("Mission 0")
   
    #hub.imu.reset_heading(0)
   # wait(200)

    drive_cm(25, 30, 50)
    # gyro_turn(-12)
    #wait(200)
    gyro_turn_phase1(-25,settle_timeout_ms=1000)
    #wait(200)

    drive_cm(44, 50, 50)
    wait(200)
    #gyro_turn(55, mode="slow")
    gyro_turn_phase1(69,mode="slow")
   
    wait(200)
    drive_cm(14, 30, 20)
   
    run_motor_for_degrees(motor_d, -1000, 1000) # SPIKE: move_sidearm_mission9(port.D, -1000, 1000, 500)
    run_motor_for_degrees(motor_c, 50, 500) # SPIKE: move_sidearm_mission9(port.C, 100, 1000, 500)
    # print("Turn completed")
    gyro_turn_phase1(-40, mode="medium")
   
    drive_cm(-60, 100, 500)
    gyro_turn_phase1(-90,mode="fast",settle_timeout_ms=1000)
    #gyro_turn(-90, mode="fast")
    drive_cm(-40, 100, 500)
    print("Total motor run time,mission0:", watch.time(), "ms")
   
def mission_1():
    setup_drive(0)
    # print("Mission 1")
   
    hub.imu.reset_heading(0)
    wait(200)

    drive_cm(12, 30, 50)
    gyro_turn_phase1(-45, mode="fast")
    run_motor_for_degrees(motor_c, 150, 300)     # move_sidearm_mission9(port.C, 150, 720, 1000)
    drive_cm(39, 30, 50)

    motor_c.run_until_stalled(-100,Stop.BRAKE,50)    # move_sidearm_mission9(port.C, -120, 100, 100)
    run_motor_for_degrees(motor_d, -500, 500)  
    run_motor_for_degrees(motor_c, 180, 1000)
    drive_cm_stall(-15, 30, 20)
    drive_cm_stall(10, 30, 10)
    run_motor_for_degrees(motor_d, 600, 1000)     # move_sidearm_mission9(port.D, 720, 360, 1000)
    drive_cm(-35, 30, 100)
    print("Total motor run time,mission1:", watch.time(), "ms")

def mission_2():
    setup_drive(0)

    # SPIKE had a complex stall-detect version; here is a simpler "repeat wiggle" version:
    for _ in range(3):  # repetitions=3
        run_motor_for_degrees(motor_d, 180, 270)
        run_motor_for_degrees(motor_d, -180, 270)
    print("Total motor run time,mission2:", watch.time(), "ms")

def mission_3():
    setup_drive(0)
    drive_cm(200, 30, 100)
    print("Total motor run time,mission3:", watch.time(), "ms")

def mission_4(): #Adi Mission - Get the broom Start from left edge of E
    setup_drive()

    drive_cm(69, 30, 50)
    gyro_turn(-45)

    drive_cm(21, 20, 30)

    run_motor_for_degrees(motor_d, 180, 1000) #raise garden bed
    motor_c.run_until_stalled(200, then=Stop.BRAKE, duty_limit=60) #drop net
    #motor_c.run_until_stalled(-200, then=Stop.BRAKE, duty_limit=80) #raise net to get broom 
    run_motor_for_degrees(motor_c, -180, 300) #raise brush
 
    drive_cm(-22, 17, 500) #back away from garden bed
    gyro_turn(-120, mode="fast") #turn towards home

    drive_cm(61, 30, 500) #drive towards home

def mission_5():
    setup_drive(0)

    drive_cm(42, 20, 20)
    drive_cm(-12, 10, 15)
    drive_cm(19.5, 30, 30)
    motor_c.run_until_stalled(-300, then=Stop.BRAKE, duty_limit=50) #drop gear mechanism
    drive_cm(2, 30, 10)
    motor_d.run_until_stalled(-500, then=Stop.BRAKE, duty_limit=60) #run gear mechanism
    motor_c.run_until_stalled(400, then=Stop.BRAKE, duty_limit=100) #raise gear mechanism //TODO: Change to turn motor
    drive_cm(-50, 30, 50) #back to home
 
def mission_6(): #Deposit Stuff
    setup_drive(0)
    drive_cm(50, 100, 50) #drop stuff in deposit zone
    drive_cm(-50, 30, 50) #back to home


def mission_7(): #Raise Hell
    setup_drive()
   
    hub.imu.reset_heading(0)
    wait(100)

    drive_cm(79, 30, 50) #move forward to wall
    gyro_turn(90)
    motor_c.run_until_stalled(600, then=Stop.BRAKE, duty_limit=35) #lower arm trolley
    drive_cm(13, 30, 50) 
    run_motor_for_degrees(motor_c, -400, 300) #raise arm trolley
    gyro_turn(45) #turn towards dinaosaur fossil
    motor_d.run_until_stalled(-600, then=Stop.BRAKE, duty_limit=35) #lower arm to hit fossil
    drive_cm(18, 30, 50)

    run_motor_for_degrees(motor_d, 300, 500) #raise arm after hitting fossil
    gyro_turn(-30, mode="medium")

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
    hub.display.char(str(index))

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
