import math
import time
import krpc
import argparse


def str2bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def commandLine() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description = "Rocket launch and circularization burn script")
    parser.add_argument("-V", "--version", action='version', version='%(prog)s 1.0')
    parser.add_argument('--target', type=int, help='target altitude (default: 85000)', required = False, default = 85000) # metavar='target_altitude' instead of 'TA'
    parser.add_argument('--compass', type=int, help='horizontal compass direction (default: 90)', required = False, default = 90)
    parser.add_argument('--throttle', type=str2bool, nargs='?', const=True, default=False, help='Auto Throttle (default: False)')
    parser.add_argument('--ag5', type=str2bool, nargs='?', const=True, default=False, help='A boolean flag for Action Group 5 (default: False)') # escape tower or fairing deployment
    return parser.parse_args()


def gravity_turn(altitude) -> float:
    return 1.48272E-8 * altitude**2 - 0.00229755 * altitude + 90


def is_stage_empty(conn) -> bool:
    vessel = conn.space_center.active_vessel
    for part in vessel.parts.in_stage(vessel.control.current_stage):
        # for resource in part.resources.all:
        #     if resource.amount > 0:
        #         return False
        if part.resources.amount('LiquidFuel') > 0:
            return False
    return True
# if is_stage_empty(conn):
#     print("Staging...")
#     vessel.control.activate_next_stage()


def setup_ui(conn, auto_throttle) -> tuple:
    # Access the stock user interface (UI) canvas
    canvas = conn.ui.stock_canvas
    screen_size = canvas.rect_transform.size

    # Add a panel to contain the UI elements
    panel = canvas.add_panel()
    rect = panel.rect_transform
    rect.size = (200, 85)
    rect.position = (screen_size[0]/4, screen_size[1]/2.3)

    # Add a button and text
    button = panel.add_button("Off")
    button.rect_transform.position = (0, -12)
    text = panel.add_text("Auto Throttle")
    text.rect_transform.position = (0, 12)
    text.color = (1, 1, 1)
    text.size = 18
    
    # Hide UI elements for the beginning
    panel.visible = False
    button.visible = False

    # Set up a stream to monitor the throttle button
    button_clicked = conn.add_stream(getattr, button, 'clicked')
    
    return panel, button, text, button_clicked


def setup_telemetry_streams(conn, vessel) -> tuple:
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    # stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
    # srb_fuel = conn.add_stream(stage_2_resources.amount, 'SolidFuel')
    return ut, altitude, apoapsis


def pre_launch_setup(vessel) -> None:
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, vessel.flight().heading)  # (pitch, yaw)
    vessel.auto_pilot.target_roll = vessel.flight().roll


def launch_sequence(vessel) -> None:
    print("\033c", end="")  # Clear screen equivalent
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Launch!')
    vessel.control.activate_next_stage()


def roll_program(vessel, compass) -> None:
    # After x seconds, set target pitch and heading ("activate roll program")
    count_seconds = 0
    while True:
        vertical_speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
        if vertical_speed >= 60:
            break
        if count_seconds > 15:
            break
        count_seconds += 1
        time.sleep(1)
    print("Roll")
    vessel.auto_pilot.target_roll = 0
    vessel.auto_pilot.target_pitch_and_heading(90, compass)


def main() -> None:
    # Parse command line arguments
    argument = commandLine()
    target_altitude = argument.target  # int
    compass = argument.compass         # int
    auto_throttle = argument.throttle  # bool
    ag5 = argument.ag5                 # bool

    # Connect to KRPC
    conn = krpc.connect(name='Launch into orbit')
    vessel = conn.space_center.active_vessel
    
    # Setup UI
    panel, button, text, button_clicked = setup_ui(conn, auto_throttle)
    
    # Setup telemetry streams
    ut, altitude, apoapsis = setup_telemetry_streams(conn, vessel)
    
    # Pre-launch setup
    pre_launch_setup(vessel)
    
    # Launch sequence
    launch_sequence(vessel)

    # Begin roll program for reorienting the vessel pitch and heading
    roll_program(vessel, compass)

    # Unhide UI elements
    panel.visible = True
    button.visible = True

    # Main ascent loop
    print("Gravity turn")
    while True:
        pitch = gravity_turn(altitude())
        pitch = max(pitch, 2.0)
        if auto_throttle:
            throttle = max(0.55, (1/90) * pitch)
            vessel.control.throttle = throttle
        vessel.auto_pilot.target_pitch_and_heading(pitch, compass)

        # Handle the throttle button being clicked
        if button_clicked():
            auto_throttle = not auto_throttle
            button.text.content = "On" if auto_throttle else "Off"
            button.clicked = False
        
        # Decrease throttle when approaching target apoapsis
        if apoapsis() > target_altitude*0.95:
            print('Approaching target apoapsis')
            break
        
        time.sleep(0.5)
    
    # Stop the button_clicked stream (must happen before removing UI elements)
    button_clicked.remove()
    
    # Remove UI elements
    button.remove()
    text.remove()
    panel.remove()

    # Disable engines when target apoapsis is reached
    while apoapsis() < target_altitude:
        pass
    print('Target apoapsis reached')
    vessel.control.throttle = 0.0

    # Wait until out of atmosphere
    print('Coasting out of atmosphere')
    while altitude() < 70005:
        time.sleep(1)
        if altitude() > 62000 and ag5:
            vessel.control.toggle_action_group(5)
            print("Action group 5 activated above 62 km")
            ag5 = False

    # Plan circularization burn (using vis-viva equation)
    print('Planning circularization burn')
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate
    print(f"Circularization burn time: {burn_time:.2f}s")

    # Orientate ship
    print('Orientating ship for circularization burn')
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    time.sleep(5)

    # Wait until burn
    print('Waiting until circularization burn')
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
    lead_time = 20
    conn.space_center.warp_to(burn_ut - lead_time)
    print('20 seconds...Ready to execute burn')

    # Execute burn
    time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burn_time/2.) > 0:
        pass

    print('Executing burn')
    vessel.control.throttle = 1.0
    time.sleep(burn_time)
    vessel.control.throttle = 0.0
    current_orientation = vessel.flight().direction
    vessel.auto_pilot.target_direction = current_orientation
    node.remove()  # Remove maneuver node
    print("Burn finished")
    
    # Finalize launch
    finalize_launch(vessel)


def finalize_launch(vessel) -> None:
    vessel.auto_pilot.disengage() # Give control back to the pilot
    vessel.control.sas = True  # Activate SAS
    print("Waiting for steering to settle down")
    time.sleep(3)
    print("Launch complete")


if __name__ == '__main__':
    main()
