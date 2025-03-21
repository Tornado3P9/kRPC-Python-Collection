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
    parser.add_argument('--target', type=int, help='target altitude (default: 90000)', required = False, default = 90000) # metavar='target_altitude' instead of 'TA'
    parser.add_argument('--compass', type=int, help='horizontal compass direction in degrees (default: 90)', required = False, default = 90)
    parser.add_argument('--auto_throttle', type=str2bool, nargs='?', const=True, default=False, help='Auto Throttle (default: False)')
    parser.add_argument('--ag5', type=str2bool, nargs='?', const=True, default=False, help='A boolean flag for Action Group 5: escape tower or fairing deployment at 65 km (default: False)')
    return parser.parse_args()


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
    button = panel.add_button("On" if auto_throttle else "Off")
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
        if vertical_speed >= 60 or count_seconds > 15:
            break
        count_seconds += 1
        time.sleep(1)
    print("Roll")
    vessel.auto_pilot.target_roll = 0
    vessel.auto_pilot.target_pitch_and_heading(90, compass)


def gravity_turn(altitude) -> float:
    return 1.48272E-8 * altitude**2 - 0.00229755 * altitude + 90


def auto_staging(vessel) -> None:
    if vessel.thrust == 0:
        print("Thrust is zero, activating next stage.")
        vessel.control.activate_next_stage()
        time.sleep(1)  # Wait a bit to avoid rapid staging


def current_twr(vessel) -> float:
    """ calculate thrust-to-weight ratio """
    thrust = vessel.available_thrust
    weight = vessel.mass * vessel.orbit.body.surface_gravity
    theoretical_twr = thrust / weight
    return theoretical_twr * vessel.control.throttle


def main() -> None:
    try:
        # Parse command line arguments
        parsed_args = commandLine()
        target_altitude: int = parsed_args.target
        compass: int = parsed_args.compass
        auto_throttle: float = parsed_args.auto_throttle
        ag5: float = parsed_args.ag5

        # Connect to KRPC
        try:
            conn = krpc.connect(name='Launch into orbit')
        except krpc.error.RPCError as e:
            print(f"A KRPC error occurred during krpc.connect(): {e}")
            return
        except Exception as e:
            print(f"Failed to connect to KRPC: {e}")
            return
        
        # Select active vessel
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
            
            # Stop main loop and disable engines when target apoapsis is reached
            if apoapsis() > target_altitude:
                print('Target apoapsis reached')
                vessel.control.throttle = 0.0
                break
            
            # Handle auto staging once thrust is no longer generated
            auto_staging(vessel)
            
            time.sleep(0.1)
        
        # Stop the button_clicked stream (must happen before removing UI elements)
        button_clicked.remove()
        
        # Remove UI elements
        button.remove()
        text.remove()
        panel.remove()

        # Wait until out of atmosphere
        print('Coasting out of atmosphere')
        while altitude() < 70005:
            time.sleep(1)
            if altitude() > 65000 and ag5:
                vessel.control.toggle_action_group(5)
                print("Action group 5 activated above 65 km")
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
        lead_time = 40
        conn.space_center.warp_to(burn_ut - lead_time)
        print(f"{lead_time} seconds...Ready to execute burn")

        # Execute burn
        time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
        while time_to_apoapsis() - (burn_time/2.) > 0:
            time.sleep(0.1)

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
        
    except krpc.error.RPCError as e:
        print(f"A KRPC error occurred: {e}")
    except Exception as e:
        print(f"An unexpected error occurred (note: keep navball extended): {e}")


def finalize_launch(vessel) -> None:
    vessel.auto_pilot.disengage() # Give control back to the pilot
    vessel.control.sas = True  # Activate SAS
    print("Waiting for steering to settle down")
    time.sleep(3)
    orbit = vessel.orbit
    print(f"  Apoapsis: {orbit.apoapsis_altitude/1000:.3f} km, Periapsis: {orbit.periapsis_altitude/1000:.3f} km"
        f"\n  Semi-major axis: {orbit.semi_major_axis/1000:.3f} km, Eccentricity: {orbit.eccentricity:.4f}"
        f"\n  Inclination: {orbit.inclination:.2f} degrees")
    print("Launch complete")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nLaunch sequence interrupted by user.")
