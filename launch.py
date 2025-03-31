import math
import os
import time
import krpc
import argparse
from simple_pid import PID


def clear_screen() -> None:
    # print("\033c", end="")  # Clear screen equivalent on Unix-like systems
    try:
        os.system('clear' if os.name == 'posix' else 'cls')
    except OSError as e:
        print(f"An OSError occurred in clear_screen(): {e}")
    except Exception as e:
        print(f"An error occurred in clear_screen(): {e}")


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
    parser.add_argument('--target', type=int, required = False, default = 90000, help='target altitude (default: 90000)')
    parser.add_argument('--compass', type=int, required = False, default = 90, help='horizontal compass direction in degrees (default: 90)')
    parser.add_argument('--auto_throttle', type=str2bool, nargs='?', const=True, default=False, help='Auto Throttle (default: False)')
    parser.add_argument('--ag5', type=str2bool, nargs='?', const=True, default=False, help='A boolean flag for Action Group 5: escape tower or fairing deployment above 65 km (default: False)')
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
    # ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    return altitude, apoapsis


def pre_launch_setup(vessel, initial_throttle) -> None:
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = initial_throttle
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, vessel.flight().heading)  # (pitch, yaw)
    vessel.auto_pilot.target_roll = vessel.flight().roll


def launch_sequence(vessel) -> None:
    print("...3")
    time.sleep(1)
    print("...2")
    time.sleep(1)
    print("...1")
    time.sleep(1)
    print("Launch!")
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


def twr_error(vessel, target_twr) -> float:
    thrust = vessel.available_thrust * vessel.control.throttle
    weight = vessel.mass * vessel.orbit.body.surface_gravity
    current_twr = thrust / weight
    return current_twr - target_twr


def main() -> None:
    # Parse command line arguments
    parsed_args = commandLine()
    target_altitude: int = parsed_args.target
    compass: int = parsed_args.compass
    auto_throttle: bool = parsed_args.auto_throttle
    ag5: bool = parsed_args.ag5
    
    clear_screen()
    
    # Connect to KRPC
    try:
        conn = krpc.connect(name='Launch into orbit')
    except krpc.error.RPCError as e:
        print(f"A KRPC error occurred during krpc.connect(): {e}")
        return
    except Exception as e:
        print(f"Failed to connect to KRPC: {e}")
        return
    
    try:
        # Select active vessel
        vessel = conn.space_center.active_vessel
        
        # Setup UI
        panel, button, text, button_clicked = setup_ui(conn, auto_throttle)
        
        # Setup telemetry streams
        altitude, apoapsis = setup_telemetry_streams(conn, vessel)
        
        # Pre-launch setup
        initial_throttle = 1.0
        pre_launch_setup(vessel, initial_throttle)
        
        # Launch sequence
        launch_sequence(vessel)

        # Begin roll program for reorienting the vessel pitch and heading
        roll_program(vessel, compass)

        # Unhide UI elements
        panel.visible = True
        button.visible = True
        
        # Set up the PID controller
        pid = PID(Kp=0.005, Ki=0.5, Kd=0, setpoint=0, auto_mode=False)
        pid.output_limits = (0, 1)  # Throttle limits
        target_twr = 1.6  # Auto throttle constant
        pid_init = True # In order to prevent accumulated errors before starting the control process,
                        # i want to Start the controller in manual mode and switch to auto mode when the system is ready.
        
        # Main ascent loop
        print("Gravity turn")
        atc = 0  # Auto stage counter
        while True:
            pitch = gravity_turn(altitude())
            pitch = max(pitch, 2.0)
            vessel.auto_pilot.target_pitch_and_heading(pitch, compass)
            
            if auto_throttle:
                # Change throttle proportionally to the pitch of the rocket
                # new_throttle = max(0.55, (1/90) * pitch)
                # Change throttle by minimizing TWR error using "from scipy.optimize import minimize"
                # new_throttle = minimize(twr_error, vessel.control.throttle, args=(vessel, target_twr), bounds=[(0, 1)]).x[0]
                # Change throttle by minimizing TWR error using "from simple_pid import PID"
                if pid_init:
                    pid.set_auto_mode(True, last_output=initial_throttle)
                    pid_init = False
                new_error = twr_error(vessel, target_twr)
                new_throttle = pid(new_error)
                # Set the throttle
                vessel.control.throttle = new_throttle

            # Handle the throttle button being clicked
            if button_clicked():
                auto_throttle = not auto_throttle
                button.text.content = "On" if auto_throttle else "Off"
                button.clicked = False
            
            # Stop main loop and disable engines when target apoapsis is reached
            if apoapsis() > target_altitude:
                print("Target apoapsis reached")
                vessel.control.throttle = 0.0
                pid.set_auto_mode(False)
                break
            
            # Handle auto staging once thrust is no longer generated
            atc += 1
            if vessel.thrust == 0 and atc % 10 == 0:
                atc = 0
                print("Thrust is zero, activating next stage.")
                vessel.control.activate_next_stage()
                # Between each staging let one second pass to avoid rapid staging
            
            time.sleep(0.1)
        
        # Stop Redundant Streams and Remove UI elements
        apoapsis.remove()
        button_clicked.remove()
        button.remove()
        text.remove()
        panel.remove()
        
        # Wait until out of atmosphere
        print("Coasting out of atmosphere")
        while altitude() < 70_050:
            if ag5 and altitude() > 65_000:
                vessel.control.toggle_action_group(5)
                print("Action group 5 activated above 65 km")
                ag5 = False
            time.sleep(1)
        altitude.remove()

        # Plan circularization burn (using vis-viva equation)
        print("Planning circularization burn")
        mu = vessel.orbit.body.gravitational_parameter
        r = vessel.orbit.apoapsis
        a1 = vessel.orbit.semi_major_axis
        a2 = r
        v1 = math.sqrt(mu * ((2 / r) - (1 / a1)))
        v2 = math.sqrt(mu * ((2 / r) - (1 / a2)))
        delta_v = v2 - v1
        node = vessel.control.add_node(conn.space_center.ut + vessel.orbit.time_to_apoapsis, prograde=delta_v)

        # Calculate burn time (using rocket equation)
        F = vessel.available_thrust
        Isp = vessel.specific_impulse * vessel.orbit.body.surface_gravity
        m0 = vessel.mass
        m1 = m0 / math.exp(delta_v/Isp)
        flow_rate = F / Isp
        burn_time = (m0 - m1) / flow_rate
        print(f"Circularization burn time: {burn_time:.2f}s")

        print("Set new ship orientation")
        vessel.auto_pilot.reference_frame = node.reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)

        print("Time warp to circularization maneuver")
        half_burn_time = burn_time / 2
        burn_ut = conn.space_center.ut + vessel.orbit.time_to_apoapsis - half_burn_time
        lead_time = 50
        conn.space_center.warp_to(burn_ut - lead_time)
        
        print(f"{lead_time} seconds...Ready to execute circularization burn")
        time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
        while time_to_apoapsis() - half_burn_time > 0:
            time.sleep(0.1)

        print('Executing burn')
        vessel.control.throttle = 1.0
        time.sleep(burn_time)
        vessel.control.throttle = 0.0
        
        print("Burn finished")
        time_to_apoapsis.remove()
        node.remove()  # Remove maneuver node
        vessel.auto_pilot.disengage() # Give control back to the pilot
        vessel.control.sas = True  # Activate SAS
        vessel.control.sas_mode = conn.space_center.SASMode.stability_assist
        print("Waiting for steering to settle down")
        time.sleep(3)
        finalize_launch(vessel.orbit)
        
    except krpc.error.RPCError as e:
        print(f"A KRPC error occurred: {e}")
    except Exception as e:
        print(f"An unexpected error occurred (note: keep navball extended): {e}")


def finalize_launch(orbit) -> None:
    print(f"  Apoapsis: {orbit.apoapsis_altitude/1000:.3f} km, Periapsis: {orbit.periapsis_altitude/1000:.3f} km"
        f"\n  Semi-major axis: {orbit.semi_major_axis/1000:.3f} km, Eccentricity: {orbit.eccentricity:.4f}"
        f"\n  Inclination: {orbit.inclination:.2f} degrees")
    print("Launch complete")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nLaunch sequence interrupted by user")
