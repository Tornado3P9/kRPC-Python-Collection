import math
import os
import time
import krpc
import argparse
from simple_pid import PID


def commandLine() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rocket launch and circularization burn script"
    )
    parser.add_argument("-V", "--version", action="version", version="%(prog)s 1.0")
    parser.add_argument(
        "--target",
        type=int,
        required=False,
        default=90000,
        help="target altitude (default: 90000)",
    )
    parser.add_argument(
        "--compass",
        type=int,
        required=False,
        default=90,
        help="horizontal compass heading in 0°-360° counterclockwise = azimuth (default: 90)",
    )
    parser.add_argument(
        "--auto_throttle",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="Auto Throttle (default: True)",
    )
    parser.add_argument(
        "--ag5",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
        help="A boolean flag for Action Group 5: escape tower or fairing deployment above 60 km (default: False)",
    )
    return parser.parse_args()


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
        conn = krpc.connect(name="Launch into orbit")
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

        # Calculate adjusted heading
        kerbin_radius = 600_000
        target_orbit = kerbin_radius + target_altitude
        Vdest = target_orbital_velocity(target_orbit)

        Vrot = kerbin_surface_rotation_speed(
            launch_latitude=0
        )  # latitude 0 degrees = equator

        azimuth_deg = compass

        launch_azimuth_deg = calculate_launch_azimuth_deg(azimuth_deg, Vdest, Vrot)
        if 90 < azimuth_deg <= 270:
            launch_azimuth_deg = 180 + launch_azimuth_deg
        elif 270 < azimuth_deg < 360:
            launch_azimuth_deg = 360 + launch_azimuth_deg
        elif (0 <= azimuth_deg < 90) and (launch_azimuth_deg < 0):
            launch_azimuth_deg = 360 + launch_azimuth_deg

        print("#####################")
        print(f"Target orbital height: {target_altitude / 1000:.2f} km")
        print(f"Target orbital velocity: {Vdest:.2f} m/s")
        print(f"Kerbin surface rotation speed: {Vrot:.2f} m/s")
        print(f"Launch azimuth: {launch_azimuth_deg:.2f}° (for {azimuth_deg:.2f}°)")
        print("#####################")

        # Launch sequence
        launch_sequence(vessel)

        # Begin roll program for reorienting the vessel pitch and heading
        roll_program(vessel, launch_azimuth_deg)

        # Unhide UI elements
        panel.visible = True
        button.visible = True

        # Set up the PID controller
        pid = PID(Kp=0.005, Ki=0.5, Kd=0, setpoint=0, auto_mode=False)
        pid.output_limits = (0, 1)  # Throttle limits
        target_twr = 1.6  # Auto throttle constant
        pid_init = True  # In order to prevent accumulated errors before starting the control process,
        # i want to Start the controller in manual mode and switch to auto mode when the system is ready.

        # Main ascent loop
        print("Gravity turn")
        atc = 0  # Auto stage counter
        while True:
            pitch = gravity_turn(altitude())
            pitch = max(pitch, 2.0)
            vessel.auto_pilot.target_pitch_and_heading(pitch, launch_azimuth_deg)

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
            if atc % 10 == 0:
                # Between each staging let one second pass to avoid rapid staging
                if vessel.thrust == 0:
                    print("Thrust is zero, activating next stage.")
                    vessel.control.activate_next_stage()
                    atc = 0  # this line can be removed

                # Also check and activate action group 5
                ag5 = check_and_activate_ag5(vessel, altitude(), ag5)

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
            # Check and activate action group 5 if not already in the "Main ascent loop"
            if ag5:
                ag5 = check_and_activate_ag5(vessel, altitude(), ag5)
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
        node = vessel.control.add_node(
            conn.space_center.ut + vessel.orbit.time_to_apoapsis, prograde=delta_v
        )
        # Calculate burn time (using rocket equation)
        F = vessel.available_thrust
        Isp = vessel.specific_impulse * vessel.orbit.body.surface_gravity
        m0 = vessel.mass
        m1 = m0 / math.exp(delta_v / Isp)
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
        time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, "time_to_apoapsis")
        while time_to_apoapsis() - half_burn_time > 0:
            time.sleep(0.1)

        print("Executing burn")
        vessel.control.throttle = 1.0
        time.sleep(burn_time)
        vessel.control.throttle = 0.0

        print("Burn finished")
        time_to_apoapsis.remove()
        node.remove()  # Remove maneuver node
        vessel.auto_pilot.disengage()  # Give control back to the pilot
        vessel.control.sas = True  # Activate SAS
        vessel.control.sas_mode = conn.space_center.SASMode.stability_assist
        print("Waiting for steering to settle down")
        time.sleep(3)
        finalize_launch(vessel.orbit)

    except krpc.error.RPCError as e:
        print(f"A KRPC error occurred: {e}")
    except Exception as e:
        print(f"An unexpected error occurred (note: keep navball extended): {e}")


def clear_screen() -> None:
    # print("\033c", end="")  # Clear screen equivalent on Unix-like systems
    try:
        os.system("clear" if os.name == "posix" else "cls")
    except OSError as e:
        print(f"An OSError occurred in clear_screen(): {e}")
    except Exception as e:
        print(f"An error occurred in clear_screen(): {e}")


def str2bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "y", "1"):
        return True
    elif v.lower() in ("no", "false", "f", "n", "0"):
        return False
    else:
        raise argparse.ArgumentTypeError("Boolean value expected.")


def setup_ui(conn, auto_throttle) -> tuple:
    # Access the stock user interface (UI) canvas
    canvas = conn.ui.stock_canvas
    screen_size = canvas.rect_transform.size

    # Add a panel to contain the UI elements
    panel = canvas.add_panel()
    rect = panel.rect_transform
    rect.size = (200, 85)
    rect.position = (screen_size[0] / 4, screen_size[1] / 2.3)

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
    button_clicked = conn.add_stream(getattr, button, "clicked")

    return panel, button, text, button_clicked


def setup_telemetry_streams(conn, vessel) -> tuple:
    # ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
    apoapsis = conn.add_stream(getattr, vessel.orbit, "apoapsis_altitude")
    return altitude, apoapsis


def pre_launch_setup(vessel, initial_throttle) -> None:
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = initial_throttle
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(
        90, vessel.flight().heading
    )  # (pitch, yaw)
    vessel.auto_pilot.target_roll = vessel.flight().roll


def target_orbital_velocity(target_orbit):
    G = 6.674 * 10**-11  # gravitational constant
    M = 5.2915793 * 10**22  # mass of Kerbin in kg
    r = target_orbit  # distance from the center of Kerbin in meters
    return math.sqrt(G * M / r)


def kerbin_surface_rotation_speed(launch_latitude=0):
    # Circumference of Kerbin's Equator is (2*π*600000m)
    # Time of Kerbin Sidereal Day = 21549.425 seconds, or 5 hours, 59 minutes, 9.425 seconds
    # VRot(φ) in m/s = ((2*π*r)/T)*cos(φ)
    return ((2 * math.pi * 600_000) / 21549.425) * math.cos(
        math.radians(launch_latitude)
    )


def calculate_launch_azimuth_deg(azimuth_deg, Vdest, Vrot):
    """_summary_
    Vlaunch = our horizontal delta-V requirement @ our rotational (compass) launch azimuth.

    Vlaunch = Vdest - Vrot.

    In order to add these vectors we split them into their X (east/west) and Y (north/south) components.

    Obtaining the X component of a vector is as simple as multiplying its magnitude by the sine of its direction. Therefore:

    Vrotx = 174.9422 m/s * sin(90) = 174.9422 m/s
    Vdestx = 2169.976 m/s * sin(45) = 1534.407 m/s

    Vlaunchx = Vdestx - Vrotx
    Vlaunchx = 1534.407 - 174.9422 = 1359.465 m/s

    And for the Y component of a vector, the magnitude by the cosine of its direction. Thus:

    Vroty = 174.9422 m/s * cos(90) = 0 m/s
    Vdesty = 2169.976 m/s * cos(45) = 1534.402

    Vlaunchy = Vdesty - Vroty
    Vlaunchy = 1534.402 - 0 = 1534.402 m/s

    We finally have X and Y components of our rotational launch vector.
    By combining them, we can finally see how much horizontal dV we need and what way to point it!
    It is at this point where I finally get to call in my good friend Pythagoreas.
    Since our X and Y velocities are perpendicular, we can treat them as sides of a right triangle.

    Vlaunch = (Vlaunchx^2 + Vlaunchy^2)^(1/2)
    Vlaunch = 2050.009 m/s

    And the launch azimuth, the heading from North we need to launch?

    tan(β) = Vlaunchx/Vlaunchy
    β = arctan(Vlaunchx/Vlaunchy)
    β = arctan(1359.465/1534.402)

    β = 41.540 degrees.
    """

    return math.degrees(
        math.atan(
            (
                (Vdest * math.sin(math.radians(azimuth_deg)))
                - (Vrot * math.cos(math.radians(0)))
            )
            / (Vdest * math.cos(math.radians(azimuth_deg)))
        )
    )


def launch_sequence(vessel) -> None:
    print("...3")
    time.sleep(1)
    print("...2")
    time.sleep(1)
    print("...1")
    time.sleep(1)
    print("Launch!")
    vessel.control.activate_next_stage()


def roll_program(vessel, launch_heading) -> None:
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
    vessel.auto_pilot.target_pitch_and_heading(90, launch_heading)


def gravity_turn(altitude) -> float:
    return 1.48272e-8 * altitude**2 - 0.00229755 * altitude + 90


def twr_error(vessel, target_twr) -> float:
    thrust = vessel.available_thrust * vessel.control.throttle
    weight = vessel.mass * vessel.orbit.body.surface_gravity
    current_twr = thrust / weight
    return current_twr - target_twr


def check_and_activate_ag5(vessel, altitude, ag5):
    if ag5 and altitude > 65_000:
        vessel.control.toggle_action_group(5)
        print("Action group 5 activated above 65 km")
        return False
    return ag5


def finalize_launch(orbit) -> None:
    print(
        f"  Apoapsis: {orbit.apoapsis_altitude / 1000:.3f} km, Periapsis: {orbit.periapsis_altitude / 1000:.3f} km"
        f"\n  Semi-major axis: {orbit.semi_major_axis / 1000:.3f} km, Eccentricity: {orbit.eccentricity:.4f}"
        f"\n  Inclination: {orbit.inclination:.2f} degrees"
    )
    print("Launch complete")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nLaunch sequence interrupted by user")
