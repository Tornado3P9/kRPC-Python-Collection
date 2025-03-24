import os
import krpc
import time
import math
from simple_pid import PID
import argparse


def clear_screen():
    # print("\033c", end="")  # Clear screen equivalent on Unix-like systems
    try:
        os.system('clear' if os.name == 'posix' else 'cls')
    except OSError as e:
        print(f"An OSError occurred in clear_screen(): {e}")
    except Exception as e:
        print(f"An error occurred in clear_screen(): {e}")


def str2bool(v: str) -> bool:
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def commandLine() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description = "Mun lander script")
    parser.add_argument("-V", "--version", action='version', version='%(prog)s 1.0')
    parser.add_argument('--radar', type=int, help='radar sensor height on lander (default: 2.7)', required = False, default = 2.7)
    parser.add_argument('--deorbit', type=str2bool, nargs='?', const=True, default=False, help='Auto deorbit the lander from a circular orbit (default: False)')
    return parser.parse_args()


def connect_to_krpc() -> krpc.client.Client:
    return krpc.connect(name='Mun Lander')


def setup_vessel(conn) -> krpc.services.spacecenter.Vessel:
    vessel = conn.space_center.active_vessel
    vessel.control.sas = True
    time.sleep(1)
    vessel.control.sas_mode = conn.space_center.SASMode.retrograde
    return vessel


# def semi_major_axis(T, M) -> float:
#     G = 6.67430e-11  # gravitational constant
#     a_cubed = (G * M * T**2) / (4 * math.pi**2)
#     a = a_cubed**(1/3)
#     return a
#     # Example usage:
#     # T = orbital period in seconds
#     # M = mass of the central body in kg
#     # semi_major_axis(T, M)


def calculate_deorbit_parameters(vessel, body) -> float:
    mu = body.gravitational_parameter  # GM in m^3/s^2
    body_radius = body.equatorial_radius
    T = vessel.orbit.period  # Orbital period of the orbit which the vessel is on
    a = ((mu * T**2) / (4 * math.pi**2))**(1/3)  # Calculate semi-major axis of current circular orbit
    r2 = a  # Current orbit radius
    r1 = body_radius / 5000  # Target periapsis for deorbit
    delta_v = math.sqrt(mu/r2) * (1 - math.sqrt((2 * r1) / (r1 + r2)))  # Delta_v2 https://www.wikiwand.com/en/articles/Hohmann_transfer_orbit
    return delta_v


def calculate_burn_time(vessel, delta_v, g) -> float:
    F = vessel.available_thrust  # Available thrust of the vessel
    Isp = vessel.specific_impulse * g  # Specific impulse adjusted for gravity
    m0 = vessel.mass  # Initial mass of the vessel
    m1 = m0 / math.exp(delta_v/Isp)  # Final mass after burn
    flow_rate = F / Isp  # Fuel flow rate
    return (m0 - m1) / flow_rate  # Calculate burn time


def execute_deorbit_burn(conn, vessel, burn_time) -> None:
    for i in range(5, 0, -1):
        print(f"{i}...")
        time.sleep(1)
    print('Executing deorbit maneuver')
    vessel.control.throttle = 1.0  # Set throttle to maximum
    vessel.control.sas_mode = conn.space_center.SASMode.stability_assist
    time.sleep(burn_time)  # Burn for the calculated time
    vessel.control.throttle = 0.0  # Cut throttle after burn
    vessel.control.sas_mode = conn.space_center.SASMode.retrograde  # Switch back to retrograde mode because it automatically switches to stability mode
    time.sleep(1)  # Wait for changes to take effect


def perform_suicide_burn(vessel, safety_d) -> None:
    state = "armed"
    while True:
        mass = vessel.mass
        thrust = vessel.available_thrust

        # Calculate velocity and altitude
        vertical_speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed  # Speed direction
        if vertical_speed > 0:
            # The vessel is moving up
            vessel.control.throttle = 0
            break  # if vessel is going up again, the suicide burn has obviously finished
        elif vertical_speed < 0:
            # The vessel is moving down
            velocity = -vessel.flight(vessel.orbit.body.reference_frame).speed
        altitude = vessel.flight().surface_altitude - safety_d
        if altitude < 0:
            vessel.control.throttle = 0
            break

        # Calculate required deceleration
        required_deceleration = velocity**2 / (2 * altitude)

        # Calculate throttle
        ideal_throttle = required_deceleration / (thrust / mass)

        # Ensure throttle is between 0 and 1
        ideal_throttle = max(0, min(1, ideal_throttle))

        # Calculate time to impact
        time_to_impact = altitude / abs(velocity)

        # Calculate burn time
        burn_time = abs((velocity / (thrust / mass)) * ideal_throttle)

        # Start burn at the last possible second
        if state == "armed":
            if time_to_impact <= burn_time:
                state = "active"
                vessel.control.throttle = ideal_throttle
            else:
                vessel.control.throttle = 0
        elif state == "active":
            vessel.control.throttle = ideal_throttle

        print(f"\rIdeal_thrtl: {ideal_throttle * 100:5.2f}%, tti: {time_to_impact:5.2f}s, burn_t: {burn_time:4.2f}s", end=' ')

        # Sleep for a short duration to prevent excessive CPU usage
        time.sleep(0.1)


def finalize_landing(conn, vessel, pid, name) -> None:
    print("\nPerforming final landing")
    vessel.control.sas_mode = conn.space_center.SASMode.radial
    throttle = 0.05  # initial throttle
    vessel.control.throttle = throttle
    
    # vessel.control.sas_mode = conn.space_center.SASMode.radial
    time_init = conn.space_center.ut
    change_sas_state = True

    while True:
        situation = vessel.situation
        if situation == conn.space_center.VesselSituation.landed:
            print(f"\nThe vessel has landed on {name}!")
            break
        # elif situation == conn.space_center.VesselSituation.splashed:
        #     print("The vessel has splashed down!")
        #     break
        else:
            print(f"\rVessel in flight at altitude (radar sensor): {vessel.flight().surface_altitude:.2f} m", end='  ')  # last value is lander radar height at vessel
            # when velocity low, retrograde marker becomes inactive, so wait a bit
            time_now = conn.space_center.ut
            if change_sas_state and (time_now - time_init) > 2.0:
                vessel.control.sas_mode = conn.space_center.SASMode.retrograde
                change_sas_state = False
            # Get the current vertical speed
            vertical_speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
            # Calculate the throttle using the PID controller
            throttle = pid(vertical_speed)
            # Apply the throttle
            vessel.control.throttle = throttle
            # Sleep for a short duration
            time.sleep(0.1)

    # Finalize landing
    vessel.control.throttle = 0
    vessel.control.sas_mode = conn.space_center.SASMode.stability_assist


def post_touch_down(vessel) -> None:
    time.sleep(3)
    print("AG0: Performing actions after landing")  # extend solar panels, ...
    vessel.control.toggle_action_group(0)


def main() -> None:
    argument = commandLine()
    radar_alt = argument.radar
    deorbit = argument.deorbit

    clear_screen()
    
    # safety_distance: stop 30 meters above ground, also taking into account that the radar is positioned at 2.7 meters on my lander
    safety_d = 30 + radar_alt

    try:
        conn = connect_to_krpc()
        vessel = setup_vessel(conn)
        body = vessel.orbit.body  # Solar body (e.g. Mun)
        g = body.surface_gravity  # in m/s^2

        # Set up the PID controller                                                                                                                                                               
        pid = PID(0.2, 0., 0., setpoint=-1)  # Tune these parameters                                                                                                                           
        pid.output_limits = (0, 1)  # Throttle limits

        if deorbit:
            delta_v = calculate_deorbit_parameters(vessel, body)  # using the Vis-Viva equation
            print(f"Delta-v required for the deorbit burn: {delta_v:.2f} m/s")
            burn_time = calculate_burn_time(vessel, delta_v, g)  # using the rocket equation
            print(f"Burn time: {burn_time:.2f} s")
            execute_deorbit_burn(conn, vessel, burn_time)
        perform_suicide_burn(vessel, safety_d)
        finalize_landing(conn, vessel, pid, body.name)
        post_touch_down(vessel)

    except ZeroDivisionError as e:
        print(f"\nZeroDivisionError -> probably empty fuel tank or engine deactivated: {e}")
    except KeyboardInterrupt:
        print("\nScript exited by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred (note: keep navball extended): {e}")


if __name__ == "__main__":
    main()
