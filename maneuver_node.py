import os
import krpc
import time
import math
import argparse


def clear_screen():
    # print("\033c", end="")  # Clear screen equivalent on Unix-like systems
    try:
        os.system('clear' if os.name == 'posix' else 'cls')
    except OSError as e:
        print(f"An OSError occurred in clear_screen(): {e}")
    except Exception as e:
        print(f"An error occurred in clear_screen(): {e}")


def commandLine() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description = "Execute Maneuver Node")
    parser.add_argument("-V", "--version", action='version', version='%(prog)s 1.0')
    parser.add_argument('--circularize_at', choices=['ap', 'pe'], default=None, help="Choose either 'ap' or 'pe'.")
    return parser.parse_args()


def execute_maneuver_node() -> None:
    argument = commandLine()
    conn = krpc.connect(name='Maneuver Execution')
    vessel = conn.space_center.active_vessel
    vessel.control.sas = False
    
    clear_screen()
    
    if argument.circularize_at:
        planning_circularization_burn(conn, vessel, argument.circularize_at)
    
    if not vessel.control.nodes:
        print("No maneuver node exists.")
        return None
    
    node = vessel.control.nodes[0]
    start_time = calculate_start_time(conn, node)

    conn.space_center.warp_to(start_time - 30)
    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = node.burn_vector(node.reference_frame)
    print("Waiting until maneuver start...")

    countdown_to_maneuver(conn, start_time)

    vessel.control.throttle = 1.0
    original_vector = node.burn_vector(node.reference_frame)
    print("Maneuver in progress...")
    
    while not is_maneuver_complete(vessel, original_vector):
        time.sleep(0.1)

    vessel.control.throttle = 0.0
    vessel.auto_pilot.disengage()
    node.remove()
    vessel.control.sas = True
    time.sleep(1)
    print("Script exited.")


def planning_circularization_burn(conn, vessel, option) -> None:
    print('Planning circularization burn')
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis if option == 'ap' else vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu * ((2 / r) - (1 / a1)))
    v2 = math.sqrt(mu * ((2 / r) - (1 / a2)))
    delta_v = v2 - v1
    time_to_burn = vessel.orbit.time_to_apoapsis if option == 'ap' else vessel.orbit.time_to_periapsis
    vessel.control.add_node(conn.space_center.ut + time_to_burn, prograde=delta_v)
    time.sleep(1)


def calculate_start_time(conn, node) -> float:
    burn_time = calculate_burn_time(conn, node)
    return conn.space_center.ut + node.time_to - burn_time / 2


def calculate_burn_time(conn, node) -> float:
    dV = node.delta_v
    vessel = conn.space_center.active_vessel
    g0 = vessel.orbit.body.surface_gravity

    active_engines = [engine for engine in vessel.parts.engines if engine.active]
    total_isp = sum(engine.specific_impulse for engine in active_engines)
    isp = total_isp / len(active_engines) if active_engines else 0
    
    if isp == 0:
        raise ZeroDivisionError("No active engines with ISP found.")
    
    print("Available Engines:")
    for engine in active_engines:
        print(f'  Engine: {engine.part.title}, ISP: {engine.specific_impulse}')
        
    mf = vessel.mass / math.exp(dV / (isp * g0))
    fuel_flow = vessel.available_thrust / (isp * g0)
    burn_time = (vessel.mass - mf) / fuel_flow
    
    print(f"Maneuver duration: {burn_time:.2f}s.")
    return burn_time


def countdown_to_maneuver(conn, start_time) -> None:
    for i in range(5, 0, -1):
        while conn.space_center.ut < start_time - i:
            time.sleep(0.1)
        print(f"...{i}")


def is_maneuver_complete(vessel, original_vector, threshold=0.2) -> bool:
    """
    Check if the maneuver node is complete.

    :param vessel: The vessel object from kRPC.
    :param original_vector: The original maneuver node vector at maneuver start.
    :param threshold: The delta-v threshold to consider the maneuver complete.
    :return: True if the maneuver is complete, False otherwise.
    """
    node = vessel.control.nodes[0] if vessel.control.nodes else None
    if node is None:
        return True  # No maneuver node, consider complete
    
    burn_vector = node.burn_vector(node.reference_frame)
    if vector_angle(original_vector, burn_vector) > 90:
        return True
    
    return node.remaining_delta_v < threshold


def vector_angle(v1, v2) -> float:
    dot_product = sum(a*b for a, b in zip(v1, v2))
    mag_v1 = math.sqrt(sum(a*a for a in v1))
    mag_v2 = math.sqrt(sum(a*a for a in v2))
    try:
        angle = math.degrees(math.acos(dot_product / (mag_v1 * mag_v2)))
    except ValueError:
        angle = 0  # Handle edge cases where acos input is out of range
    return angle


if __name__ == "__main__":
    try:
        execute_maneuver_node()
    except ZeroDivisionError as e:
        print(f"\nZeroDivisionError -> probably empty fuel tank or engine deactivated: {e}")
    except KeyboardInterrupt:
        print("\nScript exited by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred (note: keep navball extended): {e}")
