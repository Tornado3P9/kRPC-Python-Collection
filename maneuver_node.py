import krpc
import time
import math
import argparse


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

    do_circ_countdown(conn, start_time)

    vessel.control.throttle = 1.0
    
    original_vector = node.burn_vector(node.reference_frame)
    print("Maneuver in progress...")
    while not is_maneuver_complete(vessel, original_vector):
        pass

    vessel.control.throttle = 0.0

    vessel.auto_pilot.disengage()
    node.remove()
    vessel.control.sas = True
    time.sleep(1)
    print("Script exited.")


def planning_circularization_burn(conn, vessel, option) -> None:
    print('Planning circularization burn')
    mu = vessel.orbit.body.gravitational_parameter
    if option == 'ap':
        r = vessel.orbit.apoapsis
    elif option == 'pe':
        r = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    if option == 'ap':
        vessel.control.add_node(conn.space_center.ut + vessel.orbit.time_to_apoapsis, prograde=delta_v)
    elif option == 'pe':
        vessel.control.add_node(conn.space_center.ut + vessel.orbit.time_to_periapsis, prograde=delta_v)
    time.sleep(1)


def calculate_start_time(conn, node) -> float:
    global mnv_time
    mnv_time = maneuver_burn_time(conn, node)
    return conn.space_center.ut + node.time_to - mnv_time / 2


def maneuver_burn_time(conn, node) -> float:
    dV = node.delta_v
    vessel = conn.space_center.active_vessel
    g0 = vessel.orbit.body.surface_gravity

    engines = vessel.parts.engines
    active_engines = [engine for engine in engines if engine.active]
    print("Available Engines:")
    for engine in active_engines:
        isp = engine.specific_impulse
        print(f'  Engine: {engine.part.title}, ISP: {isp}')
    total_isp = sum(engine.specific_impulse for engine in active_engines)
    # average_isp = total_isp / len(engines)
    isp = total_isp

    mf = vessel.mass / math.exp(dV / (isp * g0))
    fuel_flow = vessel.available_thrust / (isp * g0)
    t = (vessel.mass - mf) / fuel_flow

    print(f"Maneuver duration: {t:.2f}s.")
    return t


def do_circ_countdown(conn, start_time) -> None:
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
    if v_ang(original_vector, burn_vector) > 90:
        return True
    
    remaining_delta_v = node.remaining_delta_v
    return remaining_delta_v < threshold


def v_ang(v1, v2) -> float:
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
        print("\033c", end="")
        execute_maneuver_node()
    except ZeroDivisionError as e:
        print(f"\nZeroDivisionError -> probably empty fuel tank or engine deactivated: {e}")
    except KeyboardInterrupt:
        print("\nScript exited by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred (note: keep navball extended): {e}")



#########################################################
# first_data = []
# data_captured = False
# while True:
#     current_data = stream()
    
#     # Capture the first data only once
#     if not data_captured:
#         first_data.append(current_data)
#         data_captured = True
    
#     # Work with the first data
#     print(f"First data captured: {first_data[0]}")

#     time.sleep(1)

#########################################################
