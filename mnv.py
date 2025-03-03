import math
import time
import krpc
import argparse

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def commandLine():
    parser = argparse.ArgumentParser(description = "execute maneuver node script")
    parser.add_argument("-V", "--version", action='version', version='%(prog)s 1.0')
    parser.add_argument('--rtt', type=int, help='rotation time to correct maneuver orientation (default: 30)', required = False, default = 30)
    return parser.parse_args()

def connect_to_krpc():
     return krpc.connect(name='Execute Maneuver Node')

def setup_vessel(vessel):
    vessel.control.sas = False
    vessel.control.throttle = 0.0
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, vessel.flight().heading)

def calculate_burn_time(vessel, node):
    # Calculate the burn time
    g = vessel.orbit.body.surface_gravity
    isp = vessel.specific_impulse
    F = vessel.available_thrust
    m0 = vessel.mass
    delta_v = node.delta_v
    # Tsiolkovsky rocket equation to calculate burn time
    burn_time = (m0 * (1 - (1 / (1 + delta_v / (isp * g))))) / (F / (isp * g))
    return burn_time

def orientate_vessel(vessel, node):
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)

def execute_burn(vessel, burn_time, node):
    vessel.control.throttle = 1.0
    time.sleep(burn_time)
    vessel.control.throttle = 0.0

    # Check remaining delta-v and perform corrective burn if needed
    remaining_delta_v = node.remaining_delta_v
    if remaining_delta_v > 0.1:  # Threshold for corrective burn
        print(f"Corrective burn needed: {remaining_delta_v:.2f} m/s")
        vessel.control.throttle = 0.5  # Lower throttle for precision
        while node.remaining_delta_v > 0.1:
            pass
        vessel.control.throttle = 0.0

def main() -> None:
    args = commandLine()
    conn = connect_to_krpc()
    vessel = conn.space_center.active_vessel
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    setup_vessel(vessel)
    print("\033c", end="")

    if not vessel.control.nodes:
        print("No planned maneuver nodes available!")
        return None

    node = vessel.control.nodes[0]
    burn_time = calculate_burn_time(vessel, node)
    print(f"Delta-V: {node.delta_v:.2f} m/s")
    print(f"Burn time: {burn_time:.2f} s")

    print('Waiting until maneuver burn')
    burn_ut = ut() + node.time_to - (burn_time/2.)
    lead_time = args.rtt
    conn.space_center.warp_to(burn_ut - lead_time)
    print(f'{lead_time} seconds...Ready to execute burn')

    orientate_vessel(vessel, node)

    while ut() < burn_ut:
        time.sleep(0.1)

    print('Executing burn')
    execute_burn(vessel, burn_time, node)
    #current_orientation = vessel.flight().direction
    #vessel.auto_pilot.target_direction = current_orientation
    vessel.auto_pilot.disengage()
    vessel.control.sas = True
    print("Waiting for steering to settle down")
    time.sleep(1)
    node.remove()  # Remove maneuver node
    time.sleep(1)
    print("Maneuver complete")

if __name__ == '__main__':
    main()
