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
    parser = argparse.ArgumentParser(description = "Circularization burn script")
    parser.add_argument("-V", "--version", action='version', version='%(prog)s 1.0')
    parser.add_argument('--warp', type=str2bool, nargs='?', const=True, default=False, help='Auto time warping to maneuver position (default: False)')
    return parser.parse_args()

def main() -> None:
    argument = commandLine()
    warp = argument.warp

    conn = krpc.connect(name='Circularize orbit')
    vessel = conn.space_center.active_vessel

    # Set up streams for telemetry
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    # altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    # apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

    # Setup
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 0.0
    vessel.auto_pilot.engage()

    # Clear screen equivalent
    print("\033c", end="")
    print("Circularization burn where the apoapsis height is the orbit height goal.")
    time.sleep(1)

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
    g = vessel.orbit.body.surface_gravity  # Calculate gravitational acceleration (g) of the current body
    Isp = vessel.specific_impulse * g
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate
    print(f"Delta-V: {delta_v:.2f} m/s")
    print(f"Burn time: {burn_time:.2f} s")

    # Orientate ship
    print('Orientating ship for circularization burn')
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    time.sleep(1)

    # Wait until burn
    print('Waiting until circularization burn')
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
    if warp:
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
    vessel.auto_pilot.disengage() # Give control back to the pilot
    vessel.control.sas = True  # Activate SAS
    print("Waiting for steering to settle down")
    time.sleep(3)
    print("Launch complete")

if __name__ == '__main__':
    main()
