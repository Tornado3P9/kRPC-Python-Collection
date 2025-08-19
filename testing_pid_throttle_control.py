import krpc
from simple_pid import PID
import time

def twr_error(vessel, target_twr):
    thrust = vessel.available_thrust * vessel.control.throttle
    weight = vessel.mass * vessel.orbit.body.surface_gravity
    current_twr = abs(thrust / weight)
    return current_twr - target_twr

# Main control loop
def main() -> None:
    try:
        # Connect to the game
        conn = krpc.connect(name='PID Controller')
        vessel = conn.space_center.active_vessel

        # Set up the PID controller
        pid = PID(Kp=0.005, Ki=0.5, Kd=0, setpoint=0)
        pid.output_limits = (0, 1)  # Throttle limits
        target_twr = 0.5
        
        vessel.control.throttle = 1.0
        vessel.control.activate_next_stage()
        
        counter = 0
        while True:
            # Auto Staging
            counter += 1
            if vessel.thrust == 0 and counter % 10 == 0:
                counter = 0
                print("\nThrust is zero, activating next stage.")
                vessel.control.activate_next_stage()
                # Between each staging let one second pass to avoid rapid staging
            
            # Optimize throttle to minimize TWR error
            new_error = twr_error(vessel, target_twr)
            new_throttle = pid(new_error)

            # Set the throttle
            vessel.control.throttle = new_throttle

            print(f"\rThrottle set to: {new_throttle*100:6.2f}% to maintain TWR of {target_twr} with Error of {new_error:5.2f} and Thrust is {vessel.thrust:.1f}", end='      ')

            # Wait for a short period before recalculating
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nPID Controller stopped.")


if __name__ == "__main__":
    main()
