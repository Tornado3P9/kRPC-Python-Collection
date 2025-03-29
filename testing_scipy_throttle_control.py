import krpc
from scipy.optimize import minimize
import time

def twr_error(throttle, vessel, target_twr):
    thrust = vessel.available_thrust * throttle
    weight = vessel.mass * vessel.orbit.body.surface_gravity
    current_twr = thrust / weight
    return (current_twr - target_twr) ** 2

# Main control loop
def main() -> None:
    try:
        # Connect to the game
        conn = krpc.connect(name='Scipy Controller')
        vessel = conn.space_center.active_vessel

        target_twr = 1.3
        
        vessel.control.throttle = 1.0
        vessel.control.activate_next_stage()
        
        while True:
            # Auto Staging
            if vessel.thrust == 0:
                print("Thrust is zero, activating next stage.")
                vessel.control.activate_next_stage()
                time.sleep(1)  # Wait a bit to avoid rapid staging
            
            # Initial guess for throttle
            initial_throttle = vessel.control.throttle

            # Optimize throttle to minimize TWR error
            # result = minimize(twr_error, initial_throttle, bounds=[(0, 1)])
            result = minimize(twr_error, initial_throttle, args=(vessel, target_twr), bounds=[(0, 1)])

            # Set the throttle
            vessel.control.throttle = result.x[0]

            print(f"\rThrottle set to: {result.x[0]:.2f} to maintain TWR of {target_twr} ", end='')

            # Wait for a short period before recalculating
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nScipy Controller stopped.")


if __name__ == "__main__":
    main()
