import krpc
import time

try:
    # Connect to the kRPC server
    conn = krpc.connect(name='Auto Stage')

    # Get the active vessel
    vessel = conn.space_center.active_vessel
    vessel.control.throttle = 1.0

    # Main loop
    while True:
        if vessel.thrust == 0:
            print("Thrust is zero, activating next stage.")
            vessel.control.activate_next_stage()
            time.sleep(1)  # Wait a bit to avoid rapid staging
        time.sleep(0.1)  # Check every 0.1 seconds
        
except KeyboardInterrupt:
    print("Testing Auto Staging stopped.")
