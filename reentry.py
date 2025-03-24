import os
import krpc
import time


def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')


def main():
    clear_screen()
    conn = krpc.connect(name='Kerbin reentry maneuver')
    vessel = conn.space_center.active_vessel

    print("Started reentry maneuver")
    vessel.control.sas = True
    time.sleep(2)
    vessel.control.sas_mode = conn.space_center.SASMode.retrograde

    print("Waiting until altitude < 75Km")
    while vessel.flight().mean_altitude >= 75000:
        time.sleep(1)

    # Do reentry staging
    # vessel.control.activate_next_stage()
    print("AG10 active: last staging before reentry")
    vessel.control.toggle_action_group(0)  # (1,2,3,4,5,6,7,8,9,0)

    # Arm parachutes
    do_parachute(vessel)
    
    # Unlock all
    vessel.control.sas = False
    vessel.control.rcs = False

    print("Script exited.")


def do_parachute(vessel):
    while vessel.flight().surface_altitude > 5000:
        time.sleep(1)
    print("Parachutes")
    for parachute in vessel.parts.parachutes:
        parachute.arm()  # .arm() or .deploy()


if __name__ == "__main__":
    main()
