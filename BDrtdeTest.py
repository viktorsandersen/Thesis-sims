import sys

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "172.17.0.2"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"
conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()
con.get_controller_version()

con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)

if not con.send_start():
    sys.exit()

print("RTDE Communication started!")

while True:
    state = con.receive()
    if state is None:
        break

    # Step 1: Receive data from robot (output registers)
    send_data = []
    for i in range(6):
        val = getattr(state, f'output_double_register_{i}')
        send_data.append(val)
    print(f"Received from robot: {send_data}")

    # Step 2: Process data in Python (e.g., scale it)
    processed = [x * 5 for x in send_data]  # Example: reduce values by 50%

    # Step 3: Send it back (input registers)
    for i in range(6):
        setattr(setp, f'input_double_register_{i}', processed[i])

    con.send(setp)
    #time.sleep(0.05)

con.send_pause()
con.disconnect()
