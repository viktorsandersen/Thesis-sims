import sys

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


# logging.basicConfig(level=logging.INFO)

#ROBOT_HOST = "172.17.0.2" #URsim
ROBOT_HOST = "192.168.1.100"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)



#setp.input_double_register_0 = 0


# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

def unpack_mass_matrix_and_coriolis(state):
    mass_flat = [getattr(state, f'output_double_register_{i}') for i in range(0, 36)]
    mass_mat = [mass_flat[i:i+6] for i in range(0, 36, 6)]
    coriolis = [getattr(state, f'output_double_register_{i}') for i in range(36, 42)]
    return mass_mat, coriolis

if not con.send_start():
    sys.exit()


move_completed = True
while keep_running:
    state = con.receive()
    if state is None:
        break

    # Read target_command from registers 0-5
    #target_command = [getattr(state, f'output_double_register_{i}') for i in range(6)]
    #print(f"Target Command: {target_command}")

    # Read mass matrix (6x6 = 36 values starting from register 6)
    #mass_matrix_flat = [getattr(state, f'output_double_register_{i}') for i in range(6, 42)]
    #mass_matrix = [mass_matrix_flat[i:i+6] for i in range(0, 36, 6)]
    #print("Mass Matrix:")
    #for row in mass_matrix:
        #print(row)

    # Read coriolis vector (6 values from register 42 to 47)
    #coriolis_vector = [getattr(state, f'output_double_register_{i}') for i in range(42, 48)]
    #print(f"Coriolis Vector: {coriolis_vector}")
    #target_command = [getattr(state, f'output_double_register_{i}') for i in range(6)]
    #print(f"Target Command: {target_command}")
    mass_mat, coriolis = unpack_mass_matrix_and_coriolis(state)
    print("target_q: ", state.target_q)
    print("Mass Matrix:")
    for row in mass_mat:
        print(row)
    print(f"Coriolis Vector: {coriolis}")
    print("="*50)

con.send_pause()
con.disconnect()
