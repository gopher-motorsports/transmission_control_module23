import os

proj_dir = os.getcwd()                  # get current working directory of the file (c://ST/STM32.../transmission_control_module)
main_dir = os.path.dirname(proj_dir)    # get the path to the directory above the module where all other gopher motorsports projects should be located (C://ST/STM32...)
dir_name = os.path.basename(proj_dir)   # get name of current working direcoty (transmission_control_module)
gcannon_path = main_dir + '\\gophercan-lib\\network_autogen'    # get the path to gcan's current network autogen location (c://ST/STM32.../gophercan-lib/network_autogen)
car_path = gcannon_path + '\\configs\\go4-23c.yaml'             # get the path to the desired network file (master gcan config), currently needs to be manually changed per car (c://ST/STM32.../configs/go4-23c.yaml)
# TODO allow for entering this file name so each module does not need config_pre_build.py
gsense_path = main_dir + '\\Gopher_Sense'                       # get the path to gopher sense, assuming it is in the same file as all other gopher motorsports projects (c://ST/STM32.../Gopher_Sense)
config_file_path = proj_dir + '\\' + dir_name + '_config.yaml'  # get the path to the module's gsense config file, currently set up to use the name of the module with _config.yaml at the end
# TODO allow for manually entering config file name or not entering and letting it auto find
os.chdir(gcannon_path)                                                  # change directory to gcan
os.system('python ' + 'autogen.py' + ' ' + car_path)                    # run gcan autogen script with the path to the car you're configuring for
os.chdir(gsense_path)                                                   # change direcoty to gsense
os.system('python ' + 'gsense_auto_gen.py' + ' ' + config_file_path)    # run gsense autogen script with path to the modules gsense config file
print(gcannon_path)         # print paths
print(car_path)
print(gsense_path)
print(config_file_path)

# Old code for doing this, had to change when gcan/gsense paths changes
# proj_dir = os.getcwd()
# main_dir = os.path.dirname(proj_dir)
# dir_name = os.path.basename(proj_dir)
# gcannon_path = main_dir + '\\gophercan-lib\\gophercannon'
# car_path = gcannon_path + '\\networks\\go4-22c.yaml'
# gsense_path = main_dir + '\\Gopher_Sense'
# config_file_path = proj_dir + '\\' + dir_name + '_config.yaml'
# os.chdir(gcannon_path)
# os.system('python ' + 'gcan_auto_gen.py' + ' ' + car_path)
# os.chdir(gsense_path)
# os.system('python ' + 'sensor_cannon.py' + ' ' + config_file_path)