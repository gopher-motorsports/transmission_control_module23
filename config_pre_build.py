import os
import sys

# Get file locations
proj_dir = os.getcwd()
main_dir = os.path.dirname(proj_dir)
dir_name = os.path.basename(proj_dir)
gcannon_path = os.path.join(main_dir,'gophercan-lib','network_autogen')

# Check if vehicle config was specified
if (len(sys.argv) < 2):
    print('Need to provide a vehicle config from the network_autogen folder of gophercan-lib')
    print('Example: python select_car.py go4-23e')
    sys.exit()

car_path = os.path.join(gcannon_path,'configs',sys.argv[1]+'.yaml')
gsense_path = os.path.join(main_dir,'Gopher_Sense')
config_file_path = os.path.join(proj_dir,dir_name + '_config.yaml')

# Enter GopherCAN autogen command
# ---
os.chdir(gcannon_path)
os.system('python ' + 'autogen.py' + ' ' + car_path)

# Enter Gopher Sense autogen command - uncomment below on gcan boiler plate once you reach software startup docs part 2
# ---
os.chdir(gsense_path)
os.system('python ' + 'gsense_auto_gen.py' + ' ' + config_file_path)