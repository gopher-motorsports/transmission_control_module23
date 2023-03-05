import os

proj_dir = os.getcwd()
main_dir = os.path.dirname(proj_dir)
dir_name = os.path.basename(proj_dir)
gcannon_path = main_dir + '\\gophercan-lib\\network_autogen'
car_path = gcannon_path + '\\configs\\go4-23c.yaml'
gsense_path = main_dir + '\\Gopher_Sense'
config_file_path = proj_dir + '\\' + dir_name + '_config.yaml'
os.chdir(gcannon_path)
os.system('python ' + 'autogen.py' + ' ' + car_path)
os.chdir(gsense_path)
os.system('python ' + 'gsense_auto_gen.py' + ' ' + config_file_path)
print(gcannon_path)
print(car_path)
print(gsense_path)
print(config_file_path)

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