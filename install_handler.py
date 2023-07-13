import subprocess
import os
import sys
from typing import List

def run_installation_script(source=False, path:str="" , options:List[int] = [0]):

    curr_path = os.path.abspath(__file__)
    curr_dir = os.path.dirname(curr_path)
    paths = [
                os.path.join(curr_dir, "main_install.sh"),
                os.path.join(curr_dir, "ros_humble_install.sh"),
                os.path.join(curr_dir, "ignition_gazebo_fortress_install.sh"),
                os.path.join(curr_dir, "bridge_install.sh"),
                os.path.join(curr_dir, "drake_install.sh")
            ]
    
    if path == "":
        path = os.path.join("/home", os.environ["USER"]) 
    print("Installing on path: ", path)
    
    if options == []:
        options = [0]
    #filters out invalid/duplicate options
    options = list(filter(lambda option: option >= 0 and option < len(path), list(set(options))))
    installed = set()
    if 0 in options: installed.update([1, 2, 3])
    
    print("Installing options: ", options)
    for option in options:
        if option in installed: continue
        installed.add(option)

        if not os.access(paths[option], os.X_OK):
            os.chmod(paths[option], 0o755)
        subprocess.run([paths[option], path])
    #Adding path to bashrc
    if source:
        sourced = set()
        source_str = "\n"
        for option in options:
            if option in sourced: continue
            if option == 1 or option == 0: #ros2_humble
                source_str += "source /opt/ros/humble/setup.bash\n"    
                source_str += "source " + os.path.join(path, "ros2_humble/install/local_setup.bash") + '\n\n'
                sourced.add(1)
            if option == 2 or option == 0: #ign_ws 
                source_str += ". " + os.path.join(path, "ign_ws/install/local_setup.bash") + '\n\n'
                sourced.add(2)
            if option == 3 or option == 0: #ros_ign_gz_bridge\
                source_str += ". " + os.path.join(path, "ros_ign_gz_bridge/install/local_setup.bash") + '\n\n'
                sourced.add(3)
            if option == 4: #drake
                source_str += 'export PATH="/opt/drake/bin${PATH:+:${PATH}}"\n'
                source_str += '''export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"''' + '\n'
                source_str += "source " + os.path.join(path, "drake/install/setup.bash") + '\n\n'
                sourced.add(4)

        with open(os.path.join("/home", os.environ["USER"], ".bashrc") , 'a') as file:
            file.write(source_str)
        
if __name__ == "__main__":
    sys.argv.pop(0)
    if len(sys.argv) == 0:
        print("This script will install select options and add source appropriate files to PATH")
        input("Press enter to continue or Ctrl+C to exit")
        run_installation_script(True)
    else:
        source_arg = sys.argv.pop(0)
        path = "" if len(sys.argv) == 0 else sys.argv.pop(0) 
        run_installation_script(int(source_arg), path, [int(arg) for arg in sys.argv])