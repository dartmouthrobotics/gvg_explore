import os
import signal
import socket
import subprocess
import sys
import time

import numpy

import rosgraph
import roslaunch # @todo use roslaunch instead of subprocess.
import rostopic



def check_kill_process(pstring):
    for line in os.popen("ps ax | grep " + pstring + " | grep -v grep"):
        fields = line.split()
        pid = fields[0]
        print line
        os.kill(int(pid), signal.SIGKILL)

if __name__ == '__main__':
    nrobots_all = ["2"]

    methods = {"gvg": "gvgexploration", "recurrent": "recurrent_connectivity", "continuous": "continuous_connectivity"}
    runs = range(5)
    envs = {"office": [4.0, 10.0], "cave": [4.0, 10.0], "city": [4.0, 10.0]}  # open

    for env in ENVIRONMENTS[3:]:
        disp.append([])
        for i in range(1,6):
            with open(os.path.join("worlds", env, env + str(i) + "_robots8.inc"), 'r') as robot_poses:
                for j, line in enumerate(robot_poses.readlines()):
                    if j == 2:
                        disp[-1].append(line.split()[2:5]) # TODO the 5 element is actually the z
                        break

    experiments = [0]#range(4,5)

    num_dynamic_obstacles = [48]

    exp = 0 # To change when running experiment for one environemnt

    ENV = [ENVIRONMENTS[8]]
    DISP = [disp[8]]
    methods = [METHODS[1]]

    """
    # amoco
    ENV = [ENVIRONMENTS[1]]
    DISP = [disp[1]]
    methods = [METHODS[1]]
    """

    root_path = "/home/aql/catkin_ws/src/pf2d_localizer/"
    maps_path = root_path + "worlds/"
    logs_path = "/home/aql/DATA/experiments/dynamic_obstacles/logs_batch/2016-02-11/"
    yaml_extension = ".yaml"
    world_extension = ".world"


    for ndo in num_dynamic_obstacles:
        for i in experiments:
            for j, env in enumerate(ENV):
                world_file = maps_path + env + "/" + env + str(i + 1) + "_" + str(ndo) + world_extension
                yaml_file = maps_path + env + "/" + env + yaml_extension
                print DISP[j][i]
                for method in methods:
                    log_path = logs_path + env + "/" + str(ndo) + "/" + method[0] + "/" + str(i)
                    try:
                        os.makedirs(log_path)
                    except OSError:
                        pass
                    #check_kill_process("ros")
                    
                    stage_args = ['roslaunch', 'pf2d_localizer', "turtlebot_in_stage" + "_" + str(8) +".launch", "map_file:=" + yaml_file, "world_file:=" + world_file]
                    pf2d_localizer_args = ['roslaunch', 'pf2d_localizer', 'pf2dlocalizer.launch.xml', "environment:=" + yaml_file, "logs_root_path:=" + log_path + "/", "disp_x:=" + str(DISP[j][i][0]), "disp_y:=" + str(DISP[j][i][1])]

                    pf2d_localizer_args.append("strategy:=" + str(method[1]))
                    if method[0] == "random_walk":               
                        stage_args.append("random_walk:=true")


                    roscore_process = subprocess.Popen("roscore")
                    time.sleep(5)
                    pf2d_localizer_process = subprocess.Popen(pf2d_localizer_args)
                    time.sleep(5)
                    ros_started = False
                    while not ros_started:
                        try:
                            rosgraph.Master('/rostopic').getPid()
                            ros_started = True
                        except socket.error:
                            raise rostopic.ROSTopicIOException("Unable to communicate with master!")


                    stage_process = subprocess.Popen(stage_args)
                    time.sleep(5)
                    pf2d_localizer_process.wait()

                    print "finished"
                    time.sleep(3)
                    try:
                        stage_process.kill()
                    except OSError: 
                        pass
                    time.sleep(3)
                    try:
                        pf2d_localizer_process.kill()
                    except OSError:
                        pass
                    time.sleep(3)
                    try:
                        roscore_process.kill()
                    except OSError:
                        pass

                    check_kill_process("ros")
