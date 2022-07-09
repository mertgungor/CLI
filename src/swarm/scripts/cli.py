#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot

import numpy as np
import os

def clear_log():
    for i in range(50):
        try:
            os.remove('./agent{}_logs.csv'.format(i))
        except FileNotFoundError:
            return



if __name__ == "__main__":
    clear_log()
    uav_count = int(input("Enter UAV number: "))
    distance_between_agents = 3
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(uav_count, vehicle2)
    
    mission = int(input("Enter mission number: "))

    distance_between_agents = float(input("Distance between agents: "))
    swarm.form_via_potential_field(distance_between_agents)

    if mission == 1:
        mission1_options = """
        1: LAND
        2: ADD AGENT
        3: REMOVE AGENT
        4: GO
        5: ROTATE
        """

        is_mission_running = True

        

        while is_mission_running:

            sub_mission = int(input(mission1_options))

            if sub_mission == 1:
                swarm.land()
                swarm.timeHelper.sleep(5)
                break

            if sub_mission == 2:
                swarm.add_agent_to_formation()

            if sub_mission == 3:
                swarm.omit_agent()

            if sub_mission == 4:
                vector = str(input("Direction vector: "))
                vector = vector.split(" ")
                swarm.go(np.array([float(vector[0]), float(vector[1]), float(vector[2])]))

            if sub_mission == 5:
                pass
    
    if mission == 2:
        sub_missions = """
        1: Add obstacle
        2: Remove last obstacle
        3: Enter displacement vector
        4: Start mission
        """

        obstacles = []

        is_mission_running = True

        while is_mission_running:

            sub_mission = int(input(sub_missions))

            if sub_mission == 1:

                pose = str(input("Position of the obstacle: "))
                pose = pose.split(" ")
                obstacle = np.array([float(pose[0]), float(pose[1]), float(pose[2])])
                obstacles.append(obstacle)

                print("Obstacles: " + str(obstacles))

            if sub_mission == 2:

                obstacles.pop()
                print(obstacles)

            if sub_mission == 3:

                vector = str(input("Position of the obstacle: "))
                vector = vector.split(" ")
                vector = np.array([float(vector[0]), float(vector[1]), float(vector[2])])
                swarm.go(vector)

