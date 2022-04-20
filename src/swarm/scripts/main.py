#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot

import numpy as np


if __name__ == "__main__":

    uav_count = 6
    radius = 1.5
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(uav_count, vehicle2)
    #swarm.star_formation()
    swarm.timeHelper.sleep(0.5)
    swarm.form_via_potential_field(radius)
    #swarm.go(np.array([radius,0,0]))
    #swarm.go(np.array([0,-radius,0]))
    #swarm.go(np.array([-radius,0,0]))
    #swarm.go(np.array([0,radius,0]))
    #swarm.rotate()
    swarm.timeHelper.sleep(2)
    #swarm.form_pyramid()
    #swarm.add_agent_to_formation()
    #swarm.timeHelper.sleep(2)

    #for i in range(uav_count):
    #    swarm.omit_agent()
    
    #swarm.swarm_square(2)
    #swarm.rotate()

    #swarm1, swarm2 = swarm.split_formation()
    #swarm1.go(np.array([1,0,0]))
    #swarm2.go(np.array([1,0,0]))
    #swarm.omit_agent()
    #swarm.timeHelper.sleep(4)
    swarm.log_to_csv()
    swarm.land()
    
    swarm.timeHelper.sleep(4)
    #swarm.return_starting_pose()