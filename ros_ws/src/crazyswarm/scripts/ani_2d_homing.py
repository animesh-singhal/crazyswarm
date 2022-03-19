#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *


Z = 1.0
sleepRate = 30


def homingPlanar(timeHelper, cf, targetYaw, kYaw):
        startTime = timeHelper.time()
        pos = cf.position()
        print(pos)
        #print("Default Initial Position set with cf")
        #print(cf.initialPosition)
        startPos = cf.initialPosition + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        
        with open("ani_circle_vel_pos.csv","w") as f: 
            f.write("t,x,y,z\n")
        
            while True:
                time = timeHelper.time() - startTime
                if time>8: 
                    # End after 2 circles
                    break
                omega = 2 * np.pi / totalTime
                # v_vec = (w_vec) X (r_vec)
                vx = -radius * omega * np.sin(omega * time)  
                vy = radius * omega * np.cos(omega * time)
            
                cf.cmdVelocityWorld(np.array([vx, vy, 0]), yawRate=0)
                          
                # Proportional controller set up to fix error in case cf loses position            
                #desiredPos = center_circle + radius * np.array(
                #    [np.cos(omega * time), np.sin(omega * time), 0])
                #errorX = desiredPos - cf.position() 
                # cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0) 
            
                pos = cf.position()
                f.write("{},{},{},{}\n".format(time, pos[0], pos[1], pos[2]))
             
                timeHelper.sleepForRate(sleepRate)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    #homingPlanar(timeHelper, allcfs.crazyflies[0], totalTime=4, target_yaw=45, kYaw=1)
    homingPlanar(timeHelper, allcfs.crazyflies[0], targetYaw=45, kYaw=1)
