"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np
import math

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
RADIUS = 2.0

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(2.5)
    
    pos_B = [RADIUS,0.0,1.0]
    cf.goTo(goal=pos_B, yaw=0.0, duration=2.5)
    timeHelper.sleep(2.5)
    
    iter = 50
    for i in range (iter): 
    	x_nxt = RADIUS*math.cos(2.0*np.pi*(i+1)/iter)
    	y_nxt = RADIUS*math.sin(2.0*np.pi*(i+1)/iter) 
    	z_nxt = 1.0
    	yaw_nxt = 2.0*np.pi*(i+1)/iter
    	pos_nxt = [x_nxt, y_nxt, z_nxt]
    	#print(pos_nxt)
    	cf.goTo(goal=pos_nxt, yaw=yaw_nxt, duration=2.0)
    	#print(cf.position())
    	print("Yaw->"+str(cf.yaw()))
    	print("rpy->"+str(cf.rpy()))
    	print("---")
    	timeHelper.sleep(2)
    	
    
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
    
