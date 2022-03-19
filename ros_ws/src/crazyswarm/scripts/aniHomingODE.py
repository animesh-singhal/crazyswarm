"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np
import math

TAKEOFF_DURATION = 2.0

HEIGHT = 1.0
X_INITIAL = 0.0
Y_INITIAL = 0.0
X_GOAL = 30.0      # cm
Y_GOAL = 60.0      # cm
v = 10.0           # cm/s
kAlpha = 2         # k > (v/Radius) = 2

simTime = 20.0     # sec
sampleTime = 1.0   # sec
iterPerSample = 10.0
iterTime = sampleTime/iterPerSample

def odes(t, x):
    
    # assign each ODE to a vector element
    X = x[0]
    Y = x[1]
    Alpha = x[2]
    Alpha = math.atan2(np.sin(Alpha),np.cos(Alpha))
  
    # Bearing already calculated previously
    
    # define each ODE
    dXdt = v*np.cos(Alpha) 
    dYdt = v*np.sin(Alpha)
    dAlphadt = -kAlpha*np.sign(relbearing)   #np.sign is signum function
        
    #print("t: "+str(t)+"; Alpha: "+str(Alpha)+"; Alpha_dot: "+str(dAlphadt))
    return [dXdt, dYdt, dAlphadt]


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    
    # Ensure initial conditions:
    initPosnSet = [X_INITIAL,Y_INITIAL,HEIGHT]
    cf.goTo(goal=initPosnSet, yaw=0.0, duration=2.5)
    timeHelper.sleep(2.5)
    
    
    # initial conditions
    #X_init = 0
    #Y_init = 0
    #Alpha_init = 0
    #Initial = np.array([[X_init, Y_init, Alpha_init]])
    initPos = np.array(cf.position())
    
    xSol = initPos 
    tSol = [0]
    
    
    for i in range (0, simTime, sampleTime): 
        delTime = [i, i+sampleTime]  
    
        x0 = xSol[-1,:]      #last row of xSol matrix ----------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Should we take the freshly sensed values or the previous values by odeSolver? 
    
        X = x0[0]
        Y = x0[1]
        Alpha = x0[2]
    
        bearing = math.atan2((Y_GOAL-Y),(X_GOAL-X)) 
        relbearing = Alpha-bearing
        relbearing = math.atan2(np.sin(relbearing), np.cos(relbearing))
    
        tEval=np.linspace(i, i+sampleTime, iterPerSample+1)
        
        sol = solve_ivp(odes, (i, i+sampleTime), x0, t_eval=tEval)
        xInterval=sol.y                      # xInterval will include x(@i) and x(@i+sampleTime)
        xInterval=np.transpose(xInterval)    # each row should have a new set of values of x,y,alpha
    
        tInterval=sol.t                      # i <= t <= i+sampleTime
        
        # While appending solutions to master arrays, removing 1st elements as they'll be appended as the 'last element of previous iteration'
        xSol = np.vstack((xSol, xInterval[1:,:]))    #stacking new solutions (except the first row) over previous solutions
        tSol = np.hstack((tSol, tInterval[1:]))      #stacking new array without its first element to the previous array
   
    
        for j in range (iterPerSample): 
    	    x_nxt = xInterval[i+1, 0]
    	    y_nxt = xInterval[i+1, 1] 
    	    z_nxt = HEIGHT
    	    yaw_nxt = xInterval[i+1, 2]
    	    pos_nxt = [x_nxt, y_nxt, z_nxt]
    	    #print(pos_nxt)
    	    cf.goTo(goal=pos_nxt, yaw=yaw_nxt, duration=iterTime)
    	
    	    #print(cf.position())
    	    #print("Yaw->"+str(cf.yaw()))
    	    #print("rpy->"+str(cf.rpy()))
    	    #print("---")
    	    timeHelper.sleep(iterTime)
    	
    
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)
    
    X = xSol[:,0]
    Y = xSol[:,1]
    Alpha = xSol[:,2]


    # plot the results
    figure, axis = plt.subplots(2, 2)

    # Y vs X 
    axis[0, 0].plot(X, Y)
    axis[0, 0].set_title("Y vs X")
  
    # Alpha with time
    axis[1, 0].plot(tSol, Alpha)
    axis[1, 0].set_title("Alpha vs t")
  
    # Y with time
    axis[0, 1].plot(tSol, Y)
    axis[0, 1].set_title("Y vs t")
  
    # X with time
    axis[1, 1].plot(tSol, X)
    axis[1, 1].set_title("X vs t")
  
    # Combine all the operations and display
    plt.show()

    

if __name__ == "__main__":
    main()
    
