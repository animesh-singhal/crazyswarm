"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
from scipy.integrate import solve_ivp
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d



SETUP_DURATION = 2.0
TAKEOFF_DURATION = 2.0

HEIGHT = 1.0
X_INITIAL = 0.0
Y_INITIAL = 0.0
Z_INITIAL = 1.0
Alpha_INITIAL = 0.0
Beta_INITIAL = 0.0

X_GOAL = 30.0      # cm
Y_GOAL = 60.0      # cm
Z_GOAL = 50.0
v = 10.0           # cm/s
kAlpha = 2         # k > (v/Radius) = 2
kBeta = 3          # k > (v/Radius) = 2 

simTime = 40#40     # sec
sampleTime = 1   # sec
iterPerSample = 10
iterTime = sampleTime/iterPerSample

def odes(t, x, AlphaRelBearing, BetaRelBearing):
    
    # assign each ODE to a vector element
    X = x[0]
    Y = x[1]
    Z = x[2]
    Alpha = x[3]
    Beta = x[4]
    Alpha = math.atan2(np.sin(Alpha),np.cos(Alpha))
    #Beta = math.atan2(np.sin(Beta),np.cos(Beta))
    Beta = math.asin(np.sin(Beta)) ###############################
    
    # Bearing already calculated previously
    
    # define each ODE
    dXdt = v*np.cos(Alpha)*np.cos(Beta) 
    dYdt = v*np.sin(Alpha)*np.cos(Beta)
    dZdt = v*np.sin(Beta)
    dAlphadt = -kAlpha*np.sign(AlphaRelBearing)
    dBetadt = -kBeta*np.sign(BetaRelBearing)

    return [dXdt, dYdt, dZdt, dAlphadt, dBetadt]
    

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    
    # Ensure initial conditions:
    initPosnSet = [X_INITIAL,Y_INITIAL,Z_INITIAL]
    cf.goTo(goal=initPosnSet, yaw=Alpha_INITIAL, duration=SETUP_DURATION)
    timeHelper.sleep(SETUP_DURATION)
    
    
    # initial conditions
    initStateVec = np.array([[X_INITIAL, Y_INITIAL, Z_INITIAL, Alpha_INITIAL, Beta_INITIAL]]) 
    
    xSol = initStateVec    
    tSol = [0]
    
    xActual = np.array([[cf.position()[0], cf.position()[1], cf.position()[2], cf.yaw()]])
    
   
    for i in range (0, simTime, sampleTime): 
        delTime = [i, i+sampleTime]  
    
        x0 = xSol[-1,:]      # last row of xSol matrix ----------->>>>>>>>> Should we take the freshly sensed values or the previous values by odeSolver? We can't get Beta. Alpha is yaw so fine
        #x0 = np.array([cf.position()[0], cf.position()[1], cf.yaw()])
        X = x0[0]
        Y = x0[1]
        Z = x0[2]
        Alpha = x0[3]
        Beta = x0[4]
        
        
        Alpha = math.atan2(np.sin(Alpha),np.cos(Alpha))
        Alphabearing = math.atan2((Y_GOAL-Y),(X_GOAL-X)) 
        
        AlphaRelBearing = Alpha-Alphabearing
        AlphaRelBearing = math.atan2(np.sin(AlphaRelBearing), np.cos(AlphaRelBearing))
    
        
        #Betabearing = math.atan2((Z_GOAL-Z),(math.sqrt((X_GOAL-X_INITIAL)**2+(Y_GOAL-Y_INITIAL)**2))) 
        Betabearing = math.asin((Z_GOAL-Z)/(math.sqrt( (X_GOAL-X_INITIAL)**2 + (Y_GOAL-Y_INITIAL)**2 + (Z_GOAL-Z_INITIAL)**2 )))
        #Beta = math.asin(np.sin(Beta))  #########
        
        BetaRelBearing = Beta-Betabearing
        #BetaRelBearing = math.atan2(np.sin(BetaRelBearing), np.cos(BetaRelBearing))
        
        
        tEval=np.linspace(i, i+sampleTime, iterPerSample+1)
        #print(tEval)
        
        
        sol = solve_ivp(odes, (i, i+sampleTime), x0, t_eval=tEval, args=(AlphaRelBearing, BetaRelBearing))     #args pass the arguments to odes function
        xInterval=sol.y                      # xInterval will include x(@i) and x(@i+sampleTime)
        xInterval=np.transpose(xInterval)    # each row should have a new set of values of x,y,alpha
    
        tInterval=sol.t                      # i <= t <= i+sampleTime
        
        
        # While appending solutions to master arrays, removing 1st elements as they'll be appended as the 'last element of previous iteration'
        xSol = np.vstack((xSol, xInterval[1:,:]))    #stacking new solutions (except the first row) over previous solutions
        tSol = np.hstack((tSol, tInterval[1:]))      #stacking new array without its first element to the previous array
    
        for j in range (iterPerSample): 
    	    x_nxt = xInterval[j+1, 0]
    	    y_nxt = xInterval[j+1, 1] 
    	    z_nxt = xInterval[j+1, 2]
    	    yaw_nxt = xInterval[j+1, 3]
    	    pos_nxt = [x_nxt, y_nxt, z_nxt]
    	    cf.goTo(goal=pos_nxt, yaw=yaw_nxt, duration=iterTime)
    	    timeHelper.sleep(iterTime)
    	    
    	    xActualNext = np.array([[cf.position()[0], cf.position()[1], cf.position()[2], cf.yaw()]])
    	    xActual = np.vstack((xActual, xActualNext))
    	    
   
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)
    
    X = xSol[:,0]
    Y = xSol[:,1]
    Z = xSol[:,2]
    Alpha = xSol[:,3]
    Beta = xSol[:,4]

    # plot the results
    figure, axis = plt.subplots(2, 3)

    # X with time
    axis[0, 0].plot(tSol, X)
    axis[0, 0].set_title("X vs t")

    # Y with time
    axis[0, 1].plot(tSol, Y)
    axis[0, 1].set_title("Y vs t")

    # Z with time
    axis[0, 2].plot(tSol, Z)
    axis[0, 2].set_title("Z vs t")


    # Alpha with time
    axis[1, 0].plot(tSol, Alpha)
    axis[1, 0].set_title("Alpha vs t")
  
    # Beta with time
    axis[1, 1].plot(tSol, Beta)
    axis[1, 1].set_title("Beta vs t")
  
  
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(xActual[:,0], xActual[:,1], xActual[:,2], 'gray')
    #ax.scatter3D(xActual[:,0], xActual[:,1], xActual[:,2], cmap='Greens');
    
    # Combine all the operations and display
    plt.show()
    
    

if __name__ == "__main__":
    main()
    
