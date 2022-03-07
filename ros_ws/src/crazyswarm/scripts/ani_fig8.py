"""Takeoff-make_fig8-land for one CF."""

from pycrazyswarm import Crazyswarm
import uav_trajectory


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 10.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    
    #Uploading trajectory: Figure of 8
    traj = uav_trajectory.Trajectory()
    traj.loadcsv("figure8.csv")
    cf.uploadTrajectory(0, 0, traj)

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    cf.startTrajectory(0)
    timeHelper.sleep(HOVER_DURATION)
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
