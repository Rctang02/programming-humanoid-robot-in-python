'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello,leftBackToStand,leftBellyToStand,rightBackToStand,rightBellyToStand
import numpy as np
from scipy.interpolate import splrep, splev

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start = 0
        self.flag = False

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        # set the start time as an offset when first time enter this function
        if self.flag == False :
            self.start = perception.time
            self.flag = True

        #all the lists save the results
        angle_value = []
        x_m = []
        y_m = []
        splList = []


        #save all time variables from keyframes in x, and angle variables in y
        for index in range(0, keyframes[1].__len__()):
            x = np.array([])
            y = np.array([])

            for time in keyframes[1][index]:
                x = np.append(x, time)

            # target[keyframes[0][index]] = x.tolist()

            for index2,item in enumerate(keyframes[2][index]):

                y = np.append(y, keyframes[2][index][index2][0])

            x_m.append(x)
            y_m.append(y)

        #perception.time - offset as an variable for interpolation
        variable = perception.time - self.start

        #calculate the spline interpolation function for all joints
        for index in range(0, x_m.__len__()):

            if x_m[index].size == 2:
                spl = splrep(x_m[index], y_m[index], k=1)
            elif x_m[index].size == 3:
                spl = splrep(x_m[index], y_m[index], k=2)
            else:
                spl = splrep(x_m[index], y_m[index], k=3)


            splList.append(spl)

        #calculate and save the angles of all joints at the time:perception.time
        #variable should not greater than offset + max
        for index in range(0, keyframes[1].__len__()):
            if variable > x_m[index].max():
                variable = x_m[index].max()
                angleOfJoint = splev(variable, splList[index])
            elif variable < x_m[index].min():
                variable = x_m[index].min()
                angleOfJoint = perception.joint[keyframes[0][index]]
            else:
                angleOfJoint = splev(variable, splList[index])
            angle_value.append(angleOfJoint)

        #put all joints name and joint angles in dict
        for index in range(0, keyframes[1].__len__()):

            target_joints[keyframes[0][index]] = angle_value[index]




        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
