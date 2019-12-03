'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
import math
from numpy.linalg import norm,inv

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):

        joint_angles = []
        # YOUR CODE HERE
        uleg = 100
        lleg = 102.9

        if effector_name == 'LLeg':
            hipToTorso = np.matrix([[1, 0, 0, 0], [0, 1, 0, 50], [0, 0, 1, -85], [0, 0, 0, 1]])
        else:
            hipToTorso = np.matrix([[1, 0, 0, 0], [0, 1, 0,-50], [0, 0, 1, -85], [0, 0, 0, 1]])

        footToHip = transform.dot(inv(hipToTorso))
        footToHip_v = [footToHip[0, 3], footToHip[1, 3], footToHip[2, 3]]  #vector between hip and foot
        ltrans = norm(footToHip_v)

        aKneePitch = math.pi - np.arccos((uleg**2 + lleg**2 - ltrans**2)/(2*uleg*lleg))     #from B-Human brochure
        aFootPitch1 = np.arccos((lleg**2 + ltrans **2 - uleg**2 )/ (2*lleg*ltrans))

        offset = np.matrix([[1,0,0],[0,math.cos(math.pi/4),-1*math.sin(math.pi/4)],[0,math.sin(math.pi/4),math.cos(math.pi/4)]])
        offset2 = np.matrix([[1,0,0,0],[0,math.cos(math.pi/4),-1*math.sin(math.pi/4),0],[0,math.sin(math.pi/4),math.cos(math.pi/4),0],[0,0,0,1]])
        footToHipOrth_v = offset.dot(footToHip_v)
        footToHipOrth_m = offset.dot(footToHip[0:3,0:3])        #maybe wrong

        x = footToHipOrth_v[0,0]
        y = footToHipOrth_v[0,1]
        z = footToHipOrth_v[0,2]
        aFootPitch2 = math.atan2(x, math.sqrt(y**2 + z**2))
        aFootRoll = math.atan2(y,z)
        aFootPitch = aFootPitch1 + aFootPitch2

        if effector_name == 'LLeg':
            transFootRoll = self.local_trans('LAnkleRoll',aFootRoll)
            transFootPitch = self.local_trans('LAnklePitch',aFootPitch)
            transKnee = self.local_trans('LKneePitch', aKneePitch)
        else:
            transFootRoll = self.local_trans('RAnkleRoll',aFootRoll)
            transFootPitch = self.local_trans('RAnklePitch', aFootPitch)
            transKnee = self.local_trans('RKneePitch',aKneePitch)
        
        thighToFoot = transFootRoll.dot(transFootPitch.dot(transKnee))

        #TODO: need to be fixed
        hipOrthToThigh = inv(thighToFoot[0:3,0:3]).dot(inv(footToHipOrth_m))
        #hipOrthToThigh = hipOrthToThigh[0:3,0:3]

        hipRoll = np.arcsin(hipOrthToThigh[2,1]) - math.pi/4
        hipPitch = math.atan2(-1*hipOrthToThigh[2,0],hipOrthToThigh[2,2])
        hipYaw = math.atan2(-1*hipOrthToThigh[0,1],hipOrthToThigh[1,1])




        joint_angles = [0.0,0.0,0.0,aKneePitch, aFootPitch, aFootRoll]

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        joint_angles = self.inverse_kinematics(effector_name, transform)

        names = self.chains[effector_name]

        keys = []

        for key in self.chains:
            if key == 'LLeg':
                times = []
                for i,joint in enumerate(self.chains[key]):
                    times.append([1.0,2.0])
                    keys.append([[0.5*joint_angles[i], [3, 0.0, 0.0], [3, 0.0, 0.0]], [joint_angles[i], [3, 0.0, 0.0], [3, 0.0, 0.0]]])

        '''
        for i,joint in enumerate(self.chains[key]):
                    keys.append([joint_angles[i],[3,0.0,0.0],[3,0.0,0.0]])
                    times.append([1.0])
            else:
                times = []
                for i, joint in enumerate(self.chains[key]):
                    keys.append([joint_angles[i], [3, 0.0, 0.0], [3, 0.0, 0.0]])
                    times.append([1.0])
        '''



        self.keyframes = (names, times, keys)



if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[1, -1] = 0.05
    T[2, -2] = 0.26

    agent.set_transforms('LLeg', T)
    agent.run()
