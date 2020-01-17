'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from recognize_posture import PostureRecognitionAgent
from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib


class ServerAgent(InverseKinematicsAgent, PostureRecognitionAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        server = SimpleXMLRPCServer(("localhost", 8888))
        server.register_function(ServerAgent.get_angle, "get_angle")
        server.register_function(ServerAgent.set_angle, "set_angle")
        server.register_function(ServerAgent.get_posture, "get_posture")
        server.register_function(ServerAgent.execute_keyframes, "execute_keyframes")
        server.register_function(ServerAgent.get_transform, "get_transform")
        server.register_function(ServerAgent.set_transform, "set_transform")
        server.register_instance(agent)
        server.serve_forever()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.perception.joint[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        return self.recognize_posture(self.perception)  #recognize posture??

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.set_transforms(effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

