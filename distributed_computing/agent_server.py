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
import threading
import numpy as np


class ServerAgent(InverseKinematicsAgent, PostureRecognitionAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.server = SimpleXMLRPCServer(('localhost', 8888),allow_none=True)
        self.server.register_function(self.get_angle, 'get_angle')
        self.server.register_function(self.set_angle, 'set_angle')
        self.server.register_function(self.get_posture, 'get_posture')
        self.server.register_function(self.execute_keyframes, 'execute_keyframes')
        self.server.register_function(self.get_transform, 'get_transform')
        self.server.register_function(self.set_transform, 'set_transform')
        #self.server.serve_forever()
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()


    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint.get(joint_name)
    
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
        self.set_transforms(effector_name, np.matrix(transform))

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

