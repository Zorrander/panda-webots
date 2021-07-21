# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Joint state publisher."""

import rospy
from sensor_msgs.msg import JointState


class CameraPublisher(object):
    """Publish as a ROS topic the joint state."""


    def __init__(self, robot, jointPrefix, nodeName):
        """Initialize the motors, position sensors and the topic."""
        self.robot = robot
        self.jointPrefix = jointPrefix
        self.camera = robot.getDevice("panda_camera")
        self.camera.enable(500)
        self.publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

    def publish(self):
        """Publish the 'joint_states' topic with up to date value."""
        msg = JointState()
        
        img = self.camera.getImage()

        
        self.publisher.publish(msg)
