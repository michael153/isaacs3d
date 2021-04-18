import rospy
import numpy as np
from geometry_msgs.msg import Vector3

class DummyRadiationPublisher:
    def __init__(self, topic, rate=10):
        self.pub = rospy.Publisher(topic, Vector3, queue_size=10)
        self.rate = rospy.Rate(10)

    def talk(self):
        while not rospy.is_shutdown():
            vec = Vector3()
            vec.x = 1
            vec.y = 1
            vec.z = np.random.poisson(lam=10)
            self.pub.publish(vec)
            rate.sleep()

def main():
    rospy.init_node("dummy_lamp")
    p = DummyRadiationPublisher('/lamp/data')
    p.talk()
    rospy.spin()