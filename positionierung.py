from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_msgs.msg import Empty

class Positionierung ():
    def __init__(self):
        self.position = None

    def callback(self,data):
        latitude=String(data.latitude)
        longitude=String(data.longitude)
        print ("Latitude: %s"%latitude)
        print ("Longitude: %s"%longitude)

if __name__ == '__main__':
    rospy.init_node('positionierung', anonymous=True)
    positionierung=Positionierung()
    positionierung.position=NavSatFix()
    # Subscriber
    rospy.Subscriber("/wamv/sensors/gps/gps/fix",NavSatFix, positionierung.callback)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass