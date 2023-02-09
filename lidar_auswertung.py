# Import required libraries
from __future__ import print_function
import sys
import rospy #Python befehle fuer ROS
from math import * #Zum Berechnen von Wurzeln, etc
from std_msgs.msg import String
from std_msgs.msg import Float32 #Zum Senden von Gleitkommazahlen an andere Knoten
from std_msgs.msg import Empty
from gazebo_msgs.msg import ModelStates #Zum Empfangen der Position des Bootes
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion #Zur Umwandlung des Kurses (Heading)
import sensor_msgs.point_cloud2 as pc2
import pcl


#Speicherklassen fuer Objekte

class Boje (): #Speichert die Koordinaten einer Boje. Kann um Farben, Funktionen, etc erweitert werden
    def __init__(self, _posX, _posY):
        self.posX=_posX
        self.posY=_posY

class Tonnenpaar (): #Verbindet zwei Bojen zu einem Tonnenpaar
    def __init__(self, _tonneR, _tonneG):
        self.tonneR=_tonneR
        self.tonneG=_tonneG
        xAbstand=self.tonneR.posX-self.tonneG.posX
        yAbstand=self.tonneR.posY-self.tonneG.posY
        self.abstand=sqrt(xAbstand*xAbstand+yAbstand*yAbstand) #Abstand zwischen den Bojen
        self.mittelpunktX=self.tonneG.posX+xAbstand/2 #Mittelpunkt zwischen den beiden Bojen
        self.mittelpunktY=self.tonneG.posY+yAbstand/2

#Objekterkennung

class Lidar ():
    def __init__(self):
        self.posX=0
        self.posY=0
        self.counter=0
        self.heading=0 #Kurs des Schiffes
        #Bb = Backbord (In Fahrtrichtung links), Stb = Steuerbord (In Fahrtrichtung rechts)

    def bootHeading (self, position): #Umwandlung des Kurses aus Quaternion Koordinaten in Winkel von -pi bis pi
        quaternion = [position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w]
        (roll,pitch,yaw) = euler_from_quaternion(quaternion) #Nur yaw (rotation um z-Achse) wird verwendet
        return yaw    

    def callback_pos (self,data): #Funktion wird jedes Mal ausgefuehrt sobald es eine neue Position im Topic gibt
        if self.counter==100: #Um Abtastrate zu begrenzen, wurde hier ein Counter implementiert, nur jeder 100. Wert wird uebernommen
            position = data.pose[17] #In dem Topic sind die Positionsdaten von allen Objekten in der Simulation, hier muss das Boot ausgewaehlt werden
            self.posX=position.position.x #Extrahieren der benoetigten Positionsdaten zur Ausgabe
            self.posY=position.position.y
            self.heading=self.bootHeading (position)
            #print ("X: %f"%self.posX)
            #print ("Y: %f"%self.posY)
            print ("Orientierung: %f"%self.heading)
            if self.aktuellesZiel<len(self.ziele): #So lange ausgefuehren, wie Ziele in dem Array sind
                self.bootSteuern ()
            self.counter=0
        else:
            self.counter+=1

    def callback_lidar (self,cloud_msg): 
        # Convert the PointCloud2 message to a PCL PointXYZRGB point cloud
        cloud = pcl.PointCloud_PointXYZRGB()
        pcl.io.pointCloud2.read_points(cloud_msg, cloud)
    
        # Filter the point cloud using a voxel grid filter
        vox = cloud.make_voxel_grid_filter()
        vox.set_leaf_size(0.1, 0.1, 0.1)
        cloud = vox.filter()
    
        # Filter the point cloud using a statistical outlier filter
        outlier_filter = cloud.make_statistical_outlier_filter()
        outlier_filter.set_mean_k(50)
        outlier_filter.set_std_dev_mul_thresh(1.0)
        cloud = outlier_filter.filter()

        

if __name__ == '__main__': #Wird mit starten des Skrips ausgefuehrt
    rospy.init_node('lidar', anonymous=True) #ROS Knoten wird erstellt
    lidar=Lidar() #Klasse fuer die Steuerung des Bootes wird initialisiert
    rospy.Subscriber("/gazebo/model_states",ModelStates, lidar.callback_pos) #Abboniert das Topic mit den Positionsdaten und fuehrt Callback Funktion aus
    rospy.Subscriber("/wamv/sensors/lidars/lidar_wamv/points",PointCloud2, lidar.callback_lidar) #Abboniert das Topic mit den Positionsdaten und fuehrt Callback Funktion aus
    try:
        rospy.spin() #Wird bis zum manuellen Abbruch ausgefuehrt
    except rospy.ROSInterruptException:
        pass