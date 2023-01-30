from __future__ import print_function
import sys
import rospy #Python befehle fuer ROS
from math import * #Zum Berechnen von Wurzeln, etc
from std_msgs.msg import String
from std_msgs.msg import Float32 #Zum Senden von Gleitkommazahlen an andere Knoten
from std_msgs.msg import Empty
from gazebo_msgs.msg import ModelStates #Zum Empfangen der Position des Bootes
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion #Zur Umwandlung des Kurses (Heading)
pi=3.141592654

#Trajektorienplanung: Erstellen der Hindernisse und setzen der Ziele fuer das Boot

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

class Szene (): #Erstellt Tonnenpaare fuer eine Zielliste, die von der Bahnfuerung abgefagt werden kann
    def __init__(self):
        red_bound_0=Boje(-475,185) #Tonnen auf der roten Seite
        red_bound_1=Boje(-463,201)
        red_bound_2=Boje(-454,219)
        red_bound_3=Boje(-483,269)
        temp_r1=Boje(-468,245) #Nicht sichtbare Tonnen um die Bahnfuehrung zu verbessern
        temp_r2=Boje(-519,221)
        temp_g1=Boje(-450,255)
        temp_g2=Boje(-511,220)
        green_bound_0=Boje(-491,196) #Tonnen auf der gruenen Seite
        green_bound_1=Boje(-481,212)
        green_bound_2=Boje(-469,229)
        green_bound_3=Boje(-477,258)
        tp1=Tonnenpaar(red_bound_0,green_bound_0) #Erstellen der Tonnenpaare
        tp2=Tonnenpaar(red_bound_1,green_bound_1)
        tp3=Tonnenpaar(red_bound_2,green_bound_2)
        tp4=Tonnenpaar(temp_r1,temp_g1)
        tp5=Tonnenpaar(red_bound_3,green_bound_3)
        tp6=Tonnenpaar(temp_r2,temp_g2)
        self.ziele = [tp1,tp2,tp3,tp4,tp5,tp6] #Tonnenpaare werden in einem Array zusammengefasst

    def getZiele (self): #Uebergabe der Zielliste an die Bahnfuehrung
        return self.ziele

#Bahnfuehrung: Steuern des Bootes

class PIDRegler (): #Klasse fuer die PID Regelung der RPM (Staerke) des Bootsantriebes
  def __init__(self, dt, max, min, kp, ki, kd) :
    self.dt  = dt #Abtastrate, hier mit dem Counter (in callback Funktion) zu verknuepfen, der bestimmt wie oft der PID ausgefuehrt wird
    self.max = max #Maximaler Stellwert, der an die Regelstrecke uebergeben wird
    self.min = min #Minimaler Stellwert, der an die Regelstrecke uebergeben wird
    self.kp  = kp
    self.kd  = kd
    self.ki  = ki
    self.err = 0.0
    self.int = 0.0

  def berechnen (self,error): #Berechnet den Stellwert anhand der Sollgroesse und der Istgroesse
    P = self.kp * error;
    self.int += error * self.dt; #Aufintegrieren des Fehlers fuer I-Anteil
    I = self.ki * self.int;
    D = self.kd * (error - self.err) / self.dt;
    #print ("P: %f, I: %f, D: %f"%(P,I,D))
    output = P + I + D + 0.6; #0.6 ist der Stellwert fuer beide Motoren, wenn das Schiff geradeaus fahren soll
    if output > self.max :
        output = self.max
    elif output < self.min :
        output = self.min
    self.err = error; #Speichern des Fehlers fuer den D-Anteil
    return(output);

class Positionierung ():
    def __init__(self):
        umgebung = Szene () #Initialisiert Trajektorenplanung
        self.ziele = umgebung.getZiele() #Uebernimmt die Ziele von der Trajektorenplanung
        self.aktuellesZiel=0 #Nummer des aktuellen Zieles im Array
        self.posX=0
        self.posY=0
        self.counter=0
        self.heading=0 #Kurs des Schiffes
        #Bb = Backbord (In Fahrtrichtung links), Stb = Steuerbord (In Fahrtrichtung rechts)
        self.reglerBb=PIDRegler(0.01, 1.5, 0, 0.22, 0.15, 0.0375) #Je ein Regler pro Motor, empfehle keinen zu grossen P-Anteil
        self.reglerStb=PIDRegler(0.01, 1.5, 0, 0.22, 0.15, 0.0375) #O als Minimalwert, 1.5 als Maximalwert
        self.bb_pub = rospy.Publisher("/wamv/thrusters/left_thrust_cmd",Float32,queue_size=10) #Zum Senden der Stellwerte an den Motor via ROS Topic
        self.stb_pub = rospy.Publisher("/wamv/thrusters/right_thrust_cmd",Float32,queue_size=10)

    def bootSteuern (self):
        ziel=self.ziele[self.aktuellesZiel] #Waehlt aktuelles Ziel aus Array
        zielX=ziel.mittelpunktX
        zielY=ziel.mittelpunktY
        abstandX=zielX-self.posX #Abstand zum Ziel
        abstandY=zielY-self.posY
        zielAbstand=sqrt(abstandX*abstandX+abstandY*abstandY)
        zielWinkel=atan(abstandY/abstandX) #Peilung des Zieles, Winkel geht von -pi/2 bis pi/2
        if abstandX<0 and abstandY>0: #Erweiterung der Peilung auf Winkel von -pi bis pi
            zielWinkel+=pi
        elif abstandX<0 and abstandY<0:
            zielWinkel-=pi

        if (zielAbstand <=5): #Wenn der Abstand zum Ziel klein genug ist (grosszuegig gewaehlt, damit genug Zeit fuer Kursaenderung)
            self.aktuellesZiel+= 1 #Naechstes Ziel im Array
            print ("Etappenziel erreicht")
        else:
            zielWinkelDiff=zielWinkel-self.heading #Differenz fuer moegliche Ausgabe und Korrektur
            if zielWinkelDiff > pi: #Vermeidet das Schiff sich im Kreis dreht, wenn ziel +/- pi ueberschreitet
                zielWinkelDiff-=2*pi
            elif zielWinkelDiff < -pi:
                zielWinkelDiff+=2*pi
            RPMbb=self.reglerBb.berechnen(zielWinkelDiff*-1) #Berechnung des Stellwertes Backbord Motor
            RPMstb=self.reglerStb.berechnen(zielWinkelDiff) #Berechnung des Stellwertes Steuerbord Motor
            self.bb_pub.publish(RPMbb) #Veroeffentlichen des Stellwertes via des ROS Topics
            self.stb_pub.publish(RPMstb)
            #Verschiedene Ausgaben nach belieben ein-/auskommentierbar
            #print ("Ziel X: %f"%zielX)
            #print ("Ziel Y: %f"%zielY)
            print ("Zielwinkel: %f, Zielwinkel Differenz: %f"%(zielWinkel, zielWinkelDiff))
            #print ("Zielabstand: %f"%zielAbstand)
            print ("Set Backbord: %f, Set Steuerbord: %f"%(RPMbb,RPMstb))


    def bootHeading (self, position): #Umwandlung des Kurses aus Quaternion Koordinaten in Winkel von -pi bis pi
        quaternion = [position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w]
        (roll,pitch,yaw) = euler_from_quaternion(quaternion) #Nur yaw (rotation um z-Achse) wird verwendet
        return yaw    

    def callback(self,data): #Funktion wird jedes Mal ausgefuehrt sobald es eine neue Position im Topic gibt
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
        

if __name__ == '__main__': #Wird mit starten des Skrips ausgefuehrt
    rospy.init_node('positionierung', anonymous=True) #ROS Knoten wird erstellt
    positionierung=Positionierung() #Klasse fuer die Steuerung des Bootes wird initialisiert
    rospy.Subscriber("/gazebo/model_states",ModelStates, positionierung.callback) #Abboniert das Topic mit den Positionsdaten und fuehrt Callback Funktion aus
    try:
        rospy.spin() #Wird bis zum manuellen Abbruch ausgefuehrt
    except rospy.ROSInterruptException:
        pass