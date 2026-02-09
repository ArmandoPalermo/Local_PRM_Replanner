#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

#Classe utilizzata per monitorare la pericolosità di un ostacolo in relazione all'arco che il robot sta attraversando
class ObstacleMonitor:
    def __init__(self):

        self.obstacle = None
        self.current_segment = []

        #Lettura dei messaggi per la posizione dell'ostacolo e del nodo corrente che il robot sta attraversando
        rospy.Subscriber("/dynamic_obstacle", Marker, self.get_obstacles_position)
        rospy.Subscriber("/current_edge_nodes", Float64MultiArray, self.get_current_edge)
        #Pubblica sul seguente topic un messaggio contenente la posa dell'ostacolo pericoloso
        self.obstacle_notsafe_pub = rospy.Publisher("/obstacle_notsafe", Pose, queue_size=1)

    # Funzione che restituisce l'arco attuale che il robot sta attraversanto
    def get_current_edge(self, msg):
        first_edge = np.array([msg.data[0], msg.data[1]])
        second_edge = np.array([msg.data[2], msg.data[3]])
        self.current_segment = (first_edge, second_edge)


    #Ottenimento la posizione degli ostacolo -- In maniera semplificata vengono letti da un topic
    #Andrebbero rilevati in qualche modo, nei casi reali
    def get_obstacles_position(self,msg):
            center = np.array([ msg.pose.position.x,msg.pose.position.y])
            radius = msg.scale.x * 0.5  # scale.x è il diametro del ciliidro
            # tupla
            self.obstacle = (center, radius)

    #Verifica la distanza tra l'ostacolo e un segmento
    def get_dist_obs_seg(self, obstacle, segment):
        center = obstacle[0]
        radius = obstacle[1]

        start_segment = segment[0]
        end_segment = segment[1]

        ap = center - start_segment
        ab = end_segment - start_segment

        den = np.dot(ab, ab)
        if den < 1e-6:
            dist = np.linalg.norm(center - start_segment)
        else:
            t = np.dot(ap, ab) / den
            t = np.clip(t, 0.0, 1.0)
            closest = start_segment + t * ab
            dist = np.linalg.norm(center - closest)

        return dist - radius  # distanza reale bordo–segmento

    #Se la distanza tra l'ostacolo e il segmento(arco) è minore di zero allora l'arco attraversa l'ostacolo e quindi puoò generare una collisione
    #In questo caso l'ostacolo viene bollato come pericoloso e viene notificato al corrispondente topic
    def get_obstacles_on_current_segment(self):
        dist = self.get_dist_obs_seg(self.obstacle, self.current_segment)
        if dist < 0.0:
            self.pub_obstacle_not_safe(self.obstacle)

    #Definizione e publicazione del messaggio contenente la posa dell'ostacolo pericoloso
    def pub_obstacle_not_safe(self, obs):
        pose = Pose()
        pose.position.x = obs[0][0]
        pose.position.y = obs[0][1]
        pose.position.z = obs[1]  #Non è l'altezza ma il raggio dell'ostacolo

        self.obstacle_notsafe_pub.publish(pose)



if __name__ =="__main__":
    rospy.init_node("Obstacle_Monitor")
    monitor = ObstacleMonitor()
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            if monitor.current_segment and monitor.obstacle is not None:
                monitor.get_obstacles_on_current_segment()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
