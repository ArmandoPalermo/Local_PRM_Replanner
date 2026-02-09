#!/usr/bin/env python3
from symbol import continue_stmt

import rospy
import math
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import random
import numpy as np
from geometry_msgs.msg import Pose

class DiffDriveRobot:

    def __init__(self):
        #Definizione path globale e locale
        self.global_path = []
        self.local_path = []

        #Indicizzazione del nodo target dell'arco che il robot sta attraversando
        #Utile per inseguimento della traiettoria
        self.global_index = 0
        self.local_index = 0
        self.using_local = False

        # Iperparametri di rumore del robot
        self.alpha1 = 0.01
        self.alpha2 = 0.01
        self.alpha3 = 0.01
        self.alpha4 = 0.01

        # Rumore sensore
        self.sigma_z = 0.3

        # Beacon nella mappa continua (Non sul grafo)
        self.beacons = np.array([
            [-10.0, 0.0],
            [10.0, 0.0],
            [0.0, 10.0]
        ])

        # posa inziale del robot
        self.x = -18.0
        self.y = -15.0
        self.theta = 0.0

        #Il nodo legge i messaggi dal path da seguire e dal path locale quando è presente
        rospy.Subscriber("/global_graph_path/edges", Marker, self.get_global_path)
        rospy.Subscriber("/local_path", Marker, self.get_local_path)

        #Pubblica messaggi relativi alle misurazioni dei beacon
        self.meas_pub = rospy.Publisher("/beacon_measurements", Float64MultiArray, queue_size=1)
        # Pubblica messaggi relativi alle velocità del robot utili all'inseguimento dell'arco del path
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Pubblica messaggi relativi ai due nodi (start ed end) dell'arco che il robot sta attraversando
        # Utile per il local replanner
        self.edge_pub = rospy.Publisher("/current_edge_nodes", Float64MultiArray, queue_size=1)
        self.next_global_node = rospy.Publisher("/next_global_node", Pose, queue_size=1)
        self.last_time = rospy.Time.now()

    # Converte un Marker contenente una lista di segmenti (edges)
    # in una sequenza ordinata di nodi da inseguire
    def parse_marker_path(self, msg):
        path = []
        for i in range(0, len(msg.points), 2):
            p1, p2 = msg.points[i], msg.points[i + 1]
            if i == 0:
                path.append((p1.x, p1.y))
            path.append((p2.x, p2.y))
        return path


    # Funzione di callback richiamata quando arriva un messaggio sul topic contenente il path globale
    def get_global_path(self, msg):
        self.global_path = self.parse_marker_path(msg)
        if not self.using_local:
            self.global_index = 0

    # Funzione di callback richiamata quando arriva un messaggio sul topic contenente il path locale
    # Flag using_local settato a True per notificare il passsaggio dell'inseguimento dal path globale a quello locale
    def get_local_path(self, msg):
        self.local_path = self.parse_marker_path(msg)
        self.local_index = 0
        self.using_local = True

    #Seleziona il path attivo tenendo in considerazione il flag using_local e controllando che il robot non abbia attraversato tutto il path locale
    def select_active_path(self):
        if self.using_local and self.local_index < len(self.local_path):
            return self.local_path, self.local_index
        else:
            self.using_local = False
            #Pubblicazione edge di arrivo globale--> global_index
            self.pub_next_global_node(self.global_index)
            return self.global_path, self.global_index

    # Passo di movimento del robot
    def step(self):
        # Estrazione path e indice di prosecuzione attuale
        path, index = self.select_active_path()

        #COntrollo se il path è stato attraversato tutto
        if index >= len(path):
            return


        #Calcolo degli incrementi utili alla definizione delle velocità che portano il robot dalla posizione attuale al nodo successivo del path
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        # Nodo target
        xg, yg = path[index]

        dx = xg - self.x
        dy = yg - self.y
        dist = math.hypot(dx, dy)

        angle_goal = math.atan2(dy, dx)
        error_angle = angle_goal - self.theta
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
        # Legge di controllo abbastanza semplice e standard
        v = 1.2
        w = 3.0 * error_angle

        #Cambio degli indici quando il robot è abbastanza vicino al nodo target
        if dist < 0.3:
            if self.using_local:
                self.local_index += 1
                if self.local_index >= len(self.local_path):
                    self.using_local = False
            else:
                self.global_index += 1
            return

        self.publish_cmd_vel(v, w)

        # Rumore
        var_v = self.alpha1 * v ** 2 + self.alpha2 * w ** 2
        var_w = self.alpha3 * v ** 2 + self.alpha4 * w ** 2

        v = v + random.gauss(0.0, math.sqrt(var_v))
        w = w + random.gauss(0.0, math.sqrt(var_w))

        # cinematica
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        self.publish_measurements()
        self.publish_current_edge(path, index)

    def pub_next_global_node(self, global_index):

        if  self.global_path and global_index <= len(self.global_path) - 1 :

            x_node = self.global_path[global_index][0]
            y_node = self.global_path[global_index][1]

            pos = Pose()
            pos.position.x = x_node
            pos.position.y = y_node
            self.next_global_node.publish(pos)

    #Definizione e pubblicazione del messaggio relativo alle misurazioni dei beacon
    def publish_measurements(self):
        pos = np.array([self.x, self.y])
        z = []
        for b in self.beacons:
            z.append(np.linalg.norm(pos - b) + random.gauss(0.0, self.sigma_z))

        msg = Float64MultiArray()
        msg.data = z
        self.meas_pub.publish(msg)

    # Definizione e pubblicazione del messaggio relativo alle velocità definite dalla legge di controllo
    def publish_cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    #  # Definizione e pubblicazione del messaggio relativo ai nodi dell'arco attuale che il robot sta percorrendo
    def publish_current_edge(self, path, index):
        if index == 0 or index >= len(path):
            return

        x1, y1 = path[index - 1]
        x2, y2 = path[index]

        msg = Float64MultiArray()
        msg.data = [x1, y1, x2, y2]
        self.edge_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("diffdrive_robot")

    robot = DiffDriveRobot()
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            robot.step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass