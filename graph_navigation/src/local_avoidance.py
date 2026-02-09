#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from astar_algorithm import astar

#Classe che si occupa dell'avoidance degli ostacoli pericolosi generando il grafo locale e definendo il path tramite A*
class LocalAvoidance:
    def __init__(self):
        self.obstacle = None
        self.same_obstacles = False
        self.current_segment = []
        self.robot_position = None
        self.local_graph_ready = False
        self.collision_on_current_segment = False
        self.next_global_node = None
        self.local_graph = {}

        #Lettura dei messaggi relativi all'ostacolo pericoloso, all'arco di attraversamento del robot e alla posizione dello stesso
        rospy.Subscriber("/obstacle_notsafe", Pose, self.get_obstacles)
        rospy.Subscriber("/current_edge_nodes", Float64MultiArray, self.get_current_edge)
        rospy.Subscriber("/ekf_robot", Marker, self.get_actual_robot_position)
        rospy.Subscriber("/next_global_node", Pose, self.get_next_global_node)
        #Pubblicazione Marker per nodi e archi del grafo locale
        self.local_edges_pub = rospy.Publisher("/local_graph/edges", Marker, queue_size=1)
        self.local_nodes_pub = rospy.Publisher("/local_graph/nodes", Marker, queue_size=1)
        self.local_path_pub = rospy.Publisher("/local_path", Marker, queue_size=1)

    def get_next_global_node(self,msg):
        x = msg.position.x
        y = msg.position.y
        self.next_global_node = np.array([x,y])

    #Funzione di callback che setta la posa del robot stimata con ekf letta dal topic corrispondente
    def get_actual_robot_position(self, msg):
        self.robot_position = np.array([msg.pose.position.x, msg.pose.position.y])

    # Funzione di callback che setta la l'ostacolo pericoloso da evitare
    def get_obstacles(self, msg):
        self.collision_on_current_segment = False

        if not self.current_segment:
            return

        # Quando ci sono ostacoli pericolosi vengono prelevati dal topic
        center = np.array([msg.position.x, msg.position.y])
        radius = msg.position.z
        new_obs = (center, radius)

        #Gestione lettura dell'ostacolo ripetuto per evitare ripianificazione continua sullo stesso ostacolo
        if self.obstacle is None:
            self.obstacle = new_obs
            self.same_obstacles = False
        else:
            if not self.same_obstacle(self.obstacle, new_obs):
                rospy.loginfo("NEW OBSTACLE")
                self.obstacle = new_obs
                self.same_obstacles = False
            else:
                self.same_obstacles = True

        # verifica collisione solo rispetto all'arco corrente
        dist = self.get_dist_obs_seg(new_obs,self.current_segment)

        if dist < 0.0:
            self.collision_on_current_segment = True

    #FUnzione di callback che estrae dal topic l'arco attuale che il robot sta percorrendo
    #Gestione stesso arco fatta in maniera simile per lo stesso motivo dell'ostacolo
    def get_current_edge(self, msg):
        first_node = np.array([msg.data[0], msg.data[1]])
        second_node = np.array([msg.data[2], msg.data[3]])
        new_segment = (first_node, second_node)

        if self.current_segment:
            same_start = np.allclose(self.current_segment[0], new_segment[0])
            same_end = np.allclose(self.current_segment[1], new_segment[1])

            if not (same_start and same_end):
                self.local_graph_ready = False
                self.same_obstacles = False
                self.collision_on_current_segment = False

        self.current_segment = new_segment

    #Funzione che controlla l'uguaglianza tra 2 pose di 2 ostacoli
    def same_obstacle(self, obs1, obs2, pos_tol=1e-3, rad_tol=1e-3):
        c1, r1 = obs1
        c2, r2 = obs2
        return np.allclose(c1, c2, atol=pos_tol) and abs(r1 - r2) < rad_tol

    #Metodo principale che permette di evitare la collisione tra il robot e l'ostacolo
    def avoid_collision(self, obstacle):
        #Numero di nodi randomici utilizzati per costruire il grafo locale
        n_of_random_nodes = 30

        #Definizione parametri della finestra di azione del replanner
        window_center = 0.5 * (self.current_segment[0] + self.current_segment[1])
        obs_center, obs_radius = obstacle
        window_radius = np.linalg.norm(obs_center - window_center) + 2.0 * obs_radius

        # Generazione dei nodi casuali
        local_nodes = np.random.uniform(low=-window_radius,high=window_radius,size=(n_of_random_nodes, 2)) + window_center

        #Filtraggio dei nodi che intersecano l'ostacolo
        local_nodes = np.array(self.filter_valid_nodes(local_nodes, obstacle))

        # Aggiunta dei nodi iniziale e finale del grafo locale
        start_node = self.robot_position
        goal_node = self.next_global_node

        local_nodes = np.vstack([start_node, local_nodes, goal_node])

        local_edges = []
        max_edge_len = 3.5 #Lunghezza massima degli archi del grafo locale

        self.local_graph = {tuple(n): [] for n in local_nodes}

        #Costruzione degli archi tra i nodi generati casualmente
        # Non vengono considerati quelli che intersecano l'ostacolo
        for i in range(len(local_nodes)):
            for j in range(i + 1, len(local_nodes)):
                dist = np.linalg.norm(local_nodes[i] - local_nodes[j])
                if dist < max_edge_len:
                    segment = (local_nodes[i], local_nodes[j])

                    if self.get_dist_obs_seg(obstacle, segment) < 0.5:
                        continue

                    p1 = Point(segment[0][0], segment[0][1], 0.0)
                    p2 = Point(segment[1][0], segment[1][1], 0.0)
                    local_edges.append(p1)
                    local_edges.append(p2)

                    # aggiunta degli archi al grafo locale (bidirezionale)
                    n1 = tuple(segment[0])
                    n2 = tuple(segment[1])
                    self.local_graph[n1].append((n2, dist))
                    self.local_graph[n2].append((n1, dist))

        #Inserimento nodo start e goal nel grafo
        self.local_graph[tuple(start_node)] = []
        self.local_graph[tuple(goal_node)] = []

        # FORZO  il collegamento start al nodo più vicino
        dists = [
            np.linalg.norm(local_nodes[i] - start_node)
            for i in range(1, len(local_nodes) - 1)
        ]
        nearest_start = 1 + np.argmin(dists)
        dist = np.linalg.norm(local_nodes[nearest_start] - start_node)
        self.local_graph[tuple(start_node)].append((tuple(local_nodes[nearest_start]), dist))
        self.local_graph[tuple(local_nodes[nearest_start])].append((tuple(start_node), dist))

        # FORZO  il collegamento goal al nodo più vicino
        dists = [
            np.linalg.norm(local_nodes[i] - goal_node)
            for i in range(1, len(local_nodes) - 1)
        ]

        nearest_goal = 1 + np.argmin(dists)
        dist = np.linalg.norm(local_nodes[nearest_goal] - goal_node)
        self.local_graph[tuple(goal_node)].append((tuple(local_nodes[nearest_goal]), dist))
        self.local_graph[tuple(local_nodes[nearest_goal])].append((tuple(goal_node), dist))

        local_edges.append(Point(start_node[0], start_node[1], 0.0))
        local_edges.append(Point(local_nodes[nearest_start][0], local_nodes[nearest_start][1], 0.0))

        local_edges.append(Point(local_nodes[nearest_goal][0], local_nodes[nearest_goal][1], 0.0))
        local_edges.append(Point(goal_node[0], goal_node[1], 0.0))

        self.local_nodes_pub.publish(self.make_local_nodes_marker(local_nodes))
        self.local_edges_pub.publish(self.make_local_edges_marker(local_edges))

        #Generazione path locale
        local_path = astar(self.local_graph, tuple(start_node), tuple(goal_node))

        if local_path:
            self.local_path_pub.publish(self.make_local_path_marker(local_path))

    #FUnzione utilizzata per filtrare i nodi validi
    def filter_valid_nodes(self, nodes, obstacle):
        filtered_nodes = []

        center, radius = obstacle
        for n in nodes:
            if np.linalg.norm(n - center) > radius:
                filtered_nodes.append(n)

        return filtered_nodes

    #Funzione utilizzata per calcolare la distanza tra un ostacolo e un arco
    def get_dist_obs_seg(self, obstacle, segment):
        center, radius = obstacle

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

    #Definizione del messaggio utile per rappresentare in Rviz gli archi del local graph
    def make_local_edges_marker(self, edges):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "local_graph_edges"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD

        m.scale.x = 0.05  # spessore linee

        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        m.points = edges  # lista di Point (a coppie)

        return m

    # Definizione del messaggio utile per rappresentare in Rviz i nodi del local graph
    def make_local_nodes_marker(self, nodes):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "local_graph_nodes"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD

        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15

        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0

        for n in nodes:
            p = Point(n[0], n[1], 0.05)
            m.points.append(p)

        return m

    # Definizione del messaggio utile per rappresentare in Rviz il path locale calcolato con A*
    def make_local_path_marker(self, path):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "local_graph_path"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD

        m.scale.x = 0.08  # spessore linea

        # NERO
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            m.points.append(Point(p1[0], p1[1], 0.1))
            m.points.append(Point(p2[0], p2[1], 0.1))
        return m


if __name__ == "__main__":
    rospy.init_node("Local_Avoidance")
    local_re_planner = LocalAvoidance()
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():

            if not local_re_planner.current_segment:
                rate.sleep()
                continue

            # pianifica solo se c'è collisione sull'arco corrente
            if not local_re_planner.collision_on_current_segment:
                rate.sleep()
                continue

            # genera solo se non esiste o l'ostacolo è cambiato
            if (not local_re_planner.local_graph_ready or
                    not local_re_planner.same_obstacles):

                if local_re_planner.obstacle is not None:
                    local_re_planner.avoid_collision(local_re_planner.obstacle)
                    local_re_planner.local_graph_ready = True
                    local_re_planner.same_obstacles = True

            rate.sleep()
    except rospy.ROSInterruptException:
        pass