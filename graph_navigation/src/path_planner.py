#!/usr/bin/env python3

import rospy
import yaml
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from astar_algorithm import astar

#Genera un marker usato per rappresentare il path globale da seguire in Rviz
#param -- frame_id : il frame rispetto al quale viene generato('map' nel caso del progetto)
def global_edge_marker_definition(frame_id):
    edge_marker = Marker()
    edge_marker.header.frame_id = frame_id
    edge_marker.header.stamp = rospy.Time.now()
    edge_marker.ns = "global_path"
    edge_marker.id = 0
    edge_marker.type = Marker.LINE_LIST
    edge_marker.action = Marker.ADD

    edge_marker.scale.x = 0.15

    edge_marker.color.r = 1.0
    edge_marker.color.g = 0.0
    edge_marker.color.b = 0.0
    edge_marker.color.a = 1.0

    return edge_marker


if __name__ == "__main__":
    rospy.init_node("global_path_planner")

    #Topic su cui viene pubblicato il path globale
    path_edges_pub = rospy.Publisher("/global_graph_path/edges", Marker,queue_size=1,latch=True)

    # Definizione percorso della mappa in formato YAML
    YAML_PATH = "Maps/global_graph.yaml"
    PATH_TO_PACKAGE = rospy.get_param("/graph_navigation_path", "")
    graph_file = PATH_TO_PACKAGE + "/" + YAML_PATH

    #Inizializzazione startNode ed endNode
    startNode = (-18.0, -15.0)
    endNode = (9.0, 17.0)

    with open(graph_file, 'r') as f:
        data = yaml.safe_load(f)

    frame_id = data["map"]["frame_id"]

    nodes = [(n["x"], n["y"]) for n in data["nodes"]]
    graph = {node: [] for node in nodes}
    max_distance_edges = 10.0

    #Costruzione del grafo globale da dare ad A*
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            p1 = np.array(nodes[i])
            p2 = np.array(nodes[j])

            dist = np.linalg.norm(p1 - p2)

            if dist <= max_distance_edges:
                n1 = nodes[i]
                n2 = nodes[j]

                # arco bidirezionale esplicito
                graph[n1].append((n2, dist))
                graph[n2].append((n1, dist))

    #Estrazione del percorso globale tramite  A* e definizioone del messaggio Marker per visulizzazione di Rviz
    nodes_path = astar(graph, startNode, endNode)
    marker_path = global_edge_marker_definition(frame_id)

    for i in range(len(nodes_path) - 1):
        p1 = Point()
        p1.x, p1.y = nodes_path[i]
        p1.z = 0.05

        p2 = Point()
        p2.x, p2.y = nodes_path[i + 1]
        p2.z = 0.05

        marker_path.points.append(p1)
        marker_path.points.append(p2)

    #Definizione dei timestamp
    marker_path.header.stamp = rospy.Time.now()
    #Pubblicazione dei messaggi
    path_edges_pub.publish(marker_path)
    rospy.spin()
