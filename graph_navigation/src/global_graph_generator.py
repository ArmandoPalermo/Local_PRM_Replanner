#!/usr/bin/env python3


import rospy
import numpy as np
import yaml
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Calcola la distanza tra 2 nodi
# param -- node1,node2 : nodi tra cui viene calcolata la distanza
def distance(node1,node2):
    p1 = np.array([node1["x"], node1["y"]])
    p2 = np.array([node2["x"], node2["y"]])
    return np.linalg.norm(p1 - p2)

# Genera un marker usato per rappresentare i nodi in Rviz
# param -- frame_id : il frame rispetto al quale viene generato('map' nel caso del progetto)
def node_marker_definition(frame_id):
    node_marker = Marker()
    node_marker.header.frame_id = frame_id
    node_marker.header.stamp = rospy.Time.now()
    node_marker.ns = "graph_nodes"
    node_marker.id = 0
    node_marker.type = Marker.SPHERE_LIST
    node_marker.action = Marker.ADD
    node_marker.scale.x = 0.25
    node_marker.scale.y = 0.25
    node_marker.scale.z = 0.25
    node_marker.color.r = 0.0
    node_marker.color.g = 1.0
    node_marker.color.b = 0.0
    node_marker.color.a = 1.0
    return node_marker

# Genera un marker usato per rappresentare gli archi tra i nodi in Rviz
# param -- frame_id : il frame rispetto al quale viene generato('map' nel caso del progetto)
def edge_marker_definition(frame_id):
    edge_marker = Marker()
    edge_marker.header.frame_id = frame_id
    edge_marker.header.stamp = rospy.Time.now()
    edge_marker.ns = "graph_edges"
    edge_marker.id = 1
    edge_marker.type = Marker.LINE_LIST
    edge_marker.action = Marker.ADD

    edge_marker.scale.x = 0.05

    edge_marker.color.r = 0.0
    edge_marker.color.g = 0.0
    edge_marker.color.b = 1.0
    edge_marker.color.a = 1.0

    return edge_marker



if __name__ == "__main__":
    rospy.init_node("global_graph_node")

    # Publisher dei nodi e degli archi del grafo globale
    nodes_pub = rospy.Publisher("/global_graph/nodes", Marker, queue_size=1)
    edges_pub = rospy.Publisher("/global_graph/edges", Marker, queue_size=1)

    # Definizione percorso della mappa in formato YAML
    YAML_PATH = "Maps/global_graph.yaml"
    PATH_TO_PACKAGE = rospy.get_param("/graph_navigation_path", "")
    graph_file = PATH_TO_PACKAGE + "/" + YAML_PATH

    # Lettura File
    with open(graph_file, 'r') as f:
        data = yaml.safe_load(f)

    # Definizione nodi e frame id
    nodes = data["nodes"]
    frame_id = data["map"]["frame_id"]

    node_marker = node_marker_definition(frame_id)
    edge_marker = edge_marker_definition(frame_id)

    # Costruzione dei punti sulla base dei nodi inseriti nel file yaml
    for n in nodes:
        p = Point()
        p.x = n["x"]
        p.y = n["y"]
        p.z = 0.0
        node_marker.points.append(p)

    max_distance_edges = 10.0 # distanza massima per creare un arco tra due nodi

    # Costruzione degli archi sulla base dei dati inseriti nel file yaml
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if distance(nodes[i], nodes[j]) <= max_distance_edges:
                p1 = Point(nodes[i]["x"], nodes[i]["y"], 0.0)
                p2 = Point(nodes[j]["x"], nodes[j]["y"], 0.0)
                edge_marker.points.append(p1)
                edge_marker.points.append(p2)

    # Loop d pubblicazione del grafo globale
    rate = rospy.Rate(1)  # 1 Hz basta
    while not rospy.is_shutdown():
        node_marker.header.stamp = rospy.Time.now()
        edge_marker.header.stamp = rospy.Time.now()

        nodes_pub.publish(node_marker)
        edges_pub.publish(edge_marker)

        rate.sleep()