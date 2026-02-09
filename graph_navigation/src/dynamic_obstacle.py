#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

#Definizione del messaggio Marker utile per la visualizzazione in Rviz degli ostacoli
def make_obstacle(marker_id, x, y, radius):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "obstacle"
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD

    # posizione dell’ostacolo
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.5

    # raggio
    m.scale.x = radius * 2.0
    m.scale.y = radius * 2.0
    m.scale.z = 1.0

    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 0.8

    return m


if __name__ == "__main__":
    rospy.init_node("dynamic_obstacle")

    pub = rospy.Publisher("/dynamic_obstacle", Marker, queue_size=1)
    rate = rospy.Rate(10)

    # OSTACOLI su archi diversi
    obstacles = [
        # OSTACOLO 1
        dict(marker_id=0, x=5.0, y=-5.0, radius=0.8),

        # OSTACOLO 2
        dict(marker_id=0, x=10.0, y=1.5, radius=0.6),

        # OSTACOLO 3
        dict(marker_id=0, x = 10.0, y = 3.0, radius = 1),

    ]

    idx = 0
    last_switch = rospy.Time.now()

    try:
        while not rospy.is_shutdown():

            # cambio ostacolo ogni 3.8 secondi
            if (rospy.Time.now() - last_switch).to_sec() > 4.0:
                idx = (idx + 1) % len(obstacles)
                last_switch = rospy.Time.now()

            obs = obstacles[idx]
            marker = make_obstacle(obs["marker_id"],obs["x"],obs["y"],obs["radius"])

            pub.publish(marker)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass