#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

#Definizione del motion model --> classico differential drive
def motion_model(dt, x, y, v, w, theta):
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new =  theta + w * dt
    return x_new, y_new, theta_new


class EKFEstimate:

    def __init__(self):
        rospy.init_node("ekf_estimate")

        #Definizione dei 3 beacon della mappa
        self.beacons = np.array([
            [-10.0, 0.0],
            [10.0, 0.0],
            [0.0, 10.0]
        ])
        #Iperparametri di rumore sule velocità sull'angolo e sulle misurazioni
        self.alpha_v = 0.1
        self.alpha_theta = 0.05
        self.sigma_z = 0.5

        #Stima e covarianza
        self.x_est = np.array([-18.0, -15.0, 0])
        self.P = np.eye(3) * 0.5

        #v_nominale
        self.v_nominal = 0
        self.w = 0.0

        self.z = None
        self.last_time = rospy.Time.now()

        #Legge messaggi dal topic che contiene le misurazioni del robot e i comandi di velocità assegnati al robot
        rospy.Subscriber("/beacon_measurements", Float64MultiArray, self.set_meas)
        rospy.Subscriber("/cmd_vel", Twist, self.set_vel)
        #Pubblica sul  2 topic covarianza e posa stimata del rovot tramite EKF
        self.pub_est = rospy.Publisher("/ekf_robot", Marker, queue_size=1)
        self.pub_cov = rospy.Publisher("/ekf_covariance", Marker, queue_size=1)

    def set_vel(self, msg):
        self.v_nominal = msg.linear.x
        self.w = msg.angular.z

    def set_meas(self, msg):
        self.z = np.array(msg.data)

    #passo di localizzazione del robot
    def step(self):
        if self.z is None:
            return

        #Controllo del dt positivo
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0.0:
            return

        theta  = self.x_est[2]

        #Posa predetta dal modello di moto
        x_pred_x, x_pred_y, theta_pred = motion_model(dt,self.x_est[0],self.x_est[1],self.v_nominal,self.w,self.x_est[2])
        x_pred = np.array([x_pred_x, x_pred_y, theta_pred])

        #Jacobiani e matrice Q
        Fx = np.array([
            [1, 0, -self.v_nominal * math.sin(theta) * dt],
            [0, 1,  self.v_nominal * math.cos(theta) * dt],
            [0, 0,  1]
        ])

        Fu = np.array([
            [math.cos(theta) * dt, 0.0],
            [math.sin(theta) * dt, 0.0],
            [0.0,                     dt]
        ])

        Q = np.diag([
            self.alpha_v * self.v_nominal**2,
            self.alpha_theta * self.w**2
        ])

        #FASE DI PREDIZIZONE
        P_pred = Fx @ self.P @ Fx.T + Fu @ Q @ Fu.T

        #Calcolo H e z dal topic e le uso per la correzione
        H = np.zeros((len(self.beacons),3))
        z_hat = np.zeros(len(self.beacons))

        for i, b in enumerate(self.beacons):
            diff = x_pred[:2] - b   
            d = np.linalg.norm(diff)
            if d < 1e-6:
                d = 1e-6
            z_hat[i] = d
            H[i, :] = [diff[0]/d, diff[1]/d, 0.0]

        #FASE DI CORREZIONE
        R = np.eye(len(self.beacons)) * self.sigma_z**2
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
	
        #Aggiorno stima e covarianza
        self.x_est = x_pred + K @ (self.z - z_hat)
        #Normalizzo l'angolo
        self.x_est[2] = math.atan2( math.sin(self.x_est[2]),math.cos(self.x_est[2]))

        self.P = (np.eye(3) - K @ H) @ P_pred

        self.publish_markers()

    #Pubblicazione del messaggio della stima della posa del robot calcolata con EKF
    def publish_markers(self):
        self.pub_est.publish(self.make_sphere(self.x_est[0], self.x_est[1],
            "ekf_robot", 0, 1.0, 0.0, 0.0
        ))
        self.pub_cov.publish(self.make_covariance())

    # Definizione del messaggio Marker utile per rappresentare su RViz la posa del robot stimata
    def make_sphere(self, x, y, ns, mid, r, g, b):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        return m

    #Definizione del messaggio Marker utile per rappresentare su RViz la covarianza della posa del robot stimata
    def make_covariance(self):
        P_xy = self.P[:2, :2]

        eigvals, eigvecs = np.linalg.eig(P_xy)

        # protezione numerica
        eigvals = np.maximum(eigvals, 1e-6)

        angle = math.atan2(eigvecs[1, 0], eigvecs[0, 0])

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "covariance"
        m.id = 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.x_est[0]
        m.pose.position.y = self.x_est[1]
        m.pose.position.z = 0.5
        m.pose.orientation.z = math.sin(angle / 2)
        m.pose.orientation.w = math.cos(angle / 2)
        m.scale.x = 2 * math.sqrt(abs(eigvals[0]))
        m.scale.y = 2 * math.sqrt(abs(eigvals[1]))
        m.scale.z = 0.01
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.6
        return m


if __name__ == "__main__":
    ekf = EKFEstimate()
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            ekf.step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
