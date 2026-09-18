#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

from example_interfaces.msg import String

class TurtlebotCtrl(Node):
	def __init__(self):
		super().__init__("TurtlebotCtrl")

		self.laser = LaserScan()
		self.odom = Odometry()

		self.map = np.array([	[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
								[0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
						])

		self.publish_cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)
		self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
		self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
		self.timer = self.create_timer(0.5, self.cmd_vel_pub)

	def cmd_vel_pub(self):

		map_resolution = 4

		index_x = -int(self.odom.pose.pose.position.x*map_resolution)
		index_y = -int(self.odom.pose.pose.position.y*map_resolution)

		index_x += int(self.map.shape[0]/2)
		index_y += int(self.map.shape[0]/2)
		
		if (index_x < 1): index_x = 1
		if (index_x > self.map.shape[0]-1): index_x = self.map.shape[0]-1
		if (index_y < 1): index_y = 1
		if (index_y > self.map.shape[0]-1): index_y = self.map.shape[0]-1
		
		if (self.map[index_x][index_y] == 1):
			self.map[index_x][index_y] = 2

			self.get_logger().info("Another part reached ... percentage total reached...." + str(100*float(np.count_nonzero(self.map == 2))/(np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))) )
			self.get_logger().info("Discrete Map")
			self.get_logger().info("\n"+str(self.map))

                # Desenvolva seu codigo aqui

		if len(self.laser.ranges) > 0:

			if not hasattr(self, 'estado'):
				self.estado = 'buscar_pared'
				self.contador_libre = 0
				self.oportunidades = 0
				self.apertura_contada = False

			lecturas = np.array(self.laser.ranges, dtype=float)

			lecturas = np.nan_to_num(
				lecturas,
				nan=self.laser.range_max,
				posinf=self.laser.range_max,
				neginf=self.laser.range_max
			)

			cantidad = len(lecturas)

			def sector(grados_inicio, grados_fin):

				inicio = int(cantidad * (grados_inicio / 360))
				fin = int(cantidad * (grados_fin / 360))

				valores = lecturas[inicio:fin]

				if len(valores) == 0:
					return self.laser.range_max

				return np.percentile(valores, 20)

			frente_derecha = sector(325, 350)
			frente_izquierda = sector(10, 35)

			derecha_delantera = sector(285, 325)
			derecha = sector(255, 285)
			derecha_trasera = sector(225, 255)

			izquierda_delantera = sector(35, 70)
			izquierda = sector(70, 110)
			izquierda_trasera = sector(110, 145)

			trasera_izquierda = sector(145, 175)
			trasera_derecha = sector(185, 215)

			frente = min(
				np.percentile(lecturas[0:int(cantidad * (10 / 360))], 20),
				np.percentile(lecturas[int(cantidad * (350 / 360)):], 20)
			)

			mensaje = TwistStamped()
			mensaje.header.stamp = self.get_clock().now().to_msg()

			distancia_pared = 0.40
			distancia_critica = 0.32
			distancia_frontal_segura = 0.50
			espacio_para_cruzar = 1.30

			peligro_frente = (
				frente < distancia_frontal_segura
				or frente_izquierda < distancia_critica
				or frente_derecha < distancia_critica
			)

			if peligro_frente:

				mensaje.twist.linear.x = 0.0

				if izquierda_delantera > derecha_delantera:
					mensaje.twist.angular.z = 0.30
					self.estado = 'girar_izquierda'

				else:
					mensaje.twist.angular.z = -0.30
					self.estado = 'girar_derecha'

				self.contador_libre = 0

			elif self.estado == 'buscar_pared':

				if derecha < 0.70:

					self.estado = 'seguir_pared'

					mensaje.twist.linear.x = 0.12
					mensaje.twist.angular.z = 0.0

				else:

					mensaje.twist.linear.x = 0.14
					mensaje.twist.angular.z = -0.10

			elif self.estado == 'girar_izquierda':

				if derecha < 0.70 and frente > 0.65:

					self.estado = 'seguir_pared'

					mensaje.twist.linear.x = 0.10
					mensaje.twist.angular.z = 0.0

				else:

					mensaje.twist.linear.x = 0.0
					mensaje.twist.angular.z = 0.30

			elif self.estado == 'girar_derecha':

				if derecha < 0.70 and frente > 0.65:

					self.estado = 'seguir_pared'

					mensaje.twist.linear.x = 0.10
					mensaje.twist.angular.z = 0.0

				else:

					mensaje.twist.linear.x = 0.0
					mensaje.twist.angular.z = -0.30

			elif self.estado == 'seguir_pared':

				espacio_izquierdo_libre = (
					izquierda_delantera > espacio_para_cruzar
					and izquierda > espacio_para_cruzar
					and izquierda_trasera > espacio_para_cruzar
				)

				if espacio_izquierdo_libre:

					if not self.apertura_contada:
						self.contador_libre += 1

				else:

					self.contador_libre = 0
					self.apertura_contada = False

				if (
					self.contador_libre >= 3
					and not self.apertura_contada
				):

					self.apertura_contada = True
					self.contador_libre = 0
					self.oportunidades += 1

					if self.oportunidades >= 2:

						self.oportunidades = 0
						self.estado = 'girar_para_cruzar'

						mensaje.twist.linear.x = 0.0
						mensaje.twist.angular.z = 0.30

					else:

						mensaje.twist.linear.x = 0.14

						error_pared = distancia_pared - derecha

						mensaje.twist.angular.z = error_pared * 0.35

				elif derecha < 0.30:

					mensaje.twist.linear.x = 0.08
					mensaje.twist.angular.z = 0.10

				elif derecha > 0.55:

					mensaje.twist.linear.x = 0.08
					mensaje.twist.angular.z = -0.12

				else:

					mensaje.twist.linear.x = 0.14

					error_pared = distancia_pared - derecha

					mensaje.twist.angular.z = error_pared * 0.35

			elif self.estado == 'girar_para_cruzar':

				if frente > 1.20 and derecha > 0.80:

					self.estado = 'cruzar'

					mensaje.twist.linear.x = 0.12
					mensaje.twist.angular.z = 0.0

				else:

					mensaje.twist.linear.x = 0.0
					mensaje.twist.angular.z = 0.30

			elif self.estado == 'cruzar':

				if frente < 0.60:

					self.estado = 'acoplar_pared'

					mensaje.twist.linear.x = 0.0
					mensaje.twist.angular.z = 0.30

				elif frente_izquierda < 0.40:

					mensaje.twist.linear.x = 0.06
					mensaje.twist.angular.z = -0.10

				elif frente_derecha < 0.40:

					mensaje.twist.linear.x = 0.06
					mensaje.twist.angular.z = 0.10

				else:

					mensaje.twist.linear.x = 0.16
					mensaje.twist.angular.z = 0.0

			elif self.estado == 'acoplar_pared':

				if derecha < 0.60 and frente > 0.65:

					self.estado = 'seguir_pared'

					mensaje.twist.linear.x = 0.10
					mensaje.twist.angular.z = 0.0

				else:

					mensaje.twist.linear.x = 0.0
					mensaje.twist.angular.z = 0.30

			self.publish_cmd_vel.publish(mensaje)
			return

		msg = TwistStamped()
		msg.twist.linear.x = 0.1
		msg.twist.angular.z = 0.1
		self.publish_cmd_vel.publish(msg)

	def callback_laser(self, msg):
		self.laser = msg

	def callback_odom(self, msg):
		self.odom = msg

def main(args=None):
	rclpy.init(args=args)
	node = TurtlebotCtrl()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
