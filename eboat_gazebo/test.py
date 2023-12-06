#!/usr/bin/env python3

# Em um terminal com o ROS configurado e a simulação em execução
import rospy
from sensor_msgs.msg import LaserScan

print("Iniciando o script test.py")

# Inicialize o nó do ROS
rospy.init_node('laser_scan_publisher')

# Crie um publicador para o tópico
pub = rospy.Publisher('/eboat/camera_bow/scan', LaserScan, queue_size=10)

# Crie uma mensagem LaserScan
scan = LaserScan()
# Preencha os campos necessários de scan aqui...

# Publique a mensagem
pub.publish(scan)

print("Fim do script test.py")