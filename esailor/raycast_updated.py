#!/home/araujo/miniconda3/envs/esailor/bin/python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class rays():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/eboat/laser/scan", LaserScan, self._laser_scan_callback)

        self.laser_scan = None
        rospy.logdebug("Waiting for /scan to be READY...")
        while ((self.laser_scan is None) and (not rospy.is_shutdown())):
            try:
                self.laser_scan = rospy.wait_for_message("/eboat/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /eboat/laser/scan READY=>")
            except:
                rospy.logerr("Current /eboat/laser/scan not ready yet, retrying for getting laser_scan")

    def _laser_scan_callback(self, data):
        self.laser_scan = data
        
    #dividir os 120 feixes em grupos 20 feixes, e inserir na rede neural
    #cada grupo de 20 feixes vai ser um input da rede neural
    #cada input vai ser um array de 1 valor, que vai ser a media dos 20 feixes
    #calcular a media dos valores maiores que zero
    #se nao tiver nenhum valor maior que zero, input = 0
    #calcular maior valor menor que 0 e menor valor maior que 0
    #valor minimo maior que zero, valor maximo menor que 1   #
    # a rede neural vai ter 6 inputs, cada um com 1 valor
    #os 6 inputs vao ser os 6 grupos de 20 feixes 
    #se nao tiver obstaculo, valor 0.5
    #se tiver obstaculo, valor 1 - (distancia do obstaculo / 5)    

    def step(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations',
                                                 Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData: 0 distance from the goal,
            #              1 angle between the foward direction and the direction towards the goal
            #              2 surge velocity
            #              3 apparent wind speed,
            #              4 apparent wind angle,
            #              5 boom angle,
            #              6 rudder angle,
            #              7 eletric propultion power,
            #              8 roll angle
            #             10 boat's current X position
            #             11 boat's current Y position

            print(np.array(obsData, dtype=float))
            print("-------")
            print(self.laser_scan)
            print("============")

        return np.array(obsData, dtype=float)


    def _laser_scan_callback(self, data):
        # Process the incoming LIDAR data here
        ranges = np.array(data.ranges)
        # Normalize ranges by 5.0 (max range) to get values between 0 and 1
        normalized_ranges = ranges / 5.0
        # Any range equal to 1 means no obstacle detected within 5 meters in that direction
        obstacles = normalized_ranges < 1.0
        # Prepare the data for PPO
        self.processed_lidar_data = normalized_ranges if obstacles.any() else np.zeros(17)

    def get_processed_lidar_data(self):
        return self.processed_lidar_data if self.laser_scan else np.zeros(17)

if __name__ == '__main__':
    test = rays()
    for i in range(20):
        test.step()