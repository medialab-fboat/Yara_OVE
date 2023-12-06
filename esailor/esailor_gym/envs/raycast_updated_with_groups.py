#!/home/araujo/miniconda3/envs/esailor/bin/python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class rays:
    def __init__(self):
        #rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/eboat/laser/scan", LaserScan, self._laser_scan_callback)

        self.laser_scan = None
        rospy.logdebug("Waiting for /scan to be READY...")
        while ((self.laser_scan is None) and (not rospy.is_shutdown())):
            try:
                self.laser_scan = rospy.wait_for_message("/eboat/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /eboat/laser/scan READY=>")
            except:
                rospy.logerr("Current /eboat/laser/scan not ready yet, retrying for getting laser_scan")

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
    
    def _laser_scan_callback(self, data):
        self.laser_scan = data
        #rospy.loginfo("laser_scan_callback")
        
        rospy.loginfo("len(self.laser_scan.ranges)="+str(len(self.laser_scan.ranges)))
            
        rospy.loginfo("self.laser_scan.ranges[0:20]="+str(self.laser_scan.ranges[0:20]))
        
        rospy.loginfo("self.laser_scan.ranges[20:40]="+str(self.laser_scan.ranges[20:40]))
        
        rospy.loginfo("self.laser_scan.ranges[40:60]="+str(self.laser_scan.ranges[40:60]))
        
        rospy.loginfo("self.laser_scan.ranges[60:80]="+str(self.laser_scan.ranges[60:80]))
        
        rospy.loginfo("self.laser_scan.ranges[80:100]="+str(self.laser_scan.ranges[80:100]))
        
        rospy.loginfo("self.laser_scan.ranges[100:120]="+str(self.laser_scan.ranges[100:120]))
        
    
    def get_laser_scan(self):
        return self.laser_scan
    
    def get_laser_scan_as_np(self):
        return np.array(self.laser_scan.ranges)
    
    def get_laser_scan_as_np_groups(self):
        return np.array([self.laser_scan.ranges[0:20],self.laser_scan.ranges[20:40],self.laser_scan.ranges[40:60],self.laser_scan.ranges[60:80],self.laser_scan.ranges[80:100],self.laser_scan.ranges[100:120]])
    
    def get_laser_scan_as_np_groups_mean(self):
        return np.array([np.mean(self.laser_scan.ranges[0:20]),np.mean(self.laser_scan.ranges[20:40]),np.mean(self.laser_scan.ranges[40:60]),np.mean(self.laser_scan.ranges[60:80]),np.mean(self.laser_scan.ranges[80:100]),np.mean(self.laser_scan.ranges[100:120])])
    
    #Normaliza os valores do laser scan para ficarem entre 0 e 1
    def get_laser_scan_as_np_groups_mean_normalized(self):
        laser_scan_as_np_groups_mean = self.get_laser_scan_as_np_groups_mean()
        laser_scan_as_np_groups_mean_normalized = (laser_scan_as_np_groups_mean - np.min(laser_scan_as_np_groups_mean)) / (np.max(laser_scan_as_np_groups_mean) - np.min(laser_scan_as_np_groups_mean))
        return laser_scan_as_np_groups_mean_normalized
    
    #Enviar os valores do laser scan para a rede neural
    def get_laser_scan_as_np_groups_mean_normalized_reshaped(self):
        laser_scan_as_np_groups_mean_normalized = self.get_laser_scan_as_np_groups_mean_normalized()
        return laser_scan_as_np_groups_mean_normalized.reshape(1,6)
    
    
    def __init__(self):
        #rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/eboat/laser/scan", LaserScan, self._laser_scan_callback)

        self.laser_scan = None
        rospy.logdebug("Waiting for /scan to be READY...")
        while ((self.laser_scan is None) and (not rospy.is_shutdown())):
            try:
                self.laser_scan = rospy.wait_for_message("/eboat/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /eboat/laser/scan READY=>")
            except:
                rospy.logerr("Current /eboat/laser/scan not ready yet, retrying for getting laser_scan")

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
    
    def _laser_scan_callback(self, data):
        self.laser_scan = data
        #rospy.loginfo("laser_scan_callback")
        
        rospy.loginfo("len(self.laser_scan.ranges)="+str(len(self.laser_scan.ranges)))
            
        rospy.loginfo("self.laser_scan.ranges[0:20]="+str(self.laser_scan.ranges[0:20]))
        
        rospy.loginfo("self.laser_scan.ranges[20:40]="+str(self.laser_scan.ranges[20:40]))
        
        rospy.loginfo("self.laser_scan.ranges[40:60]="+str(self.laser_scan.ranges[40:60]))
        
        rospy.loginfo("self.laser_scan.ranges[60:80]="+str(self.laser_scan.ranges[60:80]))
        
        rospy.loginfo("self.laser_scan.ranges[80:100]="+str(self.laser_scan.ranges[80:100]))
        
        rospy.loginfo("self.laser_scan.ranges[100:120]="+str(self.laser_scan.ranges[100:120]))
        
    
    def get_laser_scan(self):
        return self.laser_scan
    
    def get_laser_scan_as_np(self):
        return np.array(self.laser_scan.ranges)
    
    def get_laser_scan_as_np_groups(self):
        return np.array([self.laser_scan.ranges[0:20],self.laser_scan.ranges[20:40],self.laser_scan.ranges[40:60],self.laser_scan.ranges[60:80],self.laser_scan.ranges[80:100],self.laser_scan.ranges[100:120]])
    
    def get_laser_scan_as_np_groups_mean(self):
        return np.array([np.mean(self.laser_scan.ranges[0:20]),np.mean(self.laser_scan.ranges[20:40]),np.mean(self.laser_scan.ranges[40:60]),np.mean(self.laser_scan.ranges[60:80]),np.mean(self.laser_scan.ranges[80:100]),np.mean(self.laser_scan.ranges[100:120])])
    
    #Normaliza os valores do laser scan para ficarem entre 0 e 1
    def get_laser_scan_as_np_groups_mean_normalized(self):
        laser_scan_as_np_groups_mean = self.get_laser_scan_as_np_groups_mean()
        laser_scan_as_np_groups_mean_normalized = (laser_scan_as_np_groups_mean - np.min(laser_scan_as_np_groups_mean)) / (np.max(laser_scan_as_np_groups_mean) - np.min(laser_scan_as_np_groups_mean))
        return laser_scan_as_np_groups_mean_normalized
    
    #Enviar os valores do laser scan para a rede neural
    def get_laser_scan_as_np_groups_mean_normalized_reshaped(self):
        laser_scan_as_np_groups_mean_normalized = self.get_laser_scan_as_np_groups_mean_normalized()
        return laser_scan_as_np_groups_mean_normalized.reshape(1,6)
    
    
    
            
        