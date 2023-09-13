from Yara_OVE.esailor.esailor_gym.envs.gazebo_ocean_eboat_CC import GazeboOceanEboatEnvCC35v0
import rospy

LaserScan = None
try:
    from sensor_msgs.msg import LaserScan
except ImportError:
    pass

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
import numpy as np


class EboatEnv(GazeboOceanEboatEnvCC35v0):
    def __init__(self):
        super(EboatEnv, self).__init__()

        # Subscribing to LIDAR data
        self.lidar_data = None
        # Subscribing to LIDAR data from the ROS topic
        rospy.Subscriber('/eboat/lidar/scan', LaserScan, self.lidar_callback)
        
        # Definindo a distância limite abaixo da qual consideramos o barco muito perto de um obstáculo
        self.SOME_THRESHOLD = 5.0  # por exemplo, 1 metro

    def lidar_callback(self, data):
        self.lidar_data = data

    def _get_observation(self):
        # This function collects sensor data and prepares it for the neural network
        obs = {
            'lidar': [],
            'boat_speed': 0.0,
            'sail_orientation': 0.0,
        }

        # Obtendo dados do LIDAR
        if self.lidar_data:
            # Processing LIDAR data for the neural network
            obs['lidar'] = list(self.lidar_data.ranges)
            # ... (possivelmente processar ou reduzir a dimensionalidade dos dados do LIDAR)

        # Adicionar aqui outros sensores que você possa ter no barco, como velocidade, direção, orientação da vela, etc.
        # Por exemplo:
        obs['boat_speed'] = self.get_boat_speed()
        obs['sail_orientation'] = self.get_sail_orientation()
        

        return obs    
   
    def setState(self, model_name, pose, theta):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        # Definindo a pose
        state.pose.position.x = pose[0]
        state.pose.position.y = pose[1]
        state.pose.position.z = pose[2]
        quaternion = quaternion_from_euler(0, 0, theta)
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]
        # Definindo a velocidade (twist) como zero
        state.twist.linear.x = 0
        state.twist.linear.y = 0
        state.twist.linear.z = 0
        state.twist.angular.x = 0
        state.twist.angular.y = 0
        state.twist.angular.z = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = self.set_state
            result = set_state(state)
            #print(f"SetStae Result: {result}")
            assert result.success is True
            #print(f"Model {model_name} set to position {pose} with orientation theta={theta}.")
        except rospy.ServiceException:
            print("/gazebo/set_model_state service call failed")    

    def step(self, action):
        pass
        reward = 0
        done = False
        info = {}

        # Aplicando ações com base na entrada
        if action == 0:  # turn left
            self.boomAng_pub.publish(45.0)
            self.rudderAng_pub.publish(-60.0)
            pass
        elif action == 1:  # turn right
            self.boomAng_pub.publish(45.0)
            self.rudderAng_pub.publish(60.0)
            pass
        else:  # move forward
            self.boomAng_pub.publish(45.0)
            self.rudderAng_pub.publish(0.0)
            pass

        # Definindo recompensa com base nos dados do LIDAR
        if self.lidar_data:
            # Processing LIDAR data for the neural network
            if min(self.lidar_data.ranges) < self.SOME_THRESHOLD:  # se estiver muito perto de um obstáculo
                reward = -1
                done = True  # Pode considerar terminar o episódio se estiver muito perto de um obstáculo
            else:
                reward = 1
     
                
        boat_position = self.get_boat_position()
        waypoint_position = self.get_waypoint_position()
      
        previous_distance_to_waypoint = np.linalg.norm(boat_position - waypoint_position)
      
        print(f"Boat position: {boat_position}")
        print(f"Waypoint position: {waypoint_position}")
        print(f"Previous distance to waypoint: {previous_distance_to_waypoint}")
       
        
        
        # Definindo recompensa com base na distância para o waypoint
        if previous_distance_to_waypoint < 5:
            reward = 100
            done = True
        else:
            reward = -1
            done = False  

        # Penalidade por não estar indo na direção do waypoint
        penalty = 0
        if self.get_boat_speed() > 0.1:
            penalty = 1
        else:
            penalty = 0
        reward -= penalty
        
        # Recompensa por chegar no waypoint
        if previous_distance_to_waypoint < 5:
            reward += 100
            done = True
        else:
            reward -= 1
            done = False            
        
        # Penalidade por ficar empurrando a boia para longe
        pushBuoy = 0
        if self.get_boat_speed() > 0.1:
            pushBuoy = 1
        else:
            pushBuoy = 0
        reward -= pushBuoy     
        
        # Penalidade por ficar muito perto de obstáculos
        obsPenalty = 0
        if min(self.lidar_data.ranges) < 5:
            obsPenalty = 1
        else:
            obsPenalty = 0
        reward -= obsPenalty        

        
        # Obtenha a próxima observação após tomar a ação
        next_obs = self._get_observation()
    

        return next_obs, reward, done, info, previous_distance_to_waypoint, boat_position, waypoint_position
    