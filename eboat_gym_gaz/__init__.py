import logging

from gym.envs.registration import register

logger = logging.getLogger(__name__)

#--------------------------------------------
#--> GAZEBO

#-> Eboat envs
register(
    id='GazeboOceanEboatEnvCC-v0',
    entry_point='eboat_gym_gaz.envs:GazeboOceanEboatEnvCC',
    max_episode_steps=1000000,
)

register(
    id='GazeboOceanEboatEnvCC-v1',
    entry_point='eboat_gym_gaz.envs:GazeboOceanEboatEnvCC1',
    max_episode_steps=1000000,
)

#register(
#    id='GazeboOceanEboatEnvCC-v2',
#    entry_point='eboat_gym_gaz.envs:GazeboOceanEboatEnvCC2',
#    max_episode_steps=1000000,
#)
