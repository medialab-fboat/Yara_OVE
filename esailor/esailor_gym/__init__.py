import logging

from gymnasium.envs.registration import register

logger = logging.getLogger(__name__)

#--------------------------------------------
#--> GAZEBO

#-> Eboat gymnasium_envs
register(
    id='GazeboOceanEboatEnvCC35-v0',
    entry_point='esailor_gym.envs:GazeboOceanEboatEnvCC35v0',
    max_episode_steps=1000000,
)

register(
    id='GazeboOceanEboatEnvCC39-v0',
    entry_point='esailor_gym.envs:GazeboOceanEboatEnvCC39v0',
    max_episode_steps=1000000,
)

register(
    id='GazeboOceanEboatEnvCC25-v0',
    entry_point='esailor_gym.envs:GazeboOceanEboatEnvCC25v0',
    max_episode_steps=1000000,
)
