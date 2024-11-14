import logging

from gymnasium.envs.registration import register

logger = logging.getLogger(__name__)

#--------------------------------------------
#--> GAZEBO

#-> Eboat gymnasium_envs
register(
    id='Eboat53-v0',
    entry_point='esailor_gym.envs:Eboat53_v0',
    max_episode_steps=1000000,
)

register(
    id='Eboat103-v0',
    entry_point='esailor_gym.envs:Eboat103_v0',
    max_episode_steps=1000000,
)

register(
    id='Eboat101-v0',
    entry_point='esailor_gym.envs:Eboat101_v0',
    max_episode_steps=1000000,
)