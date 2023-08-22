import logging

from gymnasium.envs.registration import register

logger = logging.getLogger(__name__)

#--------------------------------------------
#--> GAZEBO

#-> Eboat gymnasium_envs
register(
    id='EboatSingleWayPointEnvCC35-v0',
    entry_point='esailor_gym.envs:EboatSingleWayPointEnvCC35v0',
    max_episode_steps=1000000,
)

register(
    id='EboatSingleWayPointEnvCC25-v0',
    entry_point='esailor_gym.envs:EboatSingleWayPointEnvCC25v0',
    max_episode_steps=1000000,
)

register(
    id='EboatStraightLineEnvCC29-v0',
    entry_point='esailor_gym.envs:EboatStraightLineEnvCC29v0',
    max_episode_steps=1000000,
)
