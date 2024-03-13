import logging
import numpy as np
import copy
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..', 'leaderboard'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..',  'scenario_runner'))
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from leaderboard.envs.sensor_interface import SensorInterface
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from omegaconf import OmegaConf

import carla
import gym
from carla_gym.utils.config_utils import load_entry_point
from carla_gym.core.obs_manager.obs_manager_handler import ObsManagerHandler
from carla_gym.core.task_actor.ego_vehicle.ego_vehicle_handler import EgoVehicleHandler
from carla_gym.core.task_actor.common.task_vehicle import TaskVehicle
from carla_gym.utils.traffic_light import TrafficLightHandler

def get_entry_point():
    return 'RlBirdviewAgent'

class RlBirdviewAgent(AutonomousAgent):
    def __init__(self, host, port, debug):
        self._logger = logging.getLogger(__name__)
        self._render_dict = None
        self.supervision_dict = None

        super().__init__(host, port, debug)

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        self.origin_global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1]) for x in range(len(global_plan_world_coord))]
        super().set_global_plan(global_plan_gps, global_plan_world_coord)

    def sensors(self):
        return []

    def setup(self, path_to_conf_file):
        self.track = Track.MAP
        self.cfg = OmegaConf.load(path_to_conf_file)
        self.cfg = OmegaConf.to_container(self.cfg)
        self._obs_configs = self.cfg['obs_configs']
        self._train_cfg = self.cfg['training']

        self._policy_class = load_entry_point(self.cfg['policy']['entry_point'])
        self._policy_kwargs = self.cfg['policy']['kwargs']
        self._ckpt, self._policy = None, None

        self._wrapper_class = load_entry_point(self.cfg['env_wrapper']['entry_point'])
        self._wrapper_kwargs = self.cfg['env_wrapper']['kwargs']

        self._om_handler = ObsManagerHandler({0:self._obs_configs})
        self._acc_as_action = self._wrapper_kwargs['acc_as_action']
        self.initialized = False

    def local_init(self):
        self._client = CarlaDataProvider.get_client()

        self._ev_handler = EgoVehicleHandler(self._client, None, None)
        self._vehicle = CarlaDataProvider.get_hero_actor()

        self._spawn_transforms = self._ev_handler._get_spawn_points(self._ev_handler._map)

        self._world = CarlaDataProvider.get_world()
        # register traffic lights
        TrafficLightHandler.reset(self._world)

        self.route = []
        for point_world_coord in self._global_plan_world_coord:
            wp = CarlaDataProvider.get_map().get_waypoint(point_world_coord[0].location)
            self.route.append(carla.Transform(wp.transform.location, wp.transform.rotation))
        self._ev_handler.ego_vehicles[0] = TaskVehicle(self._vehicle, self.route, self._spawn_transforms, False)

        # ev_spawn_locations = self._ev_handler.reset({0:test})

        self._om_handler.reset(self._ev_handler.ego_vehicles)

        state_spaces = []
        if 'speed' in self.cfg['obs_configs']:
            state_spaces.append(self._om_handler.observation_space[0]['speed']['speed_xy'])
        if 'speed_limit' in self.cfg['obs_configs']:
            state_spaces.append(self._om_handler.observation_space[0]['control']['speed_limit'])
        if 'control' in self.cfg['obs_configs']:
            state_spaces.append(self._om_handler.observation_space[0]['control']['throttle'])
            state_spaces.append(self._om_handler.observation_space[0]['control']['steer'])
            state_spaces.append(self._om_handler.observation_space[0]['control']['brake'])
            state_spaces.append(self._om_handler.observation_space[0]['control']['gear'])
        if 'acc_xy' in self.cfg['obs_configs']:
            state_spaces.append(self._om_handler.observation_space[0]['velocity']['acc_xy'])
        if 'vel_xy' in self.cfg['obs_configs']:
            state_spaces.append(self._om_handler.observation_space[0]['velocity']['vel_xy'])
        if 'vel_ang_z' in self.cfg['obs_configs']:
            state_spaces.append(self._om_handler.observation_space[0]['velocity']['vel_ang_z'])

        state_low = np.concatenate([s.low for s in state_spaces])
        state_high = np.concatenate([s.high for s in state_spaces])

        self.observation_space = gym.spaces.Dict(
            {'state': gym.spaces.Box(low=state_low, high=state_high, dtype=np.float32),
             'birdview': self._om_handler.observation_space[0]['birdview']['masks']})

        if self._acc_as_action:
            # act: acc(throttle/brake), steer
            self.action_space = gym.spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
        else:
            # act: throttle, steer, brake
            self.action_space = gym.spaces.Box(low=np.array([0, -1, 0]), high=np.array([1, 1, 1]), dtype=np.float32)

        if self._policy is None:
            self._policy = self._policy_class(self.observation_space, self.action_space, **self._policy_kwargs).to(
                'cuda')

        dir_path = os.path.dirname(os.path.realpath(__file__))
        self._policy, self._train_cfg['kwargs'] = self._policy.load(dir_path + '/rl_ckpt_11833344.pth')
        self._policy = self._policy.eval()

        self.initialized = True

    def run_step(self, input_data, timestamp):
        if(self.initialized == False):
            self.local_init()

        _ = self._ev_handler.ego_vehicles[0]._truncate_global_route_till_local_target()


        obs_dict = self._om_handler.get_observation(timestamp)
        input_data = obs_dict[0]
        input_data = copy.deepcopy(input_data)

        #Debug
        from matplotlib import pyplot as plt
        plt.ion()
        plt.imshow(input_data['birdview']['rendered'])
        plt.show()

        policy_input = self._wrapper_class.process_obs(input_data, self._wrapper_kwargs['input_states'], train=False)

        actions, values, log_probs, mu, sigma, features = self._policy.forward(
            policy_input, deterministic=True, clip_action=True)
        control = self._wrapper_class.process_act(actions, self._wrapper_kwargs['acc_as_action'], train=False)
        return control

    @property
    def obs_configs(self):
        return self._obs_configs

    def destroy(self):
        pass


if __name__ =="__main__":
    path = '/home/lzqw/PycharmProject/Carla-RL/CARLA/CODE/agent/rl_agent/roach.yaml'
    RL_agent = RlBirdviewAgent(path)
    RL_agent.setup(path)
