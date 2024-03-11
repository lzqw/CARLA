from __future__ import print_function

import carla
from agents.navigation.basic_agent import BasicAgent
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from agents.tools.misc import get_speed
import os
import lmdb
import datetime
import numpy as np

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
def get_entry_point():
    return 'Auto_Agent'

class Auto_Agent(AutonomousAgent):

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        self.origin_global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1]) for x in range(len(global_plan_world_coord))]
        super().set_global_plan(global_plan_gps, global_plan_world_coord)

    def setup(self, config):
        self.track = Track.MAP
        self._route_assigned = False
        self._agent = None
        self.num_frames = 0
        self.stop_counter = 0
        self.config = config
        self.rgbs, self.sems, self.info, self.brak = [], [], [], []

    def sensors(self):
        camera_w = 1024
        camera_h = 288
        fov = 100
        sensors = [
            {'type': 'sensor.camera.rgb', 'x': 1.5, 'y': 0.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': camera_w, 'height': camera_h, 'fov': fov, 'id': 'RGB'},
            # {'type': 'sensor.camera.semantic_segmentation', 'x': 1.5, 'y': 0.0, 'z': 2.4,  'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            # 'width': camera_w, 'height': camera_h, 'fov': fov, 'id': 'SEM'},
            {'type': 'sensor.speedometer', 'id': 'EGO'},
        ]
        return sensors
    
    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        control = carla.VehicleControl(steer=0, throttle=0, brake=1)

        if not self._agent:
            self.init_set_routes()
            return control  

        print( self._agent.run_step())
        control= self._agent.run_step()

        _, rgb = input_data.get('RGB')
        _, ego = input_data.get('EGO')
        spd = ego.get('speed')

        if spd < 0.5:
            self.stop_counter += 1
        else:
            self.stop_counter = 0

        self.num_frames += 1

        return control
    
    def force_destory_actor(self, obs, light, walker):
        if obs:
            self._world.get_actor(obs.id).destroy()
            self.stop_counter = 0
            print(f"{self.num_frames}, {bcolors.WARNING}ATTENTION:{bcolors.ENDC} force to detroy actor {obs.id} stopping for a long time")
        elif walker:
            self._world.get_actor(walker.id).destroy()
            self.stop_counter = 0
            print(f"{self.num_frames}, {bcolors.WARNING}ATTENTION:{bcolors.ENDC} force to detroy actor {walker.id} stopping for a long time")
        elif light:
            light.set_green_time(10.0)
            light.set_state(carla.TrafficLightState.Green)
            self.stop_counter = 0
            print(f"{self.num_frames}, {bcolors.WARNING}ATTENTION:{bcolors.ENDC} force to setting green light {light.id}")
        else:
            print(f"{bcolors.WARNING}==========>{bcolors.ENDC}  error!!!! None factor trigger the stop!!!")
            return

    def init_set_routes(self):
        self._vehicle = CarlaDataProvider.get_hero_actor()
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._agent = BasicAgent(self._vehicle)
        plan = []
        prev_wp = None
        for point_world_coord in self._global_plan_world_coord:
            wp = CarlaDataProvider.get_map().get_waypoint(point_world_coord[0].location)
            if prev_wp:
                plan.extend(self._agent.trace_route(prev_wp, wp))
            prev_wp = wp

        self._agent.set_global_plan(plan)

    def change_weather(self):
        # TODO
        return

    # def flush_data(self):
    #     # Save data
    #     now = datetime.datetime.now()
    #     folder_name = f'rid_{self.config.route_id:02d}_'
    #     time_now = '_'.join(map(lambda x: '%02d' % x, (now.month, now.day, now.hour, now.minute, now.second)))
    #     data_path = os.path.join(self.config.data_save, folder_name+time_now)
    #
    #     if not os.path.exists(data_path):
    #         os.makedirs(data_path)
    #         print ('======> Saving to {}'.format(data_path))
    #
    #     lmdb_env = lmdb.open(data_path, map_size=int(1e10))
    #     db_length = len(self.info)
    #
    #     with lmdb_env.begin(write=True) as txn:
    #         txn.put('len'.encode(), str(db_length).encode())
    #         for i in range(db_length):
    #             txn.put(
    #                 f'info_{i:05d}'.encode(),
    #                 np.ascontiguousarray(self.info[i]).astype(np.float32),
    #             )
    #             txn.put(
    #                 f'rgbs_{i:05d}'.encode(),
    #                 np.ascontiguousarray(self.rgbs[i]).astype(np.uint8)
    #             )
    #
    #             txn.put(
    #                 f'sems_{i:05d}'.encode(),
    #                 np.ascontiguousarray(self.sems[i]).astype(np.uint8)
    #             )
    #
    #     self.rgbs.clear()
    #     self.sems.clear()
    #     self.info.clear()
    #     lmdb_env.close()
    #
    #     return

    def destroy(self):
        if len(self.rgbs) == 0:
            return

        self.flush_data()
