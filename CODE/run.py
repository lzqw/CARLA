
import importlib
import os
import sys
import argparse
from argparse import RawTextHelpFormatter

#set the path to leaderboard and scenario_runner
sys.path.append(os.path.join(os.path.dirname(__file__), '..','leaderboard'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..','scenario_runner'))
# set the path to carla python api
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','CARLA_Leaderboard_20','PythonAPI','carla'))

from leaderboard.leaderboard_evaluator import LeaderboardEvaluator
from leaderboard.utils.statistics_manager import StatisticsManager

# from  expert_agent.auto.auto_expert import Auto_Agent
def main():
    description = "CARLA AD Leaderboard Evaluation: evaluate your Agent in CARLA scenarios\n"

    # general parameters
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--host',
                        # default='172.26.149.156',# 1080ti
                        default='localhost',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default=2000, type=int,
                        help='TCP port to listen to (default: 2000)')
    parser.add_argument('--traffic-manager-port', default=8000, type=int,
                        help='Port to use for the TrafficManager (default: 8000)')
    parser.add_argument('--traffic-manager-seed', default=0, type=int,
                        help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--debug', type=int,
                        help='Run with debug output', default=1)
    parser.add_argument('--record', type=str, default='',
                        help='Use CARLA recording feature to create a recording of the scenario')
    parser.add_argument('--timeout', default=300.0, type=float,
                        help='Set the CARLA client timeout value in seconds')

    # simulation setup
    parser.add_argument('--routes',
                        #default='/home/lzqw/PycharmProject/Carla-RL/CARLA/leaderboard/data/routes_devtest.xml',
                        default='/home/lzqw/PycharmProject/Carla-RL/CARLA/leaderboard/data/routes_town10.xml',
                        help='Name of the routes file to be executed.')
    parser.add_argument('--routes-subset', default='', type=str,
                        help='Execute a specific set of routes')
    parser.add_argument('--repetitions', type=int, default=1,
                        help='Number of repetitions per route.')

    # agent-related options
    parser.add_argument("-a", "--agent", type=str,
                        help="Path to Agent's py file to evaluate",
                        default='/home/lzqw/PycharmProject/Carla-RL/CARLA/leaderboard/leaderboard/autoagents/human_agent.py'
                        #default='/home/lzqw/PycharmProjects/Carla-RL/CARLA_leaderboard/CODE/expert_agent/auto/auto_expert.py'
                        )
    parser.add_argument("--agent-config", type=str,
                        help="Path to Agent's configuration file",
                        default="/home/lzqw/PycharmProject/Carla-RL/CARLA/leaderboard/leaderboard/autoagents/human_agent_config.txt")
    parser.add_argument("--track", type=str, default='MAP',
                        help="Participation track: SENSORS, MAP")
    parser.add_argument('--resume', type=bool, default=False,
                        help='Resume execution from last checkpoint?')
    parser.add_argument("--checkpoint", type=str,
                        default='/home/lzqw/PycharmProject/Carla-RL/CARLA/leaderboard/results.json',
                        help="Path to checkpoint used for saving statistics and resuming")
    parser.add_argument("--debug-checkpoint", type=str, default='.',
                        help="Path t checkpoint used for saving live results")

    arguments = parser.parse_args()
    print(arguments)
    statistics_manager = StatisticsManager(arguments.checkpoint, arguments.debug_checkpoint)
    leaderboard_evaluator = LeaderboardEvaluator(arguments, statistics_manager)
    crashed = leaderboard_evaluator.run(arguments)

if __name__ == '__main__':
    main()
    #At leaderboard/leaderboard/leaderboard_evaluator.py, line 178, delete parameter spectator_as_ego = False .