import numpy as np
import socket
import os
from time import sleep
from random import randint
from joystick.joystick_base import JoystickBase
from params.central_params import create_agent_params
from utils.utils import generate_config_from_pos_3

class JoystickRVO(JoystickBase):
    def __init__(self):
        super().__init__("RVO_w_ckpt")
        self.agents = None
        self.agent_radius = None
        self.robot = None

        HOST = "127.0.0.1"
        PORT = 2111
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((HOST, PORT))
        return

    def init_obstacle_map(self, env):
        scale = float(env["map_scale"])
        map_trav = env["map_traversible"]
        edges_raw = set()
        for i in range(len(map_trav)):
            for j in range(len(map_trav[0])):
                if map_trav[i][j] == False:
                    edges_cand = [((i, j), (i + 1, j)),
                                  ((i, j), (i, j + 1)),
                                  ((i + 1, j), (i + 1, j + 1)),
                                  ((i, j + 1), (i + 1, j + 1))]
                    for ed in edges_cand:
                        if ed in edges_raw:
                            edges_raw.remove(ed)
                        else:
                            edges_raw.add(ed)

        edges_set = set()
        while len(edges_raw) > 0:
            ed = edges_raw.pop()
            if (ed[1][0] - ed[0][0]) == 1:
                dir_vector = (1, 0)
            elif (ed[1][1] - ed[0][1]) == 1:
                dir_vector = (0, 1)
            else:
                raise Exception("Edge direction determination error!")
            end_pt1 = ed[0]
            end_pt2 = ed[1]
            end_pt = ed[0]
            end_pt_nxt = (end_pt[0] - dir_vector[0], end_pt[1] - dir_vector[1])
            while (end_pt_nxt, end_pt) in edges_raw:
                edges_raw.remove((end_pt_nxt, end_pt))
                end_pt1 = end_pt_nxt
                end_pt = end_pt_nxt
                end_pt_nxt = (end_pt[0] - dir_vector[0], end_pt[1] - dir_vector[1])
            end_pt = ed[1]
            end_pt_nxt = (end_pt[0] + dir_vector[0], end_pt[1] + dir_vector[1])
            while (end_pt, end_pt_nxt) in edges_raw:
                edges_raw.remove((end_pt, end_pt_nxt))
                end_pt2 = end_pt_nxt
                end_pt = end_pt_nxt
                end_pt_nxt = (end_pt[0] + dir_vector[0], end_pt[1] + dir_vector[1])
            end_pt1 = (end_pt1[0] * scale, end_pt1[1] * scale)
            end_pt2 = (end_pt2[0] * scale, end_pt2[1] * scale)
            edges_set.add((end_pt1, end_pt2))
                            
        return edges_set

    def init_control_pipeline(self):
        self.goal_config = self.get_robot_goal()
        env = self.current_ep.get_environment()
        self.environment = self.init_obstacle_map(env)
        self.robot = self.get_robot_start()
        self.agents = {}
        agents_info = self.current_ep.get_agents()
        for key in list(agents_info.keys()):
            agent = agents_info[key]
            self.agents[key] = agent.get_current_config().to_3D_numpy()
        self.commands = None
        return

    def convert_to_string(self):
        info_string = ""
        info_string += "delta_t\n"
        info_string += str(self.sim_delta_t) + "\n"
        info_string += "obstacles\n"
        for ed in self.environment:
            info_string += str(ed[0][1]) + "," + str(ed[0][0]) + "," \
                           + str(ed[1][1]) + "," + str(ed[1][0]) + "\n"
        info_string += "robot\n"
        info_string += str(self.robot[0]) + "," + str(self.robot[1]) + "," \
                       + str(self.robot[2]) + "," + str(self.robot_v[0]) + "," \
                       + str(self.robot_v[1]) + "," + str(self.robot_v[2]) + "," \
                       + str(self.goal_config[0]) + "," + str(self.goal_config[1]) + "," \
                       + str(self.goal_config[2]) + "," + str(self.robot_radius + 0.001) + "\n"
        info_string += "agents\n"
        for key in list(self.agents.keys()):
            agent = self.agents[key]
            agent_v = self.agents_v[key]
            agent_goal = self.agents_goals[key]
            agent_radius = self.agents_radius[key] + 0.001
            info_string += key + "," \
                           + str(agent[0]) + "," + str(agent[1]) + "," \
                           + str(agent[2]) + "," + str(agent_v[0]) + "," \
                           + str(agent_v[1]) + "," + str(agent_v[2]) + "," \
                           + str(agent_goal[0]) + "," + str(agent_goal[1]) + "," \
                           + str(agent_goal[2]) + "," + str(agent_radius) + "\n"
        info_string += "End*"
        return info_string

    def send_info_to_planner(self):
        info_string = self.convert_to_string()
        self.socket.sendall(info_string.encode("utf-8"))
        return

    def joystick_sense(self):
        self.send_to_robot("sense")
        if (not self.listen_once()):
            self.joystick_on = False

        robot_prev = self.robot.copy()
        agents_prev = {}
        for key in list(self.agents.keys()):
            agent = self.agents[key]
            agents_prev[key] = agent.copy()
            
        # NOTE: self.sim_delta_t is available
        self.agents = {}
        self.agents_radius = {}
        agents_info = self.sim_state_now.get_all_agents()
        for key in list(agents_info.keys()):
            agent = agents_info[key]
            self.agents[key] = agent.get_current_config().to_3D_numpy()
            self.agents_radius[key] = agent.get_radius()
        robot_tmp = list(self.sim_state_now.get_robots().values())[0]
        self.robot = robot_tmp.get_current_config().to_3D_numpy()
        self.robot_radius = robot_tmp.get_radius()

        self.robot_v = (self.robot - robot_prev) / self.sim_delta_t
        self.agents_v = {}
        for key in list(self.agents.keys()):
            if key in agents_prev:
                v = (self.agents[key] - agents_prev[key]) / self.sim_delta_t
            else:
                v = np.array([0, 0, 0], dtype=np.float32)
            self.agents_v[key] = v
        return

    def joystick_plan(self):
        horizon_scale = 10
        horizon = self.sim_delta_t * horizon_scale
        self.agents_goals = {}
        for key in list(self.agents.keys()):
            goal = self.agents[key] + self.agents_v[key] * horizon
            self.agents_goals[key] = goal

        self.send_info_to_planner()
        self.data = self.socket.recv(1024)
        return

    def joystick_act(self):
        data_b = self.data.decode('utf-8')
        coordinate_str = data_b.split(',')
        x = float(coordinate_str[0])
        y = float(coordinate_str[1])
        print([self.robot, (x,y), self.sim_state_now.sim_t])
        th = np.arctan2(y - self.robot[1], x - self.robot[0])
        if self.joystick_on:
            self.send_cmds([(x, y, th, 0)], send_vel_cmds=False)
        return

    def update_loop(self):
        self.robot_receiver_socket.listen(1)
        self.joystick_on = True
        while (self.joystick_on):
            self.joystick_sense()
            self.joystick_plan()
            self.joystick_act()
        self.socket.sendall(b"OFF")
        self.finish_episode()
        return

