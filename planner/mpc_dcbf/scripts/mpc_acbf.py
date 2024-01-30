#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import threading
import numpy as np
from visualization_msgs.msg import Marker
import casadi as ca

controller_ls = ["None", "MPC-DC", "MPC-SCBF", "MPC-DCBF", "MPC-ACBF"]

def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))


class Local_Planner():
    def __init__(self, controller_n:int):
        # for replan frequency
        self.replan_period = 1/rospy.get_param('/local_planner/mpc_frequency', 5)
        # for control type
        self.controller = controller_ls[controller_n]
        rospy.logwarn('----------the controller is: {}----------'.format(self.controller))

        self.z = 0
        # self.N = 20
        self.N  = rospy.get_param('/local_planner/pre_step', 25)
        self.Ts = rospy.get_param('/local_planner/step_time', 0.1)

        self.goal_state = np.zeros([self.N, 3])
        self.curr_state = None
        self.global_path = None

        self.last_input = []
        self.last_state = []
        self.mpc_success = None

        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        self.ob = []

        self.nums_of_planning = 1
        self.cost_time_sum = 0.0

        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)

        self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10)

        self.__sub_obs = rospy.Subscriber('/obs_Manager_node/obs_predict_pub', Float32MultiArray, self.__obs_cb, queue_size=100)

        self.__sub_goal = rospy.Subscriber('/global_path', Path, self.__global_path_cb, queue_size=25)

        self.__pub_local_path_vis = rospy.Publisher('/pub_path_vis', Marker, queue_size=10)

        self.__pub_local_path = rospy.Publisher('/local_path', Path, queue_size=10)

        self.__pub_local_plan = rospy.Publisher('/local_plan', Float32MultiArray, queue_size=10)

        self.__pub_start = rospy.Publisher('/cmd_move', Bool, queue_size=10)

    def __replan_cb(self, event):
        if self.choose_goal_state():
            # add angle information
            for i in range(self.N - 1):
                y_diff = self.goal_state[i+1, 1]-self.goal_state[i, 1]
                x_diff = self.goal_state[i+1, 0]-self.goal_state[i, 0]
                if x_diff != 0 and y_diff != 0:
                    self.goal_state[i, 2] = np.arctan2(y_diff, x_diff)
                elif i != 0:
                    self.goal_state[i, 2] = self.goal_state[i-1, 2]
                else:
                    self.goal_state[i, 2] = 0

            self.goal_state[-1, 2] = self.goal_state[-2, 2]

            plan_time_start = rospy.Time.now()  # 规划开始时间

            # MPC规划
            states_sol, input_sol = self.MPC_ellip()

            plan_time_spend = (rospy.Time.now() - plan_time_start).to_sec()  # 规划结束时间
            self.cost_time_sum += plan_time_spend
            average_time = self.cost_time_sum / self.nums_of_planning
            rospy.loginfo("plan_time_spend:= %s", plan_time_spend)
            rospy.loginfo("\033[32m[DCBF]: Local-Trajectory Generation Spend Time := %s\033[0m", average_time)
            self.nums_of_planning += 1

            cmd_move = Bool()
            cmd_move.data = distance_global(self.curr_state, self.global_path[-1]) > 0.1
            self.__pub_start.publish(cmd_move)
            self.__publish_local_plan(input_sol, states_sol)

    def __curr_pose_cb(self, data):
        self.curr_pose_lock.acquire()
        self.curr_state = np.zeros(5)
        self.curr_state[0] = data.data[0]
        self.curr_state[1] = data.data[1]
        self.curr_state[2] = data.data[2]
        self.curr_state[3] = data.data[3]
        self.curr_state[4] = data.data[4]

        self.curr_pose_lock.release()

    def __obs_cb(self, data):
        self.obstacle_lock.acquire()
        self.ob = []
        size = int(len(data.data) / 7)
        for i in range(size):
            self.ob.append(np.array(data.data[7*i:7*i+7]))
        self.obstacle_lock.release()

    def __global_path_cb(self, path):
        self.global_path_lock.acquire()
        size = len(path.poses)
        if size > 0:
            self.global_path = np.zeros([size, 3])
            for i in range(size):
                self.global_path[i, 0] = path.poses[i].pose.position.x
                self.global_path[i, 1] = path.poses[i].pose.position.y
        self.global_path_lock.release()

    def __publish_local_plan(self, input_sol, state_sol):
        local_path = Path()
        local_plan = Float32MultiArray()
        local_path_vis = Marker()
        local_path_vis.type = Marker.LINE_LIST
        local_path_vis.scale.x = 0.05
        local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.a = 1.0

        local_path_vis.header.stamp = rospy.Time.now()
        local_path.header.stamp = rospy.Time.now()

        local_path.header.frame_id = "world"
        local_path_vis.header.frame_id = "world"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i, 0]
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z
            this_pose_stamped.pose.orientation.x = 0
            this_pose_stamped.pose.orientation.y = 0
            this_pose_stamped.pose.orientation.z = 0
            this_pose_stamped.pose.orientation.w = 1
            this_pose_stamped.header.seq = i
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "world"
            local_path.poses.append(this_pose_stamped)
            for j in range(2):
                if len(input_sol[i]) != 0:
                    local_plan.data.append(input_sol[i][j])

            pt = Point()
            pt.x = state_sol[i, 0]
            pt.y = state_sol[i, 1]
            pt.z = self.z

            color = ColorRGBA()
            color.r = 1
            color.g = 0.82
            color.b = 0.1
            color.a = 1

            p1 = Point()
            p2 = Point()
            p3 = Point()
            p4 = Point()
            if i < self.N-1:
                x_diff = state_sol[i+1, 0]-state_sol[i, 0]
                y_diff = state_sol[i+1, 1]-state_sol[i, 1]
                if x_diff != 0 and y_diff != 0:
                    theta = np.arctan2(y_diff, x_diff)
                else:
                    theta = 0
                w = 0.7
                l = 0.92
                p1.z = pt.z-0.01
                p1.x = 0.5*(l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p1.y = 0.5*(l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p2.z = pt.z-0.01
                p2.x = 0.5*(-l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p2.y = 0.5*(-l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p3.z = pt.z-0.01
                p3.x = 0.5*(-l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p3.y = 0.5*(-l*np.sin(theta)-w*np.cos(theta)) + pt.y
                p4.z = pt.z-0.01
                p4.x = 0.5*(l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p4.y = 0.5*(l*np.sin(theta)-w*np.cos(theta)) + pt.y

                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.pose.orientation.x = 0
                local_path_vis.pose.orientation.y = 0
                local_path_vis.pose.orientation.z = 0
                local_path_vis.pose.orientation.w = 1

        self.__pub_local_path_vis.publish(local_path_vis)
        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def choose_goal_state(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        if self.global_path is None or self.curr_state is None:
            self.curr_pose_lock.release()
            self.global_path_lock.release()
            return False

        waypoint_num = self.global_path.shape[0]
        num = np.argmin(np.array([distance_global(self.curr_state, self.global_path[i]) for i in range(waypoint_num)]))

        scale = 1
        num_list = []
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i * scale)
            num_list.append(num_path)

        for k in range(self.N):
            self.goal_state[k] = self.global_path[num_list[k]]

        self.curr_pose_lock.release()
        self.global_path_lock.release()
        return True

    def MPC_ellip(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        self.obstacle_lock.acquire()

        opti = ca.Opti()
        # parameters for optimization
        # T = 0.2
        gamma_k = 0.3
        safe_dist = 0.8 # for scout

        v_max = 1.3
        v_min = 0.1
        omega_max = 0.8

        if self.controller == "None":
            opt_x0 = opti.parameter(3)

            # state variables
            opt_states = opti.variable(self.N + 1, 3)
            opt_controls = opti.variable(self.N, 2)
            v = opt_controls[:, 0]
            omega = opt_controls[:, 1]

            def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

            def exceed_ob(ob_):
                l_long_axis = ob_[2]
                l_short_axis = ob_[3]
                long_axis = np.array([np.cos(ob_[4]) * l_long_axis, np.sin(ob_[4]) * l_long_axis])

                ob_vec = np.array([ob_[0], ob_[1]])
                center_vec = self.goal_state[self.N-1, :2] - ob_vec
                dist_center = np.linalg.norm(center_vec)
                cos_ = np.dot(center_vec, long_axis.T) / (dist_center * l_long_axis)

                if np.abs(cos_) > 0.1:
                    tan_square = 1 / (cos_ ** 2) - 1
                    d = np.sqrt((l_long_axis ** 2 * l_short_axis ** 2 * (1 + tan_square) / (
                        l_short_axis ** 2 + l_long_axis ** 2 * tan_square)))
                else:
                    d = l_short_axis

                cross_pt = ob_vec + d * center_vec / dist_center

                vec1 = self.goal_state[self.N-1, :2] - cross_pt
                vec2 = self.curr_state[:2] - cross_pt
                theta = np.dot(vec1.T, vec2)

                return theta > 0

            def h(curpos_, ob_):
                safe_dist = 0.8 # for scout
                # safe_dist = 0.5 # for jackal

                c = ca.cos(ob_[4])
                s = ca.sin(ob_[4])
                a = ca.MX([ob_[2]])
                b = ca.MX([ob_[3]])

                ob_vec = ca.MX([ob_[0], ob_[1]])
                center_vec = curpos_[:2] - ob_vec.T

                dist = b * (ca.sqrt((c ** 2 / a ** 2 + s ** 2 / b ** 2) * center_vec[0] ** 2 + (s ** 2 / a ** 2 + c ** 2 / b ** 2) *
                                    center_vec[1] ** 2 + 2 * c * s * (1 / a ** 2 - 1 / b ** 2) * center_vec[0] * center_vec[1]) - 1) - safe_dist
                
                return dist

            def quadratic(x, A):
                return ca.mtimes([x, A, x.T])

            # init_condition
            opti.subject_to(opt_states[0, :] == opt_x0.T)

            # Position Boundaries
            if(distance_global(self.curr_state, self.global_path[-1, :2]) > 1):
                opti.subject_to(opti.bounded(v_min, v, v_max))
            else:
                opti.subject_to(opti.bounded(-v_min, v, v_max))

            opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

            # System Model constraints
            for i in range(self.N):
                x_next = opt_states[i, :] + self.Ts * f(opt_states[i, :], opt_controls[i, :]).T
                opti.subject_to(opt_states[i + 1, :] == x_next)

            num_obs = int(len(self.ob)/self.N)

            # rospy.loginfo("num_obs := %d", num_obs)

            for j in range(num_obs):
                if not exceed_ob(self.ob[self.N*j]):
                    for i in range(self.N-1):
                        opti.subject_to(h(opt_states[i + 1, :], self.ob[j * self.N + i + 1]) >=
                                        (1 - gamma_k) * h(opt_states[i, :], self.ob[j * self.N + i]))

            obj = 0
            R = np.diag([0.1, 0.05])
            A = np.diag([0.1, 0.02])
            for i in range(self.N):
                Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
                if i < self.N-1:
                    obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
                else:
                    obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q)
            Q = np.diag([1.0,1.0, 0.02])*5
            obj += quadratic(opt_states[self.N-1, :] - self.goal_state[[self.N-1]], Q)

            opti.minimize(obj)
            opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                            'ipopt.acceptable_obj_change_tol': 1e-3}
            opti.solver('ipopt', opts_setting)
            opti.set_value(opt_x0, self.curr_state[:3])

        else:
            print('--------------------------- janedipan\'s model ---------------------------')
            opt_x0 = opti.parameter(5)
            # states[x, y, theta, dx, dy]
            opt_states = opti.variable(self.N+1, 5)
            opt_controls = opti.variable(self.N, 2)
            v = opt_controls[:, 0]
            omega = opt_controls[:, 1]           
            
            def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]) * self.Ts, u_[0] * ca.sin(x_[2]) * self.Ts, u_[1] * self.Ts, 
                                               u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2])])
            
            def h(curpos_:ca.SX, ob_:tuple):
                x_obs, y_obs, r_obs = ob_
                h = ca.sqrt((x_obs-curpos_[0])**2+(y_obs-curpos_[1])**2)-(safe_dist+r_obs)
                return h
            
            def h1(curpos_:ca.SX, ob_:tuple, vr_:tuple, tau_=0.0):
                x_obs, y_obs, r_obs = ob_
                h = ca.sqrt((x_obs-curpos_[0]+vr_[0]*tau_)**2 + (y_obs - curpos_[1]+vr_[1]*tau_)**2) - (safe_dist + r_obs)
                return h

            # 障碍物筛选函数
            def exceed_ob(ob_:list):
                case = 1
                if case == 1:
                    # 前进视野的障碍物
                    ob_vec = np.array([ob_[0], ob_[1]])
                    center_vec = self.goal_state[self.N-1, :2] - ob_vec
                    d = np.linalg.norm(center_vec)
                    cross_pt = ob_vec + ob_[2]/d*center_vec
                    # cross_pt = ob_vec
                    vec1 = self.goal_state[self.N-1, :2] - cross_pt
                    vec2 = self.curr_state[:2] - cross_pt

                    theta = np.dot(vec1.T, vec2)
                    return theta > 0
                elif case == 2:
                    # 机器人周围的障碍物
                    ob_vec = np.array([ob_[0], ob_[1]])
                    vec1 = ob_vec - self.curr_state[:2]
                    sign = np.linalg.norm(vec1) > 5.0
                    return sign
            
            # tau值函数
            def updata_paramter(ob_:np.ndarray) -> float: 
                tau_ = 0.0
                scale_list = 0.0
                ro_p    = self.curr_state[:2]
                ro_v    = self.curr_state[3:]
                obs_p   = ob_[:2]
                obs_v   = ob_[5:]

                p_r     = obs_p - ro_p
                v_r     = obs_v - ro_v
                tau_max = (np.linalg.norm(p_r)-safe_dist-ob_[2])*np.linalg.norm(p_r)/(-np.dot(p_r, v_r))
                if (np.dot(p_r, v_r) < 0.0) & (tau_max < 2.0):
                    tau_max = min(tau_max, 0.65)
                    obs_scale = 1-(abs(np.dot(p_r,obs_v)/np.linalg.norm(p_r)/1.5)-1)**2
                    ro_scale = 1-(np.linalg.norm(p_r)/5-1)**2
                    scale = obs_scale * ro_scale
                else:
                    scale = 0.0
                tau_ = tau_max * scale
                return tau_

            def quadratic(x, A):
                return ca.mtimes([x, A, x.T])

            # init_condition
            opti.subject_to(opt_states[0, :] == opt_x0.T)

            # Position Boundaries
            if(distance_global(self.curr_state, self.global_path[-1, :2]) > 1):
                opti.subject_to(opti.bounded(v_min, v, v_max))
            else:
                opti.subject_to(opti.bounded(-v_min, v, v_max))

            opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

            # System Model constraints
            fa = np.zeros([5,5])
            fa[:3, :3] = np.eye(3) 
            for i in range(self.N):
                x_next = opt_states[i, :]@fa + f(opt_states[i, :], opt_controls[i, :]).T
                opti.subject_to(opt_states[i + 1, :] == x_next)

            num_obs = int(len(self.ob)/self.N)

            # rospy.loginfo("num_obs := %d", num_obs)
            # Safety constraint
            for j in range(num_obs):
                # print(j)
                # print(self.ob[self.N*j])
                if not exceed_ob(self.ob[self.N*j]):
                    print('choose obs{}'.format(j))
                    # 仅根据当前机器人，障碍物当前状态进行tau值计算～待验证
                    if self.controller == "MPC-ACBF":
                        tau = updata_paramter(self.ob[self.N*j])
                        print(tau)
                    vr = self.ob[self.N*j][5:] - self.curr_state[3:]
                    obs = (self.ob[self.N*j][0], self.ob[self.N*j][1], self.ob[self.N*j][2])
                    for i in range(self.N-1):
                        # self.ob[] -> [x, y, a, b, theta, dx, dy]
                        if self.controller == "MPC-DC":
                            hk      = h(opt_states[i, :], obs)
                            opti.subject_to(hk>=0)
                        elif self.controller == "MPC-SCBF":
                            hk      = h(opt_states[i, :], obs)
                            hk1     = h(opt_states[i+1,:], obs)
                            st_cbf  = -hk1 + (1-gamma_k)*hk
                            opti.subject_to(st_cbf<=0)
                        elif self.controller == "MPC-DCBF":
                            obs     = (self.ob[j*self.N+i][0], self.ob[j*self.N+i][1], self.ob[j*self.N+i][2])
                            # obs1    = (self.ob[j*self.N+i+1][0], self.ob[j*self.N+i+1][1], self.ob[j*self.N+i+1][2])
                            hk      = h(opt_states[i, :], obs)
                            hk1     = h(opt_states[i+1,:], obs)
                            st_cbf  = -hk1 + (1-gamma_k)*hk
                            opti.subject_to(st_cbf<=0)
                        elif self.controller == "MPC-ACBF":
                            obs     = (self.ob[j*self.N+i][0], self.ob[j*self.N+i][1], self.ob[j*self.N+i][2])
                            # tau = 0.0
                            hk      = h1(opt_states[i, :], obs, vr, tau)
                            hk1      = h1(opt_states[i+1, :], obs, vr, tau)
                            st_cbf  = -hk1 + (1-gamma_k)*hk
                            opti.subject_to(st_cbf<=0)
                        else:
                            print('-----------------------error----------------------')
        
            # Object function
            obj = 0
            R = np.diag([0.1, 0.05])
            A = np.diag([0.1, 0.02])
            for i in range(self.N):
                Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
                if i < self.N-1:
                    obj += 0.1 * quadratic(opt_states[i, :3] - self.goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
                else:
                    obj += 0.1 * quadratic(opt_states[i, :3] - self.goal_state[[i]], Q)
            Q = np.diag([1.0,1.0, 0.02])*5
            obj += quadratic(opt_states[self.N-1, :3] - self.goal_state[[self.N-1]], Q)

            opti.minimize(obj)
            opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                            'ipopt.acceptable_obj_change_tol': 1e-3}
            opti.solver('ipopt', opts_setting)
            opti.set_value(opt_x0, self.curr_state)

        try:
            sol = opti.solve()
            u_res = sol.value(opt_controls)
            state_res = sol.value(opt_states)

            self.last_input = u_res
            self.last_state = state_res
            self.mpc_success = True

        except:
            rospy.logerr("Infeasible Solution")

            if self.mpc_success:
                self.mpc_success = False
            else:
                for i in range(self.N-1):
                    self.last_input[i] = self.last_input[i+1]
                    self.last_state[i] = self.last_state[i+1]
                    self.last_input[self.N-1] = np.zeros([1, 2])

            u_res = self.last_input
            state_res = self.last_state
        self.curr_pose_lock.release()
        self.global_path_lock.release()
        self.obstacle_lock.release()
        return state_res, u_res


if __name__ == '__main__':
    rospy.init_node("phri_planner")
    controller_n = rospy.get_param('/local_planner/controller_type', 0) # 需要添加前缀
    phri_planner = Local_Planner(controller_n)

    rospy.spin()
