#!/usr/bin/env python3
import rospkg
import random
import rospy
import math
import sys
import tf
import os
from math import sin, cos
from datetime import datetime
from dynamic_simulator.msg import DynTraj
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class DynCorridor:

    def __init__(self, gazebo, obstacle_params):

        random.seed(datetime.now())

        name = rospy.get_namespace()
        self.name = name[1:-1]

        print(len(obstacle_params))

        self.obstacle_params = obstacle_params
        self.total_num_obs=len(obstacle_params)
        self.bbox_dynamic =[0.3, 0.3, 1.5]    # 障碍物的尺寸大小
        self.name_obs="obs_"
        self.available_meshes_dynamic=["package://dynamic_simulator/meshes/ConcreteDamage01b/model4.dae"]
        self.marker_array=MarkerArray()
        self.all_dyn_traj=[]

        t_ros=rospy.Time.now()
        t=rospy.get_time()

        for i in range(self.total_num_obs):
            [traj_x, traj_y, traj_z, s_num, x, y, z, mesh, bbox]=self.getTrajectoryPosMeshBBox(i, t)
            self.marker_array.markers.append(self.generateMarker(mesh, bbox, i))

            dynamic_trajectory_msg = DynTraj(); 
            dynamic_trajectory_msg.header.stamp= t_ros
            dynamic_trajectory_msg.start_time.stamp = t_ros
            dynamic_trajectory_msg.s_mean = [traj_x, traj_y, traj_z]
            dynamic_trajectory_msg.s_var = ["0.001", "0.001", "0.001"]
            dynamic_trajectory_msg.bbox = [bbox[0], bbox[1], bbox[2]]
            dynamic_trajectory_msg.pos.x = x    # 障碍物初始位置x
            dynamic_trajectory_msg.pos.y = y    # 障碍物初始位置y
            dynamic_trajectory_msg.pos.z = z    # 障碍物初始位置z
            dynamic_trajectory_msg.id = 4000 + i
            dynamic_trajectory_msg.s_num = [s_num[0], s_num[1], s_num[2], s_num[3], s_num[4], s_num[5], s_num[6], s_num[7]]

            self.all_dyn_traj.append(dynamic_trajectory_msg)

        self.pubTraj = rospy.Publisher('/Dyn_Obs_trajs', DynTraj, queue_size=100, latch=True)
        self.pubShapes_dynamic_mesh = rospy.Publisher('/obstacles_mesh', MarkerArray, queue_size=1, latch=True)

        if(gazebo):
            for i in range(self.total_num_obs):
                self.spawnGazeboObstacle(i)
                rospy.sleep(0.05)

        rospy.sleep(1.0)

    def getTrajectoryPosMeshBBox(self, i, start_time):
        # 障碍物轨迹初始参数, 随机设置
        x      = self.obstacle_params[i]['x']
        y      = self.obstacle_params[i]['y']
        z      = self.obstacle_params[i]['z']
        offset = self.obstacle_params[i]['offset']
        slower = self.obstacle_params[i]['slower']
        scale = [self.obstacle_params[i]['scale_x'], self.obstacle_params[i]['scale_y'], self.obstacle_params[i]['scale_z']]

        [x_string, y_string, z_string, s_num] = self.trefoil(x,y,z, scale[0], scale[1], scale[2], offset, slower, start_time)
        return [x_string, y_string, z_string, s_num, x, y, z, self.available_meshes_dynamic[0], self.bbox_dynamic]

    def generateMarker(self, mesh, bbox, i):
        marker=Marker()
        marker.id=i
        marker.ns="mesh"
        marker.header.frame_id="world"
        marker.type=marker.MESH_RESOURCE
        marker.action=marker.ADD
        marker.pose.position.x=0.0 #Will be updated later
        marker.pose.position.y=0.0 #Will be updated later
        marker.pose.position.z=0.0 #Will be updated later
        marker.pose.orientation.x=0.0
        marker.pose.orientation.y=0.0
        marker.pose.orientation.z=0.0
        marker.pose.orientation.w=1.0
        marker.lifetime = rospy.Duration.from_sec(0.0)
        marker.mesh_use_embedded_materials=True
        marker.mesh_resource=mesh
        marker.scale.x=bbox[0]
        marker.scale.y=bbox[1]
        marker.scale.z=bbox[2]
        return marker

    def pubTF(self, timer):
        if(gazebo):
          br = tf.TransformBroadcaster()

        for i in range(self.total_num_obs): 
            t_ros=rospy.Time.now()
            t=rospy.get_time()
          
            x = eval(self.all_dyn_traj[i].s_mean[0])
            y = eval(self.all_dyn_traj[i].s_mean[1])
            z = eval(self.all_dyn_traj[i].s_mean[2])

            # Set the stamp and the current pos
            self.all_dyn_traj[i].header.stamp= t_ros
            self.all_dyn_traj[i].pos.x=x # 障碍物当前位置x
            self.all_dyn_traj[i].pos.y=y # 障碍物当前位置y
            self.all_dyn_traj[i].pos.z=z # 障碍物当前位置z
            self.pubTraj.publish(self.all_dyn_traj[i])

            if(gazebo):
              br.sendTransform((x, y, z), (0, 0 ,0 ,1), t_ros, self.name_obs+str(self.all_dyn_traj[i].id), "world")

            self.marker_array.markers[i].pose.position.x=x
            self.marker_array.markers[i].pose.position.y=y
            self.marker_array.markers[i].pose.position.z=z
        
        self.pubShapes_dynamic_mesh.publish(self.marker_array)

    def trefoil(self,x,y,z,scale_x, scale_y, scale_z, offset, slower, start_time):

        tt='(t - ' + str(start_time) + ')/' + str(slower)+' + '

        x_string=str(scale_x/6.0)+' *(sin('+tt +str(offset)+' )+2*sin(2*'+tt +str(offset)+' ))' +'+ ' + str(x)
        y_string=str(scale_y/5.0)+' *(cos('+tt +str(offset)+' )-2*cos(2*'+tt +str(offset)+' ))' +'+ ' + str(y)
        z_string=str(scale_z/2.0)+' *(-sin(3*'+tt +str(offset)+ '))' + '+ ' + str(z)

        s_num = [scale_x/6.0, scale_y/5.0, scale_z/2.0, x, y, z, slower,  offset]

        return [x_string, y_string, z_string, s_num]

    def spawnGazeboObstacle(self, i):

            rospack = rospkg.RosPack()
            path_dynamic_simulator=rospack.get_path('dynamic_simulator')
            path_file=path_dynamic_simulator+"/meshes/tmp_"+str(i)+".urdf"

            f = open(path_file, "w") #TODO: This works, but it'd better not having to create this file
            scale=self.marker_array.markers[i].scale
            scale='"'+str(scale.x)+" "+str(scale.y)+" "+str(scale.z)+'"'

            x=self.all_dyn_traj[i].pos.x
            y=self.all_dyn_traj[i].pos.y
            z=self.all_dyn_traj[i].pos.z

            f.write("""
<robot name="name_robot">
  <link name="name_link">
    <inertial>
      <mass value="0.200" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="5.8083e-4" ixy="0" ixz="0" iyy="3.0833e-5" iyz="0" izz="5.9083e-4" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="""+'"'+self.marker_array.markers[i].mesh_resource+'"'+""" scale="""+scale+"""/>
      </geometry>
    </visual>
  </link>
  <gazebo>
    <plugin name="move_model" filename="libmove_model.so">
    <traj_x>"""+self.all_dyn_traj[i].s_mean[0]+"""</traj_x>
    <traj_y>"""+self.all_dyn_traj[i].s_mean[1]+"""</traj_y>
    <traj_z>"""+self.all_dyn_traj[i].s_mean[2]+"""</traj_z>
    </plugin>
  </gazebo>
</robot>
                """)
            f.close()

            os.system("rosrun gazebo_ros spawn_model -file "+path_file+" -urdf -x " + str(x) + " -y " + str(y) + " -z " + str(z) + " -model "+self.name_obs+str(i)+" && rm "+path_file + " &"); #all_


def startNode(gazebo):
    obstacle_params = rospy.get_param('obstacle_params')

    # 处理障碍物数据
    for obstacle in obstacle_params:
        x = obstacle['x']
        y = obstacle['y']
        z = obstacle['z']
        offset = obstacle['offset']
        slower = obstacle['slower']
        rospy.loginfo('Obstacles param: (x: %f, y: %f, z: %f, offset: %f, slower: %f)', x, y, z, offset, slower)

    c = DynCorridor(gazebo, obstacle_params)

    rospy.Timer(rospy.Duration(0.050), c.pubTF)
    rospy.spin()

if __name__ == '__main__':
    print("********************************")
    print(sys.argv)

    gazebo=((sys.argv[2]=='True') or (sys.argv[2]=='true'))

    try:
        rospy.init_node('dynamic_obstacles_sim')
        startNode(gazebo)
    except rospy.ROSInterruptException:
        pass