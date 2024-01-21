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

    def getTrajectoryPosMeshBBox(self, i, start_time):

        delta_beginning = 2.0
        delta  = (self.x_max - self.x_min - delta_beginning) / self.total_num_obs
        mesh=self.available_meshes_dynamic[0]
        bbox=self.bbox_dynamic

        # 障碍物轨迹初始参数, 随机设置
        x      = delta_beginning + self.x_min + i*delta
        y      = random.uniform(self.y_min, self.y_max)
        z      = random.uniform(self.z_min, self.z_max)
        offset = random.uniform(-2*math.pi, 2*math.pi)
        slower = random.uniform(self.slower_min, self.slower_max)

        [x_string, y_string, z_string, s_num] = self.trefoil(x,y,z, self.scale[0],self.scale[1],self.scale[2], offset, slower, start_time)
        return [x_string, y_string, z_string, s_num, x, y, z, mesh, bbox]

    def __init__(self, total_num_obs,gazebo):

        random.seed(datetime.now())

        name = rospy.get_namespace()
        self.name = name[1:-1]

        print(total_num_obs)
        self.total_num_obs=total_num_obs
        self.x_min= 2.0
        self.x_max= 18.0
        self.y_min= -2.0
        self.y_max= 2.0
        self.z_min= 0.75
        self.z_max= 0.75
        self.scale= [8.0, 4.0, 0.0]
        self.slower_min=10.0   #1.2 or 2.3
        self.slower_max=10.0   #1.2 or 2.3
        self.bbox_dynamic=[0.3, 0.3, 1.5]
        self.name_obs="obs_"
        self.available_meshes_dynamic=["package://dynamic_simulator/meshes/ConcreteDamage01b/model4.dae"]
        self.marker_array=MarkerArray()
        self.all_dyn_traj=[]

        t_ros=rospy.Time.now()
        t=rospy.get_time()

        for i in range(self.total_num_obs): 
            [traj_x, traj_y, traj_z, s_num, x, y, z, mesh, bbox]=self.getTrajectoryPosMeshBBox(i, t);           
            self.marker_array.markers.append(self.generateMarker(mesh, bbox, i))

            dynamic_trajectory_msg=DynTraj(); 
            dynamic_trajectory_msg.use_pwp_field=False
            dynamic_trajectory_msg.is_agent=False
            dynamic_trajectory_msg.header.stamp= t_ros
            dynamic_trajectory_msg.start_time.stamp = t_ros
            dynamic_trajectory_msg.s_mean = [traj_x, traj_y, traj_z]
            dynamic_trajectory_msg.s_var = ["0.001", "0.001", "0.001"]
            dynamic_trajectory_msg.bbox = [bbox[0], bbox[1], bbox[2]]
            dynamic_trajectory_msg.pos.x=x    # 障碍物初始位置x
            dynamic_trajectory_msg.pos.y=y    # 障碍物初始位置y
            dynamic_trajectory_msg.pos.z=z    # 障碍物初始位置z
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

        s_num = [scale_x/6.0, scale_y/5.0, scale_z/2.0, x, y, z, slower, offset]

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


def startNode(total_num_obs, gazebo):
    c = DynCorridor(total_num_obs,gazebo)
    rospy.Timer(rospy.Duration(0.050), c.pubTF)
    rospy.spin()

if __name__ == '__main__':
    print("********************************")
    print(sys.argv)
    if(len(sys.argv)<=1):
        total_num_obs=10
    else:
        total_num_obs=int(sys.argv[1])

    gazebo=((sys.argv[2]=='True') or (sys.argv[2]=='true'))

    try:
        rospy.init_node('dynamic_obstacles_sim')
        startNode(total_num_obs,gazebo)
    except rospy.ROSInterruptException:
        pass