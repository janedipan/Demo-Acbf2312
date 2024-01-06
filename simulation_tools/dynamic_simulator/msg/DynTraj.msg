Header header       # 消息时间
Header start_time   # 障碍物的生成时间
bool use_pwp_field  # Whether to use pwp or the string

string[] s_mean 
string[] s_var 

float32[] s_num

PieceWisePolTraj pwp_mean
PieceWisePolTraj pwp_var

float32[] bbox             #bounding box. It has three elements: hx, hy, hz (bbox of size hx x hy x hz)
geometry_msgs/Vector3 pos  #current position (x,y,z). #TODO (improvement): remove this field (as the current pos will depend when the trajectory is received in the callback)
int32 id                   #id of the dynamic obstacle
bool is_agent              #1 if is an agent, and 0 if it is a static or dynamic obstacle
