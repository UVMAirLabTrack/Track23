import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_msgs.msg import WorldMarkers,MarkerLoc
from rclpy.node import Node
from x_core2 import pose_strip, open_world_data,formulas


class PoseRecNode(Node):
    def __init__(self):
        super().__init__('pose_test_node')

        #copy the lines below into any marker nodes, dont forget the import either
        #from x_core2 import pose_strip
        #from custom_msgs.msg import WorldMarkers,MarkerLoc
        self.subscription2 = self.create_subscription(MarkerLoc, 'marker_loc', self.loc_call, 10)
        self.subscription = self.create_subscription(WorldMarkers, 'custom_poses', self.pose_call, 10)
        self.zone= 'empty'
        self.loc = 'empty'
        self.pose = Pose()
        self.marker = 'Sign2' #set for testing, use later in other classes.

    def pose_call(self,msg):
        self.pose = pose_strip.strip_pose(msg,self.zone,self.loc)
        print(self.pose)

    def loc_call(self,msg):
        self.zone,self.loc = pose_strip.strip_marker_loc(msg,self.marker)
        print(f'{self.zone} {self.loc}')
        #end copy
        

def strip_pose(msg, desired_title, desired_entry1):
    for i in range(len(msg.title)):
       # print(f'--{msg.title[i]}--{desired_title}--')     
        if msg.title[i].strip() == desired_title.strip():
            if msg.entry1[i].strip() == desired_entry1.strip():
                pose = Pose()
                pose.position.x = msg.x[i]
                pose.position.y = msg.y[i]
                pose.position.z = msg.z[i]
                pose.orientation.x = msg.qx[i]
                pose.orientation.y = msg.qy[i]
                pose.orientation.z = msg.qz[i]
                pose.orientation.w = msg.qw[i]  # Assuming no rotation

                return pose  # Return the pose if match found
    if desired_title != "na":
        print(f'{desired_title}:{desired_entry1}, Matching Pose not found, returning empty pose.')
    pose = Pose()
    return pose

def strip_pose_quat(msg, desired_title, desired_entry1):
    for i in range(len(msg.title)):
       # print(f'--{msg.title[i]}--{desired_title}--')     
        if msg.title[i].strip() == desired_title.strip():
            if msg.entry1[i].strip() == desired_entry1.strip():
                pose = [
                msg.qx[i],
                msg.qy[i],
                msg.qz[i],
                 msg.qw[i] ] # Assuming no rotation

                return pose  # Return the pose if match found
    if desired_title != "na":
        print(f'{desired_title}:{desired_entry1}, Matching Pose not found, returning empty pose.')
    pose = Pose()
    return pose

def pose_xyz_shift(pose,marker_pose):
    pose.position.x = pose.position.x + marker_pose.position.x
    pose.position.y = pose.position.y + marker_pose.position.y
    pose.position.z = pose.position.z + marker_pose.position.z
    print(f'X:{pose.position.x} Y:{pose.position.y} Z:{pose.position.z}')

    return pose

def z_rotation(pose,Marker_pose):
        q = [Marker_pose.orientation.x,Marker_pose.orientation.y,Marker_pose.orientation.z]
        z_rot = pose.orientation.z
        q2 = formulas.euler_to_quat(q[0],q[1],q[2]+z_rot)
        print(f'angles: {q}  Z_rot: {z_rot}' )

        pose.orientation.x = q2[0]
        pose.orientation.y = q2[1]
        pose.orientation.z = q2[2]
        pose.orientation.w = q2[3] 

        return pose
def strip_eulers(pose,Marker_pose):
        q = [Marker_pose.orientation.x,Marker_pose.orientation.y,Marker_pose.orientation.z]
        q[2] = pose.orientation.z
        return q

def strip_marker_loc(msg,marker):
    for i in range(len(msg.title)):
        if msg.title[i] == marker:
            zone = msg.entry1[i]
            loc = msg.entry2[i]
            return zone,loc
        
    print(f'{marker}: Matching Zone or Loc not found, defaulting to na.')
    zone = 'na'
    loc = 'na'
    return zone, loc

def read_marker_param(marker_name): #also not in use
        file_path = open_world_data.find_marker_adjust_path(marker_name)
        with open(file_path, 'r') as file:
            lines = file.readlines()

        pose_data = {}
        for line in lines:
            key, value = line.strip().split()
            try:
                pose_data[key] = float(value)
            except ValueError:
                # Handle non-numeric values, for example, by keeping them as strings
                pose_data[key] = value

        return pose_data    

def strip_marker_pose(pose_data): #currently not in use
    marker = Pose()
    marker.position.x = pose_data.get('X_position', 0.0)
    marker.position.y = pose_data.get('Y_position', 1.0)
    marker.position.z = pose_data.get('Z_position', 1.0)

    marker.orientation.x = pose_data.get('Quaternion_x', 0.0)
    marker.orientation.y = pose_data.get('Quaternion_y', 0.0)
    marker.orientation.z = pose_data.get('Quaternion_z', 0.0)
    marker.orientation.w = pose_data.get('Quaternion_w', 1.0)

    return marker
    


def shift_pose_xy(msg,msg2):
     pass


def main(args=None):
    rclpy.init(args=args)
    node = PoseRecNode()
    rclpy.spin(node)
    rclpy.shutdown()
    





if __name__ == '__main__':
    main()
