import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_msgs.msg import WorldMarkers,MarkerLoc
from rclpy.node import Node
from x_core2 import pose_strip


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
                pose.orientation.w = 1.0  # Assuming no rotation

                return pose  # Return the pose if match found

    print("Matching Pose not found, returning empty pose.")
    pose = Pose()
    return pose

def strip_marker_loc(msg,marker):
    for i in range(len(msg.title)):
        if msg.title[i] == marker:
            zone = msg.entry1[i]
            loc = msg.entry2[i]
            return zone,loc
        
    print("Matching Zone or Loc not found, defaulting to na.")
    zone = 'na'
    loc = 'na'
    return zone, loc


def shift_pose_xy(msg,msg2):
     pass


def main(args=None):
    rclpy.init(args=args)
    node = PoseRecNode()
    rclpy.spin(node)
    rclpy.shutdown()
    





if __name__ == '__main__':
    main()
