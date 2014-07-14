#!/usr/bin/env python  
import roslib
from tf.transformations import identity_matrix, euler_matrix, translation_matrix,\
    concatenate_matrices, quaternion_from_matrix, quaternion_from_euler
roslib.load_manifest('industrial_pcl_utilities')
import rospy
import tf.transformations
import sensor_msgs
from sensor_msgs.msg import PointCloud2

INPUT_POINT_CLOUD_TOPIC = "cloud_in"
OUTPUT_POINT_CLOUD_TOPIC = "cloud_out"
TRANSFORM_PARAM = "parent_to_child_pose"
   
class PointCloudTransform:
    
    def __init__(self):
        
        # parameters
        self.parent_frame_ = ''
        self.child_frame_ = ''
        self.cloud_frame_ = ''
        self.tf_broadcaster_ = tf.TransformBroadcaster()
        self.transform_ = identity_matrix()
        self.point_cloud_subs_ = None
        self.point_cloud_pub_ = None
        
        # ros init
        rospy.init_node('pointcloud_transform_node')
        self.point_cloud_subs = rospy.Subscriber(INPUT_POINT_CLOUD_TOPIC,PointCloud2,self.pointcloud_callback)
        self.point_cloud_pub_ = rospy.Publisher(OUTPUT_POINT_CLOUD_TOPIC,PointCloud2,queue_size=1)
        
    def run(self):
        
        loop_duration = rospy.Rate(2)
        while ((not rospy.is_shutdown()) and self.load_parameters()):
            
            loop_duration.sleep()
            #quaternion_from_matrix(matrix)
            #q = quaternion_from_euler(angles)
            scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(self.transform_)
            self.tf_broadcaster_.sendTransform(trans, quaternion_from_euler(angles[0],angles[1],angles[2]), rospy.Time.now(),
                                                self.child_frame_,self.parent_frame_)
            
    def load_parameters(self):
        
        try:
            self.parent_frame_ = rospy.get_param("~parent_frame")
            self.child_frame_ = rospy.get_param("~child_frame")
            self.cloud_frame_ = rospy.get_param("~cloud_frame")
            self.transform_ = self.load_transform_param(TRANSFORM_PARAM)
            
            #rospy.loginfo("parameters loaded")
        except KeyError:
            rospy.logerr("parameters not found")
            return False
        return True
        
    def load_transform_param(self,param_name):
        
        t = rospy.get_param('~' + param_name)
        x,y,z,rx,ry,rz = t['x'],t['y'],t['z'],t['rx'],t['ry'],t['rz']
        
        xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
        Rx = tf.transformations.rotation_matrix(rx, xaxis)
        Ry = tf.transformations.rotation_matrix(ry, yaxis)
        Rz = tf.transformations.rotation_matrix(rz, zaxis)
        
        trasn = translation_matrix((x,y,z))
        rot = concatenate_matrices(Rx, Ry, Rz)
        return concatenate_matrices(trasn,rot)    
        
            
       
    def pointcloud_callback(self,msg):
        #new_msg = PointCloud2(msg)
        msg.header.frame_id = self.cloud_frame_
        #rospy.loginfo("received msg") 
        
        # publishing cloud
        self.point_cloud_pub_.publish(msg)
        
   
if __name__ == '__main__':
    
    p = PointCloudTransform()
    p.run()