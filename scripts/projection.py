#!/usr/bin/env python
import rospy
import tf2_ros, tf
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from projector_sim import ProjectorSim
from camerainfo_calculator import CameraInfoCalculator

class Projection:
    def __init__(self, projector, use_fake_img, camera_z_meters, has_extrinsics):
        rospy.init_node('projection', anonymous=True)
        
        self.projector = projector
        
        self.bridge = CvBridge()
        imgTopic = '/src_img/image_raw' if use_fake_img else '/camera1/image'

        # subscribe to source image
        self.src_sub = rospy.Subscriber(imgTopic, Image, self.src_cb)
        # publish prewarped source image
        self.warped_src_pub = rospy.Publisher('/processed_img', Image, queue_size=1)

        # publish projection's tf and img
        self.quad_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.quad_pub = rospy.Publisher('/original_projection', Image, queue_size=1)
        self.quad_correct_pub = rospy.Publisher('/corrected_projection', Image, queue_size=1)

        # obtain quad position
        self.quad_pos, camera_w, camera_h = self.projector.calcFrame()
            
        if not use_fake_img:
            # calcaulte camera info
            rospy.loginfo('Camera frame width:%s, height:%s'%(camera_w, camera_h))
            calc = CameraInfoCalculator(z_meters=camera_z_meters, camframe_width_meters=camera_w, camframe_height_meters=camera_h)
            self.cameraInfo = calc.get_info()
            
            # publish camera info
            self.caminfo_pub = rospy.Publisher('/camera1/camera_info', CameraInfo, queue_size=1)
        
        rate = rospy.Rate(10)

        if has_extrinsics:
            # broadcast quad tf (parent: projector_link)
            self.broadcast_quad_tf()

            if not use_fake_img:
                while not rospy.is_shutdown():
                    self.caminfo_pub.publish(self.cameraInfo)   
                    rate.sleep()
        else:
            # broadcast projector tf (parent: quad1_link)
            # broadcast projector optical tf (parent: projector_link)
            r, p, y = self.projector.roll, self.projector.pitch, self.projector.yaw
    
            while not rospy.is_shutdown():
                br = tf.TransformBroadcaster()
                br.sendTransform((self.quad_pos[1], self.quad_pos[0], self.quad_pos[2]),
                            tf.transformations.quaternion_from_euler(3.14, 0.0, -1.57),
                            rospy.Time.now(),
                            "projector_link",
                            "quad1_link")
                br.sendTransform((0.0, 0.0, 0.0),
                            tf.transformations.quaternion_from_euler(r, p, y),
                            rospy.Time.now(),
                            "projector_optical_link",
                            "projector_link")
                
                if not use_fake_img:
                    self.caminfo_pub.publish(self.cameraInfo)   
                
                rate.sleep()

    def broadcast_quad_tf(self):
        """
        broadcast tf (parent: projector_link, child: quad1_link)      
        """
        quad_tf = TransformStamped()
        quad_tf.header.stamp = rospy.Time.now()
        quad_tf.header.frame_id = "projector_link"
        quad_tf.child_frame_id = "quad1_link"

        quad_tf.transform.translation.x = self.quad_pos[0]
        quad_tf.transform.translation.y = self.quad_pos[1]
        quad_tf.transform.translation.z = self.quad_pos[2]

        # adapt world orientation to quad image orientation
        q = tf.transformations.quaternion_from_euler(3.14, 0.0, -1.57)

        quad_tf.transform.rotation.x = q[0]
        quad_tf.transform.rotation.y = q[1]
        quad_tf.transform.rotation.z = q[2]
        quad_tf.transform.rotation.w = q[3]

        quad_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        quad_tf_broadcaster.sendTransform(quad_tf)
        
    def src_cb(self, msg):
        img_bgr = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        
        # projection of source image
        projection = self.projector.projectImage(img_bgr)
        self.quad_pub.publish(self.bridge.cv2_to_imgmsg(projection, "bgr8"))

        # projection of preprocessed source image
        warped = self.projector.prewarpImage(img_bgr)
        correct_projection = self.projector.projectImage(warped)
        
        self.warped_src_pub.publish(self.bridge.cv2_to_imgmsg(warped, "bgr8"))
        self.quad_correct_pub.publish(self.bridge.cv2_to_imgmsg(correct_projection, "bgr8"))
        
if __name__=="__main__": 
    try:
        has_extrinsics = rospy.get_param('projector/has_extrinsics')

        # projector optical parameters
        h_fov = rospy.get_param('projector/hfov')
        v_fov = rospy.get_param('projector/vfov')
        aspect_w = rospy.get_param('projector/aspect_w')
        aspect_h = rospy.get_param('projector/aspect_h')
        size_factor = rospy.get_param('projector/size_factor')

        use_fake_img = rospy.get_param('projector/use_fake_image')
        camera_z_meters = rospy.get_param('/camera1/z_meters')

        if has_extrinsics:
            # load tf (parent: world, child: projector_optical_link)
            z = rospy.get_param('projector/z')
            r = rospy.get_param('projector/roll')
            p = rospy.get_param('projector/pitch')
            y = rospy.get_param('projector/yaw')
            projector = ProjectorSim(z, (r, p, y), None, (h_fov, v_fov), (aspect_w, aspect_h), size_factor)
        else:
            # load projection dimension (meters, obtained by measuring the projection on floor)
            proj_height = rospy.get_param('projector/result/height')
            proj_lower_width = rospy.get_param('projector/result/lower_width')
            proj_upper_width = rospy.get_param('projector/result/upper_width')
            projector = ProjectorSim(None, None, (proj_height, proj_lower_width, proj_upper_width), (h_fov, v_fov), (aspect_w, aspect_h), size_factor)
            
        p = Projection(projector, use_fake_img, camera_z_meters, has_extrinsics)

        rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)