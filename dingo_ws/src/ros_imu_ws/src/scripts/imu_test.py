import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from sensor_msgs.msg import MagneticField

def imu_publisher():
    rospy.init_node("imu_test_publisher", anonymous=True)
    pub = rospy.Publisher("/wit/imu", Imu, queue_size=10)
    mag_pub = rospy.Publisher("wit/mag", MagneticField, queue_size=100)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        imu_msg = Imu()
        mag_msg = MagneticField()
        stamp = rospy.get_rostime()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "base_link"

        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = "base_link"
        


        imu_msg.orientation.x = 0
        imu_msg.orientation.y = 0
        imu_msg.orientation.z = 0
        imu_msg.orientation.w = 0

        imu_msg.angular_velocity.x = 0
        imu_msg.angular_velocity.y = 0
        imu_msg.angular_velocity.z = 0

        imu_msg.linear_acceleration.x = 0
        imu_msg.linear_acceleration.y = 0
        imu_msg.linear_acceleration.z = 0

        mag_msg.magnetic_field.x = 0
        mag_msg.magnetic_field.y = 0
        mag_msg.magnetic_field.z = 0

        pub.publish(imu_msg)
            #rospy.loginfo("Published IMU data: %s", imu_msg)
        mag_pub.publish(mag_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass