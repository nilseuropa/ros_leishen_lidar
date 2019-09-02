#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <std_msgs/UInt16.h>
#include <n301n_lidar.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string port;
    std::string frame_id;

    priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
    priv_nh.param("frame_id", frame_id, std::string("laser_link"));

    boost::asio::io_service io;
    try{
        n301_lidar_driver::n301n_lidar laser(port, 115200, io);
        ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

        while(ros::ok()){
            sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
            if (scan == NULL) ROS_WARN("No data available.");
            scan->header.frame_id = frame_id;
            scan->header.stamp = ros::Time::now();
            laser.poll(scan, 1);
            laser_pub.publish(scan);
        }
        laser.close();
        return 0;
    }
    catch (boost::system::system_error ex){
        ROS_ERROR("Error instantiating lidar object. Are you sure you set the correct port? Error was %s", ex.what());
        return -1;
    }
}
