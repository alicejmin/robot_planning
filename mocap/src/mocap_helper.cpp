#include <iostream>
#include <vector>
#include <geometry_msgs/Point.h>

#include <ros/ros.h>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_helper");

  ros::NodeHandle nl("~");

  std::string motionCaptureType;
  nl.param<std::string>("motion_capture_type", motionCaptureType, "vicon");

  ros::Publisher goal_pub = nl.advertise<geometry_msgs::Point>("/goal_pos", 1000);
  ros::Rate loop_rate(20);

  std::map<std::string, std::string> cfg;
  std::string hostname;
  nl.getParam("motion_capture_host_name", hostname);
  cfg["hostname"] = hostname;
  if (nl.hasParam("motion_capture_interface_ip")) {
    std::string interface_ip;
    nl.param<std::string>("motion_capture_interface_ip", interface_ip);
    cfg["interface_ip"] = interface_ip;
  }

  std::unique_ptr<libmotioncapture::MotionCapture> mocap(libmotioncapture::MotionCapture::connect(motionCaptureType, cfg));
  if (!mocap) {
    throw std::runtime_error("Unknown motion capture type!");
  }

  for (size_t frameId = 0; ros::ok(); ++frameId) {
    std::cout << "frame " << frameId << ":" << std::endl;
    // Get a frame
    mocap->waitForNextFrame();
    const auto& markers = mocap->pointCloud();

    std::cout << "    points:" << std::endl;

    for (size_t i = 0; i < markers.rows(); ++i) {
      const auto& point = markers.row(i);
      std::cout << "      \"" << i << "\": [" << point(0) << "," << point(1) << "," << point(2) << "]" << std::endl;
    }

    try{
        if (mocap->supportsRigidBodyTracking()) {
        const auto& rigidBodies = mocap->rigidBodies();

        std::cout << "    rigidBodies:" << std::endl;

        for (auto const& kv: rigidBodies) {
          const auto& body = kv.second;
          std::cout << "      \"" << body.name() << "\":" << std::endl;

          Eigen::Vector3f position = body.position();
          Eigen::Quaternionf rotation = body.rotation();
          std::cout << "       position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
          std::cout << "       rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
                                            << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;

          geometry_msgs::Point goal_msg;
          goal_msg.x = position(0);
          goal_msg.y = position(1);
          goal_msg.z = position(2);
          

          goal_pub.publish(goal_msg);

        }
      }
    }
    catch(...){
      continue;
    }

    ros::spinOnce();
  }

  return 0;
}
