#include "slam_node.hpp"

SlamNode::SlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node)
: Node("ORB_SLAM3_Inertial"), m_SLAM(pSLAM), node_(node)
{
    tf_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
    pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    pathpublisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    posepublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    statepublisher = this->create_publisher<std_msgs::msg::String>("state", 10);
    flagpublisher = this->create_publisher<std_msgs::msg::Bool>("flag", 10);

    this->declare_parameter("frame_id", "orbslam3");
    this->declare_parameter("child_frame_id", "left_camera_link");

}
SlamNode::~SlamNode() {
    // Para todas as threads
    m_SLAM->Shutdown();
}
void SlamNode::Update(){
    int state_num = m_SLAM->GetTrackingState();
    
    auto statemsg = std_msgs::msg::String();
    switch (state_num)
    {
    case -1:
        statemsg.data = "SYSTEM_NOT_READY";
        break;
    case 0:
        statemsg.data = "NO_IMAGES_YET";
        break;
    case 1:
        statemsg.data = "NOT_INITIALIZED";
        break;
    case 2:
        statemsg.data = "OK";
        break;
    case 3:
        statemsg.data = "RECENTLY_LOST";
        break;
    case 4:
        statemsg.data = "LOST";
        break;
    case 5:
        statemsg.data = "OK_KLT";
        break;
    }
    
    auto flagmsg = std_msgs::msg::Bool();
    flagmsg.data = m_SLAM->MapChanged();

    flagpublisher->publish(flagmsg);
    statepublisher->publish(statemsg);
    PublishTransform();
    PublishTrackedPointCloud();
    PublishPose();
    PublishPath();
}


void SlamNode::PublishTrackedPointCloud(){
    std::vector<int> indexes;
    std::vector<ORB_SLAM3::MapPoint*> points = m_SLAM->GetAllMapPoints();

    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();

    int count = 0;
    
    for (size_t i = 0; i < points.size(); i++)
    {
        if(points[i] != 0){
            count++;
            indexes.push_back(i);

        }
    }
    
    pointcloudmsg.header.stamp = current_frame_time_;
    pointcloudmsg.header.frame_id = "down";
    pointcloudmsg.height = 1;
    pointcloudmsg.width = count;
    pointcloudmsg.is_dense = true;
    pointcloudmsg.fields.resize(3);

    // Populate the fields
    pointcloudmsg.fields[0].name = "x";
    pointcloudmsg.fields[0].offset = 0;
    pointcloudmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[0].count = 1;

    pointcloudmsg.fields[1].name = "y";
    pointcloudmsg.fields[1].offset = 4;
    pointcloudmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[1].count = 1;

    pointcloudmsg.fields[2].name = "z";
    pointcloudmsg.fields[2].offset = 8;
    pointcloudmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[2].count = 1;

    pointcloudmsg.point_step = 12; // Size of a single point in bytes (3 floats * 4 bytes/float)
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
    pointcloudmsg.is_bigendian = false;
    pointcloudmsg.data.resize(pointcloudmsg.point_step*count);

    for (size_t i = 0; i < count; i++)
    {
        float x = points[indexes[i]]->GetWorldPos()(2);
        float y = -1.0 * points[indexes[i]]->GetWorldPos()(0);
        float z = -1.0 * points[indexes[i]]->GetWorldPos()(1);

        memcpy(&pointcloudmsg.data[i*12], &x, 4);
        memcpy(&pointcloudmsg.data[i*12 + 4], &y, 4);
        memcpy(&pointcloudmsg.data[i*12 + 8], &z, 4);
    }
    pclpublisher->publish(pointcloudmsg);

}
void SlamNode::PublishCurrentPointCloud(){
    std::vector<int> indexes;
    std::vector<ORB_SLAM3::MapPoint*> points = m_SLAM->GetTrackedMapPoints();
    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();
    

    int count = 0;
    
    for (size_t i = 0; i < points.size(); i++)
    {
        if(points[i] != 0){
            count++;
            indexes.push_back(i);

        }
    }
    
    pointcloudmsg.header.stamp = current_frame_time_;
    pointcloudmsg.header.frame_id = this->get_parameter("frame_id").as_string();
    pointcloudmsg.height = 1;
    pointcloudmsg.width = count;
    pointcloudmsg.is_dense = true;
    pointcloudmsg.fields.resize(3);

    // Populate the fields
    pointcloudmsg.fields[0].name = "x";
    pointcloudmsg.fields[0].offset = 0;
    pointcloudmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[0].count = 1;

    pointcloudmsg.fields[1].name = "y";
    pointcloudmsg.fields[1].offset = 4;
    pointcloudmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[1].count = 1;

    pointcloudmsg.fields[2].name = "z";
    pointcloudmsg.fields[2].offset = 8;
    pointcloudmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[2].count = 1;

    pointcloudmsg.point_step = 12; // Size of a single point in bytes (3 floats * 4 bytes/float)
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
    pointcloudmsg.is_bigendian = false;
    pointcloudmsg.data.resize(pointcloudmsg.point_step*count);

    for (size_t i = 0; i < count; i++)
    {
        float x = points[indexes[i]]->GetWorldPos()(2);
        float y = -1.0 * points[indexes[i]]->GetWorldPos()(0);
        float z = -1.0 * points[indexes[i]]->GetWorldPos()(1);

        memcpy(&pointcloudmsg.data[i*12], &x, 4);
        memcpy(&pointcloudmsg.data[i*12 + 4], &y, 4);
        memcpy(&pointcloudmsg.data[i*12 + 8], &z, 4);
    }
    pclpublisher->publish(pointcloudmsg);
}
void SlamNode::PublishPath(){
    std::vector<ORB_SLAM3::KeyFrame*> trajectory = m_SLAM->GetTrajectory();
    auto path_msg = nav_msgs::msg::Path();

    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = this->get_parameter("frame_id").as_string();;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        Sophus::SE3f SE3 =  trajectory[i]->GetPose();
        pose.header.stamp = current_frame_time_;
        pose.header.frame_id = this->get_parameter("frame_id").as_string();;

        // Transform to ROS coordinates
        tf2::Transform grasp_tf = TransformFromSophus(SE3);
        tf2::toMsg(grasp_tf, pose.pose);

        path_msg.poses.push_back(pose);

    }
    pathpublisher->publish(path_msg);
    
}
void SlamNode::PublishPose() {
    tf2::Transform grasp_tf = TransformFromSophus(SE3);
    auto pose_msg = geometry_msgs::msg::PoseStamped();

    pose_msg.header.stamp = current_frame_time_;
    pose_msg.header.frame_id = this->get_parameter("frame_id").as_string();;
    tf2::toMsg(grasp_tf, pose_msg.pose);

    posepublisher->publish(pose_msg);

}

void SlamNode::PublishTransform(){
    auto sendmsg = geometry_msgs::msg::TransformStamped();
    tf2::Transform grasp_tf = TransformFromSophus(SE3);

    sendmsg.header.stamp = current_frame_time_;
    sendmsg.header.frame_id = this->get_parameter("frame_id").as_string();;
    sendmsg.child_frame_id = this->get_parameter("child_frame_id").as_string();;
    tf2::toMsg(grasp_tf, sendmsg.transform);

    tf_publisher->publish(sendmsg);
}

tf2::Transform SlamNode::TransformFromSophus(Sophus::SE3f &se3)
{
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  Eigen::Matrix4f transformation_matrix = se3.matrix();
  tf2::Matrix3x3 tf_camera_rotation (transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                    transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                    transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2)
                                   );

  tf2::Vector3 tf_camera_translation (se3.translation().x(), se3.translation().y(), se3.translation().z());

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}
