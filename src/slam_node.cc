#include "slam_node.hpp"

SlamNode::SlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, const std::string &strSettingsFile, const std::string &strDoRectify)
: Node("ORB_SLAM3"), m_SLAM(pSLAM), node_(node)
{
    tf_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
    pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    pathpublisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    posepublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
}
SlamNode::~SlamNode() {
    // Para todas as threads
    m_SLAM->Shutdown();
}