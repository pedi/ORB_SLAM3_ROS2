#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImu_ = this->create_subscription<ImuMsg>("/imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    
    tf_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
    pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    Sophus::SE3f SE3;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            SE3 = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            // StereoInertialNode::Transform_orbslam2cam(SE3.translation(), SE3.unit_quaternion());
            // ROS Transform message initialization
            auto sendmsg = geometry_msgs::msg::TransformStamped();

            sendmsg.header.stamp = this->get_clock()->now();
            sendmsg.header.frame_id = "orbslam3";
            sendmsg.child_frame_id = "left_camera_link";

            // Set translation
            sendmsg.transform.translation.x = SE3.params()(4);
            sendmsg.transform.translation.y = SE3.params()(5);
            sendmsg.transform.translation.z = SE3.params()(6);

            // Set rotation
            sendmsg.transform.rotation.x = SE3.params()(0);
            sendmsg.transform.rotation.y = SE3.params()(1);
            sendmsg.transform.rotation.z = SE3.params()(2);
            sendmsg.transform.rotation.w = SE3.params()(3);

            // Publish the transform message
            tf_publisher->publish(sendmsg);
            StereoInertialNode::PublishPointCloud(SLAM_->GetTrackedMapPoints());

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void StereoInertialNode::Transform_orbslam2cam(const Eigen::Vector3f translation, const Eigen::Quaternionf rotation){
    auto sendmsg = geometry_msgs::msg::TransformStamped();

        // ROS Transform message initialization
        sendmsg.header.stamp = this->get_clock()->now();
        sendmsg.header.frame_id = "orbslam3";
        sendmsg.child_frame_id = "left_camera_link";

        // Set the translation
        sendmsg.transform.translation.x = translation.x();
        sendmsg.transform.translation.y = translation.y();
        sendmsg.transform.translation.z = translation.z();

        // Set the rotation (using quaternion)
        sendmsg.transform.rotation.x = rotation.x();
        sendmsg.transform.rotation.y = rotation.y();
        sendmsg.transform.rotation.z = rotation.z();
        sendmsg.transform.rotation.w = rotation.w();

        // Publish the transform message
        tf_publisher->publish(sendmsg);
}

void StereoInertialNode::PublishPointCloud(std::vector<ORB_SLAM3::MapPoint*> points){
    std::vector<int> indexes;
    // std::vector<ORB_SLAM3::MapPoint*> points = m_SLAM->GetTrackedMapPoints();
    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();

    int count = 0;
    
    for (size_t i = 0; i < points.size(); i++)
    {
        if(points[i] != 0){
            count++;
            indexes.push_back(i);

        }
    }
    
    pointcloudmsg.header.stamp = this->get_clock()->now();
    pointcloudmsg.header.frame_id = "orbslam3";
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
        float x = points[indexes[i]]->GetWorldPos()(0);
        float y = points[indexes[i]]->GetWorldPos()(1);
        float z = points[indexes[i]]->GetWorldPos()(2);

        memcpy(&pointcloudmsg.data[i*12], &x, 4);
        memcpy(&pointcloudmsg.data[i*12 + 4], &y, 4);
        memcpy(&pointcloudmsg.data[i*12 + 8], &z, 4);
    }
    pclpublisher->publish(pointcloudmsg);
}
