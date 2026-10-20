#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/logging.hpp>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>// Store the latest laser scans into laserMsg
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <chrono>
#include "ros2_livox/livox_points_plugin.h"
#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_ode_multiray_shape.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace gazebo
{

    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    LivoxPointsPlugin::LivoxPointsPlugin() {}

    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; //转化成标准的右手系角度
                // Precompute direction unit vector (avoids trig in hot path)
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, avia_infos.back().zenith, avia_infos.back().azimuth));
                avia_infos.back().direction = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            } else {
            RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
        }
        }
    }

    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        node_ = gazebo_ros::Node::Get(sdf);
        
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {   
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");

        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        if (sdf->HasElement("namespace"))
        {
            ns_ = sdf->Get<std::string>("namespace");
        }
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "namespace: %s", ns_.c_str());

        child_name = raySensor->Name();
        parent_name = raySensor->ParentName();
        size_t delimiter_pos = parent_name.find("::");
        parent_name = parent_name.substr(delimiter_pos + 2);

        rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
        sensor_qos.best_effort().durability_volatile();

        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        // PointCloud2 publisher
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + "_PointCloud2", sensor_qos);
        // CustomMsg publisher
        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, sensor_qos);

        scanPub = node->Advertise<msgs::LaserScanStamped>(curr_scan_topic+"laserscan", 50);

        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());
        parentEntity = this->world->EntityByName(_parent->ParentName());
        //SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1)
        {
            downSample = 1;
        }
        if (sdfPtr->HasElement("accumulate_scans"))
        {
            accumulateScans = sdfPtr->Get<int>("accumulate_scans");
        }
        if (accumulateScans < 1)
        {
            accumulateScans = 1;
        }
        if (sdfPtr->HasElement("num_phases"))
        {
            numPhases = sdfPtr->Get<int>("num_phases");
        }
        if (numPhases < 1)
        {
            numPhases = 1;
        }
        // Initialize phase offsets evenly spaced across the scan pattern
        phaseOffsets.resize(numPhases);
        for (int64_t i = 0; i < numPhases; i++)
        {
            phaseOffsets[i] = i * maxPointSize / numPhases;
        }
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "downsample: %ld", downSample);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "accumulate_scans: %ld", accumulateScans);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "num_phases: %ld", numPhases);
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample)
        {
            int index = j % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            auto axis = offset.Rot() * rotate_info.direction;
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }

    }

    void LivoxPointsPlugin::OnNewLaserScans()
    {
        // Check if rayShape has been initialized
        if (rayShape)
        {
            std::vector<std::pair<int, AviaRotateInfo>> points_pair;
            // Initialize ray scan point pairs
            InitializeRays(points_pair, rayShape);
            rayShape->Update();

            // The sim time this scan was cast at; it stamps every message below.
            // Not the node clock: the plugin node is forced onto use_sim_time and
            // /clock is published at 10 Hz by default, so now() lags by up to 0.1 s
            // and repeats across scans.
            const common::Time scanTime = world->SimTime();

            // Create laser scan message and set the timestamp
            msgs::Set(laserMsg.mutable_time(), scanTime);
            msgs::LaserScan *scan = laserMsg.mutable_scan();
            InitializeScan(scan);

            // Collect current scan's points into the accumulation buffer
            std::vector<std::pair<ignition::math::Vector3d, float>> currentScanPoints;
            currentScanPoints.reserve(points_pair.size());

            for (auto &pair : points_pair)
            {
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);

                // Handle out-of-range data
                if (range >= maxDist)
                {
                    range = maxDist;
                }
                else if (range <= RangeMin())
                {
                    range = RangeMin();
                }

                // Calculate point cloud data using precomputed direction
                auto point = range * pair.second.direction;
                currentScanPoints.emplace_back(point, static_cast<float>(intensity));
            }

            // Push current scan and evict oldest if buffer is full
            pointHistory.push_back({scanTime, std::move(currentScanPoints)});
            while (static_cast<int64_t>(pointHistory.size()) > accumulateScans)
            {
                pointHistory.pop_front();
            }

            // Build messages from ALL accumulated scans. As the Livox driver does,
            // the header stamp and timebase are the time of the oldest point and
            // offset_time is each point's time after it: Gazebo casts a whole scan
            // at one instant, so offsets step per accumulated scan and are all 0
            // with accumulate_scans 1.
            const common::Time baseTime = pointHistory.front().time;
            std::string frame_id = ns_ + "/" + raySensor->Name();

            livox_ros_driver2::msg::CustomMsg pp_livox;
            pp_livox.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(baseTime);
            pp_livox.header.frame_id = frame_id;
            pp_livox.timebase = rclcpp::Time(pp_livox.header.stamp).nanoseconds();

            sensor_msgs::msg::PointCloud cloud;
            cloud.header = pp_livox.header;
            cloud.channels.resize(1);
            cloud.channels[0].name = "intensity";

            // Reserve total capacity across all accumulated scans
            size_t totalPoints = 0;
            for (auto &scan : pointHistory)
            {
                totalPoints += scan.points.size();
            }
            pp_livox.points.reserve(totalPoints);
            cloud.points.reserve(totalPoints);
            cloud.channels[0].values.reserve(totalPoints);

            int count = 0;

            for (auto &scan : pointHistory)
            {
                const common::Time offset = scan.time - baseTime;
                const uint32_t offset_ns = static_cast<uint32_t>(offset.sec * 1000000000LL + offset.nsec);
                for (auto &[point, intensity] : scan.points)
                {
                    livox_ros_driver2::msg::CustomPoint p;
                    p.x = point.X();
                    p.y = point.Y();
                    p.z = point.Z();
                    p.reflectivity = intensity;
                    p.offset_time = offset_ns;

                    pp_livox.points.push_back(p);

                    cloud.points.emplace_back();
                    cloud.points.back().x = point.X();
                    cloud.points.back().y = point.Y();
                    cloud.points.back().z = point.Z();
                    cloud.channels[0].values.push_back(intensity);

                    count++;
                }
            }

            if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);

            // Set the number of point cloud data and publish the CustomMsg message
            pp_livox.point_num = count;
            custom_pub->publish(pp_livox);

            // Publish PointCloud2 type message (x, y, z, intensity), same stamp
            sensor_msgs::msg::PointCloud2 cloud2;
            sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
            cloud2_pub->publish(cloud2);
        }
    }

    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                           boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        ignition::math::Vector3d start_point, end_point;
        auto offset = laserCollision->RelativePose();
        long unsigned int ray_index = 0;
        auto ray_size = rays.size();
        points_pair.reserve(rays.size());

        int64_t samplesPerPhase = samplesStep / numPhases;

        for (int64_t phase = 0; phase < numPhases; phase++)
        {
            int64_t phaseEnd = phaseOffsets[phase] + samplesPerPhase;
            for (int64_t k = phaseOffsets[phase]; k < phaseEnd; k += downSample)
            {
                auto index = k % maxPointSize;
                auto &rotate_info = aviaInfos[index];
                auto axis = offset.Rot() * rotate_info.direction;
                start_point = minDist * axis + offset.Pos();
                end_point = maxDist * axis + offset.Pos();
                if (ray_index < ray_size)
                {
                    rays[ray_index]->SetPoints(start_point, end_point);
                    points_pair.emplace_back(ray_index, rotate_info);
                }
                ray_index++;
            }
            phaseOffsets[phase] += samplesPerPhase;
        }
    }

    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    double LivoxPointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    double LivoxPointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double LivoxPointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    int LivoxPointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    int LivoxPointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int LivoxPointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    int LivoxPointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    double LivoxPointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }


}
