#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>

using namespace std::chrono_literals;

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

class ObjectDetector : public rclcpp::Node {
public:
    ObjectDetector() : Node("object_detector") {
        // 参数声明
        resolution_ = this->declare_parameter("resolution", 0.1);
        cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.3);
        min_cluster_size_ = this->declare_parameter("min_cluster_size", 5);
        threshold_ = this->declare_parameter("threshold", 5);

        // 订阅点云话题
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", 10,
            std::bind(&ObjectDetector::cloud_callback, this, std::placeholders::_1));

        // 发布标记
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/removed_objects", 10);

        // 保存背景服务
        save_background_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_background",
            std::bind(&ObjectDetector::save_background_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Object detector initialized");
    }

private:
    void save_background_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_cloud_ && !latest_cloud_->empty()) {
            background_cloud_ = latest_cloud_;
            response->success = true;
            response->message = "Background saved successfully";
            RCLCPP_INFO(this->get_logger(), "Background saved");
        } else {
            response->success = false;
            response->message = "No point cloud available";
            RCLCPP_WARN(this->get_logger(), "Failed to save background");
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *current_cloud);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_cloud_ = current_cloud;
        }

        if (!background_cloud_ || background_cloud_->empty()) return;

        // 处理点云差异
        auto markers = process_clouds(background_cloud_, current_cloud);
        marker_pub_->publish(markers);
    }

    visualization_msgs::msg::MarkerArray process_clouds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& background,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& current) {
        
        visualization_msgs::msg::MarkerArray markers;
        std::unordered_map<std::pair<int, int>, int, pair_hash> bg_grid, curr_grid;

        // 填充网格数据
        fill_grid(background, bg_grid);
        fill_grid(current, curr_grid);

        // 检测差异区域
        pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& [cell, bg_count] : bg_grid) {
            int curr_count = curr_grid[cell];
            if (bg_count - curr_count >= threshold_) {
                float x = (cell.first + 0.5f) * resolution_;
                float y = (cell.second + 0.5f) * resolution_;
                diff_cloud->push_back(pcl::PointXYZ(x, y, 0));
            }
        }

        // 下采样
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(diff_cloud);
        vg.setLeafSize(resolution_, resolution_, resolution_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*filtered_cloud);

        // 聚类检测
        std::vector<pcl::PointIndices> cluster_indices;
        if (!filtered_cloud->empty()) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(filtered_cloud);

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cluster_tolerance_);
            ec.setMinClusterSize(min_cluster_size_);
            ec.setSearchMethod(tree);
            ec.setInputCloud(filtered_cloud);
            ec.extract(cluster_indices);
        }

        // 创建标记
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);

        int id = 0;
        for (const auto& indices : cluster_indices) {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*filtered_cloud, indices, centroid);

            // 球体标记
            visualization_msgs::msg::Marker sphere;
            // sphere.header.frame_id = msg->header.frame_id;
            sphere.header.frame_id = current->header.frame_id;
            sphere.header.stamp = this->now();
            sphere.ns = "objects";
            sphere.id = id++;
            sphere.type = visualization_msgs::msg::Marker::SPHERE;
            sphere.action = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x = centroid[0];
            sphere.pose.position.y = centroid[1];
            sphere.pose.position.z = 0;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3;
            sphere.color.r = 1.0;
            sphere.color.a = 1.0;
            markers.markers.push_back(sphere);

            // 文字标记
            visualization_msgs::msg::Marker text;
            text.header = sphere.header;
            text.ns = "labels";
            text.id = id++;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.text = "(" + std::to_string(centroid[0]) + ", " + std::to_string(centroid[1]) + ")";
            text.pose.position.x = centroid[0];
            text.pose.position.y = centroid[1];
            text.pose.position.z = 0.5;
            text.scale.z = 0.2;
            text.color.r = text.color.g = text.color.b = 1.0;
            text.color.a = 1.0;
            markers.markers.push_back(text);
        }

        return markers;
    }

    void fill_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                   std::unordered_map<std::pair<int, int>, int, pair_hash>& grid) {
        grid.clear();
        for (const auto& point : *cloud) {
            int x = static_cast<int>(point.x / resolution_);
            int y = static_cast<int>(point.y / resolution_);
            grid[{x, y}]++;
        }
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_background_service_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    std::mutex mutex_;

    float resolution_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetector>());
    rclcpp::shutdown();
    return 0;
}
