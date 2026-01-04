#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "futuraps_perception/srv/get_closest_grid.hpp"
#include "futuraps_perception/closest_grid.hpp"

using PointT = pcl::PointXYZ;
using GetSrv = futuraps_perception::srv::GetClosestGrid;

class ClosestGridServer : public rclcpp::Node {
public:
  ClosestGridServer() : Node("closest_grid_server"),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/plants_only");
    base_link_   = declare_parameter<std::string>("base_link", "base_link");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ClosestGridServer::onCloud, this, std::placeholders::_1));

    srv_ = create_service<GetSrv>(
      "get_closest_grid",
      std::bind(&ClosestGridServer::onReq, this,
                std::placeholders::_1, std::placeholders::_2));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { last_cloud_ = msg; }

  void onReq(const std::shared_ptr<GetSrv::Request> req,
             std::shared_ptr<GetSrv::Response> res) {
    if (!last_cloud_) { res->found.assign(req->rows*req->cols,false); return; }

    // Ensure cloud in base_link
    sensor_msgs::msg::PointCloud2 cloud_bl;
    if (last_cloud_->header.frame_id == base_link_) {
      cloud_bl = *last_cloud_;
    } else {
      try {
        auto tf = tf_buffer_.lookupTransform(base_link_, last_cloud_->header.frame_id, tf2::TimePointZero);
        Eigen::Isometry3d T = tf2::transformToEigen(tf);
        pcl::PointCloud<PointT> pc_in; pcl::fromROSMsg(*last_cloud_, pc_in);
        pcl::PointCloud<PointT> pc_out; pc_out.reserve(pc_in.size());
        for (const auto& p : pc_in.points) {
          Eigen::Vector3d v(p.x,p.y,p.z), w = T * v;
          pc_out.emplace_back(PointT{float(w.x()), float(w.y()), float(w.z())});
        }
        pcl::toROSMsg(pc_out, cloud_bl);
        cloud_bl.header.frame_id = base_link_;
        cloud_bl.header.stamp = last_cloud_->header.stamp;
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "TF to base_link unavailable: %s", e.what());
        res->found.assign(req->rows*req->cols,false); return;
      }
    }

    futuraps::GridReq Rq;
    Rq.s = req->cell_size;
    Rq.rows = req->rows; Rq.cols = req->cols;
    Rq.x0 = req->x0; Rq.z0 = req->z0;
    Rq.yL = req->y_left_max; Rq.yR = req->y_right_max;
    Rq.side = req->side;
    Rq.p_front = req->front_percentile;
    Rq.min_pts = req->min_points_per_cell;

    pcl::PointCloud<PointT> pc; pcl::fromROSMsg(cloud_bl, pc);
    auto out = futuraps::computeClosestGrid(pc, Rq);

    const size_t N = out.size();
    res->found.resize(N); res->x.resize(N); res->y.resize(N); res->z.resize(N);
    res->dist.resize(N); res->count.resize(N); res->confidence.resize(N);
    for (size_t k=0; k<N; ++k) {
      res->found[k] = out[k].found;
      res->x[k] = out[k].x; res->y[k] = out[k].y; res->z[k] = out[k].z;
      res->dist[k] = out[k].dist; res->count[k] = out[k].count; res->confidence[k] = out[k].confidence;
    }
  }

  // params & members
  std::string cloud_topic_, base_link_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<GetSrv>::SharedPtr srv_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestGridServer>());
  rclcpp::shutdown();
  return 0;
}
