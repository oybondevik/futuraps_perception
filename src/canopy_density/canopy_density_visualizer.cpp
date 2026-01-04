#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <futuraps_perception/msg/canopy_density_grid.hpp>
#include <cmath>

class CanopyDensityViz : public rclcpp::Node {
public:
  CanopyDensityViz() : rclcpp::Node("canopy_density_visualizer") {
    sub_ = create_subscription<futuraps_perception::msg::CanopyDensityGrid>(
      "/canopy_density/grid", rclcpp::QoS(1),
      std::bind(&CanopyDensityViz::cb, this, std::placeholders::_1));
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/canopy_density/markers", rclcpp::QoS(1));
    alpha_ = declare_parameter<double>("alpha", 0.85);
    height_offset_ = declare_parameter<double>("height_offset", 0.01);
    v_min_  = declare_parameter<double>("v_min", 0.0);     // clip low
    v_max_  = declare_parameter<double>("v_max", 1.0);     // clip high
    gain_   = declare_parameter<double>("gain", 1.5);      // >1 = more punch
    bias_   = declare_parameter<double>("bias", 0.0);      // shift before scaling
    gamma_  = declare_parameter<double>("gamma", 0.7);     // <1 brightens
    auto_stretch_ = declare_parameter<bool>("auto_stretch", false); // optional
  }
private:
  void cb(const futuraps_perception::msg::CanopyDensityGrid::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    
    float a_min = 1.0f, a_max = 0.0f;
    if (auto_stretch_) {
    for (uint32_t i = 0; i < msg->width; ++i) {
        for (uint32_t j = 0; j < msg->height; ++j) {
        const size_t idx = static_cast<size_t>(i) * msg->height + j;
        float d = msg->data[idx];
        if (std::isnan(d)) continue;
        a_min = std::min(a_min, d);
        a_max = std::max(a_max, d);
        }
    }
    if (a_max <= a_min) { a_min = 0.0f; a_max = 1.0f; }
    }
    const float clip_min = auto_stretch_ ? a_min : static_cast<float>(v_min_);
    const float clip_max = auto_stretch_ ? a_max : static_cast<float>(v_max_);
    const float clip_span = std::max(1e-6f, clip_max - clip_min);
    
    for (uint32_t i = 0; i < msg->width; ++i) {
      for (uint32_t j = 0; j < msg->height; ++j) {
        const size_t idx = static_cast<size_t>(i) * msg->height + j;
        float d = msg->data[idx];
        if (std::isnan(d)) continue;

        visualization_msgs::msg::Marker m;
        m.header = msg->header;
        m.header.stamp = rclcpp::Time(0);
        m.frame_locked = true;
        m.ns = "canopy_density";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;

        const double x = msg->origin_x + (i + 0.5) * msg->cell_size;
        const double z = msg->origin_z + (j + 0.5) * msg->cell_size;

        m.pose.position.x = x;
        m.pose.position.y = height_offset_;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;

        m.scale.x = msg->cell_size;
        m.scale.y = 0.005;
        m.scale.z = msg->cell_size;

        // normalize + boost
        float v = (d - clip_min) / clip_span;     // 0..1
        v = std::clamp(v - static_cast<float>(bias_), 0.0f, 1.0f);
        v = std::pow(v, static_cast<float>(gamma_));
        v = std::clamp(v * static_cast<float>(gain_), 0.0f, 1.0f);

        // map to color (blue->red)
        m.color.a = alpha_;
        m.color.r = v;
        m.color.g = 0.0f;
        m.color.b = 1.0f - v;

        arr.markers.push_back(std::move(m));
      }
    }
    pub_->publish(arr);
  }

  rclcpp::Subscription<futuraps_perception::msg::CanopyDensityGrid>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  double alpha_{0.85};
  double height_offset_{0.01};
  double v_min_{0.0}, v_max_{1.0}, gain_{1.5}, bias_{0.0}, gamma_{0.7};
  bool auto_stretch_{false};

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanopyDensityViz>());
  rclcpp::shutdown();
  return 0;
}
