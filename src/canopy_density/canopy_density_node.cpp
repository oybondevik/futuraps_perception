#include <deque>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <futuraps_perception/msg/canopy_density_grid.hpp>
#include <futuraps_perception/srv/get_canopy_density_grid.hpp>

#include <futuraps_perception/grid_spec.hpp>

using futuraps_perception::msg::CanopyDensityGrid;
using futuraps_perception::srv::GetCanopyDensityGrid;
using futuraps::GridSpec;

namespace {

inline bool nearly_equal(double a, double b, double eps=1e-6) {
  return std::abs(a-b) < eps;
}

struct PrevSignature {
  double min_x{}, max_x{}, min_z{}, max_z{}, cell_size{};
  bool operator==(const PrevSignature& o) const {
    return nearly_equal(min_x,o.min_x) && nearly_equal(max_x,o.max_x) &&
           nearly_equal(min_z,o.min_z) && nearly_equal(max_z,o.max_z) &&
           nearly_equal(cell_size,o.cell_size);
  }
};

} // namespace

class CanopyDensityNode : public rclcpp::Node {
public:
  CanopyDensityNode()
  : rclcpp::Node("canopy_density_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    // ---- Parameters (I/O) ----
    cloud_topic_     = declare_parameter<std::string>("cloud_topic", "/local_map/points");
    default_frame_   = declare_parameter<std::string>("default_frame", "base_link");
    publish_default_ = declare_parameter<bool>("publish_default", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    max_buffer_sec_  = declare_parameter<double>("max_buffer_sec", 2.0);

    // ---- Mode selection: "column" (default) or "ortho" ----
    mode_ = declare_parameter<std::string>("mode", "column");

    // ---- Default request used by periodic publisher (kept for both modes) ----
    default_req_.min_x   = declare_parameter<double>("min_x", -3.0);
    default_req_.max_x   = declare_parameter<double>("max_x",  2.0);
    default_req_.min_z   = declare_parameter<double>("min_z", -0.2);
    default_req_.max_z   = declare_parameter<double>("max_z",  2.0);
    default_req_.cell_size = declare_parameter<double>("cell_size", 0.20);
    default_req_.min_y   = declare_parameter<double>("min_y", -2.0);
    default_req_.max_y   = declare_parameter<double>("max_y",  2.0);
    default_req_.vy      = declare_parameter<double>("vy", 0.10);
    default_req_.k_min_points = static_cast<uint16_t>(declare_parameter<int>("k_min_points", 3));
    default_req_.y_min_canopy = 0.0;               // unused
    default_req_.use_column_density = true;        // ignored
    default_req_.apply_ema = declare_parameter<bool>("apply_ema", true);
    default_req_.alpha     = declare_parameter<double>("alpha", 0.2);
    default_req_.frame_id  = declare_parameter<std::string>("frame_id", default_frame_);
    default_req_.time_window = declare_parameter<double>("time_window", 0.75);

    // ---- Ortho mode parameter ----
    ortho_pixel_size_ = declare_parameter<double>("ortho_pixel_size", 0.01); // meters/pixel on x–z plane

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CanopyDensityNode::cloud_cb, this, std::placeholders::_1));

    pub_grid_ = create_publisher<CanopyDensityGrid>("/canopy_density/grid", rclcpp::QoS(1));

    srv_ = create_service<GetCanopyDensityGrid>(
      "/canopy_density/get_grid",
      std::bind(&CanopyDensityNode::srv_cb, this, std::placeholders::_1, std::placeholders::_2));

    if (publish_default_) {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_)),
        std::bind(&CanopyDensityNode::on_timer, this));
    }
  }

private:
  // ----------------- Subscriptions & Buffer -----------------
  struct TimedCloud {
    rclcpp::Time stamp;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  };
  void cloud_cb(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    buffer_.push_back({msg->header.stamp, msg});
    prune_buffer(now());
  }
  void prune_buffer(const rclcpp::Time& tnow) {
    while (!buffer_.empty()) {
      if ((tnow - buffer_.front().stamp).seconds() > max_buffer_sec_) buffer_.pop_front();
      else break;
    }
  }

  // ----------------- Service -----------------
  void srv_cb(
    const std::shared_ptr<GetCanopyDensityGrid::Request> req,
    std::shared_ptr<GetCanopyDensityGrid::Response> res)
  {
    if (mode_ == "ortho") {
      res->grid = compute_grid_ortho(*req);
    } else {
      if (!req->use_column_density) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "use_column_density=false requested, but this node runs column mode in 'column'. Proceeding with column mode.");
      }
      res->grid = compute_grid_column(*req);
    }
  }

  // ----------------- Periodic publish -----------------
  void on_timer() {
    auto grid = (mode_ == "ortho")
                  ? compute_grid_ortho(default_req_)
                  : compute_grid_column(default_req_);
    pub_grid_->publish(grid);
  }

  // ----------------- COLUMN MODE (original) -----------------
  CanopyDensityGrid compute_grid_column(const GetCanopyDensityGrid::Request& req) {
    CanopyDensityGrid out;
    out.header.frame_id = req.frame_id.empty() ? default_frame_ : req.frame_id;
    out.header.stamp = now();

    // 1) Configure grid
    GridSpec g{};
    g.min_x = req.min_x; g.max_x = req.max_x;
    g.min_z = req.min_z; g.max_z = req.max_z;
    g.cell_size = req.cell_size;
    g.min_y = req.min_y; g.max_y = req.max_y;
    g.origin_x = g.min_x;  // cell (0,0) anchored at min bounds
    g.origin_z = g.min_z;

    const double span_x = std::max(0.0, g.max_x - g.min_x);
    const double span_z = std::max(0.0, g.max_z - g.min_z);
    g.width  = static_cast<unsigned>(std::ceil(span_x / g.cell_size));
    g.height = static_cast<unsigned>(std::ceil(span_z / g.cell_size));

    out.cell_size = static_cast<float>(g.cell_size);
    out.origin_x  = static_cast<float>(g.origin_x);
    out.origin_z  = static_cast<float>(g.origin_z);
    out.min_y     = static_cast<float>(g.min_y);
    out.max_y     = static_cast<float>(g.max_y);
    out.width     = g.width;
    out.height    = g.height;

    const size_t Ncells = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);
    out.data.assign(Ncells, std::numeric_limits<float>::quiet_NaN());
    out.point_counts.assign(Ncells, 0);
    out.occupied_voxels.assign(Ncells, 0);

    // Empty grid edge case
    if (g.width == 0 || g.height == 0) {
      out.num_vertical_voxels = 0;
      return out;
    }

    // 2) Vertical bins
    const double vy = std::max(1e-3, static_cast<double>(req.vy));
    const uint16_t k_min_points = req.k_min_points;

    const uint16_t Ny = static_cast<uint16_t>(std::max(0, (int)std::ceil((g.max_y - g.min_y) / vy)));
    out.num_vertical_voxels = Ny;

    std::vector<uint16_t> col_counts; // (i,j,k) counts
    col_counts.assign(static_cast<size_t>(Ncells) * std::max<uint16_t>(Ny, 1), 0);

    // 3) Gather clouds within time window
    const double time_window = std::max(0.0, static_cast<double>(req.time_window));
    const rclcpp::Time tnow = now();
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> selected;
    if (time_window <= 0.0) {
      if (!buffer_.empty()) selected.push_back(buffer_.back().cloud);
    } else {
      for (auto it = buffer_.rbegin(); it != buffer_.rend(); ++it) {
        if ((tnow - it->stamp).seconds() <= time_window) selected.push_back(it->cloud);
        else break;
      }
    }
    if (selected.empty()) return maybe_apply_ema(out, g, req);

    // 4) Transform & accumulate
    for (auto& cptr : selected) {
      sensor_msgs::msg::PointCloud2 cloud_tf;
      try {
        if (cptr->header.frame_id == out.header.frame_id) {
          cloud_tf = *cptr;
        } else {
          geometry_msgs::msg::TransformStamped T =
            tf_buffer_->lookupTransform(out.header.frame_id, cptr->header.frame_id, tf2::TimePointZero);
          tf2::doTransform(*cptr, cloud_tf, T);
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF transform failed: %s", e.what());
        continue;
      }

      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(cloud_tf, pcl_cloud);

      for (const auto& p : pcl_cloud.points) {
        const double x = p.x, y = p.y, z = p.z;
        if (!futuraps::in_bounds(g, x, z)) continue;
        if (y < g.min_y || y >= g.max_y) continue;

        auto ij = futuraps::to_index(g, x, z);
        const int i = ij.first, j = ij.second;
        if (i < 0 || j < 0 || i >= static_cast<int>(g.width) || j >= static_cast<int>(g.height)) continue;
        const size_t idx = futuraps::flat_index(g, i, j);

        out.point_counts[idx]++;

        if (Ny == 0) continue; // guard
        int k = static_cast<int>(std::floor((y - g.min_y) / vy));
        if (k < 0) k = 0;
        if (k >= Ny) k = Ny - 1;
        col_counts[idx * Ny + static_cast<size_t>(k)]++;
      }
    }

    // 5) Finalize per-cell (density = occupied_voxels / Ny)
    for (size_t idx = 0; idx < Ncells; ++idx) {
      if (Ny == 0) { out.data[idx] = std::numeric_limits<float>::quiet_NaN(); continue; }
      uint16_t occ = 0;
      const size_t base = idx * Ny;
      for (uint16_t k = 0; k < Ny; ++k) {
        if (col_counts[base + k] >= k_min_points) occ++;
      }
      out.occupied_voxels[idx] = occ;
      const float d = (Ny > 0) ? static_cast<float>(occ) / static_cast<float>(Ny)
                               : std::numeric_limits<float>::quiet_NaN();
      out.data[idx] = d;
    }

    return maybe_apply_ema(out, g, req);
  }

  // ----------------- ORTHOGRAPHIC (x–z plane) MODE -----------------
  CanopyDensityGrid compute_grid_ortho(const GetCanopyDensityGrid::Request& req) {
    CanopyDensityGrid out;
    out.header.frame_id = req.frame_id.empty() ? default_frame_ : req.frame_id;
    out.header.stamp = now();

    // Grid on x–z in out.header.frame_id
    GridSpec g{};
    g.min_x = req.min_x; g.max_x = req.max_x;
    g.min_z = req.min_z; g.max_z = req.max_z;
    g.cell_size = req.cell_size;
    g.min_y = req.min_y; g.max_y = req.max_y;
    g.origin_x = g.min_x; g.origin_z = g.min_z;

    const double span_x = std::max(0.0, g.max_x - g.min_x);
    const double span_z = std::max(0.0, g.max_z - g.min_z);
    g.width  = static_cast<unsigned>(std::ceil(span_x / g.cell_size));
    g.height = static_cast<unsigned>(std::ceil(span_z / g.cell_size));

    out.cell_size = static_cast<float>(g.cell_size);
    out.origin_x  = static_cast<float>(g.origin_x);
    out.origin_z  = static_cast<float>(g.origin_z);
    out.min_y     = static_cast<float>(g.min_y);
    out.max_y     = static_cast<float>(g.max_y);
    out.width     = g.width;
    out.height    = g.height;

    const size_t Ncells = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);
    out.data.assign(Ncells, std::numeric_limits<float>::quiet_NaN());
    out.point_counts.assign(Ncells, 0);
    out.occupied_voxels.assign(Ncells, 0);
    out.num_vertical_voxels = 0;

    if (g.width == 0 || g.height == 0) return out;

    // Recent cloud
    const double time_window = std::max(0.0, static_cast<double>(req.time_window));
    const rclcpp::Time tnow = now();
    sensor_msgs::msg::PointCloud2::SharedPtr latest{};
    if (time_window <= 0.0) {
      if (!buffer_.empty()) latest = buffer_.back().cloud;
    } else {
      for (auto it = buffer_.rbegin(); it != buffer_.rend(); ++it) {
        if ((tnow - it->stamp).seconds() <= time_window) { latest = it->cloud; break; }
      }
    }
    if (!latest) return maybe_apply_ema(out, g, req);

    // Transform to plane frame (usually base_link)
    sensor_msgs::msg::PointCloud2 cloud_plane;
    try {
      if (latest->header.frame_id == out.header.frame_id) {
        cloud_plane = *latest;
      } else {
        geometry_msgs::msg::TransformStamped T =
          tf_buffer_->lookupTransform(out.header.frame_id, latest->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*latest, cloud_plane, T);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF to ortho plane failed: %s", e.what());
      return maybe_apply_ema(out, g, req);
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_pts;
    pcl::fromROSMsg(cloud_plane, pcl_pts);

    // Per-cell raster on x–z plane
    const double pix = std::max(1e-6, ortho_pixel_size_);
    const int px_per_cell = std::max(1, (int)std::ceil(g.cell_size / pix));
    const int denom_pixels = px_per_cell * px_per_cell;

    std::vector<std::unordered_set<int>> cell_pixels(Ncells);
    for (auto& s : cell_pixels) s.reserve(128);

    for (const auto& p : pcl_pts.points) {
      const double x = p.x, y = p.y, z = p.z;

      // Optional vertical band
      if (y < g.min_y || y >= g.max_y) continue;

      if (!futuraps::in_bounds(g, x, z)) continue;

      auto ij = futuraps::to_index(g, x, z);
      const int i = ij.first, j = ij.second;
      if (i < 0 || j < 0 || i >= (int)g.width || j >= (int)g.height) continue;

      const size_t cidx = futuraps::flat_index(g, i, j);
      out.point_counts[cidx]++;

      // Local coords inside cell (drop Y -> orthographic)
      const double x0 = g.origin_x + i * g.cell_size;
      const double z0 = g.origin_z + j * g.cell_size;
      const double lx = x - x0;
      const double lz = z - z0;

      int u = (int)std::floor(lx / pix);
      int v = (int)std::floor(lz / pix);
      u = std::clamp(u, 0, px_per_cell - 1);
      v = std::clamp(v, 0, px_per_cell - 1);

      cell_pixels[cidx].insert(v * px_per_cell + u);
    }

    // Finalize density = unique_pixels / total_pixels (per cell)
    for (size_t cidx = 0; cidx < Ncells; ++cidx) {
      if (denom_pixels > 0) {
        const int occ = static_cast<int>(cell_pixels[cidx].size());
        out.data[cidx] = static_cast<float>(
          std::clamp((double)occ / (double)denom_pixels, 0.0, 1.0));
      } else {
        out.data[cidx] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    return maybe_apply_ema(out, g, req);
  }

  // ----------------- EMA (shared) -----------------
  CanopyDensityGrid maybe_apply_ema(CanopyDensityGrid& out, const GridSpec& g, const GetCanopyDensityGrid::Request& req) {
    // Reset EMA state if grid geometry changed
    PrevSignature sig{g.min_x, g.max_x, g.min_z, g.max_z, g.cell_size};
    if (!(sig == prev_sig_) || prev_data_.size() != out.data.size()) {
      prev_data_.assign(out.data.size(), std::numeric_limits<float>::quiet_NaN());
      prev_sig_ = sig;
    }

    if (!req.apply_ema) return out;

    const float alpha = std::clamp(static_cast<float>(req.alpha), 0.0f, 1.0f);
    for (size_t idx = 0; idx < out.data.size(); ++idx) {
      const float cur = out.data[idx];
      const float prev = prev_data_[idx];
      float fused;
      if (std::isnan(prev)) fused = cur;
      else if (std::isnan(cur)) fused = prev; // keep previous if no new info
      else fused = alpha * cur + (1.0f - alpha) * prev;
      out.data[idx] = fused;
    }
    prev_data_ = out.data; // persist for next call
    return out;
  }

  // ----------------- Members -----------------
  std::string cloud_topic_, default_frame_;
  bool   publish_default_{true};
  double publish_rate_hz_{2.0};
  double max_buffer_sec_{2.0};

  // Mode + ortho param
  std::string mode_;
  double ortho_pixel_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<CanopyDensityGrid>::SharedPtr               pub_grid_;
  rclcpp::Service<GetCanopyDensityGrid>::SharedPtr              srv_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  // Buffer
  std::deque<TimedCloud> buffer_;

  // TF
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // EMA
  std::vector<float> prev_data_;
  PrevSignature      prev_sig_;

  // Default req used by timer
  GetCanopyDensityGrid::Request default_req_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanopyDensityNode>());
  rclcpp::shutdown();
  return 0;
}
