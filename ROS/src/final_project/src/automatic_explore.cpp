// Mode-2 adapter around explore_lite's BSD frontier search (see third_party).
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <explore/frontier_search.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>

class AutomaticExplore {
  using Point = geometry_msgs::Point;
  using Frontier = frontier_exploration::Frontier;
  struct Failure { Point frontier; unsigned attempts; };
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_, scan_sub_;
  ros::Publisher status_, markers_;
  ros::ServiceClient planner_, clear_;
  ros::Timer timer_;
  tf::TransformListener tf_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
  sensor_msgs::LaserScan::ConstPtr scan_;
  costmap_2d::Costmap2D grid_;
  std::vector<double> bounds_, anchor_;
  std::vector<Failure> failures_;
  std::vector<Point> frontier_points_;
  std::vector<std::pair<int, int>> footprint_;
  Point goal_, frontier_;
  bool active_ = false, cancelling_ = false;
  double radius_, minimum_, timeout_, sensor_timeout_, last_progress_ = 0., best_ = 0.;
  unsigned known_start_ = 0;

  static double distance(const Point& a, const Point& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }
  void report(const std::string& state, size_t frontiers = 0) {
    std::ostringstream out;
    out << "{\"state\":\"" << state << "\",\"frontiers\":" << frontiers
        << ",\"failures\":" << failures_.size() << ",\"frontier_points\":[";
    for (size_t i = 0; i < frontier_points_.size(); ++i) {
      if (i) out << ",";
      out << "[" << frontier_points_[i].x << "," << frontier_points_[i].y << "]";
    }
    out << "]}";
    std_msgs::String msg; msg.data = out.str(); status_.publish(msg);
  }
  bool inside(double x, double y) const {
    const double dx = x - anchor_[0], dy = y - anchor_[1];
    const double c = std::cos(anchor_[2]), s = std::sin(anchor_[2]);
    const double sx = c * dx + s * dy, sy = -s * dx + c * dy;
    return sx >= bounds_[0] && sy >= bounds_[1] && sx <= bounds_[2] && sy <= bounds_[3];
  }
  bool safe(const Point& p) const {
    unsigned x, y;
    if (!grid_.worldToMap(p.x, p.y, x, y)) return false;
    for (const auto& offset : footprint_) {
      const int col = static_cast<int>(x) + offset.first;
      const int row = static_cast<int>(y) + offset.second;
      if (col < 0 || row < 0 || col >= static_cast<int>(grid_.getSizeInCellsX()) ||
          row >= static_cast<int>(grid_.getSizeInCellsY()) ||
          grid_.getCost(col, row) != costmap_2d::FREE_SPACE) return false;
    }
    return true;
  }
  unsigned known() const {
    return std::count_if(map_->data.begin(), map_->data.end(), [](int value) { return value >= 0; });
  }
  bool visible(const Point& start, const Point& end) const {
    unsigned sx, sy, ex, ey;
    if (!grid_.worldToMap(start.x, start.y, sx, sy) ||
        !grid_.worldToMap(end.x, end.y, ex, ey)) return false;
    int x = sx, y = sy, tx = ex, ty = ey;
    const int dx = std::abs(tx - x), dy = -std::abs(ty - y);
    const int step_x = x < tx ? 1 : -1, step_y = y < ty ? 1 : -1;
    int error = dx + dy;
    while (x != tx || y != ty) {
      if (grid_.getCost(x, y) != costmap_2d::FREE_SPACE) return false;
      const int twice = 2 * error;
      if (twice >= dy) { error += dy; x += step_x; }
      if (twice <= dx) { error += dx; y += step_y; }
    }
    return true; // Only the frontier endpoint may be unknown.
  }
  bool observed(const Point& p) const {
    unsigned x, y;
    return grid_.worldToMap(p.x, p.y, x, y) && grid_.getCost(x, y) != costmap_2d::NO_INFORMATION;
  }
  unsigned attempts(const Point& p) const {
    for (const auto& item : failures_) if (distance(item.frontier, p) < .35) return item.attempts;
    return 0;
  }
  bool spansMinimum(const Frontier& frontier) const {
    // Cell count overestimates short curved rims around small unknown patches.
    for (size_t i = 0; i < frontier.points.size(); ++i)
      for (size_t j = i + 1; j < frontier.points.size(); ++j)
        if (distance(frontier.points[i], frontier.points[j]) >= minimum_) return true;
    return false;
  }
  void cancelActive() {
    if (active_ && !nav_.getState().isDone()) {
      nav_.cancelGoal();
      cancelling_ = true;
    }
    active_ = false;
  }
  void failed(const Point& p, const std::string& reason) {
    ROS_WARN("Exploration failure: %s; frontier=(%.3f,%.3f) goal=(%.3f,%.3f) action=%s known=%u start=%u",
             reason.c_str(), p.x, p.y, goal_.x, goal_.y,
             nav_.getState().toString().c_str(), known(), known_start_);
    cancelActive();
    for (auto& item : failures_) {
      if (distance(item.frontier, p) < .35) { ++item.attempts; return; }
    }
    failures_.push_back({p, 1});
    std_srvs::Empty request;
    clear_.call(request);  // One bounded retry per unresolved frontier.
  }
  bool prepareGrid() {
    if (!map_ || !scan_ || map_->header.frame_id != "map" || map_->info.resolution <= 0 ||
        map_->data.size() != static_cast<size_t>(map_->info.width) * map_->info.height ||
        std::abs(tf::getYaw(map_->info.origin.orientation)) > 1e-6) return false;
    const ros::Time now = ros::Time::now();
    if (std::abs((now - scan_->header.stamp).toSec()) > sensor_timeout_ ||
        std::abs((now - map_->header.stamp).toSec()) > 3. ||
        !std::any_of(scan_->ranges.begin(), scan_->ranges.end(), [this](float value) {
          return std::isfinite(value) && value >= scan_->range_min && value <= scan_->range_max;
        })) return false;
    // Scan transport can beat the matching odometry TF by one publication.
    if (!tf_.waitForTransform("map", scan_->header.frame_id, scan_->header.stamp, ros::Duration(.1))) return false;
    grid_.resizeMap(map_->info.width, map_->info.height, map_->info.resolution,
                    map_->info.origin.position.x, map_->info.origin.position.y);
    for (unsigned y = 0; y < map_->info.height; ++y) {
      for (unsigned x = 0; x < map_->info.width; ++x) {
        double wx, wy; grid_.mapToWorld(x, y, wx, wy);
        const int value = map_->data[y * map_->info.width + x];
        const unsigned char cost = !inside(wx, wy) ? costmap_2d::LETHAL_OBSTACLE :
          value < 0 ? costmap_2d::NO_INFORMATION :
          value <= 25 ? costmap_2d::FREE_SPACE : costmap_2d::LETHAL_OBSTACLE;
        grid_.setCost(x, y, cost);
      }
    }
    footprint_.clear();
    const double resolution = grid_.getResolution(), radius = radius_ + resolution / std::sqrt(2.);
    const int n = std::ceil(radius / resolution);
    for (int y = -n; y <= n; ++y) for (int x = -n; x <= n; ++x)
      if (std::hypot(x * resolution, y * resolution) <= radius) footprint_.push_back({x, y});
    failures_.erase(std::remove_if(failures_.begin(), failures_.end(),
                     [this](const Failure& item) { return observed(item.frontier); }), failures_.end());
    return true;
  }
  std::vector<Point> candidates(const Frontier& frontier, const Point& robot) const {
    std::vector<Point> result;
    const size_t stride = std::max<size_t>(1, frontier.points.size() / 20);
    for (size_t i = 0; i < frontier.points.size(); i += stride) {
      for (double radius : {.35, .55, .80}) for (int j = 0; j < 16; ++j) {
        Point p = frontier.points[i];
        const double angle = j * 2. * std::acos(-1.) / 16.;
        p.x += radius * std::cos(angle); p.y += radius * std::sin(angle); p.z = 0.;
        if (!safe(p) || !visible(p, frontier.points[i])) continue;
        if (std::none_of(result.begin(), result.end(), [&p](const Point& q) { return distance(p, q) < .12; }))
          result.push_back(p);
      }
    }
    std::sort(result.begin(), result.end(), [&robot, &frontier](const Point& a, const Point& b) {
      return distance(a, robot) + .5 * distance(a, frontier.initial) <
             distance(b, robot) + .5 * distance(b, frontier.initial);
    });
    return result;
  }
  geometry_msgs::PoseStamped pose(const Point& point, double yaw) const {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map"; msg.header.stamp = ros::Time::now();
    msg.pose.position = point; msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return msg;
  }
  void visualize(const std::vector<Frontier>& frontiers) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; marker.header.stamp = ros::Time::now();
    marker.ns = "automatic_frontiers"; marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.; marker.scale.x = marker.scale.y = .04;
    marker.color.b = 1.; marker.color.g = .7; marker.color.a = 1.;
    for (const auto& frontier : frontiers)
      marker.points.insert(marker.points.end(), frontier.points.begin(), frontier.points.end());
    array.markers.push_back(marker); markers_.publish(array);
  }
  void tick(const ros::TimerEvent&) {
    // make_plan rejects requests while a navigation action is still active.
    // Wait for cancellation acknowledgement before evaluating any new route.
    if (cancelling_) {
      if (!nav_.getState().isDone()) { report("WAITING"); return; }
      cancelling_ = false;
    }
    tf::StampedTransform transform;
    try {
      if (!prepareGrid()) throw std::runtime_error("Waiting for fresh map/scan");
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
      if (std::abs((ros::Time::now() - transform.stamp_).toSec()) > sensor_timeout_)
        throw std::runtime_error("Stale robot TF");
    } catch (const std::exception& error) {
      cancelActive();
      report("WAITING"); ROS_WARN_THROTTLE(5., "%s", error.what()); return;
    }
    Point robot; robot.x = transform.getOrigin().x(); robot.y = transform.getOrigin().y();
    unsigned robot_x, robot_y;
    if (!grid_.worldToMap(robot.x, robot.y, robot_x, robot_y) ||
        grid_.getCost(robot_x, robot_y) != costmap_2d::FREE_SPACE) {
      cancelActive(); report("WAITING"); return;
    }
    frontier_exploration::FrontierSearch search(&grid_, 1e-3, 1., minimum_);
    auto frontiers = search.searchFrom(robot);
    frontiers.erase(std::remove_if(frontiers.begin(), frontiers.end(),
                     [this](const Frontier& item) { return !spansMinimum(item); }), frontiers.end());
    frontier_points_.clear();
    for (const auto& frontier : frontiers)
      frontier_points_.insert(frontier_points_.end(), frontier.points.begin(), frontier.points.end());
    failures_.erase(std::remove_if(failures_.begin(), failures_.end(),
      [&frontiers](const Failure& failure) {
        return std::none_of(frontiers.begin(), frontiers.end(), [&failure](const Frontier& frontier) {
          return std::any_of(frontier.points.begin(), frontier.points.end(), [&failure](const Point& point) {
            return distance(failure.frontier, point) < .35;
          });
        });
      }), failures_.end());
    visualize(frontiers);
    bool hold = false;
    nh_.getParam("/final_project/automatic/exploration_hold", hold);
    if (hold) {
      cancelActive(); report("PAUSED", frontiers.size()); return;
    }
    if (active_) {
      const auto state = nav_.getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        active_ = false;
        if (!observed(frontier_) && known() <= known_start_) failed(frontier_, "goal reached without map gain");
      } else if (state.isDone()) {
        failed(frontier_, "navigation action ended: " + state.getText());
      } else {
        const double remaining = distance(robot, goal_);
        if (remaining + .02 < best_) { best_ = remaining; last_progress_ = ros::Time::now().toSec(); }
        if (ros::Time::now().toSec() - last_progress_ > timeout_) failed(frontier_, "navigation progress timeout");
        else { report("ACTIVE", frontiers.size()); return; }
      }
    }
    if (cancelling_) { report("RETRYING", frontiers.size()); return; }
    bool retry_pending = false;
    for (const auto& frontier : frontiers) {
      if (attempts(frontier.initial) >= 2) continue;
      const auto options = candidates(frontier, robot);
      if (options.empty()) {
        ROS_INFO_THROTTLE(10., "Frontier has no safe visible vantage: (%.3f,%.3f), cells=%u", frontier.initial.x, frontier.initial.y, frontier.size);
        continue; // No footprint-safe vantage in known free space.
      }
      bool found = false;
      geometry_msgs::PoseStamped target;
      for (size_t i = 0; i < std::min<size_t>(options.size(), 12); ++i) {
        nav_msgs::GetPlan request;
        request.request.start = pose(robot, tf::getYaw(transform.getRotation()));
        target = pose(options[i], std::atan2(frontier.centroid.y - options[i].y, frontier.centroid.x - options[i].x));
        request.request.goal = target; request.request.tolerance = 0.;
        if (planner_.call(request) && !request.response.plan.poses.empty()) { found = true; break; }
      }
      if (!found) { failed(frontier.initial, "no plan among nearest candidate vantages"); retry_pending = true; continue; }
      goal_ = target.pose.position; frontier_ = frontier.initial; known_start_ = known();
      best_ = distance(robot, goal_); last_progress_ = ros::Time::now().toSec();
      ROS_INFO("Exploration goal=(%.3f,%.3f) frontier=(%.3f,%.3f) cells=%u candidates=%zu known=%u attempts=%u",
               goal_.x, goal_.y, frontier_.x, frontier_.y, frontier.size, options.size(), known_start_, attempts(frontier_));
      move_base_msgs::MoveBaseGoal goal; goal.target_pose = target;
      nav_.sendGoal(goal); active_ = true; report("ACTIVE", frontiers.size()); return;
    }
    report(failures_.empty() ? "EXHAUSTED" : (retry_pending ? "RETRYING" : "BLOCKED"), frontiers.size());
  }
public:
  AutomaticExplore() : nav_("/move_base", true) {
    const std::string ns = "/final_project/automatic/";
    if (!nh_.getParam(ns + "scene_bounds", bounds_) || bounds_.size() != 4 ||
        !nh_.getParam(ns + "scene_to_map", anchor_) || anchor_.size() != 3)
      throw std::runtime_error("Missing scene bounds/map anchor");
    nh_.param(ns + "safe_radius", radius_, .30);
    // A goal may be accepted anywhere inside the planner's arrival tolerance.
    // Keep that whole region safe for the actual padded navigation footprint.
    double padding, tolerance;
    nh_.param("/move_base/global_costmap/footprint_padding", padding, .01);
    nh_.param("/move_base/DWAPlannerROS/xy_goal_tolerance", tolerance, .10);
    ros::NodeHandle costmap_nh("/move_base/global_costmap");
    auto footprint = costmap_2d::makeFootprintFromParams(costmap_nh);
    if (footprint.empty()) throw std::runtime_error("Missing navigation footprint");
    costmap_2d::padFootprint(footprint, padding);
    for (const auto& point : footprint)
      radius_ = std::max(radius_, std::hypot(point.x, point.y));
    radius_ += tolerance;
    ROS_INFO("Exploration target clearance=%.4f m, including padding and arrival tolerance", radius_);
    nh_.param(ns + "min_frontier_size", minimum_, .30);
    nh_.param(ns + "progress_timeout", timeout_, 60.);
    nh_.param(ns + "sensor_timeout", sensor_timeout_, 1.);
    status_ = nh_.advertise<std_msgs::String>("/final_project/exploration/status", 1, true);
    markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/final_project/exploration/frontiers", 1, true);
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, [this](const nav_msgs::OccupancyGrid::ConstPtr& m) { map_ = m; });
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, [this](const sensor_msgs::LaserScan::ConstPtr& m) { scan_ = m; });
    planner_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    clear_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    report("WAITING");
    if (!nav_.waitForServer(ros::Duration(60)) || !planner_.waitForExistence(ros::Duration(60)))
      throw std::runtime_error("Exploration navigation unavailable");
    timer_ = nh_.createTimer(ros::Duration(1.), &AutomaticExplore::tick, this);
  }
  ~AutomaticExplore() { nav_.cancelAllGoals(); }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "automatic_explore");
  try { AutomaticExplore explorer; ros::spin(); }
  catch (const std::exception& error) { ROS_FATAL("Exploration stopped: %s", error.what()); return 1; }
  return 0;
}
