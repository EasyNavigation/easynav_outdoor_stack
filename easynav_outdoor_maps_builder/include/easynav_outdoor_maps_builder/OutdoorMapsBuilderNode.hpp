#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDERNODE_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_outdoor_maps_builder/types/MapsBuilder.hpp"

namespace easynav
{

  /**
   * @class OutdoorMapsBuilderNode
   * @brief Lifecycle node managing multiple MapsBuilder instances for outdoor map generation.
   *
   * This node subscribes to sensor data (e.g., point clouds) and manages a collection
   * of MapsBuilder objects. Each MapsBuilder processes perception data and publishes
   * map representations (e.g., filtered point clouds). The node implements ROS2 lifecycle
   * management, enabling clean startup, shutdown, and runtime control.
   */
class OutdoorMapsBuilderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(OutdoorMapsBuilderNode)

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief Constructor
     * @param options NodeOptions for lifecycle node initialization.
     */
  explicit OutdoorMapsBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /// Destructor
  ~OutdoorMapsBuilderNode();

    /**
     * @brief Configure lifecycle callback.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Activate lifecycle callback.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Deactivate lifecycle callback.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Cleanup lifecycle callback.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Run a processing cycle on all MapsBuilder instances.
     *
     * Typically called periodically to process perception data,
     * update maps, and publish results.
     */
  void cycle();

    /**
     * @brief Accessor for the current perceptions data.
     * @return Const reference to Perceptions.
     */

private:
    /// Container holding the various map builders for different map types.
  std::vector<std::unique_ptr<MapsBuilder>> builders_;

    /// Sensor topic name to subscribe to point cloud data.
  std::string sensor_topic_;

    /// Aggregated perception data collected and processed by this node.
  Perceptions perceptions_;

    /// Shared pointer to processed perception data used by map builders.
  std::shared_ptr<PerceptionsOpsView> processed_perceptions_;

    /// Subscription to sensor_msgs::msg::PointCloud2 messages.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    /// Callback group for handling callbacks concurrently if needed.
  rclcpp::CallbackGroup::SharedPtr cbg_;

    /// Resolution parameter used for downsampling point cloud data.
  double downsample_resolution_;

    /// Default frame ID assigned to published perception data.
  std::string perception_default_frame_;
};

} // namespace easynav

#endif // EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDERNODE_HPP_
