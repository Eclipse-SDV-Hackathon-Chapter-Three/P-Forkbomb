#pragma once
#include "carla_rust.hpp"
#include "carla/Time.h"
#include "carla/Memory.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/rpc/AttachmentType.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehicleDoor.h"
#include "carla/rpc/VehicleWheels.h"
#include "carla/rpc/VehicleAckermannControl.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/rpc/GearPhysicsControl.h"
#include "carla/rpc/WeatherParameters.h"
#include "carla/rpc/ObjectLabel.h"
#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/TrafficManager.h"
#include "carla/trafficmanager/SimpleWaypoint.h"
#include "carla/client/Waypoint.h"
#include "carla/client/Sensor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Walker.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/TrafficSign.h"
#include "carla/client/Junction.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/ActorList.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Landmark.h"
#include "carla/client/LaneInvasionSensor.h"
#include "carla/client/Light.h"
#include "carla/client/LightManager.h"
#include "carla/client/Map.h"
#include "carla/client/World.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/client/Timestamp.h"
#include "carla/client/Light.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/RadarMeasurement.h"
#include "carla/sensor/data/RadarData.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla/sensor/data/SemanticLidarMeasurement.h"
#include "carla/sensor/data/LidarData.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/ObstacleDetectionEvent.h"
#include "carla/sensor/data/CollisionEvent.h"
#include "carla/sensor/data/LaneInvasionEvent.h"
#include "autocxxgen_ffi.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#if __cplusplus >= 201703L
#include <string_view>
#endif

namespace rust {
inline namespace cxxbridge1 {
// #include "rust/cxx.h"

namespace {
template <typename T>
class impl;
} // namespace

class String;

#ifndef CXXBRIDGE1_RUST_STR
#define CXXBRIDGE1_RUST_STR
class Str final {
public:
  Str() noexcept;
  Str(const String &) noexcept;
  Str(const std::string &);
  Str(const char *);
  Str(const char *, std::size_t);

  Str &operator=(const Str &) & noexcept = default;

  explicit operator std::string() const;
#if __cplusplus >= 201703L
  explicit operator std::string_view() const;
#endif

  const char *data() const noexcept;
  std::size_t size() const noexcept;
  std::size_t length() const noexcept;
  bool empty() const noexcept;

  Str(const Str &) noexcept = default;
  ~Str() noexcept = default;

  using iterator = const char *;
  using const_iterator = const char *;
  const_iterator begin() const noexcept;
  const_iterator end() const noexcept;
  const_iterator cbegin() const noexcept;
  const_iterator cend() const noexcept;

  bool operator==(const Str &) const noexcept;
  bool operator!=(const Str &) const noexcept;
  bool operator<(const Str &) const noexcept;
  bool operator<=(const Str &) const noexcept;
  bool operator>(const Str &) const noexcept;
  bool operator>=(const Str &) const noexcept;

  void swap(Str &) noexcept;

private:
  class uninit;
  Str(uninit) noexcept;
  friend impl<Str>;

  std::array<std::uintptr_t, 2> repr;
};
#endif // CXXBRIDGE1_RUST_STR
} // namespace cxxbridge1
} // namespace rust

using std_pair_float_float_AutocxxConcrete = ::std_pair_float_float_AutocxxConcrete;
using std_basic_ostream_char_AutocxxConcrete = ::std_basic_ostream_char_AutocxxConcrete;
using carla_client_detail_EpisodeProxyImpl_std_weak_ptr_carla_client_detail_Simulator_AutocxxConcrete = ::carla_client_detail_EpisodeProxyImpl_std_weak_ptr_carla_client_detail_Simulator_AutocxxConcrete;
using std_pair_carla_rpc_ActorId_carla_rpc_VehicleLightState_flag_type_AutocxxConcrete = ::std_pair_carla_rpc_ActorId_carla_rpc_VehicleLightState_flag_type_AutocxxConcrete;
using std_pair_carla_traffic_manager_RoadOption_carla_traffic_manager_WaypointPtr_AutocxxConcrete = ::std_pair_carla_traffic_manager_RoadOption_carla_traffic_manager_WaypointPtr_AutocxxConcrete;
namespace carla {
  namespace sensor {
    namespace data {
      using Image = ::carla::sensor::data::Image;
    }
  }
}
