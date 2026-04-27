cmake_minimum_required(VERSION 3.8)
project(plato_pod_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Localization-agnostic pose messages
  "msg/RobotPose.msg"
  "msg/RobotPoses.msg"
  # Legacy AprilTag messages (kept for vision node compatibility)
  "msg/TagPose.msg"
  "msg/TagDetections.msg"
  # Arena model
  "msg/HomographyMatrix.msg"
  "msg/Obstacle.msg"
  "msg/Zone.msg"
  "msg/ArenaModel.msg"
  # Robot status
  "msg/RobotStatus.msg"
  "msg/RobotStatusList.msg"
  # CoT integration
  "msg/NavGoal.msg"
  # Tactical engagement (typed contracts replacing String + JSON)
  "msg/FireIntent.msg"
  "msg/EngagementOutcome.msg"
  "msg/Observation.msg"
  # Sensor envelope
  "msg/SensorReading.msg"
  # World state (published by world_state_node, latched)
  "msg/CoverPolygon.msg"
  "msg/WorldCover.msg"
  "msg/Civilian.msg"
  "msg/CivilianList.msg"
  "msg/IedZone.msg"
  "msg/IedZoneList.msg"
  "msg/EwEmitter.msg"
  "msg/EwEmitterList.msg"
  "msg/JammingZone.msg"
  "msg/JammingZoneList.msg"
  "msg/DeadZone.msg"
  "msg/DeadZoneList.msg"
  "msg/WeatherState.msg"
  "msg/RoeRules.msg"
  "msg/WeaponSpec.msg"
  "msg/WeaponCatalog.msg"
  # Services
  "srv/RegisterPhysical.srv"
  "srv/SpawnVirtual.srv"
  "srv/RemoveRobot.srv"
  "srv/ResetRobot.srv"
  "srv/ListRobots.srv"
  "srv/ApplyDamage.srv"
  "srv/EvaluateLos.srv"
)

ament_package()
