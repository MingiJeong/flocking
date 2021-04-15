define lidar ranger
(
  sensor
  (
    range [0 5]
    fov 360.0
    samples 721
  )
  # generic model properties
  color "black"
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  localization "gps"  

  velocity_bounds [-2 2 -1 1 -1 1 -90 90 ]					
  acceleration_bounds [-2 2 -1 1 -1 1 -90 90]

  size [ 0.4 0.3 0.3]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  lidar(pose [ 0.0 0.0 -0.1 0.0 ])
)




