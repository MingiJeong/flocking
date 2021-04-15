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

  size [ 0.3 0.3 0.4 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  lidar(pose [ 0.0 0.0 -0.1 0.0 ])
)




