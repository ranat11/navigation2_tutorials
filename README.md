# navigation2_tutorials
Tutorial code referenced in https://navigation.ros.org/

This fork has added the navigation package

## run
```
 ros2 launch sam_bot_description bringup.launch.py 
 ros2 launch nav2_bringup navigation_launch.py params_file:=dev_ws/src/navigation2_tutorials/sam_bot_description/config/nav2_params.yaml 
```

## TODO lists
- map do not start when run with use_sim_time and navigation package could not launch with the same launch file
- Add maps for localization
