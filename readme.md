
Launch robot in rviz with joints and model (update path to suit).

    ros2 launch urdf_tutorial display.launch.py model:='/home/vivi/ws_moveit2/src/custom_robot/dreadnought.urdf'

Update links dynamically in rviz, rather than having to reload

    python3 urdf_updater/updater.py  --urdf /home/vivi/ws_moveit2/src/custom_robot/dreadnought.urdf

