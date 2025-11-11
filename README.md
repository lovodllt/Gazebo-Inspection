# Gazebo-Inspection
gazebo仿真巡检

    catkin_make
    //编译
    source devel/setup.bash
    
    roslaunch gazebo_pkg race.launch
    //gazebo加载机器人
    
    roslaunch gazebo_nav nav.launch
    //加载导航模块
    
    rosrun gazebo_nav auto_inspection.py
    //导航地点






    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    //键盘控制