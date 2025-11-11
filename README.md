# Gazebo-Inspection

依赖

pip3 install ultralytics

gazebo仿真巡检

    catkin_make
    //编译
    source devel/setup.bash
    
    roslaunch gazebo_pkg race.launch
    //gazebo加载机器人
    
    识别版(要改绝对路径)：
    rosrun gazebo_pkg random_model_for_room.py 
    
    roslaunch gazebo_nav nav.launch
    //加载导航模块
    
    rosrun gazebo_nav auto_inspection.py
    //导航地点
    
    识别：
    rosrun detection detection.py 


    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    //键盘控制