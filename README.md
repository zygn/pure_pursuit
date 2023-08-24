# pure_pursuit

## Quickstart

1. Make your workspace

```
mkdir -p f1tenth_ws/src
cd f1tenth_ws/src
```

2. Clone this repository in your workspace and checkout noetic branch and, build with catkin
```bash
git clone https://github.com/zygn/pure_pursuit
cd pure_pursuit
git checkout noetic
cd ../..
catkin_make
```

3. Run 
```
source ~/f1tenth_ws/devel/setup.bash
roslaunch pure_pursuit launch.launch
```

## Parameters

In the `launch/launch.launch` file you can found these parameters. Change that as needed.
```
<launch>
    <node name="pure_pursuit" pkg="pure_pursuit" type="pursuit.py" output="screen">
        <param name="wheelbase" value="0.3302" />
        <param name="lookahead_distance" value="0.82461887897713965"/>
	    <param name="sprint_speed" value="4.0" />
	    <param name="velocity_gain" value="1.375" />
	    <param name="max_reacquire" value="20.0" />
	    <param name="waypoint_path" value="$(find pure_pursuit)/waypoints/example_waypoints.csv" />

        <param name="scan_topic" value="/scan" />
        <param name="odom_topic" value="/odom" />
        <param name="drive_topic" value="/drive" />    
        <param name="tf_topic" value="/tf" />    
        <param name="joy_topic" value="/joy" />    
    </node>
</launch>
```
