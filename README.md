# firefly_waypoint_navigator

The package contains Model Predictive Controller respectively for x and y position and a PD controller for z position of the UAV asctec Firefly.

In order to compile this package correctly, the following packages are required:  
rotors_simulator: https://github.com/ethz-asl/rotors_simulator  
waypoint_navigator: https://github.com/ethz-asl/waypoint_navigator  
mav_control_rw: https://github.com/ethz-asl/mav_control_rw  

## Subpackages  
`firefly_controller`: PD control, MPC and Kalman filter  
`firefly_gazebo_sim`: launch files   
`firefly_waypoint_navigator`: package list of a metapackage  

## Instructions
only PD control for height (hovering):  
<pre><code> roslaunch firefly_gazebo_sim firefly_height.launch </code></pre>    
>Send a reference waypoint using a service from the package `waypoint_navigator`, but only the z value will be used as target height:
<pre><code> rosservice call /firefly/go_to_waypoint "point: [0, 0, 1]" </code></pre>
>Open the GUI of dynamic reconfigure, where `softlanding` can be enabled:
<pre><code> rosrun rqt_reconfigure rqt_reconfigure </code></pre>
>The UAV which has reached the target height 1m, will then land softly to the ground. If we disable `softlanding` again, the UAV will return to the previous target heigh 1m.


MPC for x and y, and PD control for z:
<pre><code> roslaunch firefly_gazebo_sim firefly_embedded_mpc.launch </code></pre>
>Need to wait for a while, until the message `[INFO] embedded MPC: initialized correctly` shows up in the command window. Because the calculation for initializing matrices of model and controller takes time. Then we can send a reference waypoint using a service from the package `waypoint_navigator`:
<pre><code> rosservice call /firefly/go_to_waypoint "point: [0.1, 0.2, 0.5]" </code></pre>
>The parameters of model, Kalman filter and controller can be changed in `firefly_controller/params/embedded_mpc.yaml` and `firefly_controller/params/kf_observer.yaml`. Note that the model parameters `A` and `B` need to be the same in the two yaml files.
