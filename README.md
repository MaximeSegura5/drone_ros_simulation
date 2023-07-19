# drone_ros_simulation

## topicService
This allows publisher and suscriber to be unified under the same class.

## rospyHandler
This class allows you to init a rospy node and shutdown it.   
There are also two methods you can use on a topic of msg type allowing you to subscribe (receive data from UAV) or publish (send data/order to UAV).   
Then you have the method service_caller that you can call on a topic of type srv to use the ROS service concerned.

## multiCopterHandler
This is the main file in which all the functions of your drone are defined.

First you have the variables, which are for the most part containing the data you receive from the drone every *x* seconds thanks to your subscribers.

Then you define all the topics you need (msg, srv, etc...).

You also start a thread calling the method *update_parameters_from_topic* in which you can use all the data collected from the drone by your subscribers topics to update your variables as you want.

After this, you have all the functions that you can use on your drone:
* arm (you need to arm before takeoff)
* takeoff (at the altitude given)
* change_mode
* move_global (send your drone to the gps coordinates you gave)
* move_global (send your drone to the local coordinates you gave)
* set and get param
* return_to_home (move to the home point you defined and land)

