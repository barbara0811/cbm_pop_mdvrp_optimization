# CBM-pop optimization for MDVRP
Distributed Coalition-Based Metaheuristic optimization for Multi-Depot Vehicle Routing Problems with capacity and precedence constraints.

Documentation:
https://barbara0811.github.io/cbm_pop_mdvrp_optimization

## Requirements:
ROS melodic (http://wiki.ros.org/melodic)

## Run the optimization example

The example CBM-pop agent (CBMPopAgentNode) is made for Cordeau benchmark examples, which are included in the data folder. For a custom optimization problem implementation, it is necessary to implement specific CBMPopAgent and CBMPopAlgorithm objects.

1. Run several CBM-pop agents (in different terminals):

```roslaunch cbm_pop_mdvrp cbm_pop_agent.launch name:=<agent name>```

2. Launch the mission:

```rostopic pub /optimize std_msgs/String "data: '<example_id>'"```
  
### Use example:
```
roslaunch cbm_pop_mdvrp cbm_pop_agent.launch name:=ag1
roslaunch cbm_pop_mdvrp cbm_pop_agent.launch name:=ag2
roslaunch cbm_pop_mdvrp cbm_pop_agent.launch name:=ag3
rostopic pub /optimize std_msgs/String "data: 'p01'"
```
