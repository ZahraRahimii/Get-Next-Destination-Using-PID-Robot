# Get Next Destination Using PID Controller Robot

To achieve this purpose we need to define two nodes:

## Mission Node: 
This node should respond to the GetNextDestination service, so that for each request, it will randomly generate two numbers in the range of -20 to 20 as the next destination x and y.

## Control Node:
This node is responsible for controlling the robot, which is done by a PID controller.


