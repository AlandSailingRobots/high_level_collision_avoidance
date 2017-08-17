# Collision avoidance algorithm

## Introduction

The goal of this algorithm is to check if there is any collision with another boat or with the border of the sailing zone in the current route, represented by a list of waypoints.
You can specify the sailing zone, the number and the trajectory of the other boat, and the waypoints list in a JSON file “config.json”.

## prerequisite

In order to display the outputs of the algorithm, you will need to install the following application :
[VIBes viewer](http://enstabretagnerobotics.github.io/VIBES/)
you will need to run vibes-viewer before running this code.

If you want to compile this code, you will need to install the following library : ibex 2.3.4
[ibex download page](http://www.ibex-lib.org/download) (watch out for the version of the library, the code was written with the 2.3.4 version)

## Input format in the json file

### border list :
[[[coordinates of the 1st point of the 1st border],... [coordinates of the last point of the 1st border]],
[[coordinates of the 1st point of the 2nd border],... [coordinates of the last point of the 2nd border]]     
[...]
]

If you don't want any border : just put an empty vector []

### waypoints :
[[coordinates of the 1st waypoint],... [coordinates of the last waypoint]]

### obstaclesInfos :
[[[uncertainty interval on the speed of the 1st obstacle], [uncertainty interval on the x coordinate of the 1st obstacle], [uncertainty interval on the y coordinate of the 1st obstacle], [uncertainty interval on the heading of the 1st obstacle]],    
[[uncertainty interval on the speed of the 2nd obstacle], [uncertainty interval on the x coordinate of the 2nd obstacle], [uncertainty interval on the y coordinate of the 2nd obstacle], [uncertainty interval on the heading of the 2nd obstacle]],    
[...]
]

If you don't want any other boats : just put an empty vector []

## Outputs of the algorithm

This algorithm outputs a drawing of the trajectory of the boat and of the other boats. The route before collision avoidance is displayed in yellow, after collision avoidance in blue. The sailing zone borders are displayed in red. The route of the other boats are displayed in black.

The blue box represent the uncertainty on the width of the channel and the uncertainty on the boat progression in this channel.
The black boxes represent the uncertainty on the other boats positions.

For each segment of the trajectory where there is a collision, the algorithm outputs a paving of the feasible speed (seen as a vector of coordinate (vx, vy)) of the boat. The speeds which lead to a collision with another boat or with the borders of the sailing zone are displayed in red and yellow. In blue are displayed the speeds which avoid collisions. In blue you can see the initial speed of the boat, before collision avoidance, and in green th feasible speed chosen by the algorithm.

## Other information

if you would like more information, please read the document below (the part entilted deliberative collision avoidance algorithm) :
[description document](https://docs.google.com/a/uas.ax/document/d/1QUy6AdNvMZuc0FNC6rVeGaShAturBskqho5N7traBUM/edit?usp=sharing)

