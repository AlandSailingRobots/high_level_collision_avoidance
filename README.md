# collision avoidance algorithm

## Introduction

The goal of this algorithm is to check if there is any collision with another boat or with the border of the sailing zone in the current route, represented by a list of waypoints.
You can specify the sailing zone, the number and the trajectory of the other boat, and the waypoints list in a JSON file “config.json”.

## input format in the json file :

### border list :
[[[coordinates of the 1st point of the 1st border],... [coordinates of the last point of the 1st border]],
[[coordinates of the 1st point of the 2nd border],... [coordinates of the last point of the 2nd border]]
[...]
]

If you want no border : just put an empty vector []

###waypoints :
[[coordinates of the 1st waypoint],... [coordinates of the last waypoint]]

### obstaclesInfos :
[[[uncertainty interval on the speed of the 1st obstacle], [uncertainty interval on the x coordinate of the 1st obstacle], [uncertainty interval on the y coordinate of the 1st obstacle], [uncertainty interval on the heading of the 1st obstacle]],
[[uncertainty interval on the speed of the 2nd obstacle], [uncertainty interval on the x coordinate of the 2nd obstacle], [uncertainty interval on the y coordinate of the 2nd obstacle], [uncertainty interval on the heading of the 2nd obstacle]],
[...]
]

If you doesn't want any other boats : just put an empty vector []