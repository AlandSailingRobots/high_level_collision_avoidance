#input format in the json file :

##border list :
[[[coordinates of the 1st point of the 1st border],... [coordinates of the last point of the 1st border]],
[[coordinates of the 1st point of the 2nd border],... [coordinates of the last point of the 2nd border]]
[...]
]

If you want no border : just put an empty vector []

##waypoints :
[[coordinates of the 1st waypoint],... [coordinates of the last waypoint]]

##obstaclesInfos :
[[[uncertainty interval on the speed of the 1st obstacle], [uncertainty interval on the x coordinate of the 1st obstacle], [uncertainty interval on the y coordinate of the 1st obstacle], [uncertainty interval on the heading of the 1st obstacle]],
[[uncertainty interval on the speed of the 2nd obstacle], [uncertainty interval on the x coordinate of the 2nd obstacle], [uncertainty interval on the y coordinate of the 2nd obstacle], [uncertainty interval on the heading of the 2nd obstacle]],
[...]
]