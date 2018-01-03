# Debugging Application

In order to assist in debugging path planner project, this electron application connects and receives web socket data. Small green dots on the world map are the way points sent by the path planner. On each iteration, the car is shown as blue dot where as other cars are shown in Red, Orange and Pink

# How to Run

npm start

# Message types

## worldmap

{
    type: "worldmap",
    worldmap_x: [],
    worldmap_y: []
}

## Future Work

Each of the other cars should be shown in a seperate lane. May be we can use the nearest way point on the world map, make a vector from the closest two points, draw a normal and then use that normal to assign different lanes.