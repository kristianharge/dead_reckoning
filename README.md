# Dead reckoning system for CarriRo AD form factor

## Index

1. Introduction to dead reckoning
1. Mathematical representation of dead reckoning
1. Functionnal overview of the code
1. How to build and test the code
1. Room for improvements
1. Bibliography

## Introduction to dead reckoning

### What is dead reckoning ?

In navigation, dead reckoning is the process of calculating current position of some moving object by using a previously determined position, or fix, and then incorporating estimates of speed, heading direction, and course over elapsed time. The corresponding term in biology, used to describe the processes by which animals update their estimates of position or heading, is path integration.[^1]

### Why would we need dead reckoning in our system ?

In our case, we consider the CarriRo AD as being the robot in wich we will implement dead reckoning:

<p align="center">
  <img src="./Images/CarriRo.jpeg" />
  <figcaption align = "center"><b>Fig.1 CarriRo Image</b></figcaption>
</p>

As far as I know, the idea behind this robot is to have an autonomous moving cart that can handle heavy loads transportation. A dead reckoning system would give to this robot, the capability of knowing its current position in a 2D plane from an initially known position. That is an essential part when comming to autonomous mooving robots.

In this use case, we have the following informations :
- The two rear wheels are driving wheels
- The two frony wheels are swivel caster wheels
- We can get the odometry of the 4 wheels
- We can get the yaw rate of the whole robot

## Mathematical representation of dead reckoning

### Dead reckoning with odometry and yaw rate

In order to estimate our position, we have two kind of informations available. The 4 wheel odometry and the yaw rate. In this case, we will use odometry to caclulate the robot shift, and the gyrometer to calculate the angle shift. We could use odometry alone to extract both of this parameters, but the gyrometer should be more precise when extracting the yaw angle.

### Coordinate system and robot initial position

We will use an orthonormal coordinate system. Where at initialization, *CarriRo AD* 0 coordinate is at the coordinate system origin. It is important to note that our *CarriRo AD* origin is the middle between the two rear wheels (for simplicity purposes). Finally, carriro is facing the positive X axis.

<p align="center">
  <img src="./Images/coordinates.png" />
  <figcaption align = "center"><b>Fig.2 Coordinates system</b></figcaption>
</p>

<p align="center">
  <img src="./Images/CarriRo_mockup.png" />
  <figcaption align = "center"><b>Fig.3 CarriRo representative illustration</b></figcaption>
</p>

<p align="center">
  <img src="./Images/CarriRo_initial_position.png" />
  <figcaption align = "center"><b>Fig.4 CarriRo inital position in the plane</b></figcaption>
</p>

The angle $\tetha$ is the angle between the direction of the robot and the x axis.

So at the begining, our coordinates are the following ones :
- $x = 0$
- $y = 0$
- $\tetha = 0$

### From the physics to the maths

In this section, we will cover how the odometry and yaw rate can be used to extract the absolute robot position.

## Bibliography

[^1]: Wikipedia : https://en.wikipedia.org/wiki/Dead_reckoning
