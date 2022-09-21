# Webots_research_line_follow_and-path_planninhg
Autonomous positioning and obstacle avoidance are important prerequisites for service robots to provide services. In
an unknown environment, autonomous mobile robots need to
rely on their own sensors to continuously obtain information
about the surrounding environment, identify the location of
obstacles, and perform calculations and autonomous decisionmaking. All mobile robots have certain characteristics of
collision avoidance, from the original algorithm to the use
of complex algorithms to prevent the robot from avoiding
obstacles. Real-time obstacle avoidance algorithms are much
more complicated, because they not only involve the detection
of obstacles, but also a certain quantitative measurement
of obstacle size. Once these are determined, the obstacle
avoidance algorithm needs to guide the robot to bypass the
obstacle and move towards the original goal. Usually, the
program requires the robot to stop from the obstacle, take a
measurement, and then resume movement. Obstacle avoidance
can lead to non-optimal paths because no existing knowledge
of the environment is used [3].
Based on the study of Webots software this semester, our
group continue to explore this practical problem as the theme
of this project. According to the investigation, it is found
that there are many kinds of robots positioning methods, and
the obstacle avoidance algorithms are endless. We choose the
Vector Field Histogram (VFH) algorithm as our method to
realize the robot’s real-time obstacle avoidance navigation,
and its function is realized by code in Webots simulation
environment, and it is verified by experiment. Then we tested
the classic simplified BUG algorithm to compare the length
of the path to show the performance.
In Section II, this paper will sort out and explain the VFH
algorithm and simplified BUG algorithm. Then, in the Section
III how to simulate the realization of the robot in the Webots
software to follow the VFH algorithm for real-time obstacle avoidance navigation is introduced. The section IV will
conduct experimental tests on the VFH algorithm developed
in this project, construct obstacles in different positions, test
the results of the algorithm to achieve obstacle avoidance
navigation, and discuss the comparison with simplified BUG
algorithm. The section V puts forward some hypotheses and
conclusions based on experimental results.
