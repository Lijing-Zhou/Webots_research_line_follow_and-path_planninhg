# Webots_research_line_follow_and-path_planninhg
这是我2021年在布里斯托大学做的在webot环境下的小车寻线。和在webots环境下的VFH+和BUG路径动态路径规划算法的实现。详情可见这个GitHub中的论文。
//


//
摘要在复杂的障碍物情况下，如何解决
移动机器人的路径规划和实时避障问题。
一直是机器人导航中的一个难题[1]。本文主要研究路径规划技术和实时避障技术。
本文主要研究移动机器人的路径规划技术和
障碍物密集场景下移动机器人的路径规划技术和避障方法的研究。
场景下的路径规划技术和避障方法。本项目采用Webots软件，基于
矢量场的避障方法的研究。
直方图（VFH）算法的基础上，实现了自主导航和避障功能。
和避障功能，并设计了不同的障碍物
情况，最后验证并讨论了算法的有效性。
最后验证并讨论了该算法的有效性。此外，我们还与简化的BUG算法做了一个简单的比较，以说明
简化的BUG算法做了简单的比较，以显示其优势。
关键词。VFH算法，BUG算法，避障，自主导航，Webots
I. 前言
自主定位和避障是服务机器人提供服务的重要先决条件。在
在一个未知的环境中，自主移动机器人需要
在一个未知的环境中，自主移动机器人需要依靠自己的传感器来不断获得关于周围环境的信息
在一个未知的环境中，自主移动机器人需要依靠自己的传感器不断获取周围环境的信息，识别障碍物的位置
障碍物，并进行计算和自主决策[2]。所有的移动机器人都有一定的特点
避免碰撞的特点，从最初的算法到使用
复杂的算法来防止机器人避开
障碍物。实时避障的算法要复杂得多
更加复杂，因为它们不仅涉及检测
障碍物，而且还要对障碍物的大小进行一定的定量测量。
障碍物的大小。一旦这些被确定，障碍物
避开障碍物的算法需要引导机器人绕过障碍物，并向原来的目标前进。
绕过障碍物，向原来的目标移动。通常情况下，程序要求机器人在遇到障碍物时停下来。
程序要求机器人从障碍物处停下来，进行一次测量，然后恢复运动。
测量，然后再恢复运动。障碍物规避
可能会导致非最佳路径，因为没有使用现有的环境知识。
环境的知识[3]。
基于本学期对Webots软件的研究，我们小组继续探索这一实际问题，因为
在本学期对Webots软件的研究基础上，我们小组继续探索这个实际问题，作为本项目的主题
本项目的主题。根据调查，我们发现
探讨，发现机器人的定位方法有很多种，避障算法也有很多。
躲避障碍物的算法也层出不穷。我们选择了
矢量场直方图（VFH）算法作为我们的方法来实现机器人的实时避障导航。
实现机器人的实时避障导航。
它的功能是在Webots仿真环境中通过代码实现的。
环境中通过代码实现其功能，并通过实验进行了验证。然后，我们测试了
然后，我们测试了经典的简化BUG算法，以比较路径的长度来显示其性能。
径的长度来显示其性能。
在第二节中，本文将对VFH算法和简化BUG算法进行梳理和解释。
算法和简化BUG算法。然后，在第
三、如何在Webots软件中模拟机器人的实现。
软件中模拟实现机器人遵循VFH算法进行实时避障导航。第四节将
对本项目开发的VFH算法进行实验测试。
本项目开发的VFH算法进行实验测试，构建不同位置的障碍物，测试
障碍物，测试该算法实现避障导航的结果，并讨论与简化的避障导航算法的比较。
障碍物导航，并讨论与简化的BUG
算法的比较。第五部分提出了一些假设和
第五部分根据实验结果提出了一些假设和结论。
A. 假设声明
通过应用8个距离传感器，尝试检测机器人360度的总范围。
机器人的总范围。利用VFH算法的思想
和简化的BUG算法来完成代码设计。
使机器人能够在简单的障碍物中达到路径规划和
在一个简单的障碍物环境中实现避障的目的。
我们通过对Webots模拟的结构化体验来研究这一假设，测试不同环境和不同算法的性能。
不同环境和不同算法的性能。
- 目的：学习VFH算法，并在Webots环境中使用该代码实现自主导航。
掌握VFH算法，并在Webots环境中使用该代码实现点对点的自主导航
实现点对点之间的自主导航和避障，并测试
并讨论实验结果。
1）"了解矢量场直方图（VFH）算法的工作原理"。
2) "通过代码实现它。"
3) "记录路径的长度作为一个因素来测试
效率"。
4) "设计不同的障碍并测试代码的可行性。
代码的可行性"。
- 目标：。这个项目的核心是理论上的
实现VFH算法。简化的BUG
算法在报告中占了较少的部分。在此基础上。
机器人在两点之间运动的一些物理数据被记录下来。
在此基础上，记录了机器人在两点之间运动的一些物理数据，并设计了多种障碍物场景，以验证代码的可行性。
来验证代码的可行性。
1) "让机器人从起点跑到终点
使机器人从起点跑到终点位置，并进行路径规划和避开
障碍物。"
2) "构建一个机器人系统，包括和长度
路径的长度，以收集有意义的数据"。
3) "在整个系统评估之前，必须对结论进行完整的分析。
在整个系统评估之前，必须进行完整的结论分析，以
进行明确的实验验证。"
- 挑战：这个项目最大的挑战是如何
如何使用代码来实现VFH避障算法？
在Webots仿真环境中实现VFH避障算法。





Autonomous positioning and obstacle avoidance are important 
prerequisites for service robots to provide services. In
an unknown environment, autonomous mobile robots need to
rely on their own sensors to continuously obtain information
about the surrounding environment, identify the location of
obstacles, and perform calculations and autonomous decision
making. All mobile robots have certain characteristics of
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
of the environment is used.


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
software to follow the VFH algorithm for real-time obstacle 
avoidance navigation is introduced. The section IV will
conduct experimental tests on the VFH algorithm developed
in this project, construct obstacles in different positions, test
the results of the algorithm to achieve obstacle avoidance
navigation, and discuss the comparison with simplified BUG
algorithm. The section V puts forward some hypotheses and
conclusions based on experimental results.
