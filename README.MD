# Overview

This application is a re-creation of Google Maps' path finding capabilities.  Its GUI component interfaces with Google Maps.

This application will show and tell you the shortest path from a start intersection to up to seven other intersections traveled in order.

This application will also show and tell you a path that is very close to the shortest path from a start intersection that visits up to seven other intersections exactly once and ends back at the starting location (the Traveling Salesperson Problem (TSP)).

# Purpose of this Application

The user can select a pre-loaded map, load their own map, select a start location, select up to seven stop locations, and select one of five pathing algorithms.  The GUI will then visually display the shortest path calculated with the selected algorithm, visually number the stops, and show some stats about the path.

The shortest path is calculated in terms of trip duration and assumes the user is driving.  Time traveled on a road is determined by the speed limit of that road, which is in turn determined by the road's type.  Speed limits apply as you might expect.  For example, major highways have a speed limit of 70 miles per hour and residential roads have a speed limit of 25 miles per hour.  Time traveled does NOT currently take traffic into account.

# Technologies Used

The graph data structures and path finding algorithms are implemented in Java.

The GUI was built using JavaFX and JavaScript.

# My Work on This Project

The initial commit for this repository is the starter code as described in the section below.  All subsequent commits are my (Ryan Connor’s) work.  I:

1. Designed and implemented the class structure for represnting roads in a geographic location as a graph.
2. Implemented five path finding algorithms (described in a section below).
3. Modified the GUI to accomodate the two TSP algorithms.
4. Modified the GUI to accept seven stops instead of just one.
5. Wrote tests to ensure the accuracy of the two TSP algorithms (the other algorithms were graded by UCSD).

# Pathing Algorithms

The user can select five different algorithms to calculate a shortest path:

1. BFS (stands for Breadth First Search)
	- Finds the shortest path from start to stop using a simple breadth first search.
	- If more than one stop is selected, will do BFS from start to the first stop, then from the first stop to the second stop, and so on.
2. Dijsktra
	- Finds the shortest path from start to stop using Dijkstra's algorithm.
	- If more than one stop is selected, will do a Dijkstra search from start to the first stop, then from the first stop to the second stop, and so on.
3. A*
	- Finds the shortest path from start to stop using the A* algorithm.
	- If more than one stop is selected, will do an A* search from start to the first stop, then from the first stop to the second stop, and on.
4. GreedyTSP
	- Attempts to finds the shortest hamiltonian cycle from start through each selected stop using a purely greedy algorithm based on an A* search from the current location to each not-yet-visited stop.
5. Greedy2OptTSP
	- Attempts to find the shortest hamiltonian cycle from start through each selected stop using a greedy 2-opt algorithm.
		- First, 2OptTSP finds a cycle using a purely greedy algorithm based on A* search from the current location to each not-yet-visited stop.
		- Then, 2OptTSP attempts to shorten the cycle by deleting pairs of paths (S1, S2), (S3, S4) between stops and rebuilding the cycle with new paths (S1, S3), (S2, S4).  If this results in a shorter path, the process is re-started with this better path.  This is what makes this 2Opt algorithm "greedy" -- to save computational time, the algorithm restarts the swapping process immediately upon finding a shorter cycle instead of "remembering" the shorter cycle and continuing through all swap possibilities for the current cycle.
			- Re-building the cycle takes one-way streets into account.  That is, swapping path pair (S1, S2), (S3, S4) out for new paths (S1, S3), (S2, S4) means some other existing paths must be reversed.  If the paths-to-be-reversed have one-way streets, the reversed paths may have different travel time than the non-reversed paths.  So, if reversing those paths results in more time lost than the path swap saves time, the new path is rejected.
		- In my only concrete test of this algorithm, the algorithm gave a solution within 1.5% of optimal in 0.005 seconds for an eight-city world tour.

# A Note on the GUI

I improved the provided GUI as was reasonable, considering my role on the project was to develop the class structure for representing the geographic location and to implement the path algorithms, but the GUI is still a bit buggy.

If the numbered stop icons do not display properly, try zooming in or out.  This seems to fix the problem and you can go back to the original zoom level.

Also, the markers don't always refresh properly after selection and route calculation.  My only known fix was to restart the application.

# Using this Application

To run the application as provided:

1. Download the file in this repo named "PathFindingApp.tar".
2. Unzip all the files in PathFindingApp.tar into the same directory on your computer.
3. In that directory, run the file named "PathFindingApp.jar".

To inspect and make changes to the provided code, you can:

1. Download all the files in this repo into a new directory on your computer.
2. Make any desired changes.
3. Run the file named "MapApp.java" as a Java application.
	- I assume if you want to make changes to the code, you are comfortable with figuring out how to accomplish this step.

### Loading your Own Data

There are utilities in this repo that will convert a map from Google Maps into a form usable by this program.

Usable data files are plain text files that end in .map in which every line defines a one-way road segment (a start intersection, a stop intersection, a road name, and a road type) in the following form:

[startLat] [startLon] [endLat] [endLon] "[road name]" [roadtype]

Note this means a two-way road between two intersections needs two lines.

To load your own data (map), you have two options:

1. Specify the path of the data in the lower left corner of the GUI. You must do this each time you restart the GUI.
2. Put a new data file in the data/maps/ directory and add the file name to the "mapfiles.list" file in that directory.  This will add the file to the map drop-down menu in the GUI.

# Units of Measure

Times are displayed in minutes, and distances are displayed in miles.

# Starter Code

*The following section shows the UCSD team's notes about the starter code that they provided.*

/-------------------------------------------------------------------------
/ Starter Code and GUI Application for Course 3 in the
/ Java Programming: Object Oriented Design of 
/ Data Structures Specialization:
/
/ Advanced Data Structures in Java
/ https://www.coursera.org/learn/advanced-data-structures
/
/ Authored by UCSD MOOC Team:
/ Mia Minnes, Christine Alvarado, Leo Porter, Alec Brickner
/ and Adam Setters
/
/ Date: 12/16/2015
/-------------------------------------------------------------------------

---------------------------------------------------------[ DESCRIPTION ]--

The files provided are skeleton code, as well as grading previews and 
testing files to be used in completing the course programming 
assignments. Additionally, you are provided a runnable JavaFX program 
which will help to test and demonstrate your implementations.

-------------------------------------------------------[ FILES BY WEEK ]--

Below are the files introduced in each week and used in each week
of the course. See file for description...

Week 1 : Introduction to the course and graphs
==============================================
basicgraph.Graph.java
basicgraph.GraphAdjList.java
basicgraph.GraphAdjMatrix.java

Week 2 : Class design and simple graph search
==================================================
roadgraph.MapGraph.java
week2example.Maze.java
week2example.MazeLoader.java
week2example.MazeNode.java

Utility files
=============
geography.GeographicPoint.java
geography.RoadSegment.java
util.GraphLoader.java

---------------------------------------------------------------[ SETUP ]-- 

Importing Project into eclipse:
	1. Create a new Java Project in your workspace
	2. Import the starter files:
	  File -> Import -> Select "File System" -> Next -> Browse and set 
	  root directory to folder contents of zip were extracted to -> Finish

Feel free to use another IDE or manually compile and run your programs.
If you need help, google is your friend.

# License

This software is licensed under the [MIT license](https://opensource.org/licenses/MIT).  See the filed named "LICENSE" in this repo for more info.
