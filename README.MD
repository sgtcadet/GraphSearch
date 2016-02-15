# Overview

This application is a re-creation of Google Maps' path finding capabilities.  Its GUI component interfaces with Google Maps.

This application will show and tell you the shortest path from a start intersection to up to seven other intersections traveled in order.  Times are displayed in minutes, and distances are displayed in miles.

This application will also show and tell you a path that is very close to the shortest path from a start intersection that visits up to seven other intersections exactly once and ends back at the starting location (the Traveling Salesperson Problem (TSP)).

# Purpose of this Application

The purpose of this application is to 1) give a user the shortest path, in terms of trip duration, from a start location to up to seven other locations (and optionally back to the start location) and 2) demonstrate the differences between various graph search algorithms.

The user can select a pre-loaded map, load their own map, select a start location, select up to seven stop locations, and select one of five pathing algorithms.  The GUI will then visually display the shortest path calculated with the selected algorithm, visualize the number the stops, and show some stats about the path.

## Meaning of "Shortest Path" in this Application

"Shortest path" in this application means "the route from one intersection to another that minimizes trip duration for a motor vehicle".

Time traveled on a road is determined solely by the length and speed limit of that road.  So, time traveled does NOT currently take into account variables like "current traffic" or "time of day" (which might in turn influence a "projected traffic" variable).

The speed limit for a road type is as you might expect for the average location in the USA: For example, major highways have a speed limit of 70 miles per hour and residential roads have a speed limit of 25 miles per hour.

# Technologies Used

The core of the program, including the graph data structures and path finding algorithms, is implemented in Java.

The GUI was built mainly in Java using JavaFX and GMapsFX, with a little HTML, JavaScript, and CSS.

# My Work on This Project

The initial commit for this repository is the starter code as described in the section below.  All subsequent commits are my (Ryan Connor’s) work.  I:

1. Designed and implemented the class structure for representing roads in a geographic location as a graph.
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
		- First, Greedy2OptTSP finds a cycle using a purely greedy algorithm based on A* search from the current location to each not-yet-visited stop.
		- Then, Greedy2OptTSP attempts to shorten the cycle by deleting pairs of paths (S1, S2), (S3, S4) between stops and rebuilding the cycle with new paths (S1, S3), (S2, S4).  If this results in a shorter path, the process is re-started with this better path.  This is what makes this 2OptTSP algorithm "greedy" -- to save computational time, the algorithm restarts the swapping process immediately upon finding a shorter cycle instead of "remembering" the shorter cycle and continuing through all swap possibilities for the current cycle.
			- Re-building the cycle takes one-way streets into account.  That is, swapping path pair (S1, S2), (S3, S4) out for new paths (S1, S3), (S2, S4) means some other existing paths must be reversed.  If the paths-to-be-reversed have one-way streets, the reversed paths may have different travel time than the non-reversed paths.  So, if reversing those paths results in more time lost than the path swap saves time, the new path is rejected.
		- In my only concrete test of this algorithm, the algorithm gave a solution within 1.5% of optimal in 0.005 seconds for an eight-city world tour.

__Visual comparison of the search strategies for the BFS and A* algorithms run with the same start and stop (a red icon marks an intersection visited by the algorithm):__

BFS:
![BFS](https://raw.githubusercontent.com/ToolsForInsight/GraphSearch/master/BFS%20Viz.png)

A*:
![A*](https://raw.githubusercontent.com/ToolsForInsight/GraphSearch/master/aStar%20Viz.png)

__Visual comparison of the TSP algorithms run with the same eight-location cycle:__

GreedyTSP:
![GreedyTSPNYC](https://raw.githubusercontent.com/ToolsForInsight/GraphSearch/master/GreedyTSP%20NYC.png)

Greedy2OptTSP:
![Greedy2OPTTSPNYC](https://raw.githubusercontent.com/ToolsForInsight/GraphSearch/master/2OptTSP%20NYC.png)

# Using this Application

**Running the Application**

At this time, the easiest way to run the application is to:

1. Download all the files in this repo into a new directory on your computer.
2. Import the the directory as a project into the [Eclipse IDE](http://www.eclipse.org/) (link provided in case you need to download Eclipse).
3. Run the file named "MapApp.java" as a Java application.

If you want to use another IDE or compile the program manually, Google is your friend.

I tried to make a standalone application by exporting the project as a JAR from Eclipse, but the Google Map does not load into the JAR.  I have contact the UCSD development team and scoured the internet about this issue to no avail.  Please see my [question on stackoverflow.com](http://stackoverflow.com/questions/35341779/google-maps-api-works-in-eclipse-not-when-exported-as-a-jar) if you are interested in helping solve this issue.  In fact, at the time of this writing, this question is the #1 hit on Google (!!) for the search term "javafx google map does +not work in JAR".  I would love it if you [emailed me](mailto:ryanwc13@gmail.com) with a solution.

**Using the GUI**

After the application opens, you will see an interactive Google Map centered on UCSD.

*To use the path search features:*

1. Load a map from the drop down menu in the top right or fetch a new map by inputting the file name in lower left.
2. Select a start location and click the "Start" button on the "Routing" tab.
3. Select a stop and click the relevant "Stop" button on the "Routing" tab.  Do this for up to seven stops.
4. Select one of the five path search strategies.
5. Click the "Show Route" button.

Once you have a route, you can click "Hide Route" to clear the route and select a new algorithm.

*To visualize the path search algorithm:*

After showing a route, click the "Start Visualization" button to see the intersections the algorithm visits on its way to determining the shortest route to each stop.

This feature does not currently work with the TSP algorithms.

**Final Notes on the GUI**

I improved the provided GUI as was reasonable, considering my role on the project was to develop the class structure for representing the geographic location and to implement the path algorithms, but the GUI is still a bit buggy.

If the numbered stop icons do not display properly, try zooming in or out.  This seems to fix the problem and you can go back to the original zoom level.

Also, the map doesn't always refresh properly after marker selection and route calculation.  Currently, there is no "reset" button (I'll add that if I have time), so if the map becomes unusable, you can load another map or restart the application.

### Loading your Own Data

There are utilities in this repo that will convert a map from Google Maps into a form usable by this program.

Usable data files are plain text files that end in .map in which every line defines a one-way road segment (a start intersection, a stop intersection, a road name, and a road type) in the following form:

[startLat] [startLon] [endLat] [endLon] "[road name]" [roadtype]

Note this means a two-way road between two intersections needs two lines.

To load your own data (map), you have two options:

1. Specify the path of the data in the lower left corner of the GUI. You must do this each time you restart the GUI.
2. Put a new data file in the data/maps/ directory and add the file name to the "mapfiles.list" file in that directory.  This will add the file to the map drop-down menu in the GUI.

# Starter Code

*The following section shows the UCSD team's notes about the starter code that they provided.  The starter code is the initial commit of this repo.*

**Starter Code and GUI Application for Course 3 in the Java Programming: Object Oriented Design of Data Structures Specialization: Advanced Data Structures in Java**

https://www.coursera.org/learn/advanced-data-structures

Authored by UCSD MOOC Team:
Mia Minnes, Christine Alvarado, Leo Porter, Alec Brickner and Adam Setters

Date: 12/16/2015

**DESCRIPTION**

The files provided are skeleton code, as well as grading previews and testing files to be used in completing the course programming assignments. Additionally, you are provided a runnable JavaFX program which will help to test and demonstrate your implementations.

**FILES BY WEEK**

Below are the files introduced in each week and used in each week
of the course. See file for description...

*Week 1 : Introduction to the course and graphs*

basicgraph.Graph.java
basicgraph.GraphAdjList.java
basicgraph.GraphAdjMatrix.java

*Week 2 : Class design and simple graph search*

roadgraph.MapGraph.java
week2example.Maze.java
week2example.MazeLoader.java
week2example.MazeNode.java

*Utility files*

geography.GeographicPoint.java
geography.RoadSegment.java
util.GraphLoader.java

**SETUP** 

Importing Project into eclipse:

1. Create a new Java Project in your workspace
2. Import the starter files:
	- File -> Import -> Select "File System" -> Next -> Browse and set root directory to folder contents of zip were extracted to -> Finish

Feel free to use another IDE or manually compile and run your programs.  If you need help, google is your friend.

# License

This software is licensed under the [MIT license](https://opensource.org/licenses/MIT).  See the filed named "LICENSE" in this repo for more info.
