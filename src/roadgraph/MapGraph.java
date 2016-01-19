package roadgraph;


import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team
 * @author ryanwilliamconnor
 * 
 * This class represents a geographic location as a directed graph.
 * Intersections between roads are nodes in the graph, and nodes are
 * represented by a custom MapIntersection object.
 * A MapIntersection object contains an intersection's geographic coordinates
 * and a list of other intersections that are reachable by a road. 
 * So, this graph uses an adjacency list representation of edges.
 * The nodes are stored for easy access by coordinates in a hashmap of hashmaps.
 */
public class MapGraph {
	
	private int numVertices;
	private int numEdges;
	
	// the set of unique geographic coordinates in the graph
	private Set<GeographicPoint> nodeCoords;
	
	// store nodes in a hashmap of hashmaps to save storage space
	// and access in constant time by geographic coordinate
	private HashMap<Double,HashMap<Double,MapIntersection>> graph;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		numVertices = 0;
		numEdges = 0;
		
		// could change initial capacity of "graph" based on how likely it is
		// that intersections will share a Double type X coordinate
		// e.g., could be likely if several long, major roads
		// exactly followed a line of longitude
		graph = new HashMap<Double,HashMap<Double,MapIntersection>>();
		nodeCoords = new HashSet<GeographicPoint>(numVertices);
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the coordinates of the intersections,
	 * which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return nodeCoords;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location == null){
			
			return false;
		}
		else if ( (graph.get(location.x) != null) &&
				  (graph.get(location.x).get(location.y) != null) ) {

			return false;
		}
		else {
			// create a new node with the location coordinates
			MapIntersection node = new MapIntersection(location.x, location.y);
			// declare a variable to hold the map from Y coord (given X coord) to node
			HashMap<Double,MapIntersection> nodes;
			
			if ( graph.get(location.x) != null ) {
				// if X coord part of a node in the graph, get X coord's map of Y coords to nodes
				nodes = graph.get(location.x);
				// put the Y coord in the X's coord's map as a key to the new node
				nodes.put((Double)location.y, node);
				nodeCoords.add(location);
			}
			else {
				// if not, create a new map first
				nodes = new HashMap<Double,MapIntersection>(4);
				// put the Y coord in the X coord's map
				nodes.put((Double)location.y, node);
				// and put the X coord's map in the graph
				graph.put((Double)location.x, nodes);
				nodeCoords.add(location);
			}
			
			numVertices++;
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		// catch all the illegal argument exceptions
		if ( (graph.get(from.x) == null) ||
			 (graph.get(from.x).get(from.y) == null) ) {
			throw new IllegalArgumentException(from.toString());
		} else if ( (graph.get(to.x) == null) ||
				    (graph.get(to.x).get(to.y) == null) ) {		
			throw new IllegalArgumentException(to.toString());
		} else if (roadName == null) {
			throw new IllegalArgumentException(roadName);
		} else if (roadType == null) {
			throw new IllegalArgumentException(roadType);
		} else if (length < 0) {
			throw new IllegalArgumentException(String.valueOf(length));
		}
		
		// get the two nodes
		MapIntersection fromNode = graph.get(from.x).get(from.y);
		MapIntersection toNode = graph.get(to.x).get(to.y);
		
		// create the MapEdge
		MapEdge edgeToAdd = new MapEdge(fromNode, toNode,
				                        roadName, roadType, length);
		
		// add "to" to "from"'s adjacency list
		fromNode.getNeighbors().add(edgeToAdd);
		numEdges++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public PathObject bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public PathObject bfs(GeographicPoint start, 
			 					     GeographicPoint goal,
			 					     Consumer<GeographicPoint> nodeSearched)
	{	
		/* NOTE: I think it is confusing to call this method "bfs" because breadth-first
		   search is the name of a search strategy, not the name of the thing
		   we want to do. I would think about renaming the method to something like
		   "getShortestPath", but this is not allowed by the specifications. */
		
		// if the start or goal do not exist in the graph, return null
		if ( (graph.get(start.x) == null) ||
				  (graph.get(start.x).get(start.y) == null) ) {
			return null;
		}
		else if ( (graph.get(goal.x) == null) ||
				  (graph.get(goal.x).get(goal.y) == null) ) {
			return null;
		}
		
		// assign the start and goal nodes
		MapIntersection startNode = graph.get(start.x).get(start.y);
		MapIntersection goalNode = graph.get(goal.x).get(goal.y);

		// initialize or declare data structures for: nodes to process, nodes processed,
		// processed node parents, current node, current neighbors, and shortest path
		ArrayDeque<MapIntersection> toProcess = new ArrayDeque<MapIntersection>();
		HashSet<MapIntersection> visited = new HashSet<MapIntersection>(numVertices*2);
		HashMap<MapIntersection,MapIntersection> parents =
				new HashMap<MapIntersection,MapIntersection>(numVertices*2);
		MapIntersection currentNode;
		List<MapEdge> currentNeighbors;
		
		// keep track of parents while doing BFS
		toProcess.add(startNode);
		visited.add(startNode);
		while (!toProcess.isEmpty()) {
			
			currentNode = toProcess.remove();
			nodeSearched.accept(currentNode);
			
			if (currentNode.equals(goalNode)) {
				// call the helper method to return shortest path
				return codifyShortestPath(startNode, goalNode, parents);
			}
				
			currentNeighbors = currentNode.getNeighbors();
			for (MapEdge neighbor : currentNeighbors) {
					
				if (!visited.contains(neighbor.getToIntersection())){
					
					visited.add(neighbor.getToIntersection());
					toProcess.add(neighbor.getToIntersection());
					parents.put(neighbor.getToIntersection(), currentNode);
				}
			}
		}
		
		return null;
	}
	
	/** Codify the shortest path from start to goal using "parents" of each node on
	 * the BFS path from goal to start
	 * 
	 * @param startNode The starting location
	 * @param goalNode The goal location
	 * @param parents The map of nodes to the node that
	 * "discovered" it during BFS
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both startNode and goalNode).
	 */
	private PathObject codifyShortestPath(MapIntersection startNode,
							MapIntersection goalNode, 
							HashMap<MapIntersection,MapIntersection> parents) {
		// create the return value and add the goalNode to it
		List<GeographicPoint> shortestPath = new ArrayList<GeographicPoint>(numVertices);
		shortestPath.add(goalNode);
		
		GeographicPoint currentNode;
		GeographicPoint parentNode;
		int pathHop = 0;
		double length = 0;
		
		/* while the last node added to the path is not the startNode,
		   add the parent of the node at the current path "hop" to the path */
		while (!shortestPath.get(shortestPath.size()-1).equals(startNode)) {
			
			currentNode = shortestPath.get(pathHop);
			parentNode = parents.get(currentNode);
			shortestPath.add(parentNode);
			length += parentNode.distance(currentNode);
			pathHop++;
		}
		// reverse the path so startNode is first and goalNode is last
		Collections.reverse(shortestPath);
		PathObject path = new PathObject(shortestPath, length);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public PathObject dijkstra(GeographicPoint start, 
							   GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public PathObject dijkstra(GeographicPoint start, 
							   GeographicPoint goal, 
							   Consumer<GeographicPoint> nodeSearched) {
		// if the start or goal do not exist in the graph, return null
		if ( (graph.get(start.x) == null) ||
				  (graph.get(start.x).get(start.y) == null) ) {
			return null;
		}
		else if ( (graph.get(goal.x) == null) ||
				  (graph.get(goal.x).get(goal.y) == null) ) {
			return null;
		}
		
		// assign the start and goal nodes
		MapIntersection startNode = graph.get(start.x).get(start.y);
		MapIntersection goalNode = graph.get(goal.x).get(goal.y);

		// initialize the priority queue to hold distances
		// of particular paths from startNodes to other nodes.
		// the priority queue holds hashmaps of "nodes" -> "distance from 
		// start node following a unique path" priority is based on the
		// following custom comparator: paths with shortest distance to
		// start node go first.
		PriorityQueue<HashMap<MapIntersection, Double>> toProcess = 
			new PriorityQueue<HashMap<MapIntersection, Double>>(10,
				new Comparator<HashMap<MapIntersection, Double>>() {
					public int compare(HashMap<MapIntersection, Double> mapOne, 
						HashMap<MapIntersection, Double> mapTwo) {
							return ((Double)mapOne.get((MapIntersection)mapOne.keySet().toArray()[0])).compareTo((Double)mapTwo.get((MapIntersection)mapTwo.keySet().toArray()[0]));
						}
				});
		
		// initialize or declare data structures for: nodes processed, 
		// processed node parents, current node, current neighbors, current 
		// best distances to nodes, and shortest path to goal
		HashSet<MapIntersection> visited = 
				new HashSet<MapIntersection>(numVertices*2);
		HashMap<MapIntersection,MapIntersection> parents =
				new HashMap<MapIntersection,MapIntersection>(numVertices*2);
		HashMap<MapIntersection, Double> distances =
				new HashMap<MapIntersection, Double>(numVertices*2);
		HashMap<MapIntersection,Double> currentNodeWithDist;
		MapIntersection currentNode;
		List<MapEdge> currentNeighbors;
		
		// initialize distance to startNode as 0
		// and distance to all other nodes as infinity
		Set<GeographicPoint> allCoordinates = getVertices();
		for (GeographicPoint coordinate : allCoordinates) {
			if (coordinate.x == start.x && coordinate.y == start.y) {
				distances.put(graph.get(coordinate.x).get(coordinate.y), 
						      0.0);
			}
			else {
				distances.put(graph.get(coordinate.x).get(coordinate.y),
						      Double.POSITIVE_INFINITY);
			}
		}
		// keep track of parents while doing Dijkstra BFS with a priority queue
		currentNodeWithDist = new HashMap<MapIntersection, Double>();
		currentNodeWithDist.put(startNode, distances.get(startNode));
		toProcess.add(currentNodeWithDist);
		
		// need to change the comparable interface for toProcess to order by
		// the value of the key of the hashmap (ascending);
		
		while (!toProcess.isEmpty()) {
			
			currentNodeWithDist = toProcess.remove();
			currentNode = (MapIntersection)currentNodeWithDist.keySet().toArray()[0];
			nodeSearched.accept(currentNode);
			
			if (!visited.contains(currentNodeWithDist)) {
				
				visited.add(currentNode);
				
				if (currentNode.equals(goalNode)) {
					// call the helper method to return shortest path
					return codifyShortestPath(startNode, goalNode, parents);
				}
				
				currentNeighbors = currentNode.getNeighbors();
				for (MapEdge neighbor : currentNeighbors) {
					// initialize values to make the logic easier to read
					MapIntersection toIntersection = neighbor.getToIntersection();
					double potentialDistance = 
							distances.get(currentNode) + neighbor.getTravelTime();
					double currentDistance = distances.get(toIntersection);
					
					if (!visited.contains(toIntersection)) {
						
						if (potentialDistance < currentDistance) {
							
							distances.put(toIntersection, potentialDistance);
							parents.put(neighbor.getToIntersection(), currentNode);
							HashMap<MapIntersection, Double> nextNodeWithDist = 
									new HashMap<MapIntersection, Double>();
							nextNodeWithDist.put(toIntersection, potentialDistance);
							toProcess.add(nextNodeWithDist);
						}
					}
				}
			}
		}
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public PathObject aStarSearch(GeographicPoint start, 
								  GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public PathObject aStarSearch(GeographicPoint start, 
								  GeographicPoint goal,
								  Consumer<GeographicPoint> nodeSearched) {
		// if the start or goal do not exist in the graph, return null
		if ( (graph.get(start.x) == null) ||
				  (graph.get(start.x).get(start.y) == null) ) {
			return null;
		}
		else if ( (graph.get(goal.x) == null) ||
				  (graph.get(goal.x).get(goal.y) == null) ) {
			return null;
		}
		
		// assign the start and goal nodes
		MapIntersection startNode = graph.get(start.x).get(start.y);
		MapIntersection goalNode = graph.get(goal.x).get(goal.y);

		// initialize the priority queue to hold distances
		// of particular paths from startNodes to other nodes.
		// the priority queue holds hashmaps of "nodes" -> "distance from 
		// start node following a unique path"
		// priority is based on the following custom comparator:
		// paths with shortest distance to start node go first.
		PriorityQueue<HashMap<MapIntersection, Double>> toProcess = 
				new PriorityQueue<HashMap<MapIntersection, Double>>(10, new Comparator<HashMap<MapIntersection, Double>>() {
					public int compare(HashMap<MapIntersection, Double> mapOne, HashMap<MapIntersection, Double> mapTwo) {
						return ((Double)mapOne.get((MapIntersection)mapOne.keySet().toArray()[0])).compareTo((Double)mapTwo.get((MapIntersection)mapTwo.keySet().toArray()[0]));
					}
				});
		
		// initialize or declare data structures for: nodes processed, processed node parents,
		// current node, current neighbors, current best distances to nodes, and shortest path to goal
		HashSet<MapIntersection> visited = new HashSet<MapIntersection>(numVertices*2);
		HashMap<MapIntersection,MapIntersection> parents =
				new HashMap<MapIntersection,MapIntersection>(numVertices*2);
		HashMap<MapIntersection, Double> distances = new HashMap<MapIntersection, Double>(numVertices*2);
		HashMap<MapIntersection,Double> currentNodeWithDist;
		MapIntersection currentNode;
		List<MapEdge> currentNeighbors;
		
		// initialize distance to startNode as 0
		// and distance to all other nodes as infinity
		Set<GeographicPoint> allCoordinates = getVertices();
		for (GeographicPoint coordinate : allCoordinates) {
			if (coordinate.x == start.x && coordinate.y == start.y) {
				distances.put(graph.get(coordinate.x).get(coordinate.y), 
						      0.0);
			}
			else {
				distances.put(graph.get(coordinate.x).get(coordinate.y),
						      Double.POSITIVE_INFINITY);
			}
		}
		// keep track of parents while doing a Dijkstra BFS with a priority queue
		currentNodeWithDist = new HashMap<MapIntersection, Double>();
		currentNodeWithDist.put(startNode, distances.get(startNode));
		toProcess.add(currentNodeWithDist);
		
		// need to change the comparable interface for toProcess to order by
		// the value of the key of the hashmap (ascending);
		
		while (!toProcess.isEmpty()) {
			
			currentNodeWithDist = toProcess.remove();
			currentNode = (MapIntersection)currentNodeWithDist.keySet().toArray()[0];
			nodeSearched.accept(currentNode);
			
			if (!visited.contains(currentNodeWithDist)) {
				
				visited.add(currentNode);
				
				if (currentNode.equals(goalNode)) {
					// call the helper method to return shortest path
					return codifyShortestPath(startNode, goalNode, parents);
				}
				
				currentNeighbors = currentNode.getNeighbors();
				for (MapEdge neighbor : currentNeighbors) {

					MapIntersection toIntersection = neighbor.getToIntersection();
					
					if (!visited.contains(toIntersection)) {
						// initialize values to make the logic easier to read
						double pathFromStartNode = 
								distances.get(currentNode) + neighbor.getTravelTime();
						double asCrowFliesToGoalDist = toIntersection.distance(goalNode);
						double aStarHeurDist = pathFromStartNode + asCrowFliesToGoalDist;
						double currentDist = distances.get(toIntersection);
						
						if (aStarHeurDist < currentDist) {
							
							distances.put(toIntersection, aStarHeurDist);
							parents.put(neighbor.getToIntersection(), currentNode);
							HashMap<MapIntersection, Double> nextNodeWithDist = 
									new HashMap<MapIntersection, Double>();
							nextNodeWithDist.put(toIntersection, aStarHeurDist);
							toProcess.add(nextNodeWithDist);
						}
					}
				}
			}
		}
		return null;
	}

	/** Return shortest path from a start vertex back to that vertex.
	 * 
	 * Each vertex in the path can only be visited once, and each edge 
	 * in the cycle can only be used once.  So, this method solves the 
	 * Traveling Salesperson Problem.
	 * 
	 * @param start: The starting (and ending) location
	 * @param stops: A list of locations to visit goal
	 * @param nodeSearched: A hook for visualization.
	 * @return The list of locations that form the shortest path from 
	 *   start to start while visiting each location in stops exactly once
	 *   and using any edges in the graph no more than once.
	 */
	public PathObject calulateShortestCycle(GeographicPoint start, 
								List<GeographicPoint> stops,
								Consumer<GeographicPoint> nodeSearched) {
		
		// if the start or any stops do not exist in the graph, return null
		if (!nodeCoords.contains(start)) {
			return null;
		}
		
		for (GeographicPoint stop : stops) {
			if (!nodeCoords.contains(stop)) {
				return null;
			}
			if (stop.equals(start)) {
				throw new IllegalArgumentException();
			}
		}
		
		List<GeographicPoint> shortestCycle = 
				new ArrayList<GeographicPoint>(stops.size()+2);
		List<MapIntersection> toVisit = 
				new ArrayList<MapIntersection>(stops.size());
		MapIntersection current;
		MapIntersection bestNext = null;
		PathObject shortestPathToNext = null;
		double potentialNextTravelTime = 0;
		PathObject pathToNext;
		double totalTravelTime = 0;
		
		if (start instanceof MapIntersection) {
			current = (MapIntersection)start;
		}
		else {
			current = graph.get(start.x).get(start.y);
		}
			
		if (stops.get(0) instanceof MapIntersection) {
			for (GeographicPoint stop : stops) {
				toVisit.add((MapIntersection)stop);
			}
		}
		else {
			for (GeographicPoint stop : stops) {
				toVisit.add(graph.get(stop.x).get(stop.y));
			}
		}
		
		shortestCycle.add(current);
		
		while (toVisit.size() > 1) {
			
			// tspAStarSearch(toVisit, current);
			
			double bestNextTravelTime = Double.POSITIVE_INFINITY;
			
			// greedily go to closet vertex from the vertices left visit
			for (MapIntersection potentialNext : toVisit) {
					
				pathToNext = aStarSearch(current, potentialNext);
					
				if (pathToNext.getLength() < 
						bestNextTravelTime) {
						
					bestNextTravelTime = potentialNextTravelTime;
					bestNext = potentialNext;
					shortestPathToNext = pathToNext;
				}
			}
			// add each hop except first (first is already added)
			for (int i = 1; i < shortestPathToNext.getPath().size(); i++) {
				
				shortestCycle.add(shortestPathToNext.getPath().get(i));
			}
			totalTravelTime += bestNextTravelTime;
			toVisit.remove(bestNext);
			current = bestNext;
		}
 
		shortestPathToNext = aStarSearch(current, start);
		
		totalTravelTime += shortestPathToNext.getLength();
		
		for (int i = 0; i < shortestPathToNext.getPath().size()-1; i++) {

			shortestCycle.add(shortestPathToNext.getPath().get(i));
		}
		shortestCycle.add(start);
		
		PathObject cycle = new PathObject(shortestCycle, totalTravelTime);
		return cycle;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
	}
	
}
