package roadgraph;


import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team
 * @author ryanwilliamconnor
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between roads
 *
 */
public class MapGraph {
	
	private int numVertices;
	private int numEdges;
	
	private Set<GeographicPoint> nodes;
	
	private HashMap<Double,HashMap<Double,MapIntersection>> graph;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		numVertices = 0;
		numEdges = 0;
		graph = new HashMap<Double,HashMap<Double,MapIntersection>>(4);
		nodes = new HashSet<GeographicPoint>(numVertices);
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
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		// returns set of GeographicPoint as specified by requirements
		// does not return set of MapIntersection
		Set<Double> uniqueXCoords = graph.keySet();
		
		Set<Double> currXYCoords;
		for (Double xCoord : uniqueXCoords) {
			
			currXYCoords = graph.get(xCoord).keySet();
			for (Double yCoord : currXYCoords) {
				nodes.add(graph.get(xCoord).get(yCoord));
			}
		}
		
		return nodes;
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
			
			MapIntersection node = new MapIntersection(location.x, location.y);
			
			HashMap<Double,MapIntersection> nodes;
			
			if ( graph.get(location.x) != null ) {
				
				nodes = graph.get(location.x);
			}
			else {
				
				nodes = new HashMap<Double,MapIntersection>(4);
			}
			
			nodes.put((Double)location.y, node);
			graph.put((Double)location.x, nodes);
			
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
		
		MapIntersection fromNode = graph.get(from.x).get(from.y);
		MapIntersection toNode = graph.get(to.x).get(to.y);
		
		// if "from"'s adjacency list has not been created, create it.
		if (fromNode.neighbors == null) {
			fromNode.neighbors = new ArrayList<MapIntersection>();
		}
		
		fromNode.neighbors.add(toNode);
		numEdges++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
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
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	
		// It is a bit confusing to call this method "bfs" because breadth-first
		// search is the name of a search strategy, not the name of the thing
		// we want to do. I suggest renaming the method to something like
		// "getShortestPath", which could implement a helper method called "bfs"
		// if that's the search strategy we want to use.
		
		//nodeSearched.accept(next.getLocation());
		
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

		// hold nodes to process, nodes processed, processed node parents,
		// current node, current neighbors, and shortest path
		ArrayDeque<MapIntersection> toProcess = new ArrayDeque<MapIntersection>();
		HashSet<MapIntersection> visited = new HashSet<MapIntersection>(numVertices*2);
		HashMap<MapIntersection,MapIntersection> parents =
				new HashMap<MapIntersection,MapIntersection>(numVertices*2);
		MapIntersection currentNode;
		List<MapIntersection> currentNeighbors;
		
		toProcess.add(startNode);
		visited.add(startNode);
		
		// keep track of parents while doing BFS
		while (!toProcess.isEmpty()) {
			
			currentNode = toProcess.remove();
			
			if (currentNode.equals(goalNode)) {
				return codifyShortestPath(startNode, goalNode, parents);
			}
			
			if (currentNode.neighbors != null) {
				
				currentNeighbors = currentNode.neighbors;
				for (MapIntersection neighbor : currentNeighbors) {
					
					if (!visited.contains(neighbor)){
					
						visited.add(neighbor);
						toProcess.add(neighbor);
						parents.put(neighbor, currentNode);
					}
				}
			}
		}
		
		return null;
	}
	
	/** Codify the shortest path from start to goal using parents of each node on
	 * the BFS path from start to goal
	 * 
	 * @param startNode The starting location
	 * @param goalNode The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both startNode and goalNode).
	 */
	private List<GeographicPoint> codifyShortestPath(MapIntersection startNode,
			MapIntersection goalNode, HashMap<MapIntersection,MapIntersection> parents) {
		
		List<GeographicPoint> shortestPath = new ArrayList<GeographicPoint>(numVertices);
		
		GeographicPoint currentParent;
		
		shortestPath.add((GeographicPoint)startNode);
		
		int pathHop = 0;
		
		while (!shortestPath.get(shortestPath.size()-1).equals(goalNode)) {
			
			currentParent = shortestPath.get(pathHop);
			shortestPath.add(parents.get(currentParent));
			pathHop++;
		}
		
		return shortestPath;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
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
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
