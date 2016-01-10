package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/** A class that represents a node in a graph that represents a geographic location. 
 * 
 * @author ryanwilliamconnor
 */
public class MapIntersection extends GeographicPoint {
	// the adjacency list
	private List<MapEdge> neighbors;
	
	/** 
	 * Create a new MapIntersection.
	 * A MapIntersection is essentially a Double (a point in a 2D plane represented
	 * by double types), but with an adjacency list.
	 * To save memory, does not instantiate a null adjacency list upon creation.
	 * However, there are probably few or no intersections that do not
	 * connect to another intersection by a road, so in practice
	 * this probably does not save much memory.
	 * @param latitude The X coordinate / latitude value of the node
	 * @param longitude The Y coordinate / longitude value of the node
	 */
	public MapIntersection(double latitude, double longitude) {

		super(latitude, longitude);
		neighbors = new ArrayList<MapEdge>();
	}
	
	/**
	 * Get the list of intersections reachable from this intersection by a road.
	 * @return The list of out neighbors of the node.
	 */
	public List<MapEdge> getNeighbors() {
		
		return neighbors;
	}
	
}
