package roadgraph;

import java.util.List;

import geography.GeographicPoint;

/** A class that represents a node in a graph of a road map. 
 * 
 * @author ryanwilliamconnor
 */
public class MapIntersection extends GeographicPoint {

	List<MapIntersection> neighbors;
	
	public MapIntersection(double latitude, double longitude) {
		
		super(latitude, longitude);
	}
	
	public List<MapIntersection> getNeighbors() {
		
		return neighbors;
	}
}
