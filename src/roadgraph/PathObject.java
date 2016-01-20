package roadgraph;

import java.util.List;

/** A class that represents information about a path in a geographic map. 
 * 
 * Useful for returning multiple data types from graph search methods.
 * 
 * @author ryanwilliamconnor
 */
import geography.GeographicPoint;

/** A class that represents a path in a graph that represents a geographic location. 
 * Useful for returning multiple data types from graph search methods (e.g., a list
 * of vertices on a path and the path's length).
 * 
 * @author ryanwilliamconnor
 */
public class PathObject {

	private List<GeographicPoint> path;
	private double length;
	
	public PathObject(List<GeographicPoint> path, double length) {
		
		this.path = path;
		this.length = length;
	}
	
	public List<GeographicPoint> getPath() {
		
		return path;
	}
	
	public double getLength() {
		
		return length;
	}
}
