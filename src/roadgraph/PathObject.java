package roadgraph;

import java.util.List;

/** A class that represents information about a path in a geographic map. 
 * 
 * Useful for returning multiple data types from graph search methods.
 * 
 * @author ryanwilliamconnor
 */
import geography.GeographicPoint;

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
