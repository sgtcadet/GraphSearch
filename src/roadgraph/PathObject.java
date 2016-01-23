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
	// having the list of nodes is redundant if you have the list of 
	// roads taken, but I needed to play nice with the "grader".
	private List<MapEdge> roadsTaken;
	private double length;
	private double travelTime;
	
	public PathObject(List<GeographicPoint> path, 
					  List<MapEdge> roadsTaken,
					  double length, 
					  double travelTime) {
		
		this.path = path;
		this.roadsTaken = roadsTaken;
		this.length = length;
		this.travelTime = travelTime;
	}
	
	public List<GeographicPoint> getPath() {
		
		return path;
	}
	
	public List<MapEdge> getRoadsTaken() {
		
		return roadsTaken;
	}
	
	public double getLength() {
		
		return length;
	}
	
	public void setLength(double length) {
		
		this.length = length;
	}
	
	public double getTravelTime() {
		
		return travelTime;
	}
	
	public void setTravelTime(double travelTime) {
		
		this.travelTime = travelTime;
	}
}
