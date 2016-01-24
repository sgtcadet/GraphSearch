package roadgraph;

/** A class that represents a road in a graph that represents a geographic location. 
 * 
 * Holds length in miles, speed limit in miles per hour, and travel time in minutes.
 * Constructor assumes length is given in kilometers.
 * 
 * Defaults to a speed limit of 40 MPH if the road type is not recognized.
 * 
 * @author ryanwilliamconnor
 */
public class MapEdge {

		private MapIntersection fromIntersection;
		private MapIntersection toIntersection;
		private String roadName;
		private String roadType;
		private double length; // in miles
		private double speedLimit; // in miles per hour
		private double travelTime; // in seconds
		
		public MapEdge(MapIntersection fromIntersection,
				       MapIntersection toIntersection,
				       String roadName, String roadType, double length) {
			
			this.fromIntersection = fromIntersection;
			this.toIntersection = toIntersection;
			this.roadName = roadName;
			this.roadType = roadType;
			this.length = length*0.621; // assumes length is given in kilometers
			
			if (roadType.equals("motorway")) {
				this.speedLimit = 70.0;
			}
			else if (roadType.equals("trunk")) {
				this.speedLimit = 60.0;
			}
			else if (roadType.equals("primary")) {
				this.speedLimit = 55.0;
			}
			else if (roadType.equals("secondary")) {
				this.speedLimit = 45.0;
			}
			else if (roadType.equals("tertiary")) {
				this.speedLimit = 40.0;
			}
			else if (roadType.equals("unclassified")) {
				this.speedLimit = 35.0;
			}
			else if (roadType.equals("residential")) {
				this.speedLimit = 25.0;
			}
			else if (roadType.equals("service")) {
				this.speedLimit = 15.0;
			}
			else if (roadType.equals("flight")) {
				this.speedLimit = 550.0;
			}
			else {
				this.speedLimit = 40.0;
				//System.out.println("Road " + roadName + " has a strange roadType. " +
					//			   "This may make results innacurate");
			}
			
			setTravelTime(this.speedLimit);
			//System.out.println("Length of " + this.roadName + " is " + this.length);
			//System.out.println("Travel time " + this.roadName + " is " + travelTime);
		}
		
		// to support changing the speed limit on this road
		public void setTravelTime(double speedLimit) {
			
			this.travelTime = (length / speedLimit)*60; // gives time in minutes
		}
		
		public MapIntersection getFromIntersection() {
			return fromIntersection;
		}
		
		public MapIntersection getToIntersection() {
			return toIntersection;
		}
		
		public String getRoadName() {
			return roadName;
		}
		
		public String getRoadType() {
			return roadType;
		}
		
		public double getLength() {
			return length;
		}
		
		public double getTravelTime() {
			return travelTime;
		}
}
