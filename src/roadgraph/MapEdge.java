package roadgraph;

/** A class that represents a road in a graph that represents a geographic location. 
 * 
 * @author ryanwilliamconnor
 */
public class MapEdge {

		private MapIntersection fromIntersection;
		private MapIntersection toIntersection;
		private String roadName;
		private String roadType;
		private double length;
		private double speedLimit;
		private double travelTime;
		
		public MapEdge(MapIntersection fromIntersection,
				       MapIntersection toIntersection,
				       String roadName, String roadType, double length) {
			
			this.fromIntersection = fromIntersection;
			this.toIntersection = toIntersection;
			this.roadName = roadName;
			this.roadType = roadType;
			this.length = length;
			
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
			else {
				this.speedLimit = 40.0;
				System.out.println("Road " + roadName + " has a strange roadType. " +
								   "This may make results innacurate");
			}
			
			setTravelTime(speedLimit);
		}
		
		// to support changing the speed limit on this road
		public void setTravelTime(double speedLimit) {
			
			this.travelTime = length / speedLimit;
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
