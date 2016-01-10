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
		
		public MapEdge(MapIntersection fromIntersection,
				       MapIntersection toIntersection,
				       String roadName, String roadType, double length) {
			
			this.fromIntersection = fromIntersection;
			this.toIntersection = toIntersection;
			this.roadName = roadName;
			this.roadType = roadType;
			this.length = length;
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
}
