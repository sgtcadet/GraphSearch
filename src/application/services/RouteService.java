package application.services;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.function.Consumer;

import application.CLabel;
import application.DataSet;
import application.MapApp;
import application.MarkerManager;
import application.RouteVisualization;
import application.SelectManager;
import application.controllers.RouteController;

import java.util.Iterator;

import geography.GeographicPoint;
import geography.RoadSegment;
import gmapsfx.GoogleMapView;
import gmapsfx.javascript.object.GoogleMap;
import gmapsfx.javascript.object.LatLong;
import gmapsfx.javascript.object.LatLongBounds;
import gmapsfx.javascript.object.MVCArray;
import gmapsfx.shapes.Polyline;
import gmapsfx.shapes.PolylineOptions;
import javafx.scene.control.Button;
import roadgraph.PathObject;

/** 
 * @author UCSD MOOC development team
 * @author ryanwilliamconnor
 */

public class RouteService {
	private GoogleMap map;

    // static variable
	private SelectManager selectManager;
    private MarkerManager markerManager;
    private Polyline routeLine;
    private String color;
    private RouteVisualization rv;

	public RouteService(GoogleMapView mapComponent, 
						SelectManager selectManager,
						MarkerManager markerManager) {
		this.map = mapComponent.getMap();
		this.selectManager = selectManager;
        this.markerManager = markerManager;

	}
    // COULD SEPARATE INTO ROUTE SERVICES IF CONTROLLER
	// GETS BIG
	// initialize??

	// add route polyline to map
	//DISPLAY ROUTE METHODS
	/**
	 * Displays route on Google Map
	 * @return returns false if route fails to display
	 */
	private boolean displayRoute(List<LatLong> route, String color) {
		
		
		PolylineOptions options = new PolylineOptions();
		options.strokeColor(color);
        if(routeLine != null) {
        	removeRouteLine();
        }
		//routeLine = new Polyline();
        routeLine = new Polyline(options);
		MVCArray path = new MVCArray();
		LatLongBounds bounds = new LatLongBounds();
		for(LatLong point : route)  {
			path.push(point);
            bounds = bounds.extend(point);
		}
		routeLine.setPath(path);

		map.addMapShape(routeLine);

		//System.out.println(bounds.getNorthEast());
		//EXCEPTION getBounds() messed up??
        //System.out.println(routeLine.getBounds());

		
		markerManager.hideIntermediateMarkers();
		map.fitBounds(bounds);
    	markerManager.disableVisButton(false);
		return true;
	}

    public void hideRoute() {
    	if(routeLine != null) {
        	map.removeMapShape(routeLine);
        	if(markerManager.getVisualization() != null) {
        		markerManager.clearVisualization();
        	}
            markerManager.restoreMarkers();
        	markerManager.disableVisButton(true);
            routeLine = null;
    	}
    }

    public void reset() {
    	markerManager.clearMarkers();
        removeRouteLine();
    }

    public boolean isRouteDisplayed() {
    	return routeLine != null;
    }
    public boolean displayRoute(geography.GeographicPoint start, 
    							List<geography.GeographicPoint> stops, 
    							int toggle) {
        if(routeLine == null) {
        	if(markerManager.getVisualization() != null) {
        		markerManager.clearVisualization();
        	}

        	if(toggle == RouteController.DIJ || toggle == RouteController.A_STAR ||
        	   toggle == RouteController.BFS || toggle == RouteController.GREEDYTSP ||
        	   toggle == RouteController._GREEDY2OPTTSP) {
        		
        		markerManager.initVisualization();
            	Consumer<geography.GeographicPoint> nodeAccepter = 
            			markerManager.getVisualization()::acceptPoint;
            	List<geography.GeographicPoint> path = 
                		new ArrayList<geography.GeographicPoint>();
            	int numStops = stops.size();
            	int stopNum = 1;
            	
            	double pathDistance = 0;
            	double pathTime = 0;
            	String dispDistance = "Distance (miles): ";
            	String dispTime = "Time (minutes): ";
            	
            	if (toggle == RouteController.BFS ||
            		toggle == RouteController.DIJ ||
            		toggle == RouteController.A_STAR) {
            		
                	PathObject servicePath;
        			path.add(start);
            		
        			geography.GeographicPoint stop;
        			
        			for (int i = 0; i < numStops; i++) {
        				
        				stop = stops.get(i);
        				
            			if (i > 0) {
            				start = stops.get(i-1);
            			}
            			
            			if (toggle == RouteController.BFS) {
            				
                			servicePath = 
                    				markerManager.getDataSet().getGraph().bfs(start, 
                    														  stop, 
                    														  nodeAccepter);
                			color = "red";
            			}
            			else if (toggle == RouteController.DIJ) {
            				
                			servicePath = 
                    				markerManager.getDataSet().getGraph().dijkstra(start, 
                    															   stop, 
                    															   nodeAccepter);
                			color = "green";
            			}
            			else {
            				
            				servicePath = 
                    				markerManager.getDataSet().getGraph().aStarSearch(start, 
                    															      stop, 
                    															      nodeAccepter,
                    															      new HashMap<Double,HashMap<Double,Integer>>());
            				color = "blue";
            			}
            			
            			geography.GeographicPoint hop;
                		for (int j = 1; j < servicePath.getPath().size(); j++) {
                			
                			hop = servicePath.getPath().get(j);
                			
                    		path.add(hop);
                    		
                			if (stops.contains(hop)) {
                				
                				markerManager.setNumStop(hop, stopNum);
                				stopNum++;
                			}
                		}
                		
            			pathDistance += servicePath.getLength();
            			pathTime += servicePath.getTravelTime();
            			
            		}
            	}
            	else {

            		List<PathObject> allServicePaths;
            		
            		if (toggle == RouteController.GREEDYTSP) {
            		
            			allServicePaths = 
            				markerManager.getDataSet().getGraph().greedyShortestCycle(start, 
            																		  stops, 
            																		  nodeAccepter, 
            																		  new HashMap<Double,HashMap<Double,Integer>>());
            			color = "purple";
            		}
            		else {
            			
            			allServicePaths = 
                			markerManager.getDataSet().getGraph().twoOptShortestCycle(start, 
                																	  stops, 
                																	  new HashMap<Double,HashMap<Double,Integer>>());
            			color = "yellow";
            		}
            		
        			path.add(start);
        			
        			PathObject servPath;
        			
        			// do not access the "meta path"
            		for (int i = 0; i < allServicePaths.size()-1; i++) {
            			
            			servPath = allServicePaths.get(i);
            			
            			geography.GeographicPoint hop;
            			// do not add the start of this path to the full route
            			// it was already added as the last stop in the previous path
            			for (int j = 1; j < servPath.getPath().size(); j++) {
            			
            				hop = servPath.getPath().get(j);
                			path.add(hop);
                			
                			if (stops.contains(hop)) {
                				
                				markerManager.setNumStop(hop, stopNum);
                				stopNum++;
                			}
            			}
            		}
            		
            		pathDistance = allServicePaths.get(allServicePaths.size()-1).getLength();
            		pathTime = allServicePaths.get(allServicePaths.size()-1).getTravelTime();
            	}

            	if(path.size() < 1) {
                    // System.out.println("In displayRoute : PATH NOT FOUND");
                    MapApp.showInfoAlert("Routing Error : ", "No path found");
                	return false;
                }
                // TODO -- debug road segments
            	List<LatLong> mapPath = constructMapPath(path);
                //List<LatLong> mapPath = new ArrayList<LatLong>();
                //for(geography.GeographicPoint point : path) {
                //    mapPath.add(new LatLong(point.getX(), point.getY()));
                //}

       			NumberFormat pathFormat = new DecimalFormat("#.##");
    			dispDistance = dispDistance + pathFormat.format(pathDistance);
    			dispTime = dispTime + pathFormat.format(pathTime);
    			
        		selectManager.setRouteInfo(dispDistance, dispTime);
                markerManager.setSelectMode(false);
                return displayRoute(mapPath, color);
    		}

    		return false;
        }
        return false;
    }




    /**
     * Construct path including road regments
     * @param path - path with only intersections
     * @return list of LatLongs corresponding the path of route
     */
    private List<LatLong> constructMapPath(List<geography.GeographicPoint> path) {
    	List<LatLong> retVal = new ArrayList<LatLong>();
        List<geography.GeographicPoint> segmentList = null;
    	geography.GeographicPoint curr;
    	geography.GeographicPoint next;

    	geography.RoadSegment chosenSegment = null;

        for(int i = 0; i < path.size() - 1; i++) {
            double minLength = Double.MAX_VALUE;
        	curr = path.get(i);
        	next = path.get(i+1);

        	if(markerManager.getDataSet().getRoads().containsKey(curr)) {
        		// get the roads heading out from the current intersection
        		HashSet<geography.RoadSegment> segments = 
        				markerManager.getDataSet().getRoads().get(curr);
        		Iterator<geography.RoadSegment> it = segments.iterator();

        		// get segments which are
            	geography.RoadSegment currSegment;
                while(it.hasNext()) {
                    //System.out.println("new segment");
                	currSegment = it.next();
                	if(currSegment.getOtherPoint(curr).equals(next)) {
                        //System.out.println("1st check passed : other point correct");
                		if(currSegment.getLength() < minLength) {
                            //System.out.println("2nd check passed : length less");
                			chosenSegment = currSegment;
                		}
                	}
                }

                if(chosenSegment != null) {
                	
                    segmentList = chosenSegment.getPoints(curr, next);
                    for(geography.GeographicPoint point : segmentList) {
                        retVal.add(new LatLong(point.getX(), point.getY()));
                    }
                }
                else {
                	System.err.println("ERROR in constructMapPath : chosenSegment was null");
                }
        		// find

        	}
        }

        // System.out.println("NOW there are " + retVal.size() + " points");
    	return retVal;
    }


	private void removeRouteLine() {
        if(routeLine != null) {
    		map.removeMapShape(routeLine);
        }
	}

//    private void setMarkerManager(MarkerManager manager) {
//    	this.markerManager = manager;
//    }




}


