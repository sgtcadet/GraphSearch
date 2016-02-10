/** Class to manage items selected in the GUI
 * 
 * @author UCSD MOOC development team
 * @author ryanwilliamconnor
 */

package application;
import java.util.List;

import application.services.GeneralService;
import geography.GeographicPoint;
import gmapsfx.javascript.object.Marker;

public class SelectManager {
	
    private CLabel<String> routeDistLabel;
    private CLabel<String> routeTimeLabel;
    private CLabel<GeographicPoint> pointLabel;
    private CLabel<GeographicPoint> startLabel;
    private List<CLabel<GeographicPoint>> stopLabels;
    private Marker startMarker;
    private Marker destinationMarker;
    private Marker selectedMarker;
    private MarkerManager markerManager;
    private DataSet dataSet;


    public SelectManager() {
        startMarker = null;
        destinationMarker = null;
        selectedMarker = null;
        routeDistLabel = null;
        routeTimeLabel = null;
        pointLabel = null;
        startLabel = null;
        stopLabels = null;
        dataSet = null;
    }


    public void resetSelect() {
        markerManager.setSelectMode(true);
        routeDistLabel.setItem(null);
        routeTimeLabel.setItem(null);
    }
    public void clearSelected() {
    	selectedMarker = null;
    	pointLabel.setItem(null);
    }

    public void setAndDisplayData(DataSet data) {
    	setDataSet(data);
        //TODO - maybe if markerManager!= null?
        if(markerManager != null) {
            markerManager.displayDataSet();
        }
        else {
        	System.err.println("Error : Marker Manager is null.");
        }
    }

    public void setMarkerManager(MarkerManager manager) { this.markerManager = manager; }
    public void setPoint(GeographicPoint point, Marker marker) {
        // System.out.println("inSetPoint.. passed : " + point);
    	pointLabel.setItem(point);
        selectedMarker = marker;
    }
    public void setDataSet(DataSet dataSet) {
    	this.dataSet = dataSet;
    	if(markerManager != null) {
    		markerManager.setDataSet(dataSet);
    	}
    }

    public void setRouteInfo(String routeDistDisp, String routeTimeDisp) {
    	
    	routeDistLabel.setItem(routeDistDisp);
    	routeTimeLabel.setItem(routeTimeDisp);
    }

	public void setRouteDistLabel(CLabel<String> label) { this.routeDistLabel = label; }
	public void setRouteTimeLabel(CLabel<String> label) { this.routeTimeLabel = label; }
    public void setPointLabel(CLabel<GeographicPoint> label) { this.pointLabel = label; }
    public void setStartLabel(CLabel<GeographicPoint> label) { this.startLabel = label; }
    public void setStopLabels(List<CLabel<GeographicPoint>> labels) { this.stopLabels = labels; }

    public GeographicPoint getPoint() { return pointLabel.getItem(); }


	public GeographicPoint getStart(){return startLabel.getItem();}
	public List<CLabel<GeographicPoint>> getStops(){return stopLabels;}
	public void setStart() {
		if(pointLabel.getItem() != null) {
        	GeographicPoint point = pointLabel.getItem();
    		startLabel.setItem(point);
            markerManager.setStart(point);
		}
	}

	public void setStop(int stopButtonNumber) {
		if(pointLabel.getItem() != null) {
        	GeographicPoint newStop = pointLabel.getItem();
        	GeographicPoint oldStop = stopLabels.get(stopButtonNumber-1).getItem();
    		stopLabels.get(stopButtonNumber-1).setItem(newStop);
    		markerManager.setStop(newStop, oldStop);
		}
	}


}