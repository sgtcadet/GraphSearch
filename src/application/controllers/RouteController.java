package application.controllers;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import application.MapApp;
import application.MarkerManager;
import application.SelectManager;
import application.CLabel;
import application.services.GeneralService;
import application.services.RouteService;
import gmapsfx.javascript.object.GoogleMap;
import gmapsfx.javascript.object.LatLong;
import gmapsfx.javascript.object.LatLongBounds;
import gmapsfx.javascript.object.MVCArray;
import gmapsfx.shapes.Polyline;
import javafx.geometry.Orientation;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleGroup;
import javafx.util.StringConverter;

/** 
 * @author UCSD MOOC development team
 * @author ryanwilliamconnor
 */

public class RouteController {
	// Strings for slider labels
	public static final int _2OPTTSP = 5;
	public static final int GREEDYTSP = 4;
	public static final int BFS = 3;
    public static final int A_STAR = 2;
    public static final int DIJ = 1;
	public static final int DISABLE = 0;
	public static final int START = 1;
	public static final int DESTINATION = 2;
	public static final int MAXSTOPS = 7;
	
    private int selectedToggle = DIJ;

    private RouteService routeService;
    private Button displayButton;
    private Button hideButton;
    private Button startButton;
    private Button resetButton;
    private List<Button> stopButtons;
    private Button visualizationButton;

    private ToggleGroup group;
    private CLabel<geography.GeographicPoint> startLabel;
    private List<CLabel<geography.GeographicPoint>> stopLabels;
    private CLabel<geography.GeographicPoint> pointLabel;
    private SelectManager selectManager;
    private MarkerManager markerManager;



	public RouteController(RouteService routeService, Button displayButton, Button hideButton,
						   Button resetButton, Button startButton, List<Button> stopButtons,
						   ToggleGroup group, List<RadioButton> searchOptions, 
						   Button visualizationButton,
						   CLabel<geography.GeographicPoint> startLabel, 
						   List<CLabel<geography.GeographicPoint>> stopLabels,
						   CLabel<geography.GeographicPoint> pointLabel, 
						   SelectManager manager, MarkerManager markerManager) {
        // save parameters
        this.routeService = routeService;
		this.displayButton = displayButton;
        this.hideButton = hideButton;
		this.startButton = startButton;
		this.resetButton = resetButton;
		this.stopButtons = stopButtons;
        this.group = group;
        this.visualizationButton = visualizationButton;

        // maybe don't need references to labels;
		this.startLabel = startLabel;
		this.stopLabels = stopLabels;
        this.pointLabel = pointLabel;
        this.selectManager = manager;
        this.markerManager = markerManager;

        setupDisplayButtons();
        setupRouteButtons();
        setupVisualizationButton();
        setupLabels();
        setupToggle();
        //routeService.displayRoute("data/sampleroute.map");
	}


	private void setupDisplayButtons() {
		displayButton.setOnAction(e -> {
            if(startLabel.getItem() == null) {
            	
            	MapApp.showErrorAlert("Route Display Error", 
						  "Choose a start point.");
            }
            else{
            	
            	for (CLabel<geography.GeographicPoint> stopLabel : stopLabels) {
            		
                	if (stopLabel.getItem() != null) {
                		
                    	List<geography.GeographicPoint> stops = 
                    			new ArrayList<geography.GeographicPoint>();
                    	
                    	for (CLabel<geography.GeographicPoint> label : stopLabels) {
                    		
                        	if (label.getItem() != null) {
                        		
                        		stops.add(label.getItem());
                        	}
                    	}
                    		
                		routeService.displayRoute(startLabel.getItem(), stops, selectedToggle);
                		break;
                	}
            	}
            }
		});

        hideButton.setOnAction(e -> {
        	routeService.hideRoute();
        });

        //TODO -- implement
        resetButton.setOnAction( e -> {

            routeService.reset();
        });
	}

    private void setupVisualizationButton() {
    	visualizationButton.setOnAction( e -> {
    		markerManager.startVisualization();
    	});
    }

    private void setupRouteButtons() {
    	startButton.setOnAction(e -> {
            //System.out.println();
            selectManager.setStart();
    	});
    	
    	Pattern stopButtonPattern = Pattern.compile("[0-9]+");

    	for (Button stopButton : stopButtons) {
    		
    		stopButton.setOnAction( e-> {
    			
    	    	Matcher stopButtonMatcher;
    	    	int stopButtonNum;
    			
    			stopButtonMatcher =  stopButtonPattern.matcher(stopButton.getText());
    			stopButtonMatcher.find();
    			stopButtonNum = Integer.parseInt(stopButtonMatcher.group());

    			selectManager.setStop(stopButtonNum);
    		});
    	}
    }


    private void setupLabels() {


    }

    private void setupToggle() {
    	group.selectedToggleProperty().addListener( li -> {
            if(group.getSelectedToggle().getUserData().equals("Dijkstra")) {
            	selectedToggle = DIJ;
            }
            else if(group.getSelectedToggle().getUserData().equals("A*")) {
            	selectedToggle = A_STAR;
            }
            else if(group.getSelectedToggle().getUserData().equals("BFS")) {
            	selectedToggle = BFS;
            }
            else if(group.getSelectedToggle().getUserData().equals("GreedyTSP")) {
            	selectedToggle = GREEDYTSP;
            }
            else if(group.getSelectedToggle().getUserData().equals("2OptTSP")) {
            	selectedToggle = _2OPTTSP;
            }
            else {
            	System.err.println("Invalid radio button selection");
            }
    	});
    }




}
