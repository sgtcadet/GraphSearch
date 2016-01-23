package roadgraph;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Scanner;
import java.util.function.Consumer;

import util.GraphLoader;
import geography.*;

/**
 * @author ryanwilliamconnor
 * Grader for MapGraph.greedyShortestCycle().
 */
public class TSPGrader implements Runnable {
	
    public String feedback;

    public int correct;

    private static final int TESTS = 4;

    /** Format readable feedback */
    public static String printOutput(double score, String feedback) {
        return "Score: " + score + "\nFeedback: " + feedback;
    }

    /** Format test number and description */
    public static String appendFeedback(int num, String test) {
        return "\n** Test #" + num + ": " + test + "...";
    }

    public static void main(String[] args) {
        TSPGrader grader = new TSPGrader();

        // Infinite loop detection
        Thread thread = new Thread(grader);
        thread.start();
        long endTime = System.currentTimeMillis() + 10000;
        boolean infinite = false;
        while(thread.isAlive()) {
            // Stop after 10 seconds
            if (System.currentTimeMillis() > endTime) {
                thread.stop();
                infinite = true;
                break;
            }
        }
        if (infinite) {
            System.out.println(printOutput((double)grader.correct / TESTS, 
            				               grader.feedback + 
            				               "\nYour program entered an infinite loop."));
        }
    }

    /** Run a test case on an adjacency list and adjacency matrix.
     * @param i The graph number
     * @param file The file to read from
     * @param desc A description of the graph
     * @param start The point to start from
     * @param end The point to end at
     */
    public void runTest(int i, String file, String desc, 
    					GeographicPoint start, List<GeographicPoint> stops,
    					HashMap<Double,HashMap<Double,Integer>> offLimits) {
        MapGraph graph = new MapGraph();

        feedback += "\n\n" + desc;

        GraphLoader.loadRoadMap("data/graders/extension/" + file, graph);
        CorrectAnswer corr = new CorrectAnswer("data/graders/extension/" + file +
        									   ".answer", false);

        judge(i, graph, corr, start, stops, offLimits);
    }

    /** Compare the user's result with the right answer.
     * @param i: The graph number
     * @param result: The user's graph
     * @param corr: The correct answer
     * @param start: The point to start from
     * @param stops: The stops to visit once each
     * @param offLimits: The locations that are illegal to visit during the tour
     */
    public void judge(int i, MapGraph result, CorrectAnswer corr, 
    				  GeographicPoint start, List<GeographicPoint> stops,
    				  HashMap<Double,HashMap<Double,Integer>> offLimits) {
    	// Correct if paths are same length and have the same elements
        feedback += appendFeedback(i, "Running greedyShortestCycle from (" + 
        							   start.getX() + ", " + start.getY() + 
        							   ") through " + stops);
        
        ArrayList<PathObject> TSPPaths;
        List<GeographicPoint> path;
        
        if (i < 5) {
            TSPPaths = result.greedyShortestCycle(start, stops, offLimits);
            path = TSPPaths.get(TSPPaths.size()-1).getPath();
        }
        else {
            TSPPaths = result.twoOptShortestCycle(start, stops, offLimits);
            path = TSPPaths.get(TSPPaths.size()-1).getPath();
        }


        if (path == null) {
            if (corr.path == null) {
                feedback += "PASSED.";
                correct++;
            } else {
                feedback += "FAILED. Your implementation returned null; expected \n" + 
                			printPath(corr.path) + ".";
            }
        } else if (path.size() != corr.path.size() || !corr.path.containsAll(path)) {
            feedback += "FAILED. Expected: \n" + printPath(corr.path) + 
            			"Got: \n" + printPath(path);
            if (path.size() != corr.path.size()) {
                feedback += "Your result has size " + path.size() + 
                			"; expected " + corr.path.size() + ".";
            } else {
                feedback += "Correct size, but incorrect path.";
            }
        } else {
            feedback += "PASSED.";
            correct++;
        }
    }

    /** Print a search path in readable form */
    public String printPath(List<GeographicPoint> path) {
        String ret = "";
        for (GeographicPoint point : path) {
            ret += point + "\n";
        }
        return ret;
    }

    /** Run the grader */
    @Override
    public void run() {
        feedback = "";

        correct = 0;

        try {
      
        	List<GeographicPoint> testStopsOneAndTwo = new ArrayList<GeographicPoint>();
        	for (int i = 1; i < 7; i++) {
            	testStopsOneAndTwo.add(new GeographicPoint(i, i));
        	}
        	
            runTest(1, "residentialLine.txt", 
            		"MAP: Straight line with only residential roads", 
            		new GeographicPoint(0, 0), 
            		testStopsOneAndTwo, 
            		new HashMap<Double,HashMap<Double,Integer>>());
            
            runTest(2, "mixedLine.txt", 
            		"MAP: Straight line with mixed road types", 
            		new GeographicPoint(0, 0), 
            		testStopsOneAndTwo,
            		new HashMap<Double,HashMap<Double,Integer>>());
            
        	List<GeographicPoint> testStopsThree = new ArrayList<GeographicPoint>();
        	for (int i = 1; i < 7; i++) {
            	testStopsThree.add(new GeographicPoint(i*10, i*10));
        	}
            
            runTest(3, "secondaryStopsLine.txt", 
            		"MAP: Line with intersections in between stops and mixed road types", 
            		new GeographicPoint(0, 0), 
            		testStopsThree,
            		new HashMap<Double,HashMap<Double,Integer>>());
            
        	List<GeographicPoint> testStopsFourAndFive = new ArrayList<GeographicPoint>();
            testStopsFourAndFive.add(new GeographicPoint(5, 0));
            testStopsFourAndFive.add(new GeographicPoint(5, -8));
            testStopsFourAndFive.add(new GeographicPoint(0, -6));
            
            runTest(4, "lectureGreedy.txt", 
            		"MAP: Simple example from UCSD lecture where 2-opt TSP improves upon greedy TSP", 
            		new GeographicPoint(0, 0), 
            		testStopsFourAndFive,
            		new HashMap<Double,HashMap<Double,Integer>>());
            
            runTest(5, "lecture2Opt.txt", 
            		"MAP: Simple example from UCSD lecture where 2-opt TSP improves upon greedy TSP", 
            		new GeographicPoint(0, 0), 
            		testStopsFourAndFive,
            		new HashMap<Double,HashMap<Double,Integer>>());
			
            if (correct == TESTS)
                feedback = "All tests passed. Great job!" + feedback;
            else
                feedback = "Some tests failed. Check your code for errors, " +
                		   "then try again: " + feedback;
			
        } catch (Exception e) {
            feedback += "\nError during runtime: " + e;
            e.printStackTrace();
        }
            
        System.out.println(printOutput((double)correct / TESTS, feedback));
    }
}
