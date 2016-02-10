/** Class to manage Markers on the Map
 * 
 * @author UCSD MOOC development team
 * @author ryanwilliamconnor
 */

package application;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import gmapsfx.javascript.event.UIEventType;
import gmapsfx.javascript.object.Animation;
import gmapsfx.javascript.object.GoogleMap;
import gmapsfx.javascript.object.LatLong;
import gmapsfx.javascript.object.Marker;
import gmapsfx.javascript.object.MarkerOptions;
import javafx.scene.control.Button;
import gmapsfx.javascript.object.LatLongBounds;
import netscape.javascript.JSObject;

public class MarkerManager {

    private static final double DEFAULT_Z = 2;
    private static final double SELECT_Z = 1;
    private static final double STRTDEST_Z = 3;

    private HashMap<geography.GeographicPoint, Marker> markerMap;
    private ArrayList<geography.GeographicPoint> markerPositions;
    private GoogleMap map;
    protected static String startURL = "http://maps.google.com/mapfiles/kml/pal3/icon40.png";
    protected static String stopURL = "http://maps.google.com/mapfiles/kml/pal2/icon5.png";
    protected static String SELECTED_URL = "http://maps.google.com/mapfiles/kml/paddle/ltblu-circle.png";
    protected static String markerURL = "http://maps.google.com/mapfiles/kml/paddle/blu-diamond-lv.png";
	protected static String visURL = "http://maps.google.com/mapfiles/kml/paddle/red-diamond-lv.png";
	
	// number icons made by Alessio Atzeni (http://www.alessioatzeni.com) 
	// from Flaticon (http://www.flaticon.com) 
	// and licensed under Creative Commons BY 3.0 (http://creativecommons.org/licenses/by/3.0/)
	// the icons were not modified
	protected static String stop1URL = "https://lh3.googleusercontent.com/DXp77sSxT6f7S1q58hjdAUrd2BVbDv5nozG6mvyMWpGI73I6648ElQyI3v332RzhLp83FV0wpcvChlCk9X_7zOTyEq4FVpfI0nq4S9TWSVZ5yk41-vq27zhhWz6BzHvuYsDZ9dk_JjmW79dlvrU76elmqAwWWIoG9jOwzIn0tGy33fuIWT--btQk18LxBCpj_OGyn76fFHzh8RbTRScKwKPKhjxxnVEZDncsU_YZs8-lVIYQuqS95TVjnAt11CHQoGMKWc9qRxGaL4VvxEjRXnhRnzmqy_07gQbYx73jJhYM3z3wgdZ7PYpDeQ-3F4e8RBX_FRqD8ROyUze2jOYTxy2leuAn4z1uvL6Mn3vd96PXfd5EKyIXLQ4IiW4q5OqH7dr4lkN6RnTK1p_RXhDbkClGcZUaeNjs9GiSFDoS7qd-l9iwF3vsCT1LC8gdOKoc_DwKIz8zvNlf6kcQuREn2a9xy8RTVNbE2DgWJ6XF4H2YZ7EzHvlZP4FTur8Vni23iTZdroGap2MhtJ6DPDu6yH9EUkofrkuFzhKcIL_jrnqEJ08H7pYgftsPupWQxMfQ3tZt=s32-no";
	protected static String stop2URL = "https://lh3.googleusercontent.com/Fv6bMoJe2jNRTdtocYTMDWGiCF0aRPxKGl6F13qm3pHcbJrRk51dEVH-jO4N3zIH3oKqwoXHzY7Jv8EC7u7Nw-pL5PcytwnjSjxjEuQxZZ7WS8wXn6UsXPqn8UZnq6GUzRIzS2dJ9A0yQG8fMwNIa6keYAMCMrBNTEcetAXZs8tZJfFxwgp-PrcWy6dpnNUvvKae-Yl2jpf8aQbcwZLZF2c3COFpqmax7nwZrvF1OAh86GbsQbb1EKFPS8AHaQWO_My-Sa-aMUPyCab_CJQP09t2af6KlodpKMbt5s_DfOa7Oi9dj_1Gq1_dz5jAV99f_06wyZovf8BQGH6MphZO006qv2WHb2TFMbETEpTN6gp1fhkKrUqqe6o-P3ZPFRKIfJXTn2Gm3KYJTlNGrlCRBiF4cmLwPIuPtfCrnDc3D5mIfxyDkLkFigw9FzL5TNeAICNdO45qqcJ54PekrqBj0qAF5yRXs5eEeGxTGSvUO8qCGGbrw16FpI7Wo9hms17t_cxrO3mMgmrzklmR9a1m_6SuYAbNPvY7jOFQNDed5KQS-ShYUSxiWKSKRckjXNynVKqe=s32-no";
	protected static String stop3URL = "https://lh3.googleusercontent.com/cRkFIzEFdKuLcDLxMqi4934Yn3eUBUtLmhpKA1QGHTjVp_LXpjd4UhOVt3jfXisa5KthW7joSohKRPINPjgfV8jaJvk6N-Q0a9nr2fo4pmb_RvkMqFVAGBKLRYrxgZbQ7iW3wG6oeGdp18o4YzTd-bYn10O1KnBTx0ltvzlhCjqa2U0JTma2fUYHaN3JzKYtByATuj-8ddQNYg7gKxbGUUpAzjbiXN7lfUzG5WhJmG7daVi_s9pxHK9vnKbWlTAS0VFsFQiIq0N3gnbSud1oQT6gn-KQS5GRcF3ILh6Iox0fwhF8c4U4RQ8ddA0MZUfukh0JCPc8s7ivHwq_F6b67hJDduitXGTB2kTtZUjBEapt1d1bOhZse8iNxBf373TdtXWdBIzNsaDJHYoEgJ94KGmnssa9llFxKwi9VqinIP20LsGgYeewysH-Q5vzrnS2VVBkfiT0w0Zesruy6C69w1MkRirJFTt4IsTKCh9Oax_07WoAChT0KhA_vr_iv3AEqk7V470N_3xAh-sy2c0tSMuXhfP0ELiqT4zuaGy6Uu2IzZgYlExhbh9b8XvRC7XXy-vr=s32-no";
	protected static String stop4URL = "https://lh3.googleusercontent.com/yQ_hOC_rpfQ0di8X3qeBhfKLgriqBtL0Tokx_gqWPtfTq5CsYoxRApNIJe20kevacvM5r2VZkYI-0R-IgQdPHskv0Hlv7k5UddogD09Ku8dF6Rig0QpPCyJW0GZNKaHq3QF_e2iBw6Ov_me4MfaO8qcU-XuFWT5F_fYyA6dov5z3QiZ9SThbNIkToPEYJ9AQ3XUBdeeSMN_lshnC2YLynh4Nkk0r0EEuHoW4SBmg3mpSoTJcz1YPkUHNK6SfVoYv-Q2dk7xRFIQwCPdALIc3kX7-ZHe2BaHE6I3xsQkf1CFeMMPV9XJ4Syrx3ZR-0PUEygVhSbv10Kpsqkx1zOmea5Vpq3ECIKh_PKfd4OBvLoJqARW_4qYqfn_saXClnIaS3mRHfhsigYxQweFwuAFPHTtIe8r2oJyyCwS3y-ccVbV1DQkBlIB6ukkJesAL47qkAUWXapr8Djba4QTrKTFZmpG7FID4zgz45o0ZhbuiB0XlrFtpyyS657iQyGxYu2CdObfSmOXlKy49H3XiEVlQzaoyqZCJ7ucZMiiJ5qVaCGtjfFFD0u3vpmu7yeI4v53ylD7i=s32-no";
	protected static String stop5URL = "https://lh3.googleusercontent.com/R--LNdYssZitXKcNeNXVD5vlx0v_zrX80WhLBMYvRioiE9WBAbSxemvOaA8QQj-2rUSSjgsBlnfGcHW04PPTnT8JsG0Ji04eMjqyfgPYKEFqcLDkU-9mcEgK2uygsWl-yamD2JEjDUFVePGYZFVLKQoX60ft6hu_6SzY77xG1xMVoGba0mIbOujKDZea0d37kNljrI4HoA3VHC5YomqIm3iLeGBev2djoiX9LMV4pRpl3KB0LOeDEyI96ivxoB0ZeD5LAPsvJyrUIflhlbCqve3vW5MbBGwp3ZvjF6kfSM3m0JeYR1TeHiXnrWQy4RFF_jOxRNpzt63vxiXa2gM7crHLyUv989jZTOAM5aJxubQvfEJ0DN1QTZqB87vc8JYHA2oybB01zJDKQVYFkXlg09JJP20RTZHdtIgaP3yne0LQFwzJBYk5Tj-xX22jP208hFtcEnppoSS6OlOk1SJoQ1mzhlUVeDvX0wlxUHU9-pkwoZ5E3cPgBDQa29JYOdX8oDxDw3b2h4QVsbBqhAfc0ncAp6OaoF0uJe7YkXVtI84Jkkpm28GFTvhzBNTDPsGmNoqo=s32-no";
	protected static String stop6URL = "https://lh3.googleusercontent.com/KGxsFkn2Od8jSgQ10ujMYsGBCx5yXbr7rHNupyXEfGfOZCBzSJMZK6CJ3nznOoHLd3p4BkKMDqRXiqphLXmmv1eXakViLREIh8NFUbOWYXDoK7Dg3jDL9MI1dKjUxc2sRNfKkyRYwejUfy7LTSqr7H4T122ImXj9IsTxeVPN2SMMRTtRs2Hx6fnv4xe14tjySIBvjtzd5a117sm1YD5MwOOHINPDl6UgTaLG1mjX53XnW9w8OJEpPfL2PEWxElssZ0Cn-YJYUJHZXN-wY7rdZTT8QJc0AKtQHv1Z6qVV1V4P_zkmZ7BK-MYEij1NvYYNwA4keC4ttPtOKDaBpmknuh-KCjLHNLM83LFNd_xozYepYTOtx6Rs2nPlyTwrJs2C-ioNK-OPSqvT8xRSd8ek560y0vYm2hHj7w7h_okHnOkizzs30DPx-NjZKgfg_zvGIJqR8ofCKmxssXcqOaeyj_kr5T79agk8P1QC77HXILhQt9Gl1eYhhqgnh9I0QXvvTpZqX_vgodXFgWVr42W8to5opCZWV4Pld6V1wx7qm0VedR5xAFmmnvXMtAFvOPDb2ppg=s32-no";
	protected static String stop7URL = "https://lh3.googleusercontent.com/X_f8UxwPaXUmfpjNSdOf7-J2JVDw5W8ZhVeJkb15lJfWz-_X4lkeYmYjnIVSrH24TlzWbBNb3s4_b7WI89F_kHydezgxz-Rrp8-OclUHgRepCkWTxiHsUZtMISwxCTEm8GxYOFmCHHakhYaTKiPQ6dr76R6skNqz8ac7X6nGjn-UMp3hOtyT69ZpnbdcfawNxA31AnPgzrqZluuAiL8pSjbFLaHM-QyjNMvPU65ZXiFt6P-qJKM3zA_IcBP7XsDYRYghe-ovuVXxoPjZxmhm2iv3yFGzQQiaGc3gReSgMu_Ovam4kZZ0bt6dxT9aShSN4oKa_cPL2qOiHnk8dyoZQkbEB6VI4S1fzKgzFx5Bm3dEsGAnvMTu5xquMKcn0gvmX65SqNnLIvE5AM7e2fx3tDM8C-Gt9fY_q-104dl4WMV8Cg4MXTFkQmHSK7tbwkjQoLhh-8oTh8XZ5Oh9t_iHXjxAYUtDMOFKl3n4RZ65HjKHOw5Fo-ztFZGVcZ4OQ6cJucARgZlHl-jTr5CahLRyIL4-fbkLsOB7uq2Iy0NU510M1AMCUileQbvupAU2Kjf9arlm=s32-no";
	
	protected static String[] numStopIconURLS = 
		{stop1URL, stop2URL, stop3URL, stop4URL, stop5URL, stop6URL, stop7URL};
	
    private Marker startMarker;
    private List<Marker> stopMarkers;
    private Marker selectedMarker;
    private DataSet dataSet;
    private LatLongBounds bounds;
    private SelectManager selectManager;
    private RouteVisualization rv;
    private Button vButton;
    private boolean selectMode = true;

    public MarkerManager() {
    	markerMap = new HashMap<geography.GeographicPoint, Marker>();
    	this.map = null;
    	this.selectManager = null;
        this.rv = null;
        markerPositions = null;
    }
    public MarkerManager(GoogleMap map, SelectManager selectManager) {
    	// TODO -- parameters?
        dataSet = null;

    }

    /**
     * Used to set reference to visualization button. Manager will be responsible
     * for disabling button
     *
     * @param vButton
     */
    public void setVisButton(Button vButton) {
    	this.vButton = vButton;
    }

    public void setSelect(boolean value) {
    	selectMode = value;
    }
    public RouteVisualization getVisualization() { return rv; }



    public GoogleMap getMap() { return this.map; }
    public void setMap(GoogleMap map) { this.map = map; }
    public void setSelectManager(SelectManager selectManager) { this.selectManager = selectManager; }

    public void putMarker(geography.GeographicPoint key, Marker value) {
    	markerMap.put(key, value);

    }

    /** Used to initialize new RouteVisualization object
     *
     */
    public void initVisualization() {
    	rv = new RouteVisualization(this);
    }

    public void clearVisualization() {
        rv.clearMarkers();
    	rv = null;
    }

    // TODO -- protect against this being called without visualization built
    public void startVisualization() {
    	if(rv != null) {
	    	rv.startVisualization();
    	}
    }

    public void setStart(geography.GeographicPoint point) {
    	if(startMarker!= null) {
            changeIcon(startMarker, markerURL);
//            startMarker.setZIndex(DEFAULT_Z);
    	}
        startMarker = markerMap.get(point);
//        startMarker.setZIndex(STRTDEST_Z);
        changeIcon(startMarker, startURL);
    }
    public void setStop(geography.GeographicPoint newStop,
    					geography.GeographicPoint oldStop) {
    	
    	Marker newStopMarker = markerMap.get(newStop);
//            destinationMarker.setZIndex(DEFAULT_Z);
    	if (stopMarkers == null) {
    		
    		stopMarkers = new ArrayList<Marker>();
    	}
    	
    	if (oldStop != null) {
    		
        	Marker oldStopMarker = markerMap.get(oldStop);
        	changeIcon(oldStopMarker, markerURL);
        	stopMarkers.remove(oldStopMarker);
    	}
    	
        changeIcon(newStopMarker, stopURL);
    	stopMarkers.add(newStopMarker);
//        destinationMarker.setZIndex(STRTDEST_Z);
    }
    
    public void setNumStop(geography.GeographicPoint stopPoint, int stopNum) {
    	
    	Marker stopMarker = markerMap.get(stopPoint);
    	String numStopIconURL = MarkerManager.numStopIconURLS[stopNum-1];
    	changeIcon(stopMarker, numStopIconURL);
    }

    public void changeIcon(Marker marker, String url) {
        marker.setVisible(false);
        marker.setIcon(url);
        marker.setVisible(true);
    }

    /**
     * TODO -- Might need to create all new markers and add them??
     */
    public void restoreMarkers() {
    	Iterator<geography.GeographicPoint> it = markerMap.keySet().iterator();
        while(it.hasNext()) {
            Marker marker = markerMap.get(it.next());
            // destination marker needs to be added because it is added in javascript
            if(marker != startMarker) {
                marker.setVisible(false);
                marker.setVisible(true);
            }
        }
        selectManager.resetSelect();
    }

    public void refreshMarkers() {

    	Iterator<geography.GeographicPoint> it = markerMap.keySet().iterator();
        while(it.hasNext()) {
        	Marker marker = markerMap.get(it.next());
        	marker.setVisible(true);
        }
    }
    public void clearMarkers() {
        if(rv != null) {
        	rv.clearMarkers();
        	rv = null;
        }
    	Iterator<geography.GeographicPoint> it = markerMap.keySet().iterator();
    	while(it.hasNext()) {
    		markerMap.get(it.next()).setVisible(false);
    	}
    }

    public void setSelectMode(boolean value) {
        if(!value) {
        	selectManager.clearSelected();
        }
    	selectMode = value;
    }

    public boolean getSelectMode() {
    	return selectMode;
    }
    public static MarkerOptions createDefaultOptions(LatLong coord) {
        	MarkerOptions markerOptions = new MarkerOptions();
        	markerOptions.animation(null)
        				 .icon(markerURL)
        				 .position(coord)
                         .title(null)
                         .visible(true);
        	return markerOptions;
    }

    public void hideIntermediateMarkers() {
        Iterator<geography.GeographicPoint> it = markerMap.keySet().iterator();
        while(it.hasNext()) {
            Marker marker = markerMap.get(it.next());
            if(marker != startMarker && !stopMarkers.contains(marker)) {
                marker.setVisible(false);
            }
//        	map.addMarker(marker);
        }
    }

    public void hideStopMarkers() {
    	
    	for (Marker stop : stopMarkers) {
    		
        	stop.setVisible(false);	
    	}
    }

    public void displayMarker(geography.GeographicPoint point) {
    	if(markerMap.containsKey(point)) {
        	Marker marker = markerMap.get(point);
            marker.setVisible(true);
            // System.out.println("Marker : " + marker + "set to visible");
    	}
    	else {
    		// System.out.println("no key found for MarkerManager::displayMarker");
    	}
    }
    
    public void displayDataSet() {
        markerPositions = new ArrayList<geography.GeographicPoint>();
        dataSet.initializeGraph();
    	Iterator<geography.GeographicPoint>it = dataSet.getIntersections().iterator();
        bounds = new LatLongBounds();
        while(it.hasNext()) {
        	geography.GeographicPoint point = it.next();
            LatLong ll = new LatLong(point.getX(), point.getY());
        	MarkerOptions markerOptions = createDefaultOptions(ll);
            bounds.extend(ll);
        	Marker marker = new Marker(markerOptions);
            registerEvents(marker, point);
        	map.addMarker(marker);
        	putMarker(point, marker);
        	markerPositions.add(point);
//            marker.setZIndex(DEFAULT_Z);
        }
        map.fitBounds(bounds);
        // System.out.println("End of display Intersections");

    }


    private void registerEvents(Marker marker, geography.GeographicPoint point) {
        /*map.addUIEventHandler(marker, UIEventType.mouseover, (JSObject o) -> {
           marker.setVisible(true);
           //marker.setAnimation(Animation.BOUNCE);
        });

        map.addUIEventHandler(marker, UIEventType.mouseout, (JSObject o) -> {
        	marker.setAnimation(null);
        });*/

        map.addUIEventHandler(marker, UIEventType.click, (JSObject o) -> {
            //System.out.println("Clicked Marker : " + point.toString());
            if(selectMode) {
                	if(selectedMarker != null && selectedMarker != startMarker
                	   && !stopMarkers.contains(selectedMarker)) {
                		selectedMarker.setIcon(markerURL);
//                		selectedMarker.setZIndex(DEFAULT_Z);
                	}
            	selectManager.setPoint(point, marker);
                selectedMarker = marker;
                selectedMarker.setIcon(SELECTED_URL);
//                selectedMarker.setZIndex(SELECT_Z);

                // re add markers to map
                // slightly glitchy
//                refreshMarkers();
            }
        });
    }

    public void disableVisButton(boolean value) {
    	if(vButton != null) {
	    	vButton.setDisable(value);
    	}
    }
	public void setDataSet(DataSet dataSet) {
		this.dataSet= dataSet;
	}


    public DataSet getDataSet() { return this.dataSet; }
}
