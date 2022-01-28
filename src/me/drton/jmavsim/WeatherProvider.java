package me.drton.jmavsim;
import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.io.File;
import java.io.FileReader;
import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonNumber;
import javax.json.JsonReader;
import javax.vecmath.Vector3d;

import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavsim.vehicle.AbstractVehicle;
import me.drton.jmavsim.ReportingObject;

import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;

import java.io.StringReader;


public class WeatherProvider implements ReportingObject, MissionDataConsumer {
    private static final String WIND_KEY = "wind";
    private static final String TEMP_KEY = "temperature";
    private static final Vector3d NO_WIND = new Vector3d();
    private static final double STANDARD_TEMPERATURE = 25.0;

    private AbstractVehicle vehicle;
    private Vector3d currentWind = NO_WIND;
    private Vector3d lastWindSetpoint = NO_WIND;
    private double currentTemp = STANDARD_TEMPERATURE;
    private double lastTemperatureSetpoint = STANDARD_TEMPERATURE;
    private int currentSeq = -1;
    private Vector3d waypointLocation = new Vector3d(-1,-1,-1);
    private double initialDistance = 0;

    private JsonObject obj;

    private static JsonArray windTriplet(double x, double y, double z) {
        JsonArrayBuilder builder = Json.createArrayBuilder();
        builder.add(x);
        builder.add(y);
        builder.add(z);
        return builder.build();
    }
    
    private static JsonObject testData() {
        JsonObjectBuilder builder = Json.createObjectBuilder();
        JsonArrayBuilder tempBuilder = Json.createArrayBuilder();
        tempBuilder.add(STANDARD_TEMPERATURE);
        tempBuilder.add(15.0);
        tempBuilder.add(22.0);
        tempBuilder.add(19.0);
        JsonArrayBuilder windBuilder = Json.createArrayBuilder();
        windBuilder.add(WeatherProvider.windTriplet(5, 5, 0));
        windBuilder.add(WeatherProvider.windTriplet(2, 5, 0));
        windBuilder.add(WeatherProvider.windTriplet(-9, 9, 0));
        windBuilder.add(WeatherProvider.windTriplet(1, -1, 0));
        builder.add(TEMP_KEY, tempBuilder.build());
        builder.add(WIND_KEY, windBuilder.build());
        return builder.build();
    }

    public WeatherProvider(AbstractVehicle vehicle) {
        this.obj = null;
        this.vehicle = vehicle;
    }

    public WeatherProvider(String fileHandle, AbstractVehicle vehicle) {
        try (JsonReader jsonReader = Json.createReader(new FileReader(new File(fileHandle)))){
            this.obj = jsonReader.readObject();
            this.vehicle = vehicle;
        } catch (Exception e) {
            System.out.println("Error in processing weather file. Mission items not initialised!");
            System.out.println(e);
            System.exit(-42);
        }
    }
    
    private Vector3d windDataforSeq(int seq) {
        if (this.obj == null) {  return NO_WIND; }
        if (seq >= 0) {
            try {
                JsonArray winds = obj.getJsonArray(WIND_KEY);
                    
                if (seq >= winds.size()) {
                    System.out.println("Sequence number ("+seq+") not present for wind data. Size "+winds.size());
                    return NO_WIND;
                } 
    
                List<JsonNumber> winds_d = winds.getJsonArray(seq).getValuesAs(JsonNumber.class);
                return new Vector3d(winds_d.get(0).doubleValue(), winds_d.get(1).doubleValue(), winds_d.get(2).doubleValue());
    
            } catch(Exception e) {
                System.out.print("Failed in reading wind data");
                System.out.print(e);
            } 
        } return NO_WIND;
    }

    private double temperatureDataFromSeq(int seq) {
        if (this.obj == null) {  return STANDARD_TEMPERATURE; }
        if (seq >= 0) {
            try {
                JsonArray temps = obj.getJsonArray(TEMP_KEY);
                
                if (seq >= temps.size() || seq < 0) {
                    System.out.println("Sequence number ("+seq+") not present for temperature data. Size "+temps.size());
                    return STANDARD_TEMPERATURE;
                } 

                return temps.getJsonNumber(seq).doubleValue();

            } catch(Exception e) {
                System.out.print("Failed in reading temperature data");
                System.out.print(e);
            }
         } return STANDARD_TEMPERATURE;
    }

    private double interpolateScalar(double a, double b, double completion) {
        return (1-completion) * a + completion * b;
    }

    private Vector3d interpolateVectors(Vector3d a, Vector3d b, double completion) {
        return new Vector3d(
            this.interpolateScalar(a.x, b.x, completion),
            this.interpolateScalar(a.y, b.y, completion),
            this.interpolateScalar(a.z, b.z, completion)
        );
    }

    public double getTemperature() {
        if (initialDistance == 0) return this.temperatureDataFromSeq(this.currentSeq);
        return currentTemp;
    }

    public Vector3d getWind() {
        if (initialDistance == 0) return this.windDataforSeq(this.currentSeq);
        return currentWind;
    }

    private double euclideanDistance(Vector3d a, Vector3d b) {
        // Horizontal distance only
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }

    private double getLegCompletion() {
        if (initialDistance < 0.1) return 0;
        double distance = this.euclideanDistance(vehicle.getPosition(), this.waypointLocation);
        return Math.min(Math.max(1 - (distance / this.initialDistance), 0), 1.0);
    }

    @Override
    public void missionDataUpdated(int seq, Vector3d wpLocation, LatLonAlt globalPosition) {
        if (currentSeq < seq && Math.abs(wpLocation.x) < 5000 && Math.abs(wpLocation.y) < 5000) {
            
            int new_seq = currentSeq++;
            this.lastWindSetpoint = seq > 0 ? this.windDataforSeq(seq - 1) : NO_WIND;
            this.lastTemperatureSetpoint =  seq > 0 ? this.temperatureDataFromSeq(seq - 1) : STANDARD_TEMPERATURE;
            
            Vector3d position = this.vehicle.getPosition();
            Vector3d localWpPosition = wpLocation;
            // First waypoint is in global frame (subtracting MLS/Ellipsoidal height) and negating (ENU -> NED)
            localWpPosition.z = seq != 1 ? localWpPosition.z : globalPosition.alt - (-1*localWpPosition.z);

            this.currentSeq = seq;
            // *0.9 to account for acceptance radius (Drone won't traverse the waypoint exactly most of the time)
            this.initialDistance = this.euclideanDistance(position, localWpPosition) * 0.9;
            this.waypointLocation = localWpPosition;
            
            // DEBUG PRINTS ---
            System.out.println(String.format("Current seq %d", seq));
            System.out.println(String.format("Drone position %f %f %f", position.x, position.y, position.z));
            System.out.println(String.format("Waypoint location %f %f %f", wpLocation.x, wpLocation.y, wpLocation.z));
            System.out.println(String.format("Initial distance %f", initialDistance));
            // ----
        }
    }

    @Override
    public void report(StringBuilder builder) {
        Vector3d windSetpoint = this.windDataforSeq(this.currentSeq);
        double tempSetpoint = this.temperatureDataFromSeq(this.currentSeq);

        builder.append("Weather Provider");
        builder.append(newLine);
        builder.append("================");
        builder.append(newLine);
        builder.append(String.format("Current mission item: %d", this.currentSeq));
        builder.append(newLine);
        builder.append(String.format("Mission leg completion: %f", this.getLegCompletion()));
        builder.append(newLine);
        
        builder.append("================");
        builder.append(newLine);
        builder.append(String.format("Wind setpoint: %f %f %f", windSetpoint.x, windSetpoint.y, windSetpoint.z));
        builder.append(newLine);
        builder.append(String.format("Current wind: %f %f %f", currentWind.x, currentWind.y, currentWind.z));
        builder.append(newLine);
        builder.append("================");
        builder.append(newLine);
        builder.append(String.format("Temperature setpoint: %f", tempSetpoint));
        builder.append(newLine);
        builder.append(String.format("Current temperature: %f", currentTemp));
        builder.append(newLine);
        builder.append(newLine);
    }

    void updateWeather() {
        double newTemp = this.interpolateScalar(this.lastTemperatureSetpoint, this.temperatureDataFromSeq(this.currentSeq), this.getLegCompletion());
        this.currentTemp = newTemp;
        
        Vector3d newWind = this.interpolateVectors(this.lastWindSetpoint, this.windDataforSeq(this.currentSeq), this.getLegCompletion());
        this.currentWind = newWind;
    }
}
