package me.drton.jmavsim;
import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonNumber;
import javax.json.JsonReader;
import javax.vecmath.Vector3d;

import me.drton.jmavsim.vehicle.AbstractVehicle;
import me.drton.jmavsim.ReportingObject;

import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;

import java.io.StringReader;


public class WeatherProvider implements ReportingObject, MissionDataConsumer {
    private static final String WIND_KEY = "wind_data";
    private static final String TEMP_KEY = "temperature_data";
    private static final Vector3d NO_WIND = new Vector3d();
    private static final double STANDARD_TEMPERATURE = 25.0;

    private AbstractVehicle vehicle;
    private double currentTemp = STANDARD_TEMPERATURE;
    private Vector3d currentWind = NO_WIND;
    private Vector3d lastWindSetpoint = NO_WIND;
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
        System.out.println("Using test data for wind and temperature. Only valid for testing purposes!");
        this.obj = testData();
        this.vehicle = vehicle;
    }

    public WeatherProvider(String fileHandle, AbstractVehicle vehicle) {
        try (JsonReader jsonReader = Json.createReader(new StringReader(fileHandle))){
            this.obj = jsonReader.readObject();
            this.vehicle = vehicle;
        } catch (Exception e) {
            System.out.println("Mission items not initialised!");
            System.exit(-42);
        }
    }
    
    private Vector3d windDataforSeq(int seq) {
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
        double newTemp = this.interpolateScalar(this.currentTemp, this.temperatureDataFromSeq(this.currentSeq), this.getLegCompletion());
        return newTemp;
    }

    public Vector3d getWind() {
        if (initialDistance == 0) return this.windDataforSeq(this.currentSeq);
        Vector3d newWind = this.interpolateVectors(this.lastWindSetpoint, this.windDataforSeq(this.currentSeq), this.getLegCompletion());
        this.currentWind = newWind;
        return newWind;
    }

    private double euclideanDistance(Vector3d a, Vector3d b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }

    private double getLegCompletion() {
        if (initialDistance < 0.1) return 0;
        double distance = this.euclideanDistance(vehicle.getPosition(), this.waypointLocation);
        return Math.min(Math.max(1 - (distance / this.initialDistance), 0), 1.0);
    }

    @Override
    public void missionDataUpdated(int seq, Vector3d wpLocation) {
        if (currentSeq != seq && Math.abs(wpLocation.x) < 5000 && Math.abs(wpLocation.y) < 5000) {
            
            if (seq > 0) {
                this.lastWindSetpoint = this.windDataforSeq(seq-1);
                this.currentTemp =  this.temperatureDataFromSeq(seq-1);
            } else {
                this.lastWindSetpoint = NO_WIND;
                this.currentTemp = STANDARD_TEMPERATURE;
            }

            this.currentSeq = seq;
            // *0.9 to account for acceptance radius (Drone won't traverse the waypoint exactly most of the time)
            this.initialDistance = this.euclideanDistance(this.vehicle.getPosition(), wpLocation) * 0.9;
            this.waypointLocation = wpLocation;
            System.out.println(String.format("Initial distance %f", initialDistance));
            System.out.println(String.format("Waypoint location %f %f %f", wpLocation.x, wpLocation.y, wpLocation.z));
        }
    }

    @Override
    public void report(StringBuilder builder) {
        builder.append("Weather Provider");
        builder.append(newLine);
        builder.append("================");
        builder.append(newLine);
        builder.append(String.format("Current mission item: %d", this.currentSeq));
        builder.append(newLine);
        builder.append(String.format("Mission leg completion: %f", this.getLegCompletion()));
        builder.append(newLine);
        Vector3d setpoint = this.windDataforSeq(this.currentSeq);
        builder.append(String.format("Wind setpoint: %f %f %f", setpoint.x, setpoint.y, setpoint.z));
        builder.append(newLine);
        builder.append(String.format("Current wind: %f %f %f", currentWind.x, currentWind.y, currentWind.z));
        builder.append(newLine);
    }
}
