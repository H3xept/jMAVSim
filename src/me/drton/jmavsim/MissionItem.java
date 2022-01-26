package me.drton.jmavsim;

public class MissionItem {
    private double latitude;
    private double longitude;
    private double altitude;
    private double sequence_number;
    
    public MissionItem(double lat, double lon, double alt, int seq) {
        this.latitude = lat;
        this.longitude = lon;
        this.altitude = alt;
        this.sequence_number = seq;
    }

}
