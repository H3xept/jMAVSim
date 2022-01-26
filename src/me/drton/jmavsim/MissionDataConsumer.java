package me.drton.jmavsim;
import javax.vecmath.Vector3d;
import me.drton.jmavlib.geo.LatLonAlt;

public interface MissionDataConsumer {
    public void missionDataUpdated(int seq, Vector3d wpLocation, LatLonAlt globalPosition);
}
