package me.drton.jmavsim;
import javax.vecmath.Vector3d;

public interface MissionDataConsumer {
    public void missionDataUpdated(int seq, Vector3d distanceToNextWp);
}
