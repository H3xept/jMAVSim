package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class AVYAera extends AbstractFixedWing {

    private final int VTOL_ROTOR_N = 4;
    private final int PUSHER_ROTOR_N = 1;

    private Vector3d[] vtolRotorPositions = new Vector3d[VTOL_ROTOR_N];
    private int[] vtolRotorRotations = new int[VTOL_ROTOR_N];
    private Vector3d[] pusherRotorPositions = new Vector3d[PUSHER_ROTOR_N];
    private int[] pusherRotorRotations = new int[PUSHER_ROTOR_N];

    public AVYAera(
        World world,
        String modelName,
        double armLength,
        double tailLength,
        double rotorThrust,
        double rotorTorque,
        double rotorTimeConst,
        Vector3d rotorsOffset,
        boolean showGui
        ) {
        super(world, modelName, showGui);

        int i;

        vtolRotorPositions[0] = new Vector3d(0.0, armLength, 0.0);
        vtolRotorPositions[1] = new Vector3d(0.0, -armLength, 0.0);
        vtolRotorPositions[2] = new Vector3d(armLength, 0.0, 0.0);
        vtolRotorPositions[3] = new Vector3d(-armLength, 0.0, 0.0);
        for (i = 0; i < VTOL_ROTOR_N; ++i) {
            vtolRotorRotations[i] = (i < 2) ? -1 : 1;
        }

        pusherRotorPositions[0] = new Vector3d(-tailLength, 0.0, 0.0);
        pusherRotorRotations[0] = 1;
        Rotor pusherRotor = pusher_rotors[0];
        pusherRotor.setFullThrust(rotorThrust);
        pusherRotor.setFullTorque(rotorTorque * pusherRotorRotations[0]);
        pusherRotor.setTimeConstant(rotorTimeConst);

        Matrix3d r = new Matrix3d();
        r.rotZ(-Math.PI / 4);
        for (i = 0; i < VTOL_ROTOR_N; i++) {
            r.transform(vtolRotorPositions[i]);
        }

        for (i = 0; i < rotors.length; i++) {
            vtolRotorPositions[i].add(rotorsOffset);
            Rotor rotor = rotors[i];
            rotor.setFullThrust(rotorThrust);
            rotor.setFullTorque(rotorTorque * vtolRotorRotations[i]);
            rotor.setTimeConstant(rotorTimeConst);
        }
    }

    @Override
    protected int getPusherRotorsNum() {
        return PUSHER_ROTOR_N;
    }

    @Override
    protected Vector3d getPusherRotorPosition(int i) {
        return this.pusherRotorPositions[i];
    }

    @Override
    protected int getRotorsNum() {
        return VTOL_ROTOR_N;
    }

    @Override
    protected Vector3d getRotorPosition(int i) {
        return this.vtolRotorPositions[i];
    }
    
}
