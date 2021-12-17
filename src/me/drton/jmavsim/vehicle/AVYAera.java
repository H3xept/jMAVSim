package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import static java.util.Map.entry;    

public class AVYAera extends AbstractFixedWing {

    private final int VTOL_ROTOR_N = 4;
    private final int PUSHER_ROTOR_N = 1;

    // AERO COEFFICIENTS
    private static final double M_C = 0.3571;
    private static final double M_B = 2.1;
    private static final double M_S = 0.75;

    private static final APM AERO_DATA = new APM(Map.ofEntries(
        entry("m_CL_0", 0.0867),
        entry("m_CL_alpha", 4.02),
        entry("m_CL_delta_e", 0.278),
        entry("m_CL_q", 3.8700),
        entry("m_CD_0", 0.0197),
        entry("m_CD_alpha", 0.0791),
        entry("m_CD_alpha2", 1.06),
        entry("m_CD_delta_e2", 0.0633),
        entry("m_CD_beta2", 0.148),
        entry("m_CD_beta", -0.00584),
        entry("m_CS_0", 0.0),
        entry("m_CS_beta", -0.224),
        entry("m_CS_delta_a", 0.0433),
        entry("m_CS_p", -0.1374),
        entry("m_CS_r", 0.0839)
    ));
    // -----


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
        super(world, modelName, showGui, M_C, M_B, M_S, AERO_DATA);

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
