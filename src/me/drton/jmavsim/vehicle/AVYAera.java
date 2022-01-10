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
    private static final double M_C = 0.26;
    private static final double M_B = 0.9;
    private static final double M_S = 0.22;

    private static final APM AERO_DATA = new APM(Map.ofEntries(
        entry("m_CL_0",0.0389),
        entry("m_CL_alpha",3.2684),
        entry("m_CL_delta_e",0.7237),
        entry("m_CL_q",6.1523),
        entry("m_CD_0",0.0208),
        entry("m_CD_alpha",0.0084),
        entry("m_CD_alpha2",1.3225),
        entry("m_CD_delta_e2",0.2),
        entry("m_CD_beta2",0.0796),
        entry("m_CD_beta",-0.0001),
        entry("m_CD_q",0.0),
        entry("m_CS_0",0.0),
        entry("m_CS_beta",-0.1285),
        entry("m_CS_delta_a",0.0299),
        entry("m_CS_delta_r",0.0),
        entry("m_CS_p",-0.0292),
        entry("m_CS_r",-0.0355),
        entry("m_Cm_0",-0.0112),
        entry("m_Cm_alpha",-0.2625),
        entry("m_Cm_delta_e",-0.2845),
        entry("m_Cm_q",-1.8522),
        entry("m_Cl_0",0.0),
        entry("m_Cl_beta",-0.0345),
        entry("m_Cl_delta_a",0.182),
        entry("m_Cl_delta_r",0.0),
        entry("m_Cl_p",-0.3318),
        entry("m_Cl_r",0.0304),
        entry("m_Cn_0",0.0),
        entry("m_Cn_beta",0.0252),
        entry("m_Cn_delta_a",-0.0102),
        entry("m_Cn_delta_r",0.0),
        entry("m_Cn_p",0.0),
        entry("m_Cn_r",-0.0192)
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
