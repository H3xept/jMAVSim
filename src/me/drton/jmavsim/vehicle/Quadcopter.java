package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.Propeller;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.Sensors;
import me.drton.jmavsim.SimpleSensors;
import me.drton.jmavsim.World;

import javax.json.JsonNumber;
import javax.json.JsonObject;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Generic quadcopter model.
 */
public class Quadcopter extends AbstractMulticopter {
    private static final int rotorsNum = 4;
    private Vector3d[] rotorPositions = new Vector3d[rotorsNum];
    private int[] rotorRotations = new int[rotorsNum];
    private static final String MODEL_NAME = "models/3dr_arducopter_quad_x.obj";

    public static Quadcopter fromJSONObject(World w, boolean showGui, JsonObject obj, double payload_mass) {
        
        JsonObject main_params = obj.getJsonObject(AbstractVehicle.MAIN_PARAMS_KEY);
        JsonObject propeller_details = requiredJsonObject(main_params, Propeller.PROPELLER_KEY);
        double mass = main_params.getJsonNumber(AbstractMulticopter.MASS_KEY).doubleValue();
        double armLength = main_params.getJsonNumber(AbstractMulticopter.ARM_LENGTH_KEY).doubleValue();
        double maxRPM = main_params.getJsonNumber(AbstractMulticopter.MAX_RPM_KEY).doubleValue();
        double maxTorque = main_params.getJsonNumber(AbstractMulticopter.MAX_TORQUE_KEY).doubleValue();
        double dragMove = optionalDoubleValue(main_params, AbstractMulticopter.DRAG_MOVE_KEY, 0.01);

        Matrix3d inertia_matrix = partseInertiaMatrix(main_params);
        
        SimpleSensors sensors = new SimpleSensors();
        sensors.setGPSInterval(50);
        sensors.setGPSDelay(200);
        sensors.setNoise_Acc(0.05f);
        sensors.setNoise_Gyo(0.01f);
        sensors.setNoise_Mag(0.005f);
        sensors.setNoise_Prs(0.1f);
        
        Quadcopter q = new Quadcopter(
            w,
            armLength,
            maxRPM,
            maxTorque,
            AbstractMulticopter.ROTOR_TIME_CONSTANT,
            AbstractMulticopter.ROTOR_OFFSET,
            Propeller.fromJSONObject(propeller_details),
            showGui
        );
        q.setMass(mass + payload_mass);
        q.setMomentOfInertia(inertia_matrix);
        q.setDragMove(dragMove);
        q.setSensors(sensors, 0);

        return q;
    }

    /**
     * Generic quadcopter constructor.
     *
     * @param world          world where to place the vehicle
     * @param modelName      filename of model to load, in .obj format
     * @param armLength      length of arm from center [m]
     * @param rotorRPM       max RPM for one rotor [Revolutions per minute]
     * @param rotorTorque    torque at full thrust of one rotor in [Nm]
     * @param rotorTimeConst spin-up time of rotor [s]
     * @param rotorsOffset   rotors positions offset from gravity center
     * @param showGui        false if the GUI has been disabled
     */
    public Quadcopter(World world, double armLength, double rotorMaxRPM, double rotorTorque,
                      double rotorTimeConst, Vector3d rotorsOffset, Propeller propeller, boolean showGui) {
        super(world, MODEL_NAME, showGui, propeller);

        int i;

        rotorPositions[0] = new Vector3d(0.0, armLength, 0.0);
        rotorPositions[1] = new Vector3d(0.0, -armLength, 0.0);
        rotorPositions[2] = new Vector3d(armLength, 0.0, 0.0);
        rotorPositions[3] = new Vector3d(-armLength, 0.0, 0.0);
        for (i = 0; i < rotorsNum; ++i) {
            rotorRotations[i] = (i < 2) ? -1 : 1;
        }

        Matrix3d r = new Matrix3d();
        r.rotZ(-Math.PI / 4);
        for (i = 0; i < rotorsNum; i++) {
            r.transform(rotorPositions[i]);
        }

        for (i = 0; i < rotors.length; i++) {
            rotorPositions[i].add(rotorsOffset);
            Rotor rotor = rotors[i];
            rotor.setMaxRPM(rotorMaxRPM);
            rotor.setFullTorque(rotorTorque * rotorRotations[i]);
            rotor.setTimeConstant(rotorTimeConst);
        }
    }

    @Override
    protected int getRotorsNum() {
        return rotorsNum;
    }

    @Override
    protected Vector3d getRotorPosition(int i) {
        return rotorPositions[i];
    }
}
