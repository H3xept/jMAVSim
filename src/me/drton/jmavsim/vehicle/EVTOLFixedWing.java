package me.drton.jmavsim.vehicle;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.SimpleSensors;
import me.drton.jmavsim.World;
import java.util.Map;
import javax.json.JsonNumber;
import javax.json.JsonObject;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import static java.util.Map.entry;    

public class EVTOLFixedWing extends AbstractFixedWing {

    private final int VTOL_ROTOR_N = 4;
    private final int PUSHER_ROTOR_N = 1;


    // -----

    private Vector3d[] vtolRotorPositions = new Vector3d[VTOL_ROTOR_N];
    private int[] vtolRotorRotations = new int[VTOL_ROTOR_N];
    private Vector3d[] pusherRotorPositions = new Vector3d[PUSHER_ROTOR_N];
    private int[] pusherRotorRotations = new int[PUSHER_ROTOR_N];

    public static EVTOLFixedWing fromJSONObject(World w, boolean showGui, JsonObject obj) {
        
        JsonObject main_config = requiredJsonObject(obj, AbstractVehicle.MAIN_PARAMS_KEY);
        double mass = requiredDoubleValue(main_config, AbstractMulticopter.MASS_KEY);
        double armLength = requiredDoubleValue(main_config, AbstractMulticopter.ARM_LENGTH_KEY);
        double maxThrust = requiredDoubleValue(main_config, AbstractMulticopter.MAX_THRUST_KEY);
        double maxTorque = requiredDoubleValue(main_config, AbstractMulticopter.MAX_TORQUE_KEY);
        double dragMove = optionalDoubleValue(main_config, AbstractMulticopter.DRAG_MOVE_KEY, 0.01);
        double tailLength = optionalDoubleValue(main_config, AbstractFixedWing.TAIL_LENGTH_KEY, 0.4);
        double maxBackThrust = requiredDoubleValue(main_config, AbstractFixedWing.MAX_BACK_PROPELLER_THRUST_KEY);

        Matrix3d inertia_matrix = partseInertiaMatrix(main_config);
        APM aeroData = AbstractFixedWing.parseAeroData(requiredJsonObject(obj, AbstractFixedWing.AERODYNAMICS_KEY));

        SimpleSensors sensors = new SimpleSensors();
        sensors.setGPSInterval(50);
        sensors.setGPSDelay(200);
        sensors.setNoise_Acc(0.05f);
        sensors.setNoise_Gyo(0.01f);
        sensors.setNoise_Mag(0.005f);
        sensors.setNoise_Prs(0.1f);
        
        EVTOLFixedWing q = new EVTOLFixedWing(
            w,
            aeroData,
            AbstractFixedWing.MODEL_NAME,
            armLength,
            tailLength,
            maxThrust,
            maxBackThrust,
            maxTorque,
            AbstractMulticopter.ROTOR_TIME_CONSTANT,
            AbstractMulticopter.ROTOR_OFFSET,
            showGui
        );
        
        q.setMass(mass);
        q.setMomentOfInertia(inertia_matrix);
        q.setDragMove(dragMove);
        q.setSensors(sensors, 0);

        return q;
    }

    public EVTOLFixedWing(
        World world,
        APM aeroData,
        String modelName,
        double armLength,
        double tailLength,
        double rotorThrust,
        double backRotorThrust,
        double rotorTorque,
        double rotorTimeConst,
        Vector3d rotorsOffset,
        boolean showGui
        ) {
        super(world, modelName, showGui, aeroData.mean_aerodynamic_chord, aeroData.wing_span, aeroData.wing_area, aeroData);

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
        pusherRotor.setFullThrust(backRotorThrust);
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
    
    @Override
    protected Vector3d getGyroSensor() {
        return this.sensors.getGyro();
    }
}
