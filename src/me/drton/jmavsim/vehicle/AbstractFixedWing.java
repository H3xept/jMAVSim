package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;

/**
 * Abstract fixed wing vehicle class
 */
public abstract class AbstractFixedWing extends AbstractMulticopter {
    protected Rotor[] pusher_rotors;

    public AbstractFixedWing(World world, String modelName, boolean showGui) {
        super(world, modelName, showGui);
        pusher_rotors = new Rotor[getPusherRotorsNum()];
        for (int i = 0; i < getPusherRotorsNum(); i++) {
            pusher_rotors[i] = new Rotor();
        }
    }

    public void report(StringBuilder builder) {
        super.report(builder);
        builder.append("FIXED WING");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);

        for (int i = 0; i < getPusherRotorsNum(); i++) {
            reportRotor(builder, i);
            builder.append(newLine);
        }

    }

    private void reportRotor(StringBuilder builder, int rotorIndex) {
        Rotor rotor = pusher_rotors[rotorIndex];

        builder.append("ROTOR #");
        builder.append(rotorIndex);
        builder.append(newLine);

        builder.append("Control: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getControl())));
        builder.append(newLine);

        builder.append("Thrust: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getThrust())));
        builder.append(" / ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getFullThrust())));
        builder.append(" [N]");
        builder.append(newLine);

        builder.append("Torque: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getTorque())));
        builder.append(" / ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getFullTorque())));
        builder.append(" [Nm]");
        builder.append(newLine);

        builder.append("Spin up: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getTimeConstant())));
        builder.append(" [s]");
        builder.append(newLine);

        builder.append("Position: ");
        builder.append(ReportUtil.vector2str(getPusherRotorPosition(rotorIndex)));
        builder.append(newLine);

    }

    /**
     * Get number of pusher rotors.
     *
     * @return number of rotors
     */
    protected abstract int getPusherRotorsNum();

    /**
     * Get rotor position relative to gravity center of vehicle.
     *
     * @param i rotor number
     * @return rotor radius-vector from GC
     */
    protected abstract Vector3d getPusherRotorPosition(int i);

    @Override
    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }
        for (Rotor rotor : pusher_rotors) {
            rotor.update(t, paused);
        }
        super.update(t, paused);
        // Control for pusher rotors is #4
        final int rotor_offset = 4;
        for (int i = 0; i < pusher_rotors.length; i++) {
            double c = control.size() > i ? control.get(i+rotor_offset) : 0.0;
            pusher_rotors[i].setControl(c);
        }
    }

    @Override
    protected Vector3d getForce() {
        int n = getPusherRotorsNum();
        Vector3d f = new Vector3d();
        for (int i = 0; i < n; i++) {
            f.x += pusher_rotors[i].getThrust();
        }
        rotation.transform(f);
        Vector3d quadForce = super.getForce();
        f.add(quadForce);
        return f;
    }

    @Override
    protected Vector3d getTorque() {
        Vector3d quadTorque = super.getTorque();
        return quadTorque;
    }

}
