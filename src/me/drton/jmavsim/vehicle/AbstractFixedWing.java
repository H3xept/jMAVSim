package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;

/**
 * Abstract fixed wing vehicle class
 */
public abstract class AbstractFixedWing extends AbstractMulticopter {
    protected Rotor[] pusher_rotors;
    private double[] ailerons_control = new double[]{0.0, 0.0};
    private double elevator_control = 0.0;

    private double maxAngleOfAttack = Math.toRadians(30);

    private double m_c = 0.0;
    private double m_b = 0.0;
    private double m_S = 0.0;
    private APM aero_data;

    public AbstractFixedWing(World world, String modelName, boolean showGui, double m_c, double m_b, double m_S, APM aero_data) {
        super(world, modelName, showGui);

        this.m_c = m_c;
        this.m_b = m_b;
        this.m_S = m_S;
        this.aero_data = aero_data;

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

        builder.append(newLine);

        Vector3d aero_force = this.getAeroForce();
        builder.append("AERODYNAMICS");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);
        builder.append(String.format("Aerodynamic force: x:%f y:%f z:%f", aero_force.x, aero_force.y, aero_force.z));
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);
        builder.append(String.format("Aileron control left:%f right:%f (%f rad)", this.ailerons_control[0], ailerons_control[1], surfaceControlToAngle(this.ailerons_control[0])));
        builder.append(newLine);
        builder.append(String.format("Elevator control %f (%f rad)", this.elevator_control, surfaceControlToAngle(this.elevator_control)));
        builder.append(newLine);
        builder.append(String.format("Angle of attack %f", (this.computeM_Alpha() * 180.0) / 3.14));
        builder.append(newLine);
        builder.append(String.format("Angle of sideslip %f", (this.computeM_Beta()* 180.0) / 3.14));
        builder.append(newLine);
        
        Vector3d aero_torque = this.getAeroTorque();
        builder.append("Aerodynamic torque");
        builder.append(newLine);
        builder.append(String.format("%f %f %f", aero_torque.x, aero_torque.y, aero_torque.z));
        builder.append(newLine);
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

        // Control for ailerons is #5 &&  #6 (aileron left and right)
        final int aileron_offset = rotor_offset + this.pusher_rotors.length;
        for (int i = 0; i < ailerons_control.length; i++) {
            double c = control.size() > i ? control.get(i+aileron_offset) : 0.0;
            ailerons_control[i] = c;
        }
        
        final int elevator_offset = aileron_offset + this.ailerons_control.length;
        this.elevator_control = control.size() > elevator_offset ? control.get(elevator_offset) : 0.0;
    }

    private double surfaceControlToAngle(double control) {
        return control * this.maxControlSurfaceDeflection();
    }

    protected double maxControlSurfaceDeflection() {
        return Math.toRadians(45);
    }
    
    protected Vector3d getVTOLForce() {
        return super.getForce();
    }
    
    protected Vector3d getPusherForce() {
        int n = getPusherRotorsNum();
        Vector3d f = new Vector3d();
        for (int i = 0; i < n; i++) {
            f.x += pusher_rotors[i].getThrust();
        }
        rotation.transform(f);
        return f;
    }

// Aerodynamics

    Matrix3d body2wind() {
        double alpha = this.computeM_Alpha();
        double beta = this.computeM_Beta();
        Matrix3d rot = new Matrix3d();
        rot.m00 = Math.cos(alpha) * Math.cos(beta);
        rot.m01 = Math.sin(beta);
        rot.m02 = Math.sin(alpha) * Math.cos(beta);
        rot.m10 = -Math.cos(alpha) * Math.sin(beta);
        rot.m11 = Math.cos(beta);
        rot.m12 = -Math.sin(alpha) * Math.sin(beta);
        rot.m20 = -Math.sin(alpha);
        rot.m22 = Math.cos(alpha);
        return rot;
    }

    double computeM_rho(){
        return 1.225;
    }

    double computeT_rho(){
        return 288.15;
    }

    double computeM_Alpha() {
        Vector3d velocity = new Vector3d(this.getVelocity());
        Matrix3d bodyRot = new Matrix3d(this.rotation);
        bodyRot.transpose();
        bodyRot.transform(velocity);
        return Math.atan2(velocity.z,velocity.x);
    }

    double computeM_Beta(){
        Vector3d velocity = new Vector3d(this.getVelocity());
        Matrix3d bodyRot = new Matrix3d(this.rotation);
        bodyRot.transpose();
        bodyRot.transform(velocity);
        return Math.asin(velocity.y/Math.sqrt(velocity.x*velocity.x+velocity.y*velocity.y+velocity.z*velocity.z));
    }

    double computeVmod(){
        Vector3d velocity = new Vector3d(this.getVelocity());
        Matrix3d bodyRot = new Matrix3d(this.rotation);
        bodyRot.transpose();
        bodyRot.transform(velocity);
        double m_Va = Math.sqrt(velocity.x*velocity.x+velocity.y*velocity.y+velocity.z*velocity.z);
        return m_Va;
    }

    private Vector3d getAeroForce() {
        
        double m_Va = this.computeVmod();
        double m_alpha = this.computeM_Alpha();
        double m_beta = this.computeM_Beta();
        double m_rho = this.computeM_rho();
        
        if (Double.isNaN(m_alpha) || Double.isNaN(m_beta) || Math.abs(m_alpha) >= maxAngleOfAttack) return new Vector3d();

        Vector3d rot_rate = this.getRotationRate();
        
        double elevator_deflection = -this.surfaceControlToAngle(this.elevator_control);
        double aileron_deflection = this.surfaceControlToAngle(this.ailerons_control[1]);

        double m_CD = aero_data.m_CD_0 + aero_data.m_CD_alpha*m_alpha + aero_data.m_CD_alpha2*m_alpha*m_alpha +
                aero_data.m_CD_delta_e2*elevator_deflection*elevator_deflection + aero_data.m_CD_beta*m_beta +
                aero_data.m_CD_beta2*m_beta*m_beta + aero_data.m_CD_q*m_c/(2.*m_Va)*rot_rate.y;
        double m_CS = aero_data.m_CS_0 + aero_data.m_CS_beta*m_beta + aero_data.m_CS_delta_a*aileron_deflection + m_b/(2.*m_Va)*
                (aero_data.m_CS_p*rot_rate.x + aero_data.m_CS_r*rot_rate.z);
        double m_CL = aero_data.m_CL_0 + aero_data.m_CL_alpha*m_alpha + aero_data.m_CL_delta_e*elevator_deflection +
                aero_data.m_CL_q*m_c/(2.*m_Va)*rot_rate.y;

        Vector3d f = new Vector3d();
        f.x = -0.5*m_rho*m_Va*m_Va*m_S*m_CD;
        f.y = 0.5*m_rho*m_Va*m_Va*m_S*m_CS;
        f.z = -0.5*m_rho*m_Va*m_Va*m_S*m_CL;
        
        Matrix3d rot = this.body2wind();
        rot.transpose();
        // // Wind to body to earth
        rot.transform(f);
        rotation.transform(f);

        return f;
    }

// -----

    @Override
    protected Vector3d getForce() {
        Vector3d total_f = new Vector3d();
        Vector3d pusher_force = this.getPusherForce();
        Vector3d vtol_force = this.getVTOLForce();
        Vector3d aero_force = this.getAeroForce();
        total_f.add(vtol_force);
        total_f.add(pusher_force);
        total_f.add(aero_force);
        return total_f;
    }
    
    protected Vector3d getAeroTorque() {
        
        double m_Va = this.computeVmod();
        double m_alpha = this.computeM_Alpha();
        double m_beta = this.computeM_Beta();
        double m_rho = this.computeM_rho();
        
        if (Double.isNaN(m_alpha) || Double.isNaN(m_beta) || Math.abs(m_alpha) >= maxAngleOfAttack) return new Vector3d();

                
        double elevator_deflection = -this.surfaceControlToAngle(this.elevator_control);
        double aileron_deflection = this.surfaceControlToAngle(this.ailerons_control[1]);

        Vector3d rot_rate = this.getRotationRate();
        double m_Cl = aero_data.m_Cl_0 + aero_data.m_Cl_beta*m_beta + aero_data.m_Cl_delta_a*aileron_deflection +
        m_b/(2.*m_Va)*(aero_data.m_Cl_p*rot_rate.x + aero_data.m_Cl_r*rot_rate.z);
        double m_Cm = aero_data.m_Cm_0 + aero_data.m_Cm_alpha*m_alpha + aero_data.m_Cm_delta_e*elevator_deflection +
                aero_data.m_Cm_q*m_c/(2.*m_Va)*rot_rate.y;
        double m_Cn = aero_data.m_Cn_0 + aero_data.m_Cn_beta*m_beta + aero_data.m_Cn_delta_a*aileron_deflection +
                m_b/(2.*m_Va)*(aero_data.m_Cn_p*rot_rate.x + aero_data.m_Cn_r*rot_rate.z);

        Vector3d m_M = new Vector3d();
        m_M.x = 0.5*m_rho*m_Va*m_Va*m_Cl*m_S*m_b;
        m_M.y = 0.5*m_rho*m_Va*m_Va*m_Cm*m_S*m_c;
        m_M.z = 0.5*m_rho*m_Va*m_Va*m_Cn*m_S*m_b;

        this.getRotation().transform(m_M);
        return m_M;
    }
    
    @Override
    protected Vector3d getTorque() {
        // TODO: calculate pusher propeller torque
        Vector3d total_torque = new Vector3d();
        Vector3d vtol_torque = super.getTorque();
        Vector3d aero_torque = this.getAeroTorque();
        total_torque.add(vtol_torque);
        total_torque.add(aero_torque);
        return total_torque;
    }

}
