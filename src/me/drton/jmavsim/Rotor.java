package me.drton.jmavsim;

/**
 * Simple rotor model. Thrust and torque are proportional to control signal filtered with simple LPF (RC filter), to
 * simulate spin up/slow down.
 */
public class Rotor {
    
    /**
     *  NOTE: I'm sorry. Everything related to thrust is in imperial units.
     *  I'm keeping it so that our implementation is consistent with the paper's.
     *  Please use SI units everywhere -- expose a SI-based interface where not possible.
     */
    private final static double AIR_DENSITY_IMPERIAL = 0.0739; // standard air density at 25 deg (lb / ft**3)

    private double tau = 1.0;
    private double fullTorque = 1.0;
    private double w = 0.0;
    private long lastTime = -1;
    private double control = 0.0;
    private double maxRPM;
    private double KF;
    private double propeller_diameter_inches;
    private double propeller_pitch_inches;
    private double propeller_blades;

    // Something's wrong, I can feel it...
    private static double cmToInches(double cm) {
        return cm / 2.54;
    }

    public Rotor(Propeller propeller) {
        this.propeller_diameter_inches = Rotor.cmToInches(propeller.propeller_diameter_cm);
        this.propeller_pitch_inches = Rotor.cmToInches(propeller.propeller_pitch_cm);
        this.propeller_blades = propeller.propeller_blades;
        this.KF = this.compute_kf();
    }

    private double compute_kf() {
        double e_d = this.getEd();
        double theta = this.getTheta();
        double k = this.getK();
        double C_t = this.getCt(k, theta, e_d);
        return this.k_f(this.propeller_diameter_inches / 2.0, e_d, C_t);
    }

    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }

        if (lastTime >= 0) {
            double dt = (t - lastTime) / 1000.0;
            w += (control - w) * (1.0 - Math.exp(-dt / tau));
        }
        lastTime = t;
    }

    /**
     * Set control signal
     * @param control control signal normalized to [0...1] for traditional or [-1...1] for reversable rotors
     */
    public void setControl(double control) {
        this.control = control;
    }

    public double getMaxRPM() {
        return this.maxRPM;
    }

    public void setMaxRPM(double maxRPM) {
        this.maxRPM = maxRPM;
    }

    /**
     * Set torque at full thrust
     * @param fullTorque [N * m]
     */
    public void setFullTorque(double fullTorque) {
        this.fullTorque = fullTorque;
    }

    /**
     * Set time constant (spin-up time).
     * @param timeConstant [s]
     */
    public void setTimeConstant(double timeConstant) {
        this.tau = timeConstant;
    }

    /**
     * Get control signal
     */
    public double getControl() {
        return control;
    }

    public double getRPM() {
        return w * maxRPM;
    }

    public double getW() {
        return w;
    }

    /**
     * Get current rotor torque [N * m]
     */
    public double getTorque() {
        return control * fullTorque;
    }

    // Blade effectiveness: (14) of https://downloads.hindawi.com/journals/ijae/2018/9632942.pdf
    private double k_f(double R, double e_d, double C_t) {
        return (1.0/16.0) * Rotor.AIR_DENSITY_IMPERIAL * Math.PI * Math.pow(R, 4) * Math.pow(e_d, 4) * C_t;
    }

    private double getCt(double k, double theta, double e_d) {
        return 4.0/3.0 * k * theta * (1 - (Math.pow((1 - e_d), 3))) - k * (Math.sqrt((k * (1 + k))) - Math.sqrt(k)) * (1 - (Math.pow((1 - e_d), 2)));
    }

    private double getEd() {
        double lambda = this.propeller_pitch_inches / this.propeller_diameter_inches;
        if (lambda < 0.4) {
            return 0.91;
        } else if (lambda >= 0.4 && lambda < 0.8) {
            return 0.88;
        } else if (lambda >= 0.8 && lambda < 0.9) {
            return 0.83;
        } else {
            return 0.80;
        }
    }

    private double getTheta() {
        return Math.atan(this.propeller_pitch_inches  / (Math.PI * this.propeller_diameter_inches));
    }

    private double getCDRatio() {
        if (this.propeller_diameter_inches < 5.0) {
            return 0.09;
        } else if (this.propeller_diameter_inches >= 5.0 && this.propeller_diameter_inches < 7.0) {
            return 0.1;
        } else if (this.propeller_diameter_inches >= 7.0 && this.propeller_diameter_inches < 10.0) {
            return 0.11;
        } else if (this.propeller_diameter_inches >= 10.0 && this.propeller_diameter_inches < 13.0) {
            return 0.12;
        } else if (this.propeller_diameter_inches >= 13.0 && this.propeller_diameter_inches < 15.0) { 
            return 0.13;
        } else if (this.propeller_diameter_inches >= 15.0 && this.propeller_diameter_inches < 17.0) {
            return 0.14;
        } else {
            System.out.println("Could not calculate C_D ratio for a propeller this big.");
            System.out.println("The thrust model does not model propellers larger than "+Rotor.cmToInches(17.0)+" centimeters.");
            System.exit(1);
            return 0.0;
        }
    }

    private double getK() {
        return this.propeller_blades * 0.5 * this.getCDRatio();
    }

    private double rpmToRadS(double rpm) {
        return rpm * 0.0010472;
    }

    public double getKF() {
        return this.KF;
    }

    /**
     * Get current rotor thrust, [N]
     */
    public double getThrust() {
        return Math.pow(this.rpmToRadS(this.getRPM()), 2) * this.getKF();
    }

    /**
     * Get torque at full thrust, [N * m]
     */
    public double getFullTorque() {
        return fullTorque;
    }

    /**
     * Get time constant (spin-up time), [s].
     */
    public double getTimeConstant() {
        return tau;
    }
}
