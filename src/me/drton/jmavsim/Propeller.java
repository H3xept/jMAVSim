package me.drton.jmavsim;
import javax.json.JsonObject;

public class Propeller {
    public static final String PROPELLER_KEY = "propeller_specs";
    public static final String DIAMETER_KEY = "diameter_cm";
    public static final String PITCH_KEY = "pitch_cm";
    public static final String BLADES_N_KEY = "blades_n";
    // The diameter of the disk swept by the rotating propeller with center in the rotor hub's center.
    public double propeller_diameter_cm;
    // The pitch is a measure of how far the propeller would move forwards in one revolution if it were
    // treated as a screw and screwed into some solid material. The size of a propeller is usually
    // expressed in the form diameter x pitch. For example, an 8×4 propeller has an 8 inch diameter and 4 inch pitch.
    public double propeller_pitch_cm;
    // Number of blades on the propeller.
    public double propeller_blades;

    public static Propeller fromJSONObject(JsonObject obj) {
        try {
            // get propeller diameter
            double propeller_diameter_cm = obj.getJsonNumber(Propeller.DIAMETER_KEY).doubleValue();
            // get propeller pitch
            double propeller_pitch_cm = obj.getJsonNumber(Propeller.PITCH_KEY).doubleValue();
            // get propeller blades n
            double propeller_blades = obj.getJsonNumber(Propeller.BLADES_N_KEY).intValue();
            return new Propeller(propeller_diameter_cm, propeller_pitch_cm, propeller_blades);
        } catch (Exception e) {
            System.out.println("Error parsing propeller parameters. Make sure the '"+PROPELLER_KEY+"' key is present in the vehicle JSON file under drone_config.");
            System.out.println(e);
            System.exit(1);
            return null;
        }
    } 

    public Propeller(double propeller_diameter_cm, double propeller_pitch_cm, double propeller_blades_n) {
        this.propeller_diameter_cm = propeller_diameter_cm;
        this.propeller_pitch_cm = propeller_pitch_cm;
        this.propeller_blades = propeller_blades_n;
    }
}
