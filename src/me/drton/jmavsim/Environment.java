package me.drton.jmavsim;
import me.drton.jmavlib.geo.LatLonAlt;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * User: ton Date: 28.11.13 Time: 20:35
 */
public abstract class Environment extends WorldObject implements ReportingObject, MissionDataConsumer{
    protected Vector3d g = new Vector3d();  // gravity
    protected Vector3d magField = new Vector3d();
    protected Vector3d wind = new Vector3d();   // base wind speed
    protected Vector3d windDeviation = new Vector3d();  // deviation magnitude
    protected Vector3d windCurrent = new Vector3d();  // current wind conditions (base plus deviation)
    protected double windT = 2.0;
    protected double groundLevel = 0.0;
    protected Float magIncl = 0.0f;
    protected Float magDecl = 0.0f;
    protected double magHIntensity;
    protected double magTIntensity;
    protected WeatherProvider weather;
    private Boolean landing_height_updated = false;

    public Environment(World world, WeatherProvider weather) {
        super(world);
        this.weather = weather;
    }

    public void report(StringBuilder builder) {
        builder.append("ENVIRONMENT");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);

        builder.append("Grav: ");
        builder.append(ReportUtil.vector2str(g));
        builder.append(newLine);

        builder.append("Mag: ");
        builder.append(ReportUtil.vector2str(magField));
        builder.append(newLine);
        builder.append(String.format("     Incl: %.5f; Decl: %.5f", magIncl, magDecl));
        builder.append(newLine);
        builder.append(String.format("     H-Magn: %.5f; T-Magn: %.5f", magHIntensity, magTIntensity));
        builder.append(newLine);

        builder.append("Wind Set: ");
        builder.append(ReportUtil.vector2str(wind));
        builder.append(newLine);
        builder.append("Wind Dev: ");
        builder.append(ReportUtil.vector2str(windDeviation));
        builder.append(newLine);
        builder.append("Wind Cur: ");
        builder.append(ReportUtil.vector2str(windCurrent));
        builder.append(newLine);
        builder.append(newLine);
        this.weather.report(builder);
    }


    public Vector3d getWind() {
        return weather.getWind();
    }

    public void setWind(Vector3d wind) {
        this.wind = wind;
    }

    public Vector3d getWindDeviation() {
        return this.windDeviation;
    }

    public void setWindDeviation(Vector3d deviation) {
        this.windDeviation = deviation;
    }

    /**
     * Get wind (air velocity) vector in specified point.
     *
     * @param point point in NED frame
     * @return wind vector
     */
    public Vector3d getCurrentWind(Vector3d point) {
        return windCurrent;
    }

    // Celsius degs
    public double getCurrentTemperature() {
        return weather.getTemperature();
    }
    
    public void setCurrentWind(Vector3d wind) {
        this.windCurrent = wind;
    }

    /**
     * The base ground level member.
     */
    public void setGroundLevel(double groundLevel) {
        this.groundLevel = groundLevel;
    }
    public double getGroundLevel() {
        return groundLevel;
    }

    /**
     * Get ground level for specified point.
     * Multilevel environment may be simulated, in this case method should return level under specified point.
     *
     * @param point point in NED frame
     * @return ground level in NED frame
     */
    public double getGroundLevelAt(Vector3d point) {
        return getGroundLevel();
    }

    /**
     * Get gravity vector.
     * Should be pointed to ground.
     *
     * @return gravity vector
     */
    public Vector3d getG() {
        return g;
    }
    public void setG(Vector3d grav) {
        if (grav == null) {
            g = new Vector3d();
        } else {
            g = grav;
        }
    }

    /**
     * Get magnetic field vector in specified point.
     *
     * @param point point in NED frame
     * @return magnetic field vector
     */
    public Vector3d getMagField(Vector3d point) {
        return magField;
    }

    /**
     * Set magnetic field using specific vector.
     *
     * @param magField vector in NED frame
     */
    public void setMagField(Vector3d magField) {
        this.magField = magField;
        magHIntensity = Math.sqrt(magField.x * magField.x + magField.y * magField.y);
        magTIntensity = Math.sqrt(magHIntensity * magHIntensity + magField.z * magField.z);
        magIncl = (float)Math.toDegrees(Math.atan2(magField.z, magField.x));
        magDecl = (float)Math.toDegrees(Math.atan2(magField.y, magField.x));
    }

    /**
     * Set magnetic field using specified inclination and declination
     *
     * @param incl Magnetic inclination in degrees
     * @param decl Magnetic declination in degrees
     */
    public void setMagFieldByInclDecl(double incl, double decl) {
        decl = Math.toRadians(decl);
        incl = Math.toRadians(incl);
        Vector3d magField = new Vector3d(Math.cos(incl), 0.0f, Math.sin(incl));
        Matrix3d declMtx = new Matrix3d();
        declMtx.rotZ(decl);
        declMtx.transform(magField);
        setMagField(magField);
    }

    @Override
    public void missionDataUpdated(int seq, Vector3d wpLocation, LatLonAlt globalPosition) {
        this.weather.missionDataUpdated(seq, wpLocation, globalPosition);
        if(seq > 1 && !this.landing_height_updated) {
            try {
                double landing_ground_height = Double.parseDouble(System.getenv("PX4_LANDING_HEIGHT"));
                System.out.println("Landing height set to: "+landing_ground_height+" MSL.");
                this.setGroundLevel(landing_ground_height);
            } catch(Exception e) {
                System.out.println("No landing height specified (ENV: PX4_LANDING_HEIGHT is undefined).");
                System.out.println("Landing height will be NED frame origin height.");
            } finally {
                this.landing_height_updated = true;
            }

        }
    }

}
