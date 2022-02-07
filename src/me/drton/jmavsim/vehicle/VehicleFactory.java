package me.drton.jmavsim.vehicle;
import java.io.FileReader;

import javax.json.Json;
import javax.json.JsonReader;

import me.drton.jmavsim.World;
import javax.json.JsonObject;

public class VehicleFactory {

    private World world;
    private boolean showGUI;
    
    private static final String TYPE_KEY = "type";
    private static final String DRONE_TYPE_QUADCOPTER = "QUADCOPTER";
    private static final String DRONE_TYPE_EVTOL_FW = "EVTOL_FW";

    public VehicleFactory(World world, boolean showGUI) { 
        this.world = world;
        this.showGUI = showGUI;
    }

    private AbstractFixedWing evtolFwFromObject(JsonObject obj) {
        System.out.println("Building EVTOL FixedWing");
        return EVTOLFixedWing.fromJSONObject(this.world, this.showGUI, obj);
    }

    private AbstractMulticopter multicopterFromObject(JsonObject obj) {
        System.out.println("Building Multicopter");
        return Quadcopter.fromJSONObject(this.world, this.showGUI, obj);
    }

    public AbstractVehicle vehicleFromFile(String filename) {
        try (JsonReader reader = Json.createReader(new FileReader(filename))) {
            JsonObject obj = reader.readObject();
            System.out.println("Successfully read drone file '"  + filename +"'");
            switch (obj.getString(TYPE_KEY)) {
                case DRONE_TYPE_QUADCOPTER:
                    return this.multicopterFromObject(obj);
                case DRONE_TYPE_EVTOL_FW:
                    return this.evtolFwFromObject(obj);
                default:
                    throw new Exception("Unknown drone type!");
            }
        } catch(Exception e) {
            System.out.println("Error when trying to read vehicle information from "+filename);
            System.out.println(e.toString());
            System.exit(1);
        }
        return null;

    }

}
