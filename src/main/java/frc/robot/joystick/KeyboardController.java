package frc.robot.joystick;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardController {
    private final NetworkTable table;

    public KeyboardController(){
        //Conect to the network table
        table = NetworkTableInstance.getDefault().getTable("OperatorController");
    }

    private Trigger createTrigger(String key){
        // function to read the key from table
        return new Trigger(() -> table.getEntry(key).getBoolean(false));
    }

    // this is how to create a trigger to be read in a robot container
    public Trigger getATrigger(){
        return createTrigger("a");
    }
}