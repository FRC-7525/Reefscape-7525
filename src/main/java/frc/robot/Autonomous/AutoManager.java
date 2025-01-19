package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Manager.Manager;

public class AutoManager {

    public static AutoManager instance;
    
    private final Manager manager = Manager.getInstance();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    private AutoManager() {
        
    }

    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }


}
