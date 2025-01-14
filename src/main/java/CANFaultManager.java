import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

public class CANFaultManager {
    public static AtomicReference<CANFaultManager> instance = new AtomicReference<>();
    
    private ArrayList<TalonFX> talons = new ArrayList<TalonFX>();
    private ArrayList<SparkMax> sparkMaxes = new ArrayList<SparkMax>();
    private ArrayList<CANcoder> encoders = new ArrayList<CANcoder>();

    public CANFaultManager() {


    }

    public CANFaultManager getInstance() {
        if (instance.get() == null) {
            instance.set(new CANFaultManager());
        }
        return instance.get();
    }

    public void periodic() {
        
    }

    public void checkDevices() {
        
    }

    public void logDevices() {
        
    }

    public void addDevice(TalonFX talonFX, int id) {

    }

    public void addDevice(SparkMax sparkMax, int id) {

    }

    public void addDevice(CANcoder encoder, int id) {

    }
}
