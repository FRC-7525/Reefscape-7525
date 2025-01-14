import java.util.concurrent.atomic.AtomicReference;

public interface FaultManager {
    public static AtomicReference<FaultManager> instance = new AtomicReference<>();

    public abstract FaultManager getInstance();

    public abstract void periodic();

    public abstract void checkDevices();

    public abstract void logDevices();

    public abstract void addDevice();
}
