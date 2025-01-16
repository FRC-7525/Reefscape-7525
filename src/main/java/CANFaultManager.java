import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.team7525.misc.Elastic;
import org.team7525.misc.Elastic.ElasticNotification;
import org.team7525.misc.Elastic.ElasticNotification.NotificationLevel;

public class CANFaultManager {

	public static AtomicReference<CANFaultManager> instance = new AtomicReference<>();

	private ArrayList<Integer> deviceOrder = new ArrayList<Integer>();

	private Map<Integer, CANDevice> deviceMap = new HashMap<>();

	public class CANDevice {
		private TalonFX talon;
		private SparkMax sparkMax;
		private CANcoder canCoder;

		public CANDeviceTypes deviceType;

		public Map<String, Integer> faults = new HashMap<String, Integer>();
		public String deviceName;

		public CANDevice(TalonFX talon, String name) {
			this.deviceType = CANDeviceTypes.TALON;
			this.talon = talon;
			this.deviceName = name;
		}

		public CANDevice(SparkMax sparkMax, String name) {
			this.deviceType = CANDeviceTypes.SPARK;
			this.sparkMax = sparkMax;
			this.deviceName = name;
		}

		public CANDevice(CANcoder canCoder, String name) {
			this.deviceType = CANDeviceTypes.CANCODER;
			this.canCoder = canCoder;
			this.deviceName = name;
		}

		public TalonFX getTalon() {
			if (deviceType != CANDeviceTypes.TALON) {
				throw new Error("This device is not a TalonFX");
			}

			return this.talon;
		}

		public SparkMax getSparkMax() {
			if (deviceType != CANDeviceTypes.SPARK) {
				throw new Error("This device is not a SparkMax");
			}

			return this.sparkMax;
		}

		public CANcoder getCANcoder() {
			if (deviceType != CANDeviceTypes.CANCODER) {
				throw new Error("This device is not a CANcoder");
			}

			return this.canCoder;
		}

		public boolean getFault() {
			return faults.size() > 0;
		}

		public String getDeviceName() {
			return deviceName;
		}

		public void addFault(String fault) {
			if (faults.get(fault) == null) {
				faults.put(fault, 1);
			} else faults.put(fault, faults.get(faults) + 1);
		}

		public void removeFault(String fault) {
			if (faults.get(fault) == null) {
				throw new Error("This fault does not exist with this device");
			}

			faults.put(fault, faults.get(faults) - 1);
			
			if (faults.get(fault) <= 0) {
				faults.remove(fault);
			}
		}
	}

	private CANFaultManager() {}

	public CANFaultManager getInstance() {
		if (instance.get() == null) {
			instance.set(new CANFaultManager());
		}
		return instance.get();
	}

	public void periodic() {
		checkDevices();
		logDevices();
	}

	public void checkDevices() {
		for (int id : deviceOrder) {
			CANDevice device = deviceMap.get(id);

			switch (device.deviceType) {
				case TALON:
					break;
				case SPARK:
					break;
				case CANCODER:
					break;
				default:
					break;
			}
		}
	}

	public void logDevices() {
		for (int id : deviceOrder) {
			CANDevice device = deviceMap.get(id);

			if (device.getFault()) {
				for (String fault : device.faults.keySet()) {
					Elastic.sendAlert(new ElasticNotification(NotificationLevel.ERROR, device.deviceName + " (" + id + ")" , fault));
				}
			}
		}
	}

	/**
	 * Input an array of integers that represent the CAN IDs of the devices organized into the order that they are in the CAN chain
	 */
	public void calibrateDeviceOrder(ArrayList<Integer> deviceOrder) {
		this.deviceOrder = deviceOrder;
	}

	public void addDevice(TalonFX talonFX, String name) {
		deviceMap.put(talonFX.getDeviceID(), new CANDevice(talonFX, name));
	}

	public void addDevice(SparkMax sparkMax, String name) {
		deviceMap.put(sparkMax.getDeviceId(), new CANDevice(sparkMax, name));
	}

	public void addDevice(CANcoder encoder, String name) {
		deviceMap.put(encoder.getDeviceID(), new CANDevice(encoder, name));
	}
}
