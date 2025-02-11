package frc.robot.FaultManager;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class FaultManager {

	public static AtomicReference<FaultManager> instance = new AtomicReference<>();

	private Map<String, ArrayList<Integer>> CANDeviceOrder = new HashMap<>();

	private Map<String, CANDevice> CANDeviceMap = new HashMap<>();

	private ArrayList<MiscDevice> miscDevices = new ArrayList<>();

	public class Device {

		public Map<String, Integer> faults = new HashMap<String, Integer>();
		public String deviceName;

		public boolean alive;

		public Device(String deviceName) {
			this.deviceName = deviceName;
			this.alive = true;
		}

		public String getDeviceName() {
			return this.deviceName;
		}

		public boolean getFault() {
			return faults.size() > 0;
		}

		private void addFault(String fault) {
			if (faults.get(fault) == null) {
				faults.put(fault, 1);
			} else faults.put(fault, faults.get(fault) + 1);
		}

		private void removeFault(String fault) {
			if (faults.get(fault) == null) {
				return;
			}

			faults.put(fault, faults.get(fault) - 1);

			if (faults.get(fault) <= 0) {
				faults.remove(fault);
			}
		}

		public void updateFault(String fault, boolean value) {
			if (value) {
				this.addFault(fault);
			} else {
				this.removeFault(fault);
			}
		}
	}

	public class CANDevice extends Device {

		private TalonFX talon;
		private SparkMax sparkMax;
		private CANcoder canCoder;
		private Pigeon2 pigeon;

		public CANDeviceTypes deviceType;
		public String busName;

		public CANDevice(TalonFX talon, String name) {
			super(name);
			this.deviceType = CANDeviceTypes.TALON;
			this.talon = talon;
		}

		public CANDevice(SparkMax sparkMax, String name) {
			super(name);
			this.deviceType = CANDeviceTypes.SPARK;
			this.sparkMax = sparkMax;
		}

		public CANDevice(CANcoder canCoder, String name) {
			super(name);
			this.deviceType = CANDeviceTypes.CANCODER;
			this.canCoder = canCoder;
		}

		public CANDevice(Pigeon2 pigeon, String name) {
			super(name);
			this.deviceType = CANDeviceTypes.PIGEON;
			this.pigeon = pigeon;
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

		public Pigeon2 getPigeon() {
			if (deviceType != CANDeviceTypes.PIGEON) {
				throw new Error("This device is not a Pigeon");
			}

			return this.pigeon;
		}
	}

	public class MiscDevice extends Device {

		private DutyCycleEncoder dutyCycleEncoder;
		private PhotonCamera photonCamera;

		public MiscDeviceTypes deviceType;

		public MiscDevice(DutyCycleEncoder dutyCycleEncoder, String name) {
			super(name);
			this.deviceType = MiscDeviceTypes.DUTY_CYCLE_ENCODER;
			this.dutyCycleEncoder = dutyCycleEncoder;
		}

		public MiscDevice(PhotonCamera photonCamera, String name) {
			super(name);
			this.deviceType = MiscDeviceTypes.PHOTON_CAMERA;
			this.photonCamera = photonCamera;
		}

		public PhotonCamera getPhotonCamera() {
			if (deviceType != MiscDeviceTypes.PHOTON_CAMERA) {
				throw new Error("This device is not a Photon Camera");
			}

			return this.photonCamera;
		}

		public DutyCycleEncoder getDutyCycleEncoder() {
			if (deviceType != MiscDeviceTypes.DUTY_CYCLE_ENCODER) {
				throw new Error("This device is not a Duty Cycle Encoder");
			}

			return this.dutyCycleEncoder;
		}
	}

	private FaultManager() {}

	public static FaultManager getInstance() {
		if (instance.get() == null) {
			instance.set(new FaultManager());
		}
		return instance.get();
	}

	public void periodic() {
		checkDevices();
		logDevices();
	}

	public void checkDevices() {
		for (String busName : CANDeviceOrder.keySet()) {
			for (int id : CANDeviceOrder.get(busName)) {
				CANDevice device = CANDeviceMap.get(busName + " " + id);

				switch (device.deviceType) {
					case TALON:
						TalonFX talon = device.getTalon();

						device.updateFault("Booting Fault", talon.getFault_BootDuringEnable().getValue());
						device.updateFault("Bridge Brownout Fault", talon.getFault_BridgeBrownout().getValue());
						device.updateFault("Device Temp Fault", talon.getFault_DeviceTemp().getValue());
						device.updateFault("Forward Hard Limit Asserted", talon.getFault_ForwardHardLimit().getValue());
						device.updateFault("Forward Soft Limit Asserted", talon.getFault_ForwardSoftLimit().getValue());
						device.updateFault("Synchronization Fault", talon.getFault_FusedSensorOutOfSync().getValue());
						device.updateFault("Hardware Fault", talon.getFault_Hardware().getValue());
						device.updateFault("Differential Talon Missing", talon.getFault_MissingDifferentialFX().getValue());
						device.updateFault("Hard Limit Switch Missing", talon.getFault_MissingHardLimitRemote().getValue());
						device.updateFault("Soft Limit Switch Missing", talon.getFault_MissingSoftLimitRemote().getValue());
						device.updateFault("Oversupply Voltage Fault", talon.getFault_OverSupplyV().getValue());
						device.updateFault("Processor Temp Fault", talon.getFault_ProcTemp().getValue());
						device.updateFault("Remote Sensor Fault", talon.getFault_RemoteSensorDataInvalid().getValue());
						device.updateFault("Position Sensor Overflow Fault", talon.getFault_RemoteSensorPosOverflow().getValue());
						device.updateFault("Remote Sensor Reset", talon.getFault_RemoteSensorReset().getValue());
						device.updateFault("Reverse Hard Limit Switch Asserted", talon.getFault_ReverseHardLimit().getValue());
						device.updateFault("Reverse Soft Limit Switch Asserted", talon.getFault_ReverseSoftLimit().getValue());
						device.updateFault("Static Brake Disabled", talon.getFault_StaticBrakeDisabled().getValue());
						device.updateFault("Stator Current Limit Occurred", talon.getFault_StatorCurrLimit().getValue());
						device.updateFault("Supply Current Limit Occurred", talon.getFault_SupplyCurrLimit().getValue());
						device.updateFault("Undervoltage Fault", talon.getFault_Undervoltage().getValue());
						device.updateFault("Unstable Supply Voltage", talon.getFault_UnstableSupplyV().getValue());

						device.alive = talon.isConnected();

						break;
					case SPARK:
						SparkMax spark = device.getSparkMax();
						Faults faults = spark.getFaults();

						device.updateFault("CAN Fault", faults.can);
						device.updateFault("Temperature Fault", faults.temperature);
						device.updateFault("ESC EEPROM Fault", faults.escEeprom);
						device.updateFault("Firmware Fault", faults.firmware);
						device.updateFault("Gate Driver Fault", faults.gateDriver);
						device.updateFault("Motor Type Fault", faults.motorType);
						device.updateFault("Fault Detected", faults.other);
						device.updateFault("Sensor Fault", faults.sensor);

						device.alive = faults.temperature;

						break;
					case CANCODER:
						CANcoder canCoder = device.getCANcoder();

						device.updateFault("Magnet Fault", canCoder.getFault_BadMagnet().getValue());
						device.updateFault("Booting Fault", canCoder.getFault_BootDuringEnable().getValue());
						device.updateFault("Hardware Fault", canCoder.getFault_Hardware().getValue());
						device.updateFault("Undervoltage Fault", canCoder.getFault_Undervoltage().getValue());

						device.alive = canCoder.isConnected();

						break;
					case PIGEON:
						Pigeon2 pigeon = device.getPigeon();

						device.updateFault("Booting Fault", pigeon.getFault_BootDuringEnable().getValue());
						device.updateFault("Motion Detected Before Bootup", pigeon.getFault_BootIntoMotion().getValue());
						device.updateFault("Accelerometer Boot-Up Fault", pigeon.getFault_BootupAccelerometer().getValue());
						device.updateFault("Gyroscope Boot-Up Fault", pigeon.getFault_BootupGyroscope().getValue());
						device.updateFault("Magnetometer Boot-Up Fault", pigeon.getFault_BootupMagnetometer().getValue());
						device.updateFault("Data Acquisition was Slower Than Usual", pigeon.getFault_DataAcquiredLate().getValue());
						device.updateFault("Hardware Fault", pigeon.getFault_Hardware().getValue());
						device.updateFault("Motion Stack Loop Time was Slower Than Expected", pigeon.getFault_LoopTimeSlow().getValue());
						device.updateFault("Accelerometer Values are Saturated", pigeon.getFault_SaturatedAccelerometer().getValue());
						device.updateFault("Gyroscope Values are Saturated", pigeon.getFault_SaturatedGyroscope().getValue());
						device.updateFault("Magnetometer Values are Saturated", pigeon.getFault_SaturatedMagnetometer().getValue());
						device.updateFault("Undervoltage Fault", pigeon.getFault_Undervoltage().getValue());

						device.alive = pigeon.isConnected();

						break;
					default:
						break;
				}
			}

			for (MiscDevice device : miscDevices) {
				switch (device.deviceType) {
					case DUTY_CYCLE_ENCODER:
						DutyCycleEncoder dutyCycleEncoder = device.getDutyCycleEncoder();

						device.updateFault("Connection Fault", dutyCycleEncoder.isConnected());

						device.alive = dutyCycleEncoder.isConnected();

						break;
					case PHOTON_CAMERA:
						PhotonCamera photonCamera = device.getPhotonCamera();

						device.updateFault("Connection Fault", photonCamera.isConnected());

						device.alive = photonCamera.isConnected();

						break;
					default:
						break;
				}
			}
		}
	}

	public void logDevices() {
		for (String busName : CANDeviceOrder.keySet()) {
			for (int id : CANDeviceOrder.get(busName)) {
				CANDevice device = CANDeviceMap.get(busName + " " + id);

				if (device.getFault()) {
					for (String fault : device.faults.keySet()) {
						Logger.recordOutput("FaultManager/CAN Device Faults/" + busName + "/" + device.deviceName + "(" + id + ")", fault);
					}
				}

				System.out.println(device.alive);
				Logger.recordOutput("FaultManager/Alive CAN Devices/" + busName + "/" + device.deviceName + "(" + id + ")", device.alive);
			}
		}

		for (MiscDevice device : miscDevices) {
			if (device.getFault()) {
				for (String fault : device.faults.keySet()) {
					Logger.recordOutput("FaultManager/Misc Device Faults/" + device.deviceName, fault);
				}
			}
			Logger.recordOutput("FaultManager/Alive Misc Devices/" + device.deviceName, device.alive);
		}
	}

	/**
	 * Input an array of integers that represent the CAN IDs of the devices
	 * organized into the order that they are in the CAN chain
	 */
	public void calibrateDeviceOrder(ArrayList<Integer> deviceOrder, String busName) {
		CANDeviceOrder.put(busName, deviceOrder);
	}

	// Adding CAN Devices
	public void addDevice(TalonFX talonFX, String name, String busName) {
		CANDeviceMap.put(busName + " " + talonFX.getDeviceID(), new CANDevice(talonFX, name));
	}

	public void addDevice(SparkMax sparkMax, String name, String busName) {
		CANDeviceMap.put(busName + " " + sparkMax.getDeviceId(), new CANDevice(sparkMax, name));
	}

	public void addDevice(CANcoder encoder, String name, String busName) {
		CANDeviceMap.put(busName + " " + encoder.getDeviceID(), new CANDevice(encoder, name));
	}

	public void addDevice(Pigeon2 pigeon, String name, String busName) {
		CANDeviceMap.put(busName + " " + pigeon.getDeviceID(), new CANDevice(pigeon, name));
	}

	// Adding Misc Devices
	public void addDevice(DutyCycleEncoder dutyCycleEncoder, String name) {
		miscDevices.add(new MiscDevice(dutyCycleEncoder, name));
	}

	public void addDevice(PhotonCamera photonCamera, String name) {
		miscDevices.add(new MiscDevice(photonCamera, name));
	}
}
