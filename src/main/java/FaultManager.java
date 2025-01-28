import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.PhotonCamera;
import org.team7525.misc.Elastic;
import org.team7525.misc.Elastic.ElasticNotification;
import org.team7525.misc.Elastic.ElasticNotification.NotificationLevel;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;

public class FaultManager {
    public static AtomicReference<FaultManager> instance = new AtomicReference<>();
    
	private ArrayList<Integer> CANDeviceOrder = new ArrayList<Integer>();
    
	private Map<Integer, CANDevice> CANDeviceMap = new HashMap<>();
    
	private ArrayList<MiscDevice> miscDevices = new ArrayList<>();

    public class Device {
        public Map<String, Integer> faults = new HashMap<String, Integer>();
        public String deviceName;

        public Device(String deviceName) {
            this.deviceName = deviceName;
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
				throw new Error("This fault does not exist with this device");
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

		public CANDeviceTypes deviceType;
        public String deviceName;

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
	}

    public class MiscDevice extends Device {
        private DigitalInput digitalInput;
        private DutyCycleEncoder dutyCycleEncoder;
        private PhotonCamera photonCamera;
        private PWM pwmDevice; 

		public MiscDeviceTypes deviceType;
        public String deviceName;

		public MiscDevice(DigitalInput digitalInput, String name) {
            super(name);

			this.deviceType = MiscDeviceTypes.DIGITALINPUT;
			this.digitalInput = digitalInput;
		}

		public MiscDevice(DutyCycleEncoder dutyCycleEncoder, String name) {
            super(name);

			this.deviceType = MiscDeviceTypes.DUTYCYCLEENCODER;
			this.dutyCycleEncoder = dutyCycleEncoder;
		}

		public MiscDevice(PhotonCamera photonCamera, String name) {
            super(name);

			this.deviceType = MiscDeviceTypes.PHOTONCAMERA;
			this.photonCamera = photonCamera;
		}

        public MiscDevice(PWM pwm, String name) {
            super(name);

			this.deviceType = MiscDeviceTypes.PWM;
			this.pwmDevice = pwm;
		}

		public DigitalInput getDigitalInput() {
			if (deviceType != MiscDeviceTypes.DIGITALINPUT) {
				throw new Error("This device is not a Digital Input");
			}

			return this.digitalInput;
		}

		public PhotonCamera getPhotonCamera() {
			if (deviceType != MiscDeviceTypes.PHOTONCAMERA) {
				throw new Error("This device is not a Photon Camera");
			}

			return this.photonCamera;
		}

		public DutyCycleEncoder getDutyCycleEncoder() {
			if (deviceType != MiscDeviceTypes.DUTYCYCLEENCODER) {
				throw new Error("This device is not a Duty Cycle Encoder");
			}

			return this.dutyCycleEncoder;
		}

        public PWM getPWM() {
			if (deviceType != MiscDeviceTypes.PWM) {
				throw new Error("This device is not a PWM");
			}

			return this.pwmDevice;
		}
	}

    private FaultManager() {}

    public FaultManager getInstance() {
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
		for (int id : CANDeviceOrder) {
			CANDevice device = CANDeviceMap.get(id);

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

					break;
				case CANCODER:
                    CANcoder canCoder = device.getCANcoder();

                    device.updateFault("Magnet Fault", canCoder.getFault_BadMagnet().getValue());
                    device.updateFault("Booting Fault", canCoder.getFault_BootDuringEnable().getValue());
                    device.updateFault("Hardware Fault", canCoder.getFault_Hardware().getValue());
                    device.updateFault("Undervoltage Fault", canCoder.getFault_Undervoltage().getValue());

					break;
				default:
					break;
			}
		}

        //TODO: I have no clue what faults to put here
        for (MiscDevice device : miscDevices) {
			switch (device.deviceType) {
				case DIGITALINPUT:
                    DigitalInput dio = device.getDigitalInput();
                    //TODO: IDK WHAT FAULTS TO PUT HERE

					break;
				case DUTYCYCLEENCODER:
                    DutyCycleEncoder dutyCycleEncoder = device.getDutyCycleEncoder();

                    device.updateFault("Connection Fault", dutyCycleEncoder.isConnected());

					break;
				case PHOTONCAMERA:
                    PhotonCamera photonCamera = device.getPhotonCamera();

                    device.updateFault("Connection Fault", photonCamera.isConnected());

					break;
                case PWM:
                    PWM pwm = device.getPWM();

                    break;
				default:
					break;
			}
		}
	}

	public void logDevices() {
		for (int id : CANDeviceOrder) {
			CANDevice device = CANDeviceMap.get(id);

			if (device.getFault()) {
				for (String fault : device.faults.keySet()) {
					Elastic.sendAlert(new ElasticNotification(NotificationLevel.ERROR, device.deviceName + " (" + id + ")" , fault));
				}
			}
		}

        for (MiscDevice device : miscDevices) {
			if (device.getFault()) {
				for (String fault : device.faults.keySet()) {
					Elastic.sendAlert(new ElasticNotification(NotificationLevel.ERROR, device.deviceName, fault));
				}
			}
		}
	}

	/**
	 * Input an array of integers that represent the CAN IDs of the devices organized into the order that they are in the CAN chain
	 */
	public void calibrateDeviceOrder(ArrayList<Integer> deviceOrder) {
		this.CANDeviceOrder = deviceOrder;
	}

    //Adding CAN Devices
	public void addDevice(TalonFX talonFX, String name) {
		CANDeviceMap.put(talonFX.getDeviceID(), new CANDevice(talonFX, name));
	}

	public void addDevice(SparkMax sparkMax, String name) {
		CANDeviceMap.put(sparkMax.getDeviceId(), new CANDevice(sparkMax, name));
	}

	public void addDevice(CANcoder encoder, String name) {
		CANDeviceMap.put(encoder.getDeviceID(), new CANDevice(encoder, name));
	}

    //Adding Misc Devices
    public void addDevice(DigitalInput digitalInput, String name) {
        miscDevices.add(new MiscDevice(digitalInput, name));
    }

    public void addDevice(DutyCycleEncoder dutyCycleEncoder, String name) {
        miscDevices.add(new MiscDevice(dutyCycleEncoder, name));
    }

    public void addDevice(PhotonCamera photonCamera, String name) {
        miscDevices.add(new MiscDevice(photonCamera, name));
    }

    public void addDevice(PWM pwm, String name) {
        miscDevices.add(new MiscDevice(pwm, name));
    }
}
