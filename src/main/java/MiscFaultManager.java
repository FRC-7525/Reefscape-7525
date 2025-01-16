import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.team7525.misc.Elastic;
import org.team7525.misc.Elastic.ElasticNotification;
import org.team7525.misc.Elastic.ElasticNotification.NotificationLevel;

public class MiscFaultManager {

	public static AtomicReference<MiscFaultManager> instance = new AtomicReference<>();

	private ArrayList<MiscDevice> devices = new ArrayList<>();

	public class MiscDevice {
        private DigitalInput digitalInput;
        private DutyCycleEncoder dutyCycleEncoder;
        private PhotonCamera photonCamera;
        private PWM pwmDevice; 

		public MiscDeviceTypes deviceType;

		public Map<String, Integer> faults = new HashMap<String, Integer>();
		public String deviceName;

		public MiscDevice(DigitalInput digitalInput, String name) {
			this.deviceType = MiscDeviceTypes.DIGITALINPUT;
			this.digitalInput = digitalInput;
			this.deviceName = name;
		}

		public MiscDevice(DutyCycleEncoder dutyCycleEncoder, String name) {
			this.deviceType = MiscDeviceTypes.DUTYCYCLEENCODER;
			this.dutyCycleEncoder = dutyCycleEncoder;
			this.deviceName = name;
		}

		public MiscDevice(PhotonCamera photonCamera, String name) {
			this.deviceType = MiscDeviceTypes.PHOTONCAMERA;
			this.photonCamera = photonCamera;
			this.deviceName = name;
		}

        public MiscDevice(PWM pwm, String name) {
			this.deviceType = MiscDeviceTypes.PWM;
			this.pwmDevice = pwm;
			this.deviceName = name;
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

	private MiscFaultManager() {}

	public MiscFaultManager getInstance() {
		if (instance.get() == null) {
			instance.set(new MiscFaultManager());
		}
		return instance.get();
	}

	public void periodic() {
		checkDevices();
		logDevices();
	}

	public void checkDevices() {
		for (MiscDevice device : devices) {
			switch (device.deviceType) {
				case DIGITALINPUT:
					break;
				case DUTYCYCLEENCODER:
					break;
				case PHOTONCAMERA:
					break;
                case PWM:
                    break;
				default:
					break;
			}
		}
	}

	public void logDevices() {
		for (MiscDevice device : devices) {
			if (device.getFault()) {
				for (String fault : device.faults.keySet()) {
					Elastic.sendAlert(new ElasticNotification(NotificationLevel.ERROR, device.deviceName, fault));
				}
			}
		}
	}

    public void addDevice(DigitalInput digitalInput, String name) {
        devices.add(new MiscDevice(digitalInput, name));
    }

    public void addDevice(DutyCycleEncoder dutyCycleEncoder, String name) {
        devices.add(new MiscDevice(dutyCycleEncoder, name));
    }

    public void addDevice(PhotonCamera photonCamera, String name) {
        devices.add(new MiscDevice(photonCamera, name));
    }

    public void addDevice(PWM pwm, String name) {
        devices.add(new MiscDevice(pwm, name));
    }
}
