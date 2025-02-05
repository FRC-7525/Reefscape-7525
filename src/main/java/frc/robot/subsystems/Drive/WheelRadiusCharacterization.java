package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class WheelRadiusCharacterization {
    // Shoutout to 2974 for this code, they highkey cooked but lowkey we should swap
    // this to a state bc commands are yuck
    private double lastGyroYawRads = 0;
    private double accumGyroYawRads = 0;
    private double averageWheelPosition = 0;

    private double[] startWheelPositions = new double[4];
    private double currentEffectiveWheelRadius = 0;

    private boolean characterizationActive = false;

    public static final double DRIVE_ROTATIONS_PER_METER = TunerConstants.kDriveGearRatio
            / (2 * Math.PI * TunerConstants.kWheelRadius.in(Meters));
    public static final double DRIVE_RADIUS = Math.hypot(TunerConstants.kFrontLeftXPos.magnitude(),
            TunerConstants.kFrontLeftYPos.magnitude());

    public static WheelRadiusCharacterization instance;

    private WheelRadiusCharacterization() {
    }

    public static WheelRadiusCharacterization getInstance() {
        if (instance == null) {
            instance = new WheelRadiusCharacterization();
            return instance;
        }
        return instance;
    }

    public Command getWheelRadiusCharacterizationCommand(double omegaDirection, Drive drive) {
        /* wheel radius characterization schtuffs */
        final DoubleSupplier m_gyroYawRadsSupplier = () -> Units
                .degreesToRadians(drive.getPigeon2().getYaw().getValueAsDouble());
        // () -> getState().Pose.getRotation().getRadians();
        final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(0.5);
        final SwerveRequest.RobotCentric m_characterizationReq = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final double m_characterizationSpeed = 1.5;
        var initialize = Commands.runOnce(() -> {
            characterizationActive = true;
            lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
            accumGyroYawRads = 0;
            currentEffectiveWheelRadius = 0;
            averageWheelPosition = 0;
            for (int i = 0; i < drive.getDriveTrain().getModules().length; i++) {
                var pos = drive.getDriveTrain().getModules()[i].getPosition(true);
                startWheelPositions[i] = pos.distanceMeters / DRIVE_ROTATIONS_PER_METER;
                startWheelPositions[i] = pos.distanceMeters * DRIVE_ROTATIONS_PER_METER;
            }
            m_omegaLimiter.reset(0);
        });

        var executeEnd = Commands.runEnd(
                () -> {
                    drive.getDriveTrain().setControl(m_characterizationReq
                            .withRotationalRate(m_omegaLimiter.calculate(m_characterizationSpeed * omegaDirection)));
                    accumGyroYawRads += MathUtil.angleModulus(m_gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
                    lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
                    double averageWheelPosition = 0;
                    averageWheelPosition = 0;
                    double[] wheelPositions = new double[4];
                    for (int i = 0; i < drive.getDriveTrain().getModules().length; i++) {
                        var pos = drive.getDriveTrain().getModules()[i].getPosition(true);
                        wheelPositions[i] = pos.distanceMeters * DRIVE_ROTATIONS_PER_METER;
                        averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
                    }
                    averageWheelPosition /= 4.0;
                    averageWheelPosition = averageWheelPosition / 4.0;
                    currentEffectiveWheelRadius = (accumGyroYawRads * DRIVE_RADIUS)
                            / averageWheelPosition;
                    // System.out.println("effective wheel radius: " + currentEffectiveWheelRadius);
                    System.out.println("Average Wheel Position: " + averageWheelPosition);
                }, () -> {
                    drive.getDriveTrain().setControl(m_characterizationReq.withRotationalRate(0));
                    if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
                        System.out.println("not enough data for characterization " + accumGyroYawRads);
                        System.out.println("not enough data for characterization " + accumGyroYawRads
                                + "\navgWheelPos: " + averageWheelPosition + "radians");
                    } else {
                        System.out.println(
                                "effective wheel radius: "
                                        + currentEffectiveWheelRadius
                                        + " inches"
                                        + "\naccumGryoYawRads: " + accumGyroYawRads + " radians"
                                        + "\navgWheelPos: " + averageWheelPosition + " radians");
                    }
                    characterizationActive = false;
                });

        return Commands.sequence(initialize, executeEnd);
    }

    public boolean isCharacterizationActive() {
        return characterizationActive;
    }

}
