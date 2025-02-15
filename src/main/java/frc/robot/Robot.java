// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutoManager.AutoManager;
import frc.robot.FaultManager.FaultManager;
import frc.robot.GlobalConstants.FaultManagerConstants;
import frc.robot.MusicManager.MusicManager;
import frc.robot.SubsystemManager.SubsystemManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team7525.misc.CommandsUtil;

public class Robot extends LoggedRobot {

	private final SubsystemManager manager = SubsystemManager.getInstance();
	private final AutoManager autoManager = AutoManager.getInstance();
	private final MusicManager musicManager = MusicManager.getInstance();
	private final FaultManager faultManager = FaultManager.getInstance();

	@Override
	public void robotInit() {
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL:
				Logger.addDataReceiver(new NT4Publisher());
				Logger.addDataReceiver(new WPILOGWriter());
				break;
			case SIM:
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case TESTING:
				Logger.addDataReceiver(new NT4Publisher());
				break;
		}

		Logger.start();
		CommandsUtil.logCommands();
		DriverStation.silenceJoystickConnectionWarning(true);

		CommandScheduler.getInstance().unregisterAllSubsystems();

		FaultManager.getInstance().calibrateDeviceOrder(FaultManagerConstants.CANIVORE_DEVICE_ORDER, "CANivore");
		FollowPathCommand.warmupCommand().schedule();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		manager.periodic();
		faultManager.periodic();
	}

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().schedule(autoManager.getSelectedCommand());
		musicManager.stopMusic();
		musicManager.removeAllMotors();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopInit() {
		musicManager.stopMusic();
		musicManager.removeAllMotors();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {
		if (musicManager.playMusicEnabled()) {
			musicManager.playMusic();
		}

		if (!musicManager.hasInstruments() && musicManager.playMusicEnabled()) {
			musicManager.addAllSubsystemInstruments();
		}
	}

	@Override
	public void testInit() {
		musicManager.stopMusic();
		musicManager.removeAllMotors();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}
}
