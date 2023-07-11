// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String defaultAuto = "Default";
  private static final String customAuto = "My Auto";
  private String autoSelected;
  private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Auto Choices");

  private Pose2d robotPosition = new Pose2d();
  private ArrayList<Pose2d> path = new ArrayList<>(Arrays.asList(
    new Pose2d(1, 0, new Rotation2d()),
    new Pose2d(0, 1, new Rotation2d())
  ));

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    if (isReal() || true) {
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit logger
    logger.start();

    // Initialize auto chooser
    chooser.addDefaultOption("Default Auto", defaultAuto);
    chooser.addOption("My Auto", customAuto);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Robot Position", robotPosition);

    for (int i = 0; i < path.size(); i++) {
      Logger.getInstance().recordOutput("Path Node " + i, path.get(i));
    }
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.get();
    System.out.println("Auto selected: " + autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
      case customAuto:
        // Put custom auto code here
        break;
      case defaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotPosition = new Pose2d(robotPosition.getX() + 0.01, robotPosition.getY(), robotPosition.getRotation());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }



 /** Takes the initial and last poses of the robot in addition to the list of locations the robot will be going on and returns a trajectory of the path*/ 
  private static Trajectory generatePoseTrajectory(ArrayList<Translation2d> poseArray, Pose2d robotInitialPose, Pose2d robotLastPose){
    //TODO edit the constants
    DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.58);
    var autoVoltageConstraint  = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(84271, 3.1946, 0.64), kDriveKinematics, 10);
    TrajectoryConfig config = new TrajectoryConfig(0.8, 6.0).setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(robotInitialPose, poseArray, robotLastPose, config);
    return trajectory;
  }
}
