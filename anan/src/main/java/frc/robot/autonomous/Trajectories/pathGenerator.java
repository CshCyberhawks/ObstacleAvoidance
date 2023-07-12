// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.Trajectories;

import java.util.ArrayList;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleopConstants;

/** Add your docs here. */
public class pathGenerator {
    private Pose2d initialPose;
    private Pose2d lastPose;
    private ArrayList<Translation2d> calculatedLocs;

    public pathGenerator(Pose2d initiaPose, Pose2d lastPose){
        this.initialPose = FieldConstants.allianceFlip(initiaPose);
        this.lastPose = FieldConstants.allianceFlip(lastPose);
        this.calculatedLocs = new ArrayList<Translation2d>();
        calculatePath();
    }

    private final void calculatePath(){
        //Code to calculate waypoints for the path, adds the calculated locations to calculatedLocs
    }




    public final Trajectory getTrajectory(){
        DifferentialDriveVoltageConstraint autoVoltageConstraint  = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

        TrajectoryConfig config  = new TrajectoryConfig(
            TeleopConstants.kMaxSpeedMetersPerSecond,
            TeleopConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        return TrajectoryGenerator.generateTrajectory(initialPose, calculatedLocs, lastPose, config);
    }
}
