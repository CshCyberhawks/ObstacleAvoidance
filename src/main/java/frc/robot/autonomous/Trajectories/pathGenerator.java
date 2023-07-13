// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.Trajectories;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.autonomous.Obstacle;
import frc.robot.autonomous.ObstacleConstants;
import frc.robot.util.PriorityQueue;

/**
 * Add your docs here.
 */
public class PathGenerator {
    private final Pose2d initialPose;
    private final Pose2d lastPose;
    private final ArrayList<Translation2d> calculatedLocs;

    private final double step = 1;

    public PathGenerator(Pose2d initialPose, Pose2d lastPose) {
        this.initialPose = FieldConstants.allianceFlip(initialPose);
        this.lastPose = FieldConstants.allianceFlip(lastPose);
        // this.calculatedLocs = buildPath(astar(new Translation2d(this.initialPose.getX(), this.initialPose.getY()), new Translation2d(this.lastPose.getX(), this.lastPose.getY())));
        Node finalNode = astar(new Translation2d(this.initialPose.getX(), this.initialPose.getY()), new Translation2d(this.lastPose.getX(), this.lastPose.getY()));
        assert finalNode != null;
        SmartDashboard.putNumber("Final Pos X", finalNode.position.getX());
        SmartDashboard.putNumber("Final Pos Y", finalNode.position.getY());

        this.calculatedLocs = buildPath(finalNode);
        Collections.reverse(this.calculatedLocs);

        for (Translation2d pos : this.calculatedLocs) {
            System.out.println(pos.getX() + ", " + pos.getY());
        }
        System.out.println(this.calculatedLocs.size());
        //        this.calculatedLocs = new ArrayList<>();

        // for (int i = 0; i < path.size(); i++) {
        //     System.out.println(String.format("%d: %d, %d", i, path.get(i).getX(), path.get(i).getY()));
        // }

        //        double yValue = 4.25;
        //        if (Math.abs(initialPose.getY() - 4.25) > Math.abs(initialPose.getY() - .75)) {
        //            yValue = .75;
        //        }
        //        if (initialPose.getX() >= 5 && lastPose.getX() <= 3) {
        //            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(5.5, yValue)));
        //            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(2.5, yValue)));
        //        } else if (initialPose.getX() <= 3 && lastPose.getX() >= 5) {
        //            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(2.5, yValue)));
        //            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(5.5, yValue)));
        //        }
    }

    private static class Node {
        Translation2d position;
        Translation2d finalPosition;
        Node parent;

        public Node(Translation2d position, Translation2d finalPosition) {
            this.position = position;
            this.finalPosition = finalPosition;
        }
    }

    private boolean containedIn(Translation2d pos, Translation2d lowerLeft, Translation2d upperRight) {
        return pos.getX() >= lowerLeft.getX() && pos.getY() >= lowerLeft.getY() && pos.getX() <= upperRight.getX() && pos.getY() <= upperRight.getY();
    }

    private boolean inObstacle(Translation2d pos) {
        for (Obstacle obstacle : ObstacleConstants.obstacleList) {
            if (containedIn(pos, obstacle.lowerLeft, obstacle.upperRight)) {
                return true;
            }
        }

        return false;
    }

    private ArrayList<Node> getNeighbors(Node node, Translation2d finalPosition) {
        ArrayList<Node> neighbors = new ArrayList<>();

        for (int x = -1; x < 2; x++) {
            for (int y = -1; y < 2; y++) {
                if (x == 0 && y == 0) continue;
                Translation2d pos = new Translation2d(node.position.getX() + x * step, node.position.getY() + y * step);
                if (!inObstacle(pos)) {
                    Node element = new Node(pos, finalPosition);
                    neighbors.add(element);
                }
            }
        }

        return neighbors;
    }

    private Node astar(Translation2d initialPosition, Translation2d finalPosition) {
        PriorityQueue<Node> frontier = new PriorityQueue<>();
        frontier.add(new Node(initialPosition, finalPosition), 0);
        ArrayList<Translation2d> visited = new ArrayList<>();

        while (!frontier.isEmpty()) {
            Node currentNode = frontier.remove();
            if (Math.abs(currentNode.position.getDistance(finalPosition)) < 1) {
                return currentNode;
            }

            for (Node child : getNeighbors(currentNode, finalPosition)) {
                if (!visited.contains(child.position)) {
                    visited.add(child.position);
                    child.parent = currentNode;
                    frontier.add(child, Math.abs(child.position.getDistance(finalPosition)) + Math.abs(child.position.getDistance(initialPosition)));
                }
            }
        }

        return null;
    }

    private ArrayList<Translation2d> buildPath(Node finalNode) {
        ArrayList<Translation2d> path = new ArrayList<>();

        Node currentNode = finalNode;
        while (currentNode.parent != null) {
            path.add(currentNode.position);
            currentNode = currentNode.parent;
        }

        return path;
    }


    public final Trajectory getTrajectory() {
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);

        TrajectoryConfig config = new TrajectoryConfig(
                TeleopConstants.kMaxSpeedMetersPerSecond,
                TeleopConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        return TrajectoryGenerator.generateTrajectory(initialPose, calculatedLocs, lastPose, config);
    }
}
