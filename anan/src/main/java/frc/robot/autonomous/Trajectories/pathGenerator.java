// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.Trajectories;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;

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

/** Add your docs here. */
public class pathGenerator {
    private Pose2d initialPose;
    private Pose2d lastPose;
    private ArrayList<Translation2d> calculatedLocs;

    private final double step = .25;

    public pathGenerator(Pose2d initialPose, Pose2d lastPose){
        this.initialPose = FieldConstants.allianceFlip(initialPose);
        this.lastPose = FieldConstants.allianceFlip(lastPose);
        // this.calculatedLocs = buildPath(astar(new Translation2d(this.initialPose.getX(), this.initialPose.getY()), new Translation2d(this.lastPose.getX(), this.lastPose.getY())));
        // Node finalNode = astar(new Translation2d(this.initialPose.getX(), this.initialPose.getY()), new Translation2d(this.lastPose.getX(), this.lastPose.getY()));
        // SmartDashboard.putNumber("Final Pos X", finalNode.position.getX());
        // SmartDashboard.putNumber("Final Pos Y", finalNode.position.getY());

        // this.calculatedLocs = buildPath(finalNode);
        // Collections.reverse(this.calculatedLocs);

        // for (Translation2d pos : this.calculatedLocs) {
        //     System.out.println(pos.getX() + ", " + pos.getY());
        // }
        // System.out.println(this.calculatedLocs.size());
        this.calculatedLocs = new ArrayList<>();

        // for (int i = 0; i < path.size(); i++) {
        //     System.out.println(String.format("%d: %d, %d", i, path.get(i).getX(), path.get(i).getY()));
        // }

        double yValue = 4.5;
        if (Math.abs(initialPose.getY() - 4.5) > Math.abs(initialPose.getY() - .75)) {
            yValue = .75;
        }
        if (initialPose.getX() >= 5 && lastPose.getX() <= 3) {
            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(5.5, yValue)));
            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(2, yValue)));
        } else if (initialPose.getX() <= 3 && lastPose.getX() >= 5) {
            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(2, yValue)));
            calculatedLocs.add(FieldConstants.allianceFlip(new Translation2d(5.5, yValue)));
        }
    }

    private final ArrayList<Node> getNeighbors(Node node, Translation2d finalPosition){
        ArrayList<Node> neighbors = new ArrayList<Node>();
        
        for (int x = -1; x < 2; x++){
            for (int y = -1; y < 2; y++){
                if (x == y) continue;
                Translation2d pos = new Translation2d(node.position.getX() + x * step, node.position.getY() + y * step);
                Node element = new Node(pos, finalPosition);
                neighbors.add(element); 
            }
        }

        return neighbors;
    }

    private class Node implements Comparable<Node> {
        Translation2d position;
        Translation2d finalPosition;
        Node parent;

        public Node(Translation2d position, Translation2d finalPosition) {
            this.position = position;
            this.finalPosition = finalPosition;
        }

        @Override
        public int compareTo(Node arg0) {
            return (int)(Math.abs(position.getX() - finalPosition.getX()) + Math.abs(position.getY() - finalPosition.getY()));
        }
    }

    /*
     while not queue.isEmpty():
        currentState = queue.pop()
        if problem.isGoalState(currentState[0]):
            break

        for v in problem.getSuccessors(currentState[0]):
            if v[0] not in visited:
                print(v[0], problem.isGoalState(v[0]))
                visited.append(v[0])
                queue.push((v[0], currentState, v[1]), heuristic(v[0], problem))


    def buildPath(state, path=[]):
        if state[2] != None:
            path.insert(0, state[2])

        if state[1] == None:
            return path
        
        return buildPath(state[1], path)

    path = buildPath(currentState)
     */

    private Node astar(Translation2d initialPosition, Translation2d finalPosition) {
        Queue<Node> frontier = new LinkedList<>();
        frontier.add(new Node(initialPosition, finalPosition));
        ArrayList<Translation2d> visited = new ArrayList<>();

        while (frontier.size() > 0) {
            Node currentNode = frontier.remove();
            if (Math.abs(currentNode.position.getDistance(finalPosition)) < 1) {
                return currentNode;
            }

            for (Node child : getNeighbors(currentNode, finalPosition)) {
                if (!visited.contains(child.position)) {
                    visited.add(child.position);
                    child.parent = currentNode;
                    frontier.add(child);
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
