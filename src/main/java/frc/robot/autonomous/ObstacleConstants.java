package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.autonomous.Obstacle;

public class ObstacleConstants {
    public static final Obstacle[] obstacleList = new Obstacle[]{
            // Charge Station
            new Obstacle(FieldConstants.allianceFlip(new Translation2d(FieldConstants.Community.chargingStationInnerX, FieldConstants.Community.chargingStationLeftY)), FieldConstants.allianceFlip(new Translation2d(FieldConstants.Community.chargingStationOuterX, FieldConstants.Community.chargingStationRightY)))
    };
}