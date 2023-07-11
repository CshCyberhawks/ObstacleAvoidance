package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;

import java.util.ArrayList;
import java.util.Collections;

public class FieldObject2d implements AutoCloseable {
    public DoubleArrayEntry entry;
    private ArrayList<Pose2d> poseList = new ArrayList<>();

    public FieldObject2d(String name) {
        new FieldObject2d(name, new Pose2d());
    }

    public FieldObject2d(String name, Pose2d initialPosition) {

    }

    @Override
    public void close() {
        if (entry != null) {
            entry.close();
        }
    }

    public void setPoses(Pose2d... poses) {
        poseList.clear();
        Collections.addAll(poseList, poses);
        updateEntry();
    }

    public void updateEntry() {
        updateEntry(false);
    }

    public void updateEntry(Boolean setDefault) {
        if (entry == null) {
            return;
        }
        final double[] arr = new double[poseList.size() * 3];
        int ndx = 0;
        for (Pose2d pose : poseList) {
            final Translation2d translation = pose.getTranslation();
            arr[ndx] = translation.getX();
            arr[ndx + 1] = translation.getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        if (setDefault) {
            entry.setDefault(arr);
        } else {
            entry.set(arr);
        }
    }

    private void updateFromEntry() {
        if (entry == null) {
            return;
        }
        final double[] arr = entry.get(null);
        if (arr != null) {
            if (arr.length % 3 != 0) {
                return;
            }
            poseList.clear();
            int i = 0;
            while (i < arr.length) {
                poseList.add(new Pose2d(arr[i], arr[i + 1], Rotation2d.fromDegrees(arr[i + 2])));
                i += 3;
            }
        }
    }
}
