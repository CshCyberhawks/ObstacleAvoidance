// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autonomous.CartesianRamseteClass;
import frc.robot.autonomous.Trajectories.PathGenerator;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();
    private final CartesianRamseteClass m_ramsete = new CartesianRamseteClass(m_robotDrive);
    //TODO change the initial and last pose of the path
    private final PathGenerator m_pathGenerator = new PathGenerator(new Pose2d(10, 3, new Rotation2d()), new Pose2d(2.5, 3, new Rotation2d()));


    private static boolean isTank = true;
    private static int reverser = -1;

    // The driver's controller
    Joystick m_rightStick = new Joystick(0);
    Joystick m_leftStick = new Joystick(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        m_robotDrive.resetOdometry(FieldConstants.allianceFlip(new Pose2d(10, 3, new Rotation2d())));

        m_robotDrive.setDefaultCommand(
                new RunCommand(() -> m_robotDrive.tankOrArcadeDrive(m_rightStick.getY() / getDriveSpeed(), -m_rightStick.getX() / getDriveSpeed(), m_leftStick.getY() / getDriveSpeed(), m_rightStick.getY() / getDriveSpeed(), isTank, reverser), m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_rightStick, 1)
                .onTrue(new InstantCommand(() -> changeDriveState()));

        new JoystickButton(m_rightStick, 2)
                .onTrue(new InstantCommand(() -> reverseChanger()));
    }

    public Drivetrain getRobotDrive() {
        return m_robotDrive;
    }

    /**
     * Zeros the outputs of all subsystems.
     */
    public void zeroAllOutputs() {
        m_robotDrive.tankDriveVolts(0, 0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //m_robotDrive.resetOdometry(FieldConstants.allianceFlip(new Pose2d(1.638, 2.725, new Rotation2d())));
        Trajectory trajectory = m_pathGenerator.getTrajectory();
        Logger.getInstance().recordOutput("Trajectory", trajectory);
        return m_ramsete.getCartesianRamseteCommand(trajectory);
    }

    private double getDriveSpeed() {
        double joyValue = 1 + m_rightStick.getRawAxis(3);
        if (joyValue <= 1) {
            return 1;
        } else {
            return joyValue;
        }
    }

    private void changeDriveState() {
        isTank = !isTank;
    }

    private void reverseChanger() {
        reverser *= -1;
    }

}
