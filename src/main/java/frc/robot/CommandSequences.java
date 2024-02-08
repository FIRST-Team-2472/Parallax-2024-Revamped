package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {
    
    public static Command robot1Command(SwerveSubsystem swerveSubsystem) {
        System.out.println("I found your stupid code but I ain't running jack shit");
        swerveSubsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);


            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), 
            List.of(
                new Translation2d(1.87, 6.95),
                new Translation2d(2.90, 6.95),
                new Translation2d(1.87, 6.95),
                new Translation2d(8.30, 7.44),
                new Translation2d(1.87, 8)
                ),
                new Pose2d(2, -1, Rotation2d.fromDegrees(0)),
                trajectoryConfig
        );

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(Math.PI, Math.PI);

      /*SwerveJoystickCmd swerveJoystickCmd = new SwerveJoystickCmd(
            trajectory,
             SwerveSubsystem [] getPose,
             DriveConstants.kDriveKinematics,
             xController,
             yController,
             thetaController,
             SwerveSubsystem [] setModuleStates,
             swerveSubsystem); */

        return null;
    }

    public Command robot2Command(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(null);

        return new SequentialCommandGroup(
            
        );
    }

    public Command robot3Command(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(null);

        return new SequentialCommandGroup(
            
        );
    }

}
