package frc.robot;

import java.util.ArrayList;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerveExtras.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandSequences {

    PosPose2d[] exampleNodes = new PosPose2d[4];

    public CommandSequences() {
        exampleNodes[0] = simplePose(1, 0, 0);
        exampleNodes[1] = simplePose(0, 1, 0);
        exampleNodes[2] = simplePose(0, 1, 0);
        exampleNodes[3] = simplePose(1, 1, 0);
    }
    
    public Command robot1Command(SwerveSubsystem swerveSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(exampleNodes[0].toString());
        swerveSubsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, new PosPose2d(), List.of(),  exampleNodes[0])
        );
    }

        public Command robot2Command(SwerveSubsystem swerveSubsystem) {

            System.out.println("Autos Happening");
            System.out.println(exampleNodes[0].toString());
            swerveSubsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));

            return new SequentialCommandGroup(
                genratePath(swerveSubsystem, new PosPose2d(), List.of(),  exampleNodes[0]),
                genratePath(swerveSubsystem, new PosPose2d(), List.of(),  exampleNodes[1])

            );
        }

        public Command robot3Command(SwerveSubsystem swerveSubsystem){
            swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

            return new SequentialCommandGroup(
                genratePath(swerveSubsystem, new PosPose2d(), List.of(), exampleNodes[2]),
                genratePath(swerveSubsystem, new PosPose2d(), List.of(), exampleNodes[3])
             );     
        }

    // generates a path via points
    private static Command genratePath(SwerveSubsystem swerveSubsystem, PosPose2d startPoint, List<PositivePoint> midPoints,
        PosPose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Pose2d driveStartPoint = startPoint.toDrivePose2d();
        Pose2d driveEndPoint = endPoint.toDrivePose2d();
        List<Translation2d> driveMidPoints = new ArrayList<Translation2d>();
        for (int i = 0; i < midPoints.size(); i++)
            driveMidPoints.add(midPoints.get(i).toDrivePos());

        // 2. Generate trajectory
        // Genrates trajectory need to feed start point, a series of inbetween points,
        // and end point
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            driveStartPoint,
            driveMidPoints,
            driveEndPoint,
            trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                // swerveSubsystm::getPose is same as () -> swerveSubsystem.getPose()
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        // creates a Command list that will reset the Odometry, then move the path, then
        // stop
        return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public PosPose2d simplePose(double x, double y, double angleDegrees) {
        return new PosPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
}