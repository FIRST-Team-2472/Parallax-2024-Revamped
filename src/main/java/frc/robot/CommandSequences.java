package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.type.CollectionLikeType;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;
import frc.robot.subsystems.swerveExtras.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class CommandSequences {

        int miscNodeLength = 4;
        PosPose2d[] miscellaneousNodes = new PosPose2d[miscNodeLength];
        
        int importantNodesLength = 6;
        PosPose2d[] importantNodes = new PosPose2d[importantNodesLength];
        
        int startingNodesLength = 5;
        PosPose2d[] startingNodes = new PosPose2d[startingNodesLength];
        
        int collectingNearNodesLength = 3;
        PosPose2d[] collectingNearNodes = new PosPose2d[collectingNearNodesLength];
        
        int shootingNearNodesLength = 3;
        PosPose2d[] shootingNearNodes = new PosPose2d[shootingNearNodesLength];
        
        PosPose2d ampNode = new PosPose2d(1.84, 7.32, -90);
    

    public CommandSequences() {

        // by the source over the line
        miscellaneousNodes[0] = new PosPose2d(3, 2, 0);
        //On top of note 2
        miscellaneousNodes[1] = new PosPose2d(2.91, 5.56, 0);
        //On the way to note 1
        miscellaneousNodes[2] = new PosPose2d(1.76, 7, 0);
        miscellaneousNodes[3] = new PosPose2d(1, 1, 0);

        // non-amp side of Speaker
        importantNodes[0] = new PosPose2d(.55, 4.10, 0);
        // In front of Note
        importantNodes[1] = new PosPose2d(2.2, 4.10, 0);
        // Near front of Speaker
        importantNodes[2] = new PosPose2d(2.2, 5.57, 0);
        // In from of amp
        importantNodes[3] = new PosPose2d(1.84, 7.32, -130);
        //amp side of stage
        importantNodes[4] = new PosPose2d(4.28, 6.30, 0);
        //under the stage
        importantNodes[5] = new PosPose2d(4.78, 4.15, 0);

        // amp start
        startingNodes[0] = new PosPose2d(1.41, 7.26, 0);
        //speaker start 1
        startingNodes[1] = new PosPose2d(0.71, 6.7, 60);
        //speaker start 2
        startingNodes[2] = new PosPose2d(1.4, 5.52, 0);
        // speakr start 3
        startingNodes[3] = new PosPose2d(0.71, 4.38, -60);

        startingNodes[4] = new PosPose2d(2.46, 7.27, 0);

        // Collecting the near nodes
        collectingNearNodes[0] = new PosPose2d(2.9, 6.92, 0);
        collectingNearNodes[1] = new PosPose2d(2.15, 5.5,	0); //same as imp. n. [2];
        collectingNearNodes[2] = new PosPose2d(2.15, 4.08, 0); //same as imp. n. [3];

        // Shooting to the speaker from the near nodes
        shootingNearNodes[0] = new PosPose2d(2.9, 7, 26);
        shootingNearNodes[1] = new PosPose2d(2.9, 5.5,	0);
        shootingNearNodes[2] = new PosPose2d(2.9, 4.08, -26);
    }

   
    public Command driveFromZone(SwerveSubsystem swerveSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(miscellaneousNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[0]);
 
        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, List.of(), startingNodes[4])
            );
    }

    public Command twoinspeakerfrompositiontwoCommand(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(miscellaneousNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[2]);

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7),
                new ParallelCommandGroup(
                    new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle), 
                    genratePath(swerveSubsystem, List.of(), importantNodes[2]), 
                    new runIntake(intakeMotorSubsystem, 0, 1.7)),
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle),
                new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7),
                genratePath(swerveSubsystem, List.of(), miscellaneousNodes[1])
        );
    }

    public Command twoinspeakerfrompositiononeCommand(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[1]);

         return new SequentialCommandGroup(
            new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
            new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7),
            new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
            genratePath(swerveSubsystem, List.of(), miscellaneousNodes[2]),
            new ParallelCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, collectingNearNodes[0]),
                new runIntake(intakeMotorSubsystem, 0, 2.2)
            ),
            new ParallelCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, shootingNearNodes[0]),
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle)),
            new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7)
            ); 
                //return new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle);
    }

    public Command twoinspeakerfrompositionthreeCommand(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem){
        swerveSubsystem.resetOdometry(startingNodes[3]);

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7),
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                genratePath(swerveSubsystem, List.of(), importantNodes[1]),
                genratePath(swerveSubsystem, List.of(), startingNodes[3]));
    }

    public Command oneinamponefromspeakerpositiononeCommand(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, List.of(), collectingNearNodes[0]),
                genratePath(swerveSubsystem, List.of(), importantNodes[3]));
    }

    public Command threeinspeakerfrompositionone(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, List.of(), collectingNearNodes[0]),
                genratePath(swerveSubsystem, List.of(), importantNodes[3]));
    }

        public Command underStage(SwerveSubsystem swerveSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(miscellaneousNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[0]);
 
        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, List.of(importantNodes[4]), importantNodes[5])
            );
        }

    public Command justShoot(PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {
        
        
        return new SequentialCommandGroup(
            new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
            new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7)
        );
    }

    public Command justShootAndMove(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {
        
        swerveSubsystem.resetOdometry(startingNodes[3]);


        return new SequentialCommandGroup(
            new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
            new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7),
            genratePath(swerveSubsystem, List.of(), miscellaneousNodes[0])
        );
    }

    public Command justMovePosition2tonote2(SwerveSubsystem swerveSubsystem) {
        
        swerveSubsystem.resetOdometry(startingNodes[2]);

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, List.of(), importantNodes[2])
        );
    }

    public Command justRunIntake(IntakeMotorSubsystem intakeMotorSubsystem) {

        return new SequentialCommandGroup(
            new runIntake(intakeMotorSubsystem, 0, 0.6)
        );
    }

    public Command shootFromFurtherAway(PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {

        return new SequentialCommandGroup(
            new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle),
            new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7)
        );
    }

    public Command justMovePosition2tonote1(SwerveSubsystem swerveSubsystem) {
        
        swerveSubsystem.resetOdometry(startingNodes[2]);

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, List.of(), collectingNearNodes[0])
        );
    }

    public Command justMovePosition2tonote3(SwerveSubsystem swerveSubsystem) {
        
        swerveSubsystem.resetOdometry(startingNodes[2]);

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, List.of(), importantNodes[1])
        );
    }

    public Command justMovePosition1tonote1(SwerveSubsystem swerveSubsystem) {
        
        swerveSubsystem.resetOdometry(startingNodes[1]);

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, List.of(), collectingNearNodes[0])
        );
    }

    public Command justMovePosition1tonote2(SwerveSubsystem swerveSubsystem) {
        
        swerveSubsystem.resetOdometry(startingNodes[1]);

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, List.of(), importantNodes[2])
        );
    }

    public Command justMovePosition1tonote3(SwerveSubsystem swerveSubsystem) {
        
        swerveSubsystem.resetOdometry(startingNodes[1]);

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, List.of(), importantNodes[1])
        );
    }

    // generates a path via points
    private static Command genratePath(SwerveSubsystem swerveSubsystem, List<PosPose2d> midPoints,
            PosPose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Pose2d driveStartPoint = new Pose2d(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY(), swerveSubsystem.getRotation2d());
        Pose2d driveEndPoint = new Pose2d(endPoint.getX(), endPoint.getY(), endPoint.getAngle());
        List<Translation2d> driveMidPoints = new ArrayList<Translation2d>();
        for (int i = 0; i < midPoints.size(); i++)
            driveMidPoints.add(new Translation2d(midPoints.get(i).getX(), midPoints.get(i).getY()));

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

    public void flipToRed(){
        for(int i = 0; i < miscNodeLength; i++){
            miscellaneousNodes[i].changeToRed();
        }
        for(int i = 0; i < importantNodesLength; i++){
            importantNodes[i].changeToRed();
        }
        for(int i = 0; i < startingNodesLength; i++){
            startingNodes[i].changeToRed();
        }
        for(int i = 0; i < collectingNearNodesLength; i++){
            collectingNearNodes[i].changeToRed();
        }
        for(int i = 0; i < shootingNearNodesLength; i++){
            shootingNearNodes[i].changeToRed();
        }
        ampNode.changeToRed();
    }

}