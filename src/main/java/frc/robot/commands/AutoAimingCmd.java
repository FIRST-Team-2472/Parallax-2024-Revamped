package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimingConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.commands.*;
import frc.robot.commands.DefaultCommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class AutoAimingCmd extends Command {
    private double x, y, z, tx, ty;
    // private double z = 10;
    // private double y = 7;
    // private double x = 18;
    private double pitchAngle, yawAngle;
    // private Pose3d targetPoseCameraSpace;
    private PitchMotorSubsystem pitchSubsystem;
    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private SwerveSubsystem swerveSubsystem;
    // private SwerveDriveToPointCmd swerveDriveToPointCmd;
    // private SwerveRotateToAngle swerveRotateToAngle;
    private Pose2d robotPos;
    private double distanceFromSpeaker;

    Pose2d blueSpeakerPos = new Pose2d(new Translation2d(0.25, 5.6), new Rotation2d());
    Pose2d redSpeakerPos = new Pose2d(new Translation2d(16.35, 5.6), new Rotation2d());
    private boolean isDone = false;

    public AutoAimingCmd(PitchMotorSubsystem pitchSubsystem, ShootingMotorSubsystem shooterSubsystem,
            IntakeMotorSubsystem intakeSubsystem) {
        this.pitchSubsystem = pitchSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

    }

    @Override
    public void initialize() {
        System.out.println("Distance Self-Test: " + getDistance(2.2, 5.6, 0.25, 5.6) + " == 1.59");
        System.out.println("Angle Equation Self-Test: " + distanceToAngle(1.15) + " == 80");
    }

    @Override
    public void execute() {

        robotPos = swerveSubsystem.getPose();

        if (isOnBlueSide(robotPos.getX())) {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(), blueSpeakerPos.getX(),
                    blueSpeakerPos.getY());
        } else {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(), redSpeakerPos.getX(),
                    redSpeakerPos.getY());
        }

        pitchAngle = distanceToAngle(distanceFromSpeaker);

        new SetArmPitchCmd(pitchSubsystem, pitchAngle);

        isDone = true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    public double distanceToAngle(double angle) {
        return (-20.6 * angle) + 104;
    }

    double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    }

    boolean isOnBlueSide(double xPos) {
        return xPos < 8.3;
    }
}