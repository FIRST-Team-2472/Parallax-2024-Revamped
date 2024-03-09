package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimingConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.commands.*;
import frc.robot.commands.DefaultCommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class AutoAimingCmd extends Command {
    private double x, y, z;
    // private double z = 10;
    // private double y = 7;
    // private double x = 18;
    private double pitchAngle, yawAngle;
    private Pose3d targetPoseCameraSpace;
    private PitchMotorSubsystem pitchSubsystem;
    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private SwerveModule swerveModule;
    private SwerveDriveToPointCmd swerveDriveToPointCmd;
    private SwerveRotateToAngle swerveRotateToAngle;
    private PosPose2d posPose2d;
    private boolean isDone = false;

    public AutoAimingCmd(PitchMotorSubsystem pitchSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem) {
        this.pitchSubsystem = pitchSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        targetPoseCameraSpace = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-shooter");
        LimelightResults llr = LimelightHelpers.getLatestResults("limelight-shooter");
        // TODO Filter out all targets exept the one we want

        if (llr.targetingResults.targets_Fiducials.length > 0) {
            x = targetPoseCameraSpace.getX();
            y = targetPoseCameraSpace.getY();
            z = targetPoseCameraSpace.getZ();

            pitchAngle = Units.radiansToDegrees(Math.atan2(z, y));
            yawAngle = Units.radiansToDegrees(Math.atan2(z, x));



            System.out.println(armAimingEquation(pitchAngle));

            new SequentialCommandGroup(
                    new SetArmPitchCmd(pitchSubsystem, armAimingEquation(pitchAngle)),
                    new runShooter(shooterSubsystem, intakeSubsystem, .5) // FIXME Get correct speed
            );
        }

        isDone = true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    public double armAimingEquation(double angle) {
        return -((angle - (-AutoAimingConstants.kLimeLightAngle + 90)) + AutoAimingConstants.kShooterAngle);
    }

}