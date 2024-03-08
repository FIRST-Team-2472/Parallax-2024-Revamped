package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimingConstants;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class AutoAimingCmd extends Command {
    //private double x, y, z;
    private double z = 10;
    private double y = 7;
    private double x = 18;
    private double pitchAngle, yawAngle;
    private Pose3d targetPoseCameraSpace;
    private PitchMotorSubsystem pitchSubsystem;
    private SwerveModule swerveModule;
    private SwerveDriveToPointCmd swerveDriveToPointCmd;
    private SwerveRotateToAngle swerveRotateToAngle;
    private PosPose2d posPose2d;

    public AutoAimingCmd() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    targetPoseCameraSpace = LimelightHelpers.getTargetPose3d_CameraSpace(""); // FIXME Add Limelight Name
    // TODO Filter out all targets exept the one we want
    
        //x = targetPoseCameraSpace.getX();
        //y = targetPoseCameraSpace.getY();
        //z = targetPoseCameraSpace.getZ();

        pitchAngle = Math.atan(y / z);
        yawAngle = Math.atan(x / z);

        System.out.println(armAimingEquation(pitchAngle));
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public double armAimingEquation(double angle) {
        return -((angle - (-AutoAimingConstants.kLimeLightAngle + 90)) + AutoAimingConstants.kShooterAngle);
    }

}