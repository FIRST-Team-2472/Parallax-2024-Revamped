package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(//
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed );

    private final SwerveModule frontRight = new SwerveModule(//
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed );

    private final SwerveModule backLeft = new SwerveModule(//
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed );

    private final SwerveModule backRight = new SwerveModule(//
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed );

    private final Pigeon2 gyro = new Pigeon2(SensorConstants.kPigeonID);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch(Exception e){

            }
        }).start();
    }

    public void zeroHeading(){
        gyro.setYaw(0);
    }

    public double getHeading(){
        return -gyro.getYaw().getValue();
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic(){
        odometer.update(getRotation2d(), getModulePositions());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    
    public SwerveModulePosition[] getModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        return temp;
    }

    // send over shuffleboard values
    public double getFLAbsEncoder(){
        return frontLeft.getUnfilteredPosition();
    }
    public double getFRAbsEncoder(){
        return frontRight.getUnfilteredPosition();
    }
    public double getBLAbsEncoder(){
        return backLeft.getUnfilteredPosition();
    }
    public double getBRAbsEncoder(){
        return backRight.getUnfilteredPosition();
    }
}
