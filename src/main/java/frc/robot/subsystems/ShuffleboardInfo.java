package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class ShuffleboardInfo {

    private SwerveSubsystem swerveSubsystem;
    public ShuffleboardInfo(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
        SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
        SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
        SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
        SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
        SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
    
    }

}
