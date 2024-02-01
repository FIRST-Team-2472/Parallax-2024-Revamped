package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private CANSparkMax topMotor = new CANSparkMax(ShooterConstants.kTopShooterMotorId, MotorType.kBrushless);
    public CANSparkMax bottomMotor = new CANSparkMax(ShooterConstants.kBottomShooterMotorId, MotorType.kBrushless);

    public void runshootermotors(Double topMotorSpeed, Double bottomMotorSpeed){
        topMotor.set(topMotorSpeed);
        bottomMotor.set(bottomMotorSpeed);
    }
}
