package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax leftMotor = new CANSparkMax(ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
    public CANSparkMax rightMotor = new CANSparkMax(ArmConstants.kRightArmMotorId, MotorType.kBrushless);

    public void runintakemotors(Double topMotorSpeed, Double bottomMotorSpeed){
        leftMotor.set(topMotorSpeed);
        rightMotor.set(bottomMotorSpeed);
    }
}
