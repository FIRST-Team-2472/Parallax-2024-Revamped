package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase{
    private CANSparkMax topMotor = new CANSparkMax(IntakeConstants.kTopIntakeMotorId, MotorType.kBrushless);
    public CANSparkMax bottomMotor = new CANSparkMax(IntakeConstants.kBottomIntakeMotorId, MotorType.kBrushless);

    public void runintakemotors(Double MotorSpeed){
        topMotor.set(MotorSpeed);
        bottomMotor.set(-MotorSpeed);
    } 
}
