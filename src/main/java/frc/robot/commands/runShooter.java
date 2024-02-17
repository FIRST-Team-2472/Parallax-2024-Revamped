package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_Motors_Subsystem;

public class runShooter extends Command { 

    private ArmMotorsCmd armCmd;
    private Arm_Motors_Subsystem armSubsystem;

    public runShooter() {

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        armSubsystem.runShooterMotors(0.5);
        new Thread(() -> {
            try{
                Thread.sleep(1500);
                armSubsystem.runPushMotor(0.5);
            }catch(Exception e){

            }
        }).start();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
