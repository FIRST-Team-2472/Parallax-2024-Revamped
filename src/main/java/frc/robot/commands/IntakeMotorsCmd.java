package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeMotorsCmd extends Command{
    private Double MotorSpeed;
    private Supplier<Boolean> Motorrunning;
    private Intake intake;
    private boolean a;
    public IntakeMotorsCmd(Intake intake, Supplier<Boolean> Motorrunning){
        this.Motorrunning = Motorrunning;
        this.intake = intake;
        addRequirements(intake);
        a = false;
    } 
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (Motorrunning.get()){
            if (a) a = false;
            else a = true;
        }
        MotorSpeed = a ? 0.1 : 0;
        intake.runintakemotors(MotorSpeed);
        System.out.println(MotorSpeed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
