package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
  Intake intake = new Intake();
  //Shooter shooter = new Shooter();
  //Arm arm = new Arm();
  XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);
  public RobotContainer() {
    intake.setDefaultCommand(new IntakeMotorsCmd(intake, () -> xbox.getAButtonPressed()));
    //shooter.setDefaultCommand(new ShooterMotorsCmd(shooter, () -> xbox.getLeftY(), () -> xbox.getRightY()));
    //arm.setDefaultCommand(new ArmMotorsCmd());
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
