package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmMotorsConstants.Encoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
  ArmMotorsSubsystem armSubsystem = new ArmMotorsSubsystem();
  XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);
  private AnalogEncoder encoder = new AnalogEncoder(Encoder.kEncoderPort);
  public RobotContainer() {
    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, encoder, () -> xbox.getLeftY(), // Pitch Motor
      () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
        () -> xbox.getRightTriggerAxis() > 0.5, // Push Motor
          () -> xbox.getRightBumper())); // Intake Motors

    
    configureBindings();
  }

  private void configureBindings() {
    if (xbox.getAButton())
      resetEncoder();
  }
  public void resetEncoder(){
    encoder.reset();
    encoder.setDistancePerRotation(360);
  }
  public Command getAutonomousCommand() {
    return null;
  }
}
