package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Limelight;
import frc.robot.RobotContainer;

public class AlignApriltagCommand extends Command {

  private PIDController controller;

  public AlignApriltagCommand() {

    controller = new PIDController(0.05, 0, 0.005); // 0.15, 0, 0.001
    controller.setTolerance(0.7);
    
    controller.setSetpoint(0);

    addRequirements(RobotContainer.s_Swerve);
  }

  @Override
  public void initialize() {
    // Limelight.setPipeline(0)
    if (Limelight.getTv() == 0) {
        //this.end(true);
        this.cancel();
    }
  }

  @Override
  public void execute() {
    double rotation = -controller.calculate(Limelight.getTx());
    RobotContainer.s_Swerve.drive(new Translation2d(), rotation, false, false);
  }
    
  @Override
  public boolean isFinished() {
    //System.out.println("aligned!!!!!!");
    //return Limelight.hasValidTarget() && controller.atSetpoint();
    return controller.atSetpoint() && Limelight.getTv() == 1;
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Swerve.stop();
  }
}