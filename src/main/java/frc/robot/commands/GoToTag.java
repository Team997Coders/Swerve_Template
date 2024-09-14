package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class GoToTag extends Command {

  private final Drivebase drivebase;
  private final Translation2d robotToTag;
  private final Double radius;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController TurningController;

  public GoToTag(Drivebase drivebase, Translation2d robotToTag, Double radius) {
    this.drivebase = drivebase;
    this.robotToTag = robotToTag;
    this.radius = radius;
    this.xController = new PIDController(0.1, 0, 0);
    this.yController = new PIDController(0.1, 0, 0);
    this.TurningController = new PIDController(0.1, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -xController.calculate(robotToTag.getX());
    double ySpeed = -yController.calculate(robotToTag.getY()); 
    double rotationSpeed = -TurningController.calculate(this.robotToTag.getAngle().getDegrees());

    drivebase.defaultDrive(xSpeed, ySpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
