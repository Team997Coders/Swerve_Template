package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class GoToTag extends Command {

  private final Drivebase drivebase;
  private final Pose2d tagPose;
  private final Double radius;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController TurningController;

  public GoToTag(Drivebase drivebase, Pose2d tagPose, Double radius) {
    this.drivebase = drivebase;
    this.tagPose = tagPose;
    this.radius = radius;
    this.xController = new PIDController(0.25, 0, 0);
    this.yController = new PIDController(0.25, 0, 0);
    this.TurningController = new PIDController(0.15, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses
    (
      this.drivebase.getPose(),
      this.tagPose
    );
    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    //return autoChooser.getSelected();
    AutoBuilder.followPath(path);
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
