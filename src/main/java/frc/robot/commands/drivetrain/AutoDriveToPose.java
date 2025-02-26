// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveToPose extends ParallelDeadlineGroup {
  /** Creates a new AutoDriveToPose. */
  public AutoDriveToPose(Pose2d desiredPose, DriveTrain driveTrain, PoseEstimator poseEstimator) {
    // deadline command vvvvvv
    super(new WaitUntilCommand(() -> poseEstimator.isAtPose(desiredPose, Units.inchesToMeters(2), Units.degreesToRadians(3))));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      poseEstimator.getPoseEstimate(),
      desiredPose
    );

    // temp values
    PathConstraints constraints = new PathConstraints(0.5, 0.5, Units.degreesToRotations(180), Units.degreesToRadians(360));

    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null,
      new GoalEndState(0.0, desiredPose.getRotation())
    );

    path.preventFlipping = true;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      AutoBuilder.followPath(path)
    );
  }
}
