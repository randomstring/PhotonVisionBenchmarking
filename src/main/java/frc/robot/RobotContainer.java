// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.team2930.vision.VisionIO;
import frc.lib.team2930.vision.VisionIOConfig;
import frc.lib.team2930.vision.VisionIOPhotonVision;
import frc.lib.team2930.vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;

  // Controller
  //private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  //private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  // No robot, so make pitch and roll zero
  private DoubleSupplier pitchSupplier = () -> { return(0.0); };
  private DoubleSupplier rollSupplier  = () -> { return(0.0); }; 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    VisionIOConfig testCameraConfig;

      AprilTagFieldLayout layout;
      try {
        layout = new AprilTagFieldLayout(
          new File(Filesystem.getDeployDirectory(), "test_layout.json").toPath());
        //layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      } catch (IOException e) {
        layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      }

    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
      case SIM:
        // This is not a robot, this is code that runs in simulation mode

        testCameraConfig = new VisionIOConfig(
          new VisionIOPhotonVision("See3CAM_24CUG"),  // FIXME: set camera name
          "testCamera",
          Constants.ROBOT_TO_TEST_CAMERA);

        vision = new Vision(pitchSupplier, rollSupplier, layout, testCameraConfig);
        break;

      // Replayed robot, disable IO implementations
      default:
        testCameraConfig =
          new VisionIOConfig(new VisionIO() {}, "testCamera", Constants.ROBOT_TO_TEST_CAMERA);

        vision = new Vision(pitchSupplier, rollSupplier, layout, testCameraConfig);
        break;
    }

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

          }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
