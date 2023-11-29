package frc.robot;

import java.nio.file.Path;
import java.security.DrbgParameters.Reseed;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzAbstractStateUtil;
import frc.robot.Utils.CatzAbstractStateUtil.GamePieceState;
import frc.robot.Utils.CatzAbstractStateUtil.SetAbstractMechanismState;
import frc.robot.commands.ManipulatorToPoseCmd;
import frc.robot.commands.DriveCmds.Trajectory.TrajectoryFollowingCmd;
import frc.robot.commands.DriveCmds.Trajectory.Paths.Trajectories;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class CatzAutonomous {
    static CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance(); 
    static CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
    static CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
    static CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
    
    //private static PathPlannerPath driveStraighFullTurn = PathPlannerPath.fromPathFile("DriveStraightFullTurn");
    //private static PathPlannerPath feildSideDriveBack = PathPlannerPath.fromPathFile("FeildSideDriveBack");


    public static LoggedDashboardChooser<AllianceColor> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<AutoModes> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    private enum AutoModes {
        TEST,
        DRIVE_STRAIGHT,
        TRANSLATE_DRIVE_STRAIGHT,
        PARALEL_SCORE_2
    }

    public CatzAutonomous()
    {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.BlUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.RED_ALLIANCE);

        autoChooser.addDefaultOption("Do Nothing", null);

        autoChooser.addOption       ("TEST PATH",  AutoModes.TEST);
        autoChooser.addOption       ("Drivestright", AutoModes.DRIVE_STRAIGHT);
        autoChooser.addOption       ("TranslateDriveStraight", AutoModes.TRANSLATE_DRIVE_STRAIGHT);
    }

    public Command getCommand()
    {
        switch(autoChooser.get())
        {
            case TEST: return testPath();
            case DRIVE_STRAIGHT: driveStraight();
            case PARALEL_SCORE_2: return parallelScoreCube();
            default: 
            return new InstantCommand();
        }
    }

    public Command testPath()
    {
        return new SequentialCommandGroup(

               new TrajectoryFollowingCmd(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(180))
                                        );
    }

    public Command driveStraight() {
        return new TrajectoryFollowingCmd(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(180));
    }

    public Command startWall()
    {
        return new SequentialCommandGroup();
    }

    public Command startField()
    {
        return new SequentialCommandGroup();
    }

    public Command startCenter()
    {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup parallelScoreCube()
    {
        return new SequentialCommandGroup(
                    new ManipulatorToPoseCmd(ManipulatorPoseConstants.SCORE_HIGH_CONE)
                        .raceWith(Commands.waitSeconds(1.0)),

                    new ParallelCommandGroup(
                       // new PPTrajectoryFollowingCmd(driveStraighFullTurn), 
                        new SequentialCommandGroup(
                            new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                           // new StateMachineCmd(SetMechanismState.PICKUP_GROUND)
                          //      .raceWith(Commands.waitSeconds(1.0)),
                            intakeSequenceCommandGroup())
                                            ),

                    new ParallelCommandGroup(
                       // new PPTrajectoryFollowingCmd(feildSideDriveBack), 
                        new SequentialCommandGroup(
                            new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                            new ManipulatorToPoseCmd(ManipulatorPoseConstants.SCORE_HIGH_CONE)
                                .raceWith(Commands.waitSeconds(1.0)))
                                            ),
                    intakeSequenceCommandGroup(),
                    new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW)
                                         );       
    }

    public static SequentialCommandGroup RightScore1HighConeBalance()
    {
        return new SequentialCommandGroup(
                    Commands.runOnce(() -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.CONE)),
                    new ManipulatorToPoseCmd(CatzConstants.ManipulatorPoseConstants.SCORE_HIGH_CONE)

                   //new PPTrajectoryFollowingCmd(driveStraighFullTurn)
            
        );
    }

    public static SequentialCommandGroup intakeSequenceCommandGroup()
    {

        return new SequentialCommandGroup(
                intake.intakeRollersIn(),
                Commands.waitSeconds(0.5),
                intake.intakeRollersOff()           
                                         );
    }








    
    



}
