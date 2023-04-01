package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorFromGround;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.Intake;

public class FeederConeRetractIntake extends SequentialCommandGroup{
    public FeederConeRetractIntake(Intake m_intake, Elevator m_elevator){
        addCommands(
            // new IntakeOff(m_intake), //TODO REMOVE?
            new ElevatorToNode(m_elevator, Elevator.L),
            new IntakeOn(m_intake),
            new ElevatorFromGround(m_elevator, Elevator.J, m_intake)
        );
    }
}
