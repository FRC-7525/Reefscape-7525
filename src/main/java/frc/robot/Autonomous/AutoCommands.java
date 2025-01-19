package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AutoCommands {

    public class IntakeCoral extends Command {
        private final Manager manager = Manager.getInstance();
        private final boolean leftSourceTargeted;

        public IntakeCoral(boolean leftSourceTargeted) {
            this.leftSourceTargeted = leftSourceTargeted;
        }

        @Override
        public void initialize() {
            manager.setLeftSourceTargeted(leftSourceTargeted);
            manager.setState(ManagerStates.INTAKING_CORALER_AA_OFF);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class ScoreReef extends Command {
        private final Manager manager = Manager.getInstance();
        private final int scoringLevel;

        public ScoreReef(int scoringLevel) {
            this.scoringLevel = scoringLevel;
        }

        @Override
        public void initialize() {
            manager.setDriverReefScoringLevel(scoringLevel);
            manager.setState(ManagerStates.TRANSITIONING_SCORING_REEF);
        }

        @Override
        public boolean isFinished() {
            return manager.getState() == ManagerStates.IDLE;
        }
    } 

    public class ProcessAlgae extends Command {
        
    }
}
