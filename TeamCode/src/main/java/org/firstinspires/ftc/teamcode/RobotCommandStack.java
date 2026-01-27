package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

public class RobotCommandStack {

    private Hardware hardware = null;
    private String stackName = null;
    public  RobotCommandStack(Hardware hardware, String stackName) {
//        this.opMode = opMode;
        this.stackName = stackName;
        this.hardware = hardware;
    }

    private final RobCommand robotCommandNull = new RobCommand();
//    private final Action robotActionNull = new Action() {
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            return false;
//        }
//    };

    public int currentRobotCommandIndex = -1;
    public int nextRobotCommandIndex = 0;
    public List<RobCommand> robotCommands = new ArrayList<RobCommand>();

//    public int currentRobotActionIndex = -1;
//    public int nextRobotActionIndex = 0;

    //Creates a list of all the actions created
//    public List<Action> robotActions = new ArrayList<Action>();




    public void processCommands(){
        if(currentRobotCommandIndex == -1){
            if(robotCommands.size() > nextRobotCommandIndex) {
                currentRobotCommandIndex = nextRobotCommandIndex;
                robotCommands.get(currentRobotCommandIndex).run();
            }
        }
        else if(currentRobotCommandIndex < robotCommands.size()){
            if(robotCommands.get(currentRobotCommandIndex).isComplete()){
                robotCommands.get(currentRobotCommandIndex).hasCompleted = true;
                nextRobotCommandIndex++;
                if(nextRobotCommandIndex < robotCommands.size()) {
                    currentRobotCommandIndex = nextRobotCommandIndex;
                    robotCommands.get(currentRobotCommandIndex).run();
                }
                else currentRobotCommandIndex = -1;
            }
        }
    }

//    public void processActions(){
//        if(currentRobotActionIndex == -1){
//            if(robotActions.size() > nextRobotActionIndex){
//                currentRobotActionIndex = nextRobotActionIndex;
//                runAction(robotActions.get(currentRobotActionIndex));
//            }
//        }
//        else if(currentRobotActionIndex < robotActions.size()){
//            if(!robotActions.get(currentRobotActionIndex).run(hardware.packet)){
//                robotActions.get(currentRobotActionIndex).run(new TelemetryPacket());
//                nextRobotActionIndex++;
//                if(nextRobotCommandIndex < robotCommands.size()) {
//                    currentRobotActionIndex = nextRobotActionIndex;
//                    runAction(robotActions.get(currentRobotActionIndex));
//                }
//                else currentRobotCommandIndex = -1;
//            }
//        }
//    }

//    public void runAction(Action action){
//        if(action instanceof RCRoadrunner1){
//            Actions.runBlocking(action);
//        }else{
//            action.run(hardware.packet);
//        }
//    }



    public void addCommand(RobCommand command){
        hardware.logMessage(false,"Robot130",stackName + "; Command Added: " + command.toString());
        robotCommands.add(command);
    }

//    public void addAction(Action action){
//        hardware.logMessage(false,"Robot130", stackName + "; Action Added: " + action.toString());
//        robotActions.add(action);
//    }

    public boolean isRoadRunnerActive(){
        if(getCurrentCommand() instanceof RCRoadrunner1){
            return true;
        }else{
            return false;
        }
    }

//    public boolean isRoadRunnerActionActive(){
//        if(getCurrentCommand() instanceof RCRoadrunner1){
//            return true;
//        }else{
//            return false;
//        }
//    }

    public int getNumCommands(){
        return robotCommands.size();
    }

//    public int getNumActions(){return robotActions.size();}
    public int getCurrentCommandIndex(){
        return currentRobotCommandIndex;
    }
//    public int getCurrentActionIndex(){
//        return currentRobotActionIndex;
//    }
    public int getNextCommandIndex(){
        return nextRobotCommandIndex;
    }
//    public int getNextActionIndex(){return nextRobotActionIndex;}

    public RobCommand getCurrentCommand(){
        if(currentRobotCommandIndex == -1){
            return robotCommandNull;
        }
        return robotCommands.get(currentRobotCommandIndex);
    }

    public ArrayList<RobCommand> getUnfinishedCommands(){
        if(currentRobotCommandIndex == -1){
            return new ArrayList<RobCommand>();
        }
        return new ArrayList<>(robotCommands.subList(currentRobotCommandIndex, robotCommands.size() - 1));
    }

//    public Action getCurrentAction(){
//        if(currentRobotActionIndex == -1){
//            return robotActionNull;
//        }
//        return robotActions.get(currentRobotActionIndex);
//    }

    public void cancelFutureCommands(){
        currentRobotCommandIndex = -1;
        nextRobotCommandIndex = robotCommands.size();
    }

//    public void cancelFutureActions(){
//        currentRobotActionIndex = -1;
//        nextRobotActionIndex = robotActions.size();
//    }

}