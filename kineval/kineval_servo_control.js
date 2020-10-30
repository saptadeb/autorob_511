
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return;
    //kineval.params.persist_pd = true;

    var temp = 0;
    // STENCIL: implement FSM to cycle through dance pose setpoints
    for(x in robot.joints){
        kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_pose_index][x];
        temp += Math.pow((kineval.params.setpoint_target[x]-robot.joints[x].angle), 2);
    }
    var root = Math.sqrt(temp);

    if(root >= 0.01)
        return;

    kineval.params.dance_pose_index += 1;

    if(kineval.params.dance_pose_index == 10)
        kineval.params.dance_pose_index = 0;


    kineval.setPoseSetpoint(kineval.params.dance_pose_index);
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (k in robot.joints) {
        var temp = kineval.params.setpoint_target[k]-robot.joints[k].angle
        temp = temp*robot.joints[k].servo.p_gain
        robot.joints[k].control = temp
    }

}


