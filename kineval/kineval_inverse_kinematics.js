
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length


    var endeXYZWorld = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    var endeRPYWorld = xformTransTheta(robot.joints[endeffector_joint].xform);
    var endeVec = [];
    var i;
    for (i = 0; i< 3; i++){
        endeVec[i] = [endeXYZWorld[i][0]];
        endeVec[i+3] = [endeRPYWorld[i][0]];
    }
    robot.dx = [];
    for (i = 0; i< 3; i++){
        robot.dx[i] = [endeffector_target_world.position[i] - endeVec[i][0]];
    }
    for (i = 3; i< 6; i++){
        if (kineval.params.ik_orientation_included){
            robot.dx[i] = [endeffector_target_world.orientation[i-3] - endeVec[i][0]];
        }
        else{
            robot.dx[i] = [0];
        }
    }


    robot.jacobian = [];
    var indexJoint = 0;
    var curJoint = endeffector_joint;
    

    
    var flagWhile = true;
    while (flagWhile){
        var tempAxis = matrix_vec_multiply(robot.joints[curJoint].xform,robot.joints[curJoint].axis)
         tempAxis = vector_normalize(tempAxis);
         if (robot.joints[curJoint].type === "prismatic"){
             robot.jacobian[indexJoint] = [tempAxis[0], tempAxis[1], tempAxis[2], 0, 0, 0];
         }else{
             var jointXYZWorld = matrix_multiply(robot.joints[curJoint].xform, [[0],[0],[0],[1]]);
             var diffJointXYZ = vec_minus(endeXYZWorld,jointXYZWorld);
             var tempC = vector_cross(tempAxis, diffJointXYZ);
             robot.jacobian[indexJoint] = [tempC[0], tempC[1], tempC[2], tempAxis[0], tempAxis[1], tempAxis[2]];
         }
         if (robot.joints[curJoint].parent === robot.base){
             flagWhile = false;
         }
         indexJoint++;
         curJoint = robot.links[robot.joints[curJoint].parent].parent;
     }
     robot.dq = [];
     if (!kineval.params.ik_pseudoinverse){
         robot.dq = matrix_multiply(robot.jacobian, robot.dx);
     }
     else{
         var jacobT = matrix_transpose(robot.jacobian);
         robot.dq = matrix_multiply(matrix_pseudoinverse(jacobT), robot.dx);
     
     }
     flagWhile = true;
     indexJoint = 0;
     curJoint = endeffector_joint;
     while (flagWhile){
         robot.joints[curJoint].control += kineval.params.ik_steplength*robot.dq[indexJoint][0];
         indexJoint++;
         if (robot.joints[curJoint].parent === robot.base){
             flagWhile = false;
         }
         curJoint = robot.links[robot.joints[curJoint].parent].parent;
     }

}

function xformTransTheta(xform){
    
    var theta1=[Math.atan2( xform[2][1], xform[2][2])];
    var theta3=[Math.atan2( xform[1][0], xform[0][0])];
    var temp = Math.pow(xform[2][1], 2)+ Math.pow(xform[2][2], 2);
    temp = Math.pow(temp,0.5);
    var theta2=[Math.atan2(-xform[2][0],temp)];
    
    var theta = [theta1,theta2,theta3];
    return theta;
}


