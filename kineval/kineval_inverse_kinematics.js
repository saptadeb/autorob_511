
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

    var endeRPYWorld = rotation_matrix_to_axisangle(robot.joints[endeffector_joint].xform);
    var endeXYZWorld = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    var delta_position = vec_minus(endeffector_target_world.position, endeXYZWorld);
    var delta_orientation = vec_minus(endeffector_target_world.orientation, endeRPYWorld);

    robot.dx = [];
    for (i = 0; i < 3; i++) {
        robot.dx[i] = [delta_position[i]];
        if (kineval.params.ik_orientation_included) {
            robot.dx[i + 3] = [delta_orientation[i]];
        } else {
            robot.dx[i + 3] = [0];
        }
    }

    var joint_chain = [];
    var curr_joint = endeffector_joint;
    while (robot.joints[curr_joint].parent !== robot.base) {
        joint_chain.push(curr_joint);
        curr_joint = robot.links[robot.joints[curr_joint].parent].parent;
    }
    joint_chain.push(curr_joint);
    joint_chain = rev_vec(joint_chain)
    
    robot.jacobian = [[], [], [], [], [], []]; 

    for (var i = 0; i < joint_chain.length; i++) {
        // var j = joint_chain.length - i - 1;
        var joint = robot.joints[joint_chain[i]];
        var joint_origin_world = matrix_multiply(joint.xform, [[0], [0], [0], [1]]);

        var axis_local = [[joint.axis[0]], [joint.axis[1]], [joint.axis[2]], [0]];
        var axis_world = matrix_multiply(joint.xform, axis_local);

        var J_cross = vector_cross(axis_world, vec_minus(endeXYZWorld, joint_origin_world));
        for (var k = 0; k < 3; k++) {
            if (joint.type === 'prismatic') {
                robot.jacobian[k][i] = axis_world[k][0];
                robot.jacobian[k + 3][i] = 0;
            } else {
                robot.jacobian[k][i] = J_cross[k];
                robot.jacobian[k + 3][i] = axis_world[k][0];
            }
        }
    }

    if (kineval.params.ik_pseudoinverse) {
        var pseudoinverse = matrix_pseudoinverse(robot.jacobian);
        robot.dq = matrix_multiply(pseudoinverse, robot.dx);
    } else {
        robot.dq = matrix_multiply(matrix_transpose(robot.jacobian), robot.dx);
    }

    for (i = 0; i < joint_chain.length; i++) {
        robot.joints[joint_chain[i]].control += kineval.params.ik_steplength * robot.dq[i][0];
    }
}


