
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();   
}

kineval.buildFKTransforms = function buildFKTransforms () {
    traverseFKBase()
    var lengthBaseChildren = robot.links[robot.base].children.length
    for (var i = 0; i < lengthBaseChildren; i++){
        traverseFKJoint(robot.links[robot.base].children[i])
    }
}

// STENCIL: reference code alternates recursive traversal over 
//   links and joints starting from base, using following functions: 
//     traverseFKBase
//     traverseFKLink
//     traverseFKJoint
//
// user interface needs the heading (z-axis) and lateral (x-axis) directions
//   of robot base in world coordinates stored as 4x1 matrices in
//   global variables "robot_heading" and "robot_lateral"
//
// if geometries are imported and using ROS coordinates (e.g., fetch),
//   coordinate conversion is needed for kineval/threejs coordinates:
//

function traverseFKBase() {
    var rotM = generate_rotation_matrix(robot.origin.rpy[0], robot.origin.rpy[1], robot.origin.rpy[2])
    var transM = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2])
    var temp1 = [[0],[0],[1],[1]]
    robot.links[robot.base].xform = matrix_multiply(transM, rotM)
    robot_heading = matrix_multiply(robot.links[robot.base].xform, temp1)
    var temp2 = [[1],[0],[0],[1]]
    robot_lateral = matrix_multiply(robot.links[robot.base].xform, temp2)
    if (robot.links_geom_imported) {
        var tempTrans = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2))
        robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, tempTrans)
    }
}

function traverseFKJoint(curJoint) {
    var rotM = generate_rotation_matrix(robot.joints[curJoint].origin.rpy[0],robot.joints[curJoint].origin.rpy[1],robot.joints[curJoint].origin.rpy[2])
    var transM = generate_translation_matrix(robot.joints[curJoint].origin.xyz[0], robot.joints[curJoint].origin.xyz[1], robot.joints[curJoint].origin.xyz[2])
    var mJ
    if (robot.links_geom_imported){
        if (robot.joints[curJoint].type === "prismatic"){
            var temp = [robot.joints[curJoint].angle, robot.joints[curJoint].angle, robot.joints[curJoint].angle]
            temp = vector_dot(temp, robot.joints[curJoint].axis)
            mJ = generate_translation_matrix(temp[0],temp[1],temp[2])
        }
        else if ((robot.joints[curJoint].type === "continuous")|(robot.joints[curJoint].type === "revolute")){
            mJ = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[curJoint].axis, robot.joints[curJoint].angle)))
        }
        else{
            mJ = generate_identity(4);
        }
    }
    else{
        mJ = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[curJoint].axis, robot.joints[curJoint].angle))) 
    }
    var tempTrans = matrix_multiply(robot.links[robot.joints[curJoint].parent].xform, matrix_multiply(transM, rotM))
    robot.joints[curJoint].xform = matrix_multiply(tempTrans, mJ);
    traverseFKLink(robot.joints[curJoint].child);
}

function traverseFKLink(curLink) {
    robot.links[curLink].xform = robot.joints[robot.links[curLink].parent].xform
    if (typeof robot.links[curLink].children !== 'undefined'){
        for (var i = 0; i < robot.links[curLink].children.length; i++){
            traverseFKJoint(robot.links[curLink].children[i])
        }
    }
    else {
        return
    }
}

