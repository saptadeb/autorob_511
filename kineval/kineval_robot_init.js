/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics 
     in HTML5/JavaScript and threejs
     
     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

*/

kineval.initRobot = function initRobot() {
        
    // ASSUME: robot kinematics are described separate js file (eg., "robot_urdf_example.js")

    // initialize and create threejs mesh objects for robot links
    kineval.initRobotLinks();

    // initialize robot joints and create threejs mesh objects for robot joints and form kinematic hiearchy
    kineval.initRobotJoints();

    // initialize robot collision state
    robot.collision = false;

}

kineval.initRobotLinks = function initRobotLinks() {

    for (x in robot.links) {
        robot.links[x].name = x;
    }

    // initialize controls for robot base link
    robot.control = {xyz: [0,0,0], rpy:[0,0,0]}; 
}

kineval.initRobotJoints = function initRobotJoints() {
    var x

    for (x in robot.joints) {
        robot.joints[x].name = x
        robot.joints[x].angle = 0
        robot.joints[x].control = 0
        robot.joints[x].servo = {}
        robot.joints[x].servo.p_gain = 0 
        robot.joints[x].servo.p_desired = 0
        robot.joints[x].servo.d_gain = 0

    var tempParent = robot.joints[x].parent
    var tempChild = robot.joints[x].child
    if (typeof robot.links[tempParent].children === 'undefined'){
            robot.links[tempParent].children=[]
        }
        robot.links[tempChild].parent = x
        robot.links[tempParent].children.push(x)    
    }
    timeStart = new Date()
}








