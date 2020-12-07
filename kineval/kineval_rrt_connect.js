
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    RRT_connect_flag = true;
    step = 0.4;
    radius = 2 * step;
    angle_scale = 0.1;
    threshold_goal = 0.7;
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED), 2: rrt_star

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
        var q_rand = random_config();

        if (rrt_alg === 1) {
            // rrt_connect
            if (rrt_extend(T_a, q_rand) !== 'trapped') {
                if (rrt_connect(T_b, T_a.vertices[T_a.newest].vertex) === "reached") {
                    rrt_iterate = false;
                    var path_a = find_path(T_a);
                    var path_b = find_path(T_b);
                    kineval.motion_plan = [];
                    kineval.motion_plan_traversal_index = 0;

                    if (RRT_connect_flag) {
                        kineval.motion_plan = path_a.reverse().concat(path_b);
                    } else {
                        kineval.motion_plan = path_b.reverse().concat(path_a);
                    }
                    return "reached";
                }
            }

            var tmp = T_a;
            T_a = T_b;
            T_b = tmp;
            RRT_connect_flag = !RRT_connect_flag;

        } else if (rrt_alg === 2) {
            // rrt_star
            var prob_goal = 0.2;
            var isSearchingGoal = Math.random() < prob_goal;
            if (isSearchingGoal) {
                q_rand = q_goal_config;
            }

            if (rrt_star_extend(T_a, q_rand) === "reached" && isSearchingGoal) {
                rrt_iterate = false;
                var path = find_path(T_a);
                kineval.motion_plan = [];
                kineval.motion_plan_traversal_index = 0;
                kineval.motion_plan = path.reverse();
                return "reached";
            }
        }
    }
    return 'extended';

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

    function rrt_extend(T, q) {
        rrt_iter_count++;
        var nearestIdx = nearest_neighbor(T, q);
        var nearestVertexConf = T.vertices[nearestIdx].vertex;
        var newVertexConf = new_config(nearestVertexConf, q);
    
        if (!kineval.poseIsCollision(newVertexConf)) {
            var idx = tree_add_vertex(T, newVertexConf);
            tree_add_edge(T, T.vertices.length - 1, nearestIdx);
    
            if (distance(newVertexConf, q) < step) {
                // tree_add_vertex(T, q);
                // tree_add_edge(T, T.vertices.length - 1, idx);
                return "reached";
            } else {
                return "advanced";
            }
        }
        return "trapped";
    }

    function rrt_connect(T, q) {
        var status = 'advanced';
        while (status === 'advanced') {
            status = rrt_extend(T, q);
        }
        return status;
    }

    function random_config() {
        var xMin = robot_boundary[0][0];
        var xMax = robot_boundary[1][0];
        var zMin = robot_boundary[0][2];
        var zMax = robot_boundary[1][2];
    
        var q_rand = [];
        q_rand[0] = 1.4 * xMin - 0.4 * xMax + 1.8 * (xMax - xMin) * Math.random();
        q_rand[1] = 0;
        q_rand[2] = 1.4 * zMin - 0.4 * zMax + 1.8 * (zMax - zMin) * Math.random();
        q_rand[3] = 0;
        q_rand[4] = 2 * Math.PI * Math.random();
        q_rand[5] = 0;
    
        for (var i = 6; i < T_a.vertices[0].vertex.length; i++) {
            var joint = robot.joints[q_index[i]];
            if (joint.type === 'revolute' || joint.type === 'prismatic') {
                q_rand[i] = joint.limit.lower + (joint.limit.upper - joint.limit.lower) * Math.random();
            } else {
                q_rand[i] = 2 * Math.PI * Math.random();
            }
        }
        return q_rand;
        // var minX = range[0][1][1];
        // var maxX = range[1][1][0];

        // var randX = minX + (maxX - minX) * Math.random();
        // var randY = minX + (maxX - minX) * Math.random();
        // return [randX, randY];
    }

    function new_config(q_from, q_to) {
        var diff = [];
        for (var i = 0; i < q_from.length; i++) {
            diff[i] = q_to[i] - q_from[i];
            if (i >= 3) {
                diff[i] *= angle_scale;
            }
        }
        var diff_normal = vector_normalize(diff);
    
        var newConf = [];
        for (i = 0; i < q_from.length; i++) {
            newConf[i] = q_from[i] + diff_normal[i] * step;
    
            var joint = robot.joints[q_index[i]];
            if (joint && (joint.type === 'revolute' || joint.type === 'prismatic')) {
                newConf[i] = Math.min(joint.limit.upper, Math.max(joint.limit.lower, newConf[i]));
            }
        }
        return newConf;
    }

    function nearest_neighbor(T, q) {
        var dist = null;
        var minDist = Number.MAX_VALUE;
        var minIdx = null;
    
        for (var i = 0; i < T.vertices.length; i++) {
            dist = distance(T.vertices[i].vertex, q);
            if (dist < minDist) {
                minDist = dist;
                minIdx = i;
            }
        }
        return minIdx;
    }

    function find_path(T) {
        var path = path_dfs(T);
        for (var i = 0; i < path.length; i++) {
            path[i].geom.material.color = {r:1, g:0, b:0};
        }
        return path;
    }
    
    function path_dfs(T) {
        var path = [];
        var curr = T.vertices[T.newest];
    
        while (curr !== T.vertices[0]) {
            path.push(curr);
            curr = curr.edges[0];
        }
        path.push(curr);
        return path;
    }
    
    function rrt_star_extend(T, q) {
        rrt_iter_count++;
        var nearestIdx = nearest_neighbor(T, q);
        var nearestVertexConf = T.vertices[nearestIdx].vertex;
        var newVertexConf = new_config(nearestVertexConf, q);
    
        if (!kineval.poseIsCollision(newVertexConf)) {
            var neighbors = [];
    
            for (var i = 0; i < T.vertices.length; i++) {
                var dist = distance(newVertexConf, T.vertices[i].vertex);
                if (dist <= radius) {
                    neighbors.push(T.vertices[i]);
                }
            }
    
            var cost = null;
            var minCost = Number.MAX_VALUE;
            var minIdx = null;
    
            for (i = 0; i < neighbors.length; i++) {
                cost = distance(newVertexConf, neighbors[i].vertex) + neighbors[i].path;
                if (cost < minCost) {
                    minCost = cost;
                    minIdx = i;
                }
            }
    
            var parentIdx = T.vertices.indexOf(neighbors[minIdx]);
            var idx = tree_add_vertex(T, newVertexConf);
            tree_add_edge(T, T.vertices.length - 1, parentIdx);
            rewrite(neighbors, T.vertices[idx]);
    
            if (distance(newVertexConf, q) < threshold_goal) {
                // tree_add_vertex(T, q);
                // tree_add_edge(T, T.vertices.length - 1, idx);
                return "reached";
            } else {
                return "advanced";
            }
        }
        return "trapped";
    }
    
    function rewrite(neighbors, newVertex) {
        for (var i = 0; i < neighbors.length; i++) {
            var parent = neighbors[i].edges[0];
            if (typeof(parent) === 'undefined') {
                continue;
            }
    
            var cost = newVertex.path + distance(newVertex.vertex, neighbors[i].vertex);
            if (cost < neighbors[i].path) {
                neighbors[i].edges[0] = newVertex;
                newVertex.edges.push(neighbors[i]);
                neighbors[i].path = cost;
                parent.edges.splice(parent.edges.indexOf(neighbors[i]), 1);
            }
        }
    }
    
    function distance(q1, q2) {
        var dist = 0;
        for (var i = 0; i < q1.length; i++) {
            if (i < 3) {
                dist += Math.pow(q1[i] - q2[i], 2);
            } else {
                dist += Math.pow((q1[i] - q2[i]) * angle_scale, 2);
            }
        }
        return Math.sqrt(dist);
    }
    

    

    









