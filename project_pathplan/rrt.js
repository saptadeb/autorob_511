/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree


}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    if (search_iter_count > search_max_iterations) {
        search_iterate = false;
        return "failed";
    }

    if (search_iter_count == 1){
        RRT_connect_flag = true
    }

    var q_random = randomConfig();

    if (extendRRT(T_a, q_random) !== 'collided') {
        if (connectRRT(T_b, T_a.vertices[T_a.newest].vertex) === "reached") {
            search_iterate = false;
            var path_a = dfsPath(T_a);
            var path_b = dfsPath(T_b);
            path_length = T_a.vertices[T_a.newest].path + T_b.vertices[T_b.newest].path;
            var path = [];
            if (RRT_connect_flag) {
                path = path_a.concat(path_b.reverse());
            } else {
                path = path_b.concat(path_a.reverse());
            }
            drawHighlightedPath(path);
            return "succeeded";
        }
    }

    var tmp = T_a;
    T_a = T_b;
    T_b = tmp;
    RRT_connect_flag = !RRT_connect_flag;

    return "extended";
}

function iterateRRTStar() {
    if (search_iter_count > search_max_iterations) {
        search_iterate = false;
        return "failed";
    }

    var prob_goal = 0.2;
    var isSearchingGoal = Math.random() < prob_goal;
    var q_random = isSearchingGoal ? q_goal : randomConfig();

    if (extendRRTStar(T_a, q_random) === "reached" && isSearchingGoal) {
        search_iterate = false;
        var path = dfsPath(T_a);
        path_length = T_a.vertices[T_a.newest].path;
        drawHighlightedPath(path);
        return "succeeded";
    }

    return "extended";
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

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

function connectRRT(T, q) {
    var status = 'advanced';
    while (status === 'advanced') {
        status = extendRRT(T, q);
    }
    return status;
}

function randomConfig() {
    var minX = range[0][1][1];
    var maxX = range[1][1][0];

    var randX = minX + (maxX - minX) * Math.random();
    var randY = minX + (maxX - minX) * Math.random();
    return [randX, randY];
}

function extendRRT(T, q) {
    var nearestIdx = findNearestNeighbor(T, q);
    // console.log(nearestIdx)
    var nearestVertex = T.vertices[nearestIdx].vertex;
    var newVertex = [];
    for (var i = 0; i < 2; i++) {
        newVertex[i] = (q[i] - nearestVertex[i]) * eps / distance(q, nearestVertex) + nearestVertex[i];
    }

    if (!testCollision(newVertex)) {
        var idx = insertTreeVertex(T, newVertex);
        insertTreeEdge(T, T.vertices.length - 1, nearestIdx);

        if (distance(newVertex, q) < eps) {
            // insertTreeVertex(T, q);
            // insertTreeEdge(T, T.vertices.length - 1, idx);
            return "reached";
        } else {
            return "iterating";
        }
    }
    return "collided";
}


function extendRRTStar(T, q) {
    search_iter_count++;
    var nearestIdx = findNearestNeighbor(T, q);
    var nearestVertex = T.vertices[nearestIdx].vertex;
    var newVertex = [];
    for (var i = 0; i < 2; i++) {
        newVertex[i] = (q[i] - nearestVertex[i]) * eps / distance(q, nearestVertex) + nearestVertex[i];
    }

    if (!testCollision(newVertex)) {
        var neighbors = [];

        for (i = 0; i < T.vertices.length; i++) {
            var dist = distance(newVertex, T.vertices[i].vertex);
            if (dist <= eps) {
                neighbors.push(T.vertices[i]);
            }
        }

        var cost = null;
        var minCost = Number.MAX_VALUE;
        var minIdx = null;

        for (i = 0; i < neighbors.length; i++) {
            if (typeof(neighbors[i].path) === 'undefined'){
                temp = 0
            }
            else{
                temp = neighbors[i].path
            }
            cost = distance(newVertex, neighbors[i].vertex) + temp;
            // console.log(cost)
            if (cost < minCost) {
                minCost = cost;
                minIdx = i;
            }
        }

        var parentIdx = T.vertices.indexOf(neighbors[minIdx]);
        var idx = insertTreeVertex(T, newVertex);

        // jugaad
        if (parentIdx == -1){
            parentIdx = 0
        }

        // console.log(parentIdx)
        insertTreeEdge(T, T.vertices.length - 1, parentIdx);
        rewrite(neighbors, T.vertices[idx]);

        if (distance(newVertex, q) < eps) {
            insertTreeVertex(T, q);
            insertTreeEdge(T, T.vertices.length - 1, idx);
            return "reached";
        } else {
            return "advanced";
        }
    }
    return "collided";
}

function newConfig(q_rand, q_near){
    var q_new = [];
    var q_distance = distance(q_rand, q_near);
    for(var i=0; i<q_rand.length; i++){
        q_new.push(q_near[i] + eps/q_distance*(q_rand[i] - q_near[i]));
    }

    if(testCollision(q_new)){
        return false;
    }
    return q_new;
}

function findNearestNeighbor(T, q) {
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

function distance(q1, q2){
    var square_sum = 0;
    for(var i=0; i<q1.length; i++){
        square_sum += Math.pow((q1[i]-q2[i]),2);
    }
    return Math.pow(square_sum, 0.5);
}

function dfsPath(T) {
    var path = [];
    var currVertex = T.vertices[T.newest];

    while (!isEqualNode(currVertex.vertex, T.vertices[0].vertex)) {
        path.unshift(currVertex);
        currVertex = currVertex.edges[0];
    }
    path.unshift(currVertex);
    return path;
}

function isEqualNode(q1, q2) {
    return hashcode(q1) === hashcode(q2);
}

function hashcode(q) {
    var hash1 = q[0].toFixed(1);
    var hash2 = q[1].toFixed(1);
    if (hash1 === '-0.0') {
        hash1 = '0.0';
    }
    if (hash2 === '-0.0') {
        hash2 = '0.0';
    }
    return hash1 + ' ' + hash2;
}