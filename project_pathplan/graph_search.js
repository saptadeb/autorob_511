/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

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

minheaper = {}; 

function initSearchGraph() {

    // create the search queue
    visit_queue = [];
    counter = 0 // for BFS priority

    // if (testCollision(q_goal) == true || testCollision(q_init) == true){
    //     search_iterate = false
    //     return "failed"
    // }

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent (g-score)
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };
            
            if (G[iind][jind].x - (eps/2) < q_init[0] && q_init[0] <= G[iind][jind].x + (eps/2) && G[iind][jind].y - (eps/2) < q_init[1] && q_init[1] <= G[iind][jind].y + (eps/2)) {
                G[iind][jind].distance = 0 // distance to start via path through parent
                G[iind][jind].visited = true // flag for whether the node has been visited
                if (search_alg == "breadth-first")
                    G[iind][jind].priority = counter
                else
                    G[iind][jind].priority = (0 + hscore(G[iind][jind].x, G[iind][jind].y))
                G[iind][jind].queued = true
                minheap_insert(visit_queue, G[iind][jind])
            }
            // STENCIL: determine whether this graph node should be the start
            //   point for the search
        }
    }
}

function iterateGraphSearch() {
    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location

    if (testCollision(q_goal) == true || testCollision(q_init) == true){
        search_iterate = false
        return "failed"
    }

    if (visit_queue.length == 0){
        return "failed"
    }
    cur_node = minheap_extract(visit_queue)
    cur_node.visited = true
    cur_node.queued = false
    search_visited += 1
    collision_check = testCollision([cur_node.x, cur_node.y])
    draw_2D_configuration([cur_node.x, cur_node.y], "visited")

    if (collision_check === true)
        return "failed";

    if (isGoal(cur_node)){
        drawHighlightedPathGraph(cur_node)
        search_iterate = false
        return "succeeded"
    }
    cur_node.visited = true
    for (u = -1; u < 2; u++){
        for (v = -1; v < 2; v++){
            if ((u == 0 && v == 0) || Math.abs(u) == Math.abs(v)){
                continue
            }
            ngbr = G[cur_node.i + u][cur_node.j + v]
            if (ngbr.visited == false && ngbr.distance > (cur_node.distance + eps) && testCollision([ngbr.x,ngbr.y]) == false){
                ngbr.parent = cur_node
                ngbr.distance = cur_node.distance + eps
                ngbr.queued = true
                ngbr.priority = ngbr.distance + hscore(ngbr.x, ngbr.y)
                minheap_insert(visit_queue, ngbr)
                draw_2D_configuration([ngbr.x, ngbr.y], "queued")
            }
        }
    }
    return "iterating"
}

/*Greedy Best first Search */
function greedy(){
    if (visit_queue.length == 0){
        return "failed"
    }
    cur_node = minheap_extract(visit_queue)
    cur_node.visited = true
    cur_node.queued = false
    search_visited += 1
    collision_check = testCollision([cur_node.x, cur_node.y])
    draw_2D_configuration([cur_node.x, cur_node.y], "visited")

    if (collision_check === true)
        return "failed";

    if (isGoal(cur_node)){
        drawHighlightedPathGraph(cur_node)
        search_iterate = false
        return "succeeded"
    }
    cur_node.visited = true
    for (u = -1; u < 2; u++){
        for (v = -1; v < 2; v++){
            if ((u == 0 && v == 0) || Math.abs(u) == Math.abs(v)){
                continue
            }
            ngbr = G[cur_node.i + u][cur_node.j + v]
            if (ngbr.visited == false && ngbr.distance > (cur_node.distance + eps) && testCollision([ngbr.x,ngbr.y]) == false){
                ngbr.parent = cur_node
                ngbr.distance = cur_node.distance + eps
                ngbr.queued = true
                ngbr.priority = hscore(ngbr.x, ngbr.y)
                minheap_insert(visit_queue, ngbr)
                draw_2D_configuration([ngbr.x, ngbr.y], "queued")
            }
        }
    }
    return "iterating"
}

/* DFS */
function dfs(){
    if (visit_queue.length == 0){
        return "failed"
    }
    cur_node = minheap_extract(visit_queue)
    cur_node.visited = true
    cur_node.queued = false
    search_visited += 1
    collision_check = testCollision([cur_node.x, cur_node.y])
    draw_2D_configuration([cur_node.x, cur_node.y], "visited")

    if (collision_check === true)
        return "failed";

    if (isGoal(cur_node)){
        drawHighlightedPathGraph(cur_node)
        search_iterate = false
        return "succeeded"
    }
    cur_node.visited = true  
    for (u = -1; u < 2; u++){
        for (v = -1; v < 2; v++){
            if ((u == 0 && v == 0) || Math.abs(u) == Math.abs(v)){
                continue
            }
            ngbr = G[cur_node.i + u][cur_node.j + v]
            if (ngbr.visited == false && ngbr.distance > (cur_node.distance + eps) && testCollision([ngbr.x,ngbr.y]) == false){
                ngbr.parent = cur_node
                ngbr.distance = cur_node.distance + eps
                ngbr.queued = true
                ngbr.priority = ngbr.distance 

                minheap_insert(visit_queue, ngbr)
                draw_2D_configuration([ngbr.x, ngbr.y], "queued")
            }
        }
    }
    return "iterating"
}

/* BFS */
function bfs(){
    if (visit_queue.length == 0){
        return "failed"
    }
    cur_node = minheap_extract(visit_queue)
    cur_node.visited = true
    cur_node.queued = false
    search_visited += 1
    collision_check = testCollision([cur_node.x, cur_node.y])
    draw_2D_configuration([cur_node.x, cur_node.y], "visited")

    if (collision_check === true)
        return "failed";

    if (isGoal(cur_node)){
        drawHighlightedPathGraph(cur_node)
        search_iterate = false
        return "succeeded"
    }
    cur_node.visited = true
    for (u = -1; u < 2; u++){
        for (v = -1; v < 2; v++){
            if ((u == 0 && v == 0) || Math.abs(u) == Math.abs(v)){
                continue
            }
            ngbr = G[cur_node.i + u][cur_node.j + v]
            if (ngbr.visited == false && ngbr.distance > (cur_node.distance + eps) && testCollision([ngbr.x,ngbr.y]) == false){
                ngbr.parent = cur_node
                ngbr.distance = cur_node.distance + eps
                ngbr.queued = true
                ngbr.priority = ++counter
                minheap_insert(visit_queue, ngbr)
                draw_2D_configuration([ngbr.x, ngbr.y], "queued")
            }
        }
    }
    return "iterating"
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.

function minheap_insert(heap, new_element) {
    heap.push(new_element)
    
    if (heap.length > 1) {
        let current = heap.length - 1
        while (current > 0 && heap[Math.floor((current - 1)/2)].priority > heap[current].priority) {
            let temp = heap[Math.floor((current - 1)/2)]
            heap[Math.floor((current - 1)/2)] = heap[current]
            heap[current] = temp
            current = Math.floor((current - 1)/2)
        }
    }
}

function minheap_extract(heap) {
    let top = heap[0]
    heap[0] = heap[heap.length-1]
    heap.pop()
    heapify(heap,0)
    // console.log(top)
    return top
}

function heapify(arr, index){
    largest = index
    leftchild = 2*index + 1
    rightchild = 2*index + 2
    if (leftchild < arr.length && arr[leftchild].priority < arr[largest].priority){
        largest = leftchild
    }
    if (rightchild < arr.length && arr[rightchild].priority < arr[largest].priority){
        largest = rightchild
    }
    if (largest != index){
        let temp = arr[index]
        arr[index] = arr[largest]
        arr[largest] = temp
        heapify(arr, largest)
    }
}

function hscore(p1, p2){
    g1 = q_goal[0]
    g2 = q_goal[1]
    d1 = p1 - g1
    d2 = p2 - g2
    heuristic = Math.sqrt(Math.pow(d1, 2) + Math.pow(d2, 2))
    // console.log(heuristic)
    return heuristic
}

function isGoal(givennode){
    if (givennode.x - (eps/2) < q_goal[0] && q_goal[0] <= givennode.x + (eps/2) && givennode.y - (eps/2) < q_goal[1] && q_goal[1] <= givennode.y + (eps/2))
        return true

    else
        return false
}

function manhattan_dist(p,n){
    if (p.x == n.x || p.y == n.y)
        return eps
    else
        return 2 * eps
}

