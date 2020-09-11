/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
    heap.push(new_element)

    if (heap.length > 1) {
        let current = heap.length - 1

        while (current > 0 && heap[Math.floor((current - 1)/2)] > heap[current]) {
            [heap[Math.floor((current - 1)/2)], heap[current]] = [heap[current], heap[Math.floor((current - 1)/2)]]
            current = Math.floor((current - 1)/2)
        }
    }
    // STENCIL: implement your min binary heap insert operation
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
    let top = heap[0]
    heap[0] = heap[heap.length-1]
    heap.length--
    heapify(heap,0)
    console.log(top)
    return top
    // STENCIL: implement your min binary heap extract operation
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object

/* code for recursive extration */
function heapify(arr, index){
    largest = index
    leftchild = 2*index + 1
    rightchild = 2*index + 2
    if (leftchild < arr.length && arr[leftchild] < arr[largest]){
        largest = leftchild
    }
    if (rightchild < arr.length && arr[rightchild] < arr[largest]){
        largest = rightchild
    }
    if (largest != index){
        let temp = arr[index]
        arr[index] = arr[largest]
        arr[largest] = temp
        heapify(arr, largest)
    }
}