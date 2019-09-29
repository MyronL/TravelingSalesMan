class Element {
  constructor (element, priority) {
    this.element = element;
    this.priority = priority;
  }
}

class PriorityQueue {
  constructor () {
    this.items = [];
  }
  // enqueue function to add element
  // to the queue as per priority
  enqueue(element, priority) {
  	// creating object from queue element
  	var element = new Element(element, priority);
  	var contain = false;

  	// iterating through the entire
  	// item array to add element at the
  	// correct location of the Queue
  	for (var i = 0; i < this.items.length; i++) {
  		if (this.items[i].priority > element.priority) {
  			// Once the correct location is found it is
  			// enqueued
  			this.items.splice(i, 0, element);
  			contain = true;
  			break;
  		}
  	}

  	// if the element have the highest priority
  	// it is added at the end of the queue
  	if (!contain) {
  		this.items.push(element);
  	}
  }

  // dequeue method to remove
  // element from the queue
  dequeue() {
  	// return the dequeued element
  	// and remove it.
  	// if the queue is empty
  	// returns Underflow
  	if (this.isEmpty())
  		return "Underflow";
  	return this.items.shift();
  }

  // isEmpty function
  isEmpty() {
  	// return true if the queue is empty.
  	return this.items.length == 0;
  }

}

module.exports = PriorityQueue;
