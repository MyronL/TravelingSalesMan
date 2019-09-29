class Graph {
	// defining vertex array and
	// adjacent list
	constructor() {
		this.AdjList = new Map();
    this.nodes = [];
	}

  // add vertex to the graph
  addNode(v) {
  	// initialize the adjacent list with an empty array
    this.nodes.push(v);
    if (!this.AdjList.get(v)) this.AdjList.set(v, []);
  }

  // add edge to the graph
  addEdge(v, w, weight) {
  	// get the list for vertex v and put the
  	// vertex w denoting edge between v and w
  	this.AdjList.get(v).push({node: w, weight});

  	// Since graph is undirected,
  	// add an edge from w to v also
    if (!this.AdjList.get(w)) {
      this.AdjList.set(w, []);
    }
  	this.AdjList.get(w).push({node: v, weight});
  }

  removeEdge(v, w) {
    let filteredArray = this.AdjList.get(v).filter((node) => {
      return node.node !== w;
    })

    this.AdjList.set(v, filteredArray);

    filteredArray = this.AdjList.get(w).filter((node) => {
      return node.node !== v;
    })

    this.AdjList.set(w, filteredArray);
  }

  getEdges () {
    return Array.from(this.AdjList.values());
  }

  // Prints the vertex and adjacency list
  printGraph () {
  	// get all the vertices
  	var get_keys = this.AdjList.keys();

  	// iterate over the vertices
  	for (var i of get_keys) {
  		// great the corresponding adjacency list
  		// for the vertex
  		var get_values = this.AdjList.get(i);
  		var conc = "";

  		// iterate over the adjacency list
  		// concatenate the values into a string
  		for (var j of get_values)
  			conc += JSON.stringify(j) + " ";

  		// print the vertex and its adjacency list
  		console.log(i + " -> " + conc);
  	}
  }
}

module.exports = Graph;
