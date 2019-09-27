// UrbanLogiq Coding Challenge

/*
Christofides Algorithm Implementation
*/

const csv = require('csv-parser');
var munkres = require('munkres-js');
const fs = require('fs');

// Parse CSV File into an array of objects
function readFile () {
  return new Promise((resolve, reject)=> {
    let cities = [];
    fs.createReadStream('cities_medium.csv')
      .on('error', (err) => {
        reject(err);
      })
      .pipe(csv(['city', 'state', 'lat', 'lon']))
      .on('data', (row) => {
        cities.push(row);
      })
      .on('end', () => {
        resolve(cities);
      });
  })
}

// Returns the distance between 2 points in meters
function distanceBetweenPoints(lat1, lon1, lat2, lon2) {
  const radius = 6371e3; // Earth's radius
  const lat1Radians = degreesToRadians(lat1);
  const lat2Radians = degreesToRadians(lat2);
  const latDifRadians = degreesToRadians(lat2-lat1);
  const lonDifRadians = degreesToRadians(lon2-lon1);

  const cordLength = Math.sin(latDifRadians/2) * Math.sin(latDifRadians/2) +
            Math.cos(lat1Radians) * Math.cos(lat2Radians) *
            Math.sin(lonDifRadians/2) * Math.sin(lonDifRadians/2);
  const angularDistance = 2 * Math.atan2(Math.sqrt(cordLength), Math.sqrt(1 - cordLength));

  return radius * angularDistance;
}

function degreesToRadians(degrees) {
  return degrees * (Math.PI/180);
}

function displayInformation(data, route) {
  console.log('Christofides Algorithm');
  route.path.forEach((city) => console.log(data[city].city));
  console.log(route.distance / 1000 + ' km');
}

function christofides(data) {
  let modifiedData = data.map((city) => {
    return [city.lat, city.lon]
  });

  let graph = buildGraph(modifiedData);
  // console.log(data);
  // console.log(data.length);

  let mstree = minimumSpanningTree(graph);
  // console.log('minimum spanning tree');
  // mstree.printGraph();

  let odd_vertexes = findOddVertexes(mstree);
  // console.log('odd_vertexes', odd_vertexes);

  let perfect_matching_edges = findPerfectMatching(graph, odd_vertexes, mstree);
  // console.log('perfect matching edges graph');
  // perfect_matching_edges.printGraph();

  // Correct up to here
  let eulerian_tour = find_eulerian_tour(perfect_matching_edges);
  // console.log('eulerian_tour');
  // console.log(eulerian_tour);

  let current = 0;
  let path = [current];
  let visited = Array(perfect_matching_edges.nodes).fill(false);

  eulerian_tour = eulerian_tour.reduce((acc, val) => {
    return acc.concat(val);
  }, []);

  visited[current] = true;

  let distance = 0;

  eulerian_tour.slice(1).forEach((node, index) => {
    if (!visited[node]) {
      path.push(node);
      visited[node] = true;

      distance += graph.AdjList.get(current).find(n => n.node === node).weight;
      current = node;
    }
  })

  distance += graph.AdjList.get(path[path.length - 1]).find(n => n.node === path[0]).weight;
  path.push(path[0]);
  displayInformation(data, {path, distance})
}

function buildGraph (data) {
  let graph = new Graph();
  for (let i = 0; i < data.length; i++) {
    graph.addNode(i);
    for (let j = i + 1; j < data.length; j++) {
      let weight = distanceBetweenPoints(data[i][0], data[i][1], data[j][0], data[j][1]);
      graph.addEdge(i, j, weight);
    }
  }
  return graph;
}

function minimumSpanningTree (graph) {
  const MST = new Graph();
  graph.nodes.forEach(node => MST.addNode(node));

  let edgeQueue = new PriorityQueue();

  graph.AdjList.forEach((node, index) => {
    node.forEach((edge) => {
      edgeQueue.enqueue([index, edge.node], edge.weight);
    })
  })

  let uf = new UnionFind(graph.nodes);

  while (!edgeQueue.isEmpty()) {
    let nextEdge = edgeQueue.dequeue();
    let nodes = nextEdge.element;
    let weight = nextEdge.priority;

    if (!uf.connected(nodes[0], nodes[1])) {
      MST.addEdge(nodes[0], nodes[1], weight);
      uf.union(nodes[0], nodes[1]);
    }
  }
  return MST;
}

function findOddVertexes (mstree) {
  let oddIndexes = []
  mstree.AdjList.forEach((node, index) => {
    if(node.length % 2 === 1) {
      oddIndexes.push(index);
    }
  })
  return oddIndexes;
}

function findPerfectMatching (graph, odd_vertexes, mstree) {
  let munkresArray = [];
  odd_vertexes.forEach((vertex) => {
    let arr = []
    odd_vertexes.forEach((ver) => {
      if (ver === vertex) {
        arr.push(Number.MAX_SAFE_INTEGER);
      } else {
        arr.push(graph.AdjList.get(vertex).find((node) => node.node === ver).weight)
      }
    })
    munkresArray.push(arr);
  })
  let result = munkres(munkresArray);

  // Remove duplicate array from result
  let hash = {};
  let filteredResult = [];
  for (var i = 0, l = result.length; i < l; i++) {
    var key = result[i].sort().join('|');
    if (!hash[key]) {
      filteredResult.push(result[i]);
      hash[key] = 'found';
    }
  }

  filteredResult.forEach((pair) => {
    let n1 = odd_vertexes[pair[0]]
    let n2 = odd_vertexes[pair[1]]
    mstree.addEdge(n1, n2, munkresArray[pair[0]][pair[1]]);
  })
  return mstree;
}

function find_eulerian_tour (matched_tree) {
  let graph = {};
  matched_tree.getEdges().forEach((edges, index) => {
    graph[index] = edges.reduce((acc, edge) => {
      return acc.concat(edge.node)
    }, [])
  });

  let vertexTracker = 0;
  let bridge = false;
  let cycle = [];
  while (convert_graph(graph).length > 0) {
    let current_vertex = vertexTracker;
    // console.log(graph);
    // console.log(current_vertex);
    for (let vertex of graph[current_vertex]) {
      vertexTracker = vertex;
      // console.log(vertexTracker);
      graph[current_vertex].splice(graph[current_vertex].indexOf(vertexTracker), 1);
      graph[vertexTracker].splice(graph[vertexTracker].indexOf(current_vertex), 1);

      bridge = !is_connected(graph);
      if (bridge) {
        graph[current_vertex].push(vertexTracker);
        graph[vertexTracker].push(current_vertex);
      } else {
        break;
      }
    }
    if (bridge) {
      graph[current_vertex].splice(graph[current_vertex].indexOf(vertexTracker), 1);
      graph[vertexTracker].splice(graph[vertexTracker].indexOf(current_vertex), 1);
      delete graph[current_vertex];
    }
    cycle.push([current_vertex, vertexTracker]);
  }
  return cycle;
}

function is_connected(graph) {
  let start_node = Object.keys(graph)[0]
  let index = 0;
  let tracker = {};

  for (let i of Object.keys(graph)) {
    tracker[i] = 1;
  }

  tracker[start_node] = 0;
  let arr = [start_node];
  while(arr.length !== 0) {
    let node = arr.pop();
    for (let i of graph[node]) {
      if (tracker[i] === 1) {
        tracker[i] = 0;
        arr.push(i);
      }
      tracker[node] = 2;
    }
  }
  return Object.values(tracker).filter(x => x === 2).length === Object.keys(graph).length;
}

function convert_graph(tree) {
  let graph = []
  Object.values(tree).forEach((neighbours, index) => {
    neighbours.forEach((neighbour) => {
      graph.push([index, neighbour])
    })
  })
  return graph;
}

function print_tour (matched_tree, vertex) {
  let neighbours = matched_tree.getEdges()[vertex];
  neighbours.forEach((neighbour) => {
    if (is_valid_edge(matched_tree, vertex, neighbour.node)) {
      console.log(vertex + '-' + neighbour.node);
      matched_tree.removeEdge(vertex, neighbour.node);
      print_tour(matched_tree, neighbour.node);
    }
  })
}

function is_valid_edge (matched_tree, u, v) {
  let neighbours = matched_tree.getEdges()[u];
  // If v is the only adjacent vertex of u
  if (neighbours.length === 1) return true;
  // Check if u-v is a bridge
  let visited = Array(matched_tree.nodes.length).fill(false);
  let count1 = dfs_count(matched_tree, u, visited);

  // console.log('count1', count1);

  let weight = matched_tree.AdjList.get(u).find((node) => node.node = v).weight;
  matched_tree.removeEdge(u, v);

  visited = Array(matched_tree.nodes.length).fill(false);
  let count2 = dfs_count(matched_tree, u, visited);

  // console.log('count2', count2);

  matched_tree.addEdge(u, v, weight);

  return count1 <= count2;
}

function dfs_count (matched_tree, v, visited) {
  let count = 1;
  visited[v] = true;

  let neighbours = matched_tree.getEdges()[v];
  neighbours.forEach((neighbour) => {
    if (!visited[neighbour.node]) {
      count += dfs_count(matched_tree, neighbour.node, visited);
    }
  })
  return count;
}

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

class UnionFind {
 constructor(elements) {
    // Number of disconnected components
    this.count = elements.length;

    // Keep Track of connected components
    this.parent = {};

    // Initialize the data structure such that all
    // elements have themselves as parents
    elements.forEach(e => (this.parent[e] = e));
 }

 union(a, b) {
    let rootA = this.find(a);
    let rootB = this.find(b);

    // Roots are same so these are already connected.
    if (rootA === rootB) return;

    // Always make the element with smaller root the parent.
    if (rootA < rootB) {
       if (this.parent[b] != b) this.union(this.parent[b], a);
       this.parent[b] = this.parent[a];
    } else {
       if (this.parent[a] != a) this.union(this.parent[a], b);
       this.parent[a] = this.parent[b];
    }
 }

 // Returns final parent of a node
 find(a) {
    while (this.parent[a] !== a) {
       a = this.parent[a];
    }
    return a;
 }

 // Checks connectivity of the 2 nodes
 connected(a, b) {
    return this.find(a) === this.find(b);
 }
}

// create a graph class
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


(function() {
  readFile().then((data) => {
    let cities = data.filter(city => Object.keys(city).length !== 0);
    christofides(cities);

  }).catch((err) => {
    console.log(err);
  })
})();
