/*
Christofides Algorithm
Step 1. Find minimum spanning tree
Step 2. Find all odd vertexes of that minimum spanning tree
Step 3. Find the minimum perfect matching of those odd vertexes
Step 4. Find an eulerian tour
Step 5. Generate the hamiltonian graph
*/

const csv = require('csv-parser');
const fs = require('fs');
const blossom = require('edmonds-blossom');
const Graph = require('./Graph');
const PriorityQueue = require('./PriorityQueue');
const UnionFind = require('./UnionFind');

// Parse CSV File into an array of objects
function readFile () {
  return new Promise((resolve, reject)=> {
    let cities = [];
    fs.createReadStream('csv/cities_all.csv')
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

// Haversine Formula
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

// Convert degrees to radians
function degreesToRadians(degrees) {
  return degrees * (Math.PI / 180);
}

// Display cities visited in order and distance travelled in km
function displayInformation(data, route) {
  console.log('Christofides Algorithm');
  route.path.forEach((city) => console.log(data[city].city));
  console.log(route.distance / 1000 + ' km');
}

// Takes in array of latitude and longitude and adds points to graph
function buildGraph (data) {
  let graph = new Graph();
  for (let i = 0; i < data.length; i++) {
    // Adds node to graph
    graph.addNode(i);
    for (let j = i + 1; j < data.length; j++) {
      // Calculates distance as weight edge
      let weight = distanceBetweenPoints(data[i][0], data[i][1], data[j][0], data[j][1]);
      // Adds edge to graph
      graph.addEdge(i, j, weight);
    }
  }
  return graph;
}

// Takes in a graph and finds minimum spanning tree using Kruskal's algorithm
// Runs in O(logV)
function minimumSpanningTree (graph) {
  // Minimum spanning tree with be a new graph
  const MST = new Graph();
  // Add nodes to graph
  graph.nodes.forEach(node => MST.addNode(node));

  // Creates a new Priority Queue
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

// Find all odd degree vertexes in the minimum spanning stree
function findOddVertexes (mstree) {
  let oddIndexes = []
  // Cycles through minimum spanning tree
  mstree.AdjList.forEach((node, index) => {
    // If there are an odd amount of neighbours add it to the array
    if(node.length % 2 === 1) {
      oddIndexes.push(index);
    }
  })
  // Returns array of odd vertexes
  return oddIndexes;
}

function findPerfectMatching (graph, odd_vertexes, mstree) {
  let blossomArray = [];
  odd_vertexes.forEach((vertex) => {
    odd_vertexes.forEach((ver) => {
      if (vertex !== ver) {
        let arr = []
        weight = Number.MAX_SAFE_INTEGER - graph.AdjList.get(vertex).find((node) => node.node === ver).weight;
        arr = [vertex, ver, weight];
        blossomArray.push(arr);
      }
    })
  })

  let result = blossom(blossomArray);
  let formattedResult = [];
  result.forEach((num, index) => {
    if (num !== -1) formattedResult.push([index, num]);
  })

  // Remove duplicate array from result
  let hash = {};
  let filteredResult = [];
  for (var i = 0, l = formattedResult.length; i < l; i++) {
    var key = formattedResult[i].sort().join('|');
    if (!hash[key]) {
      filteredResult.push(formattedResult[i]);
      hash[key] = 'found';
    }
  }

  filteredResult.forEach((pair) => {
    mstree.addEdge(pair[0], pair[1], blossomArray[pair[0]][pair[1]]);
  })

  return mstree;
}

function findEulerianTour (matched_tree) {
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
    for (let vertex of graph[current_vertex]) {
      vertexTracker = vertex;
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

function is_valid_edge (matched_tree, u, v) {
  let neighbours = matched_tree.getEdges()[u];
  // If v is the only adjacent vertex of u
  if (neighbours.length === 1) return true;
  // Check if u-v is a bridge
  let visited = Array(matched_tree.nodes.length).fill(false);
  let count1 = dfs_count(matched_tree, u, visited);

  let weight = matched_tree.AdjList.get(u).find((node) => node.node = v).weight;
  matched_tree.removeEdge(u, v);

  visited = Array(matched_tree.nodes.length).fill(false);
  let count2 = dfs_count(matched_tree, u, visited);

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

function generateHamiltonianPath (nodes, list, data, tour) {
  // Current index
  let previous = 0;
  // Current path will start at index 0
  let path = [previous];
  // Create array of size nodes filled with false
  let visited = Array(nodes).fill(false);

  // Already visited the first index
  visited[previous] = true;

  // Reduce array of edges to array of nodes
  tour = tour.reduce((acc, val) => {
    return acc.concat(val);
  }, []);

  // Distance traveled
  let distance = 0;

  // Starting from index 1
  tour.slice(1).forEach((node, index) => {
    // If the path hasn't visited the node
    if (!visited[node]) {
      // Add node to the path
      path.push(node);
      // Set visited to true
      visited[node] = true;

      // Add distance using previous node and current
      distance += list.get(previous).find(n => n.node === node).weight;
      // Set current node
      previous = node;
    }
  })

  // Add distance to starting node
  distance += list.get(path[path.length - 1]).find(n => n.node === path[0]).weight;
  // Circle path back to the beginning
  path.push(path[0]);
  // Display information
  displayInformation(data, {path, distance});
}

// Main function to run Christofides Algorithm
function run(data) {
  // Change to true if you want to view console log
  const print = false;

  if (print) console.log('\nData', data);

  // Create new array of each cities latitude and longitude coordinates
  let modifiedData = data.map((city) => {
    return [city.lat, city.lon]
  });

  if (print) console.log('\nModified Data', modifiedData);

  // Generate graph
  let graph = buildGraph(modifiedData);

  if (print) {
    console.log('\nGraph');
    graph.printGraph();
  }

  // Generate minimum spannning tree
  let mstree = minimumSpanningTree(graph);

  if (print) {
    console.log('\nMinimum Spanning Tree');
    mstree.printGraph();
  }

  // Find all odd degree nodes in minimum spanning tree
  let odd_vertexes = findOddVertexes(mstree);

  if (print) console.log('\nOdd Vertexes', odd_vertexes);

  // Find perfect matching edges of odd vertexes and return graph of
  // minimum spanning tree plus those edges
  let perfect_matching_edges = findPerfectMatching(graph, odd_vertexes, mstree);

  if (print) {
    console.log('Perfect Matching Edges Graph');
    perfect_matching_edges.printGraph();
  }

  // Generage Eulerian Tour of graph
  let eulerian_tour = findEulerianTour(perfect_matching_edges);
  if (print) console.log('\nEulerian Tour', eulerian_tour);

  // Generate hamiltonian path
  generateHamiltonianPath(perfect_matching_edges.nodes, graph.AdjList, data, eulerian_tour);
}

(function() {
  readFile().then((data) => {
    // Filter empty objects
    let cities = data.filter(city => Object.keys(city).length !== 0);
    run(cities);
  }).catch((err) => {
    console.log(err);
  })
})();
