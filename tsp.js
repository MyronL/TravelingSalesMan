// UrbanLogiq Coding Challenge

/*
Initial thoughts:
1. Traveling salesman problem
2. Brute force O(n!) definitely too slow
3. Branch and bound algorithm (probably the best)
4. Nearest neighbours algorithm (greedy, not as accurate)
*/

const csv = require('csv-parser');
const fs = require('fs');

// Parse CSV File into an array of objects
function readFile () {
  return new Promise((resolve, reject)=> {
    let cities = [];
    fs.createReadStream('cities_all.csv')
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

// Try every combination and see which one is the best.
// Not optimal as it didn't save any distances in memory to reduce
// need to run distanceBetweenPoints multiple times.
function bruteForce (currentCity, cities, path, distance, bestRoute) {
  // Add new city to path
  path.push(currentCity);

  // Add new distance
  if (path.length > 1) {
    let lastCity = path[path.length - 2];
    distance += distanceBetweenPoints(lastCity.lat, lastCity.lon, currentCity.lat, currentCity.lon);
  }

  // If path contains all cities
  if (cities.length === path.length) {
    path.push(path[0]);
    let lastCity = path[path.length - 1];
    distance += distanceBetweenPoints(lastCity.lat, lastCity.lon, currentCity.lat, currentCity.lon);
    if (!bestRoute.path.length || distance < bestRoute.distance) {
      bestRoute.path = path;
      bestRoute.distance = distance;
    }
    return bestRoute;
  }

  cities.forEach((city) => {
    if (!path.find((node) => node.city === city.city)) {
      return bruteForce(city, cities, [...path], distance, bestRoute);
    }
  })

  return bestRoute;
}

// Nearest Neighbor Algorithm
function nearestNeighbourAlgorithm (cities) {
  const startingCity = cities[0];
  const numCities = cities.length;
  let route = [];
  let totalDistance = 0;

  route.push(startingCity);
  cities.shift();

  while (route.length < numCities) {
    let lastCity = route[route.length - 1];
    let closestCity;
    let closestDistance = 0;
    let closestIndex = 0;

    cities.forEach((city, index) => {
      let distance = distanceBetweenPoints(city.lat, city.lon, route[route.length - 1].lat, route[route.length - 1].lon);
      if (!closestCity || closestDistance > distance) {
        closestCity = city;
        closestDistance = distance;
        closestIndex = index;
      }
    });
    route.push(closestCity);
    totalDistance += closestDistance;
    cities.splice(closestIndex, 1);
  }

  totalDistance += distanceBetweenPoints(route[0].lat, route[0].lon, route[route.length - 1].lat, route[route.length - 1].lon)
  route.push(route[0]);

  return {path: route, distance: totalDistance};
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

function displayInformation(route) {
  route.path.forEach((city) => console.log(city.city));
  console.log(route.distance / 1000 + ' km');
}

(function() {
  readFile().then((data) => {
    const cities = data.filter(city => Object.keys(city).length !== 0);

    // let bestRouteBruteForce = bruteForce(cities[0], cities, [], 0, {path: [], distance: 0});
    // console.log('\nBrute Force');
    // displayInformation(bestRouteBruteForce);

    let bestRoute = nearestNeighbourAlgorithm(cities);
    console.log('\nNearest Neighbours')
    displayInformation(bestRoute);


  }).catch((err) => {
    console.log(err);
  })
})();
