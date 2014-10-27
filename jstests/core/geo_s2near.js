// Test 2dsphere near search, called via find and geoNear.
t = db.geo
t.drop();

// Make sure that geoNear gives us back loc
goldenPoint = {type: "Point", coordinates: [ 31.0, 41.0]}
t.insert({geo: goldenPoint})
t.ensureIndex({ geo : "2dsphere" })
// printjson(t.find({geo: {$geoNear: {$geometry: { type: "Point", coordinates: [31.01, 41.01]}}}}).explain(true));
var polyLoop = [
[30, 40],
[32, 40],
[32, 42],
[30, 42],
[30, 40]
]
printjson(t.find({geo: {$geoWithin: {$geometry: { type: "Polygon", coordinates: [polyLoop]}}}}).explain(true));



// {"geo": {"$nearSphere": {"$geometry": { "type": "Point", "coordinates": [31.01, 41.01]}, "$maxDistance": 500000}}}

// // MongoDB
// {"geo": {"$nearSphere": {"$geometry": { "type": "Point", "coordinates": [-73.988152, 40.757563]}, "$maxDistance": 500000}}}

// 40.757563, -73.988152