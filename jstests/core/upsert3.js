// tests to make sure no dup fields are created when using query to do upsert
var res;
t = db.upsert1;
t.drop();

// make sure the new _id is not duplicated
res = t.update( {"a.b": 1, a: {a: 1, b: 1}} , {$inc: {y: 1}} , true );
if (!res.hasWriteErrors()) {
    assert(false, "a.b-1 - " + res.toString() + " doc:" + tojson(t.findOne()));
}

res = t.update( {"_id.a": 1, _id: {a: 1, b: 1}} , {$inc : {y: 1}} , true );
if (!res.hasWriteErrors()) {
    assert(false, "_id-1 - " + res.toString() + " doc:" + tojson(t.findOne()));
}

res = t.update( {_id: {a: 1, b: 1}, "_id.a": 1} , { $inc: {y: 1}} , true );
if (!res.hasWriteErrors()) {
    assert(false, "_id-2 - " + res.toString() + " doc:" + tojson(t.findOne()));
}

// Should be redundant, but including from SERVER-11363
res = t.update( {_id: {a: 1, b: 1}, "_id.a": 1} , {$setOnInsert: {y: 1}} , true );
if (!res.hasWriteErrors()) {
    assert(false, "_id-3 - " + res.toString() + " doc:" + tojson(t.findOne()));
}

//Should be redundant, but including from SERVER-11514
res = t.update( {"a": {}, "a.c": 2} , {$set: {x: 1}}, true );
if (!res.hasWriteErrors()) {
    assert(false, "a.c-1 - " + res.toString() + " doc:" + tojson(t.findOne()));
}

// Should be redundant, but including from SERVER-4830
res = t.update( {'a': {b: 1}, 'a.c': 1}, {$inc: {z: 1}}, true );
if (!res.hasWriteErrors()) {
    assert(false, "a-1 - " + res.toString() + " doc:" + tojson(t.findOne()));
}

// Should be redundant, but including from SERVER-4830
res = t.update( {a: 1, "a.b": 1, a: [1, {b: 1}]}, {$inc: {z: 1}}, true );
if (!res.hasWriteErrors()) {
    assert(false, "a-2 - " + res.toString() + " doc:" + tojson(t.findOne()));
}

// Replacement tests
// Query is ignored for replacements, except _id field.
res = t.update( {r: {a: 1, b: 1}, "r.a": 1} , {y: 1} , true );
assert(!res.hasWriteErrors(), "r-1");
assert(t.findOne().y, 1, "inserted doc missing field")
var docMinusId = t.findOne();
delete docMinusId._id
assert.docEq({y: 1}, docMinusId, "r-1")
t.drop()

res = t.update( {_id: {a:1, b:1}, "_id.a": 1} , {y: 1} , true );
assert(!res.hasWriteErrors(), "_id-4");
assert.docEq({_id: {a: 1, b: 1}, y: 1}, t.findOne(), "_id-4")
t.drop()