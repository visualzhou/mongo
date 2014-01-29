// Test that we can update DBRefs, but not dbref fields outside a DBRef

var res;
t = db.jstests_update_dbref;
t.drop();

res = t.save({_id:1, a: new DBRef("a", "b")});
assert(!res.hasWriteErrors(), "failed to save dbref");
assert.docEq({_id:1, a: new DBRef("a", "b")}, t.findOne());

res = t.update({}, {$set: {"a.$id": 2}});
assert(!res.hasWriteErrors(), "a.$id update");
assert.docEq({_id:1, a: new DBRef("a", 2)}, t.findOne());

res = t.update({}, {$set: {"a.$ref": "b"}});
assert(!res.hasWriteErrors(), "a.$ref update");

assert.docEq({_id:1, a: new DBRef("b", 2)}, t.findOne());

// Bad updates
res = t.update({}, {$set: {"$id": 3}});
assert(res.hasWriteErrors() && /\$id/.test(res.getWriteError()), "expected bad update because of $id");
assert.docEq({_id:1, a: new DBRef("b", 2)}, t.findOne());

res = t.update({}, {$set: {"$ref": "foo"}});
assert(res.hasWriteErrors() && /\$ref/.test(res.getWriteError()), "expected bad update because of $ref");
assert.docEq({_id:1, a: new DBRef("b", 2)}, t.findOne());

res = t.update({}, {$set: {"$db": "aDB"}});
assert(res.hasWriteErrors() && /\$db/.test(res.getWriteError()), "expected bad update because of $db");
assert.docEq({_id:1, a: new DBRef("b", 2)}, t.findOne());

res = t.update({}, {$set: {"b.$id": 2}});
if (!res.hasWriteErrors()) {
    assert(false, "b.$id update should fail -- doc:" + tojson(t.findOne()) + " result:" + res.toString());
}

res = t.update({}, {$set: {"b.$ref": 2}});
if (!res.hasWriteErrors()) {
    assert(false, "b.$ref update should fail -- doc:" + tojson(t.findOne()) + " result:" + res.toString());
}
