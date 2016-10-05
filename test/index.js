var tape = require("tape"),
    quat = require("..");


var x = quat.normalize(quat.create(), quat.create(1, 1, 1, 1)),
    y = quat.normalize(quat.create(), quat.create(0.5, 0.5, 0.5, 0.5));


tape("quat.mul(out, a, b)", function(assert) {
    assert.deepEquals(quat.mul(quat.create(), x, y), quat.create(0.5, 0.5, 0.5, -0.5));
    assert.end();
});

tape("quat.div(out, a, b)", function(assert) {
    assert.deepEquals(quat.div(quat.create(), x, y), quat.create(0, 0, 0, 1));
    assert.end();
});
