global.quat = require("../src/index");


var a = quat.rotate(quat.create(), 0, Math.PI, 0),
    b = quat.rotate(quat.create(), 0, Math.PI * 0.5, 0),
    c = quat.slerp(a, b, 0.5, quat.create());


console.log(c, quat.length(c));
