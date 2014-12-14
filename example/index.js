global.quat = require("../src/index");


var a = quat.create(),
    b = quat.create(),
    c = quat.create();

quat.rotate(a, a, 0, Math.PI, 0);
quat.rotate(b, b, 0, Math.PI * 0.5, 0);
quat.slerp(c, a, b, 0.5);

console.log(c, quat.length(c));
console.log(quat.rotationX(c));
console.log(quat.rotationY(c));
console.log(quat.rotationZ(c));
