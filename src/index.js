var mathf = require("@nathanfaucett/mathf"),
    vec3 = require("@nathanfaucett/vec3"),
    vec4 = require("@nathanfaucett/vec4"),
    isNumber = require("@nathanfaucett/is_number");


var quat = exports;


quat.ArrayType = typeof(Float32Array) !== "undefined" ? Float32Array : mathf.ArrayType;


quat.create = function(x, y, z, w) {
    var out = new quat.ArrayType(4);

    out[0] = isNumber(x) ? x : 0;
    out[1] = isNumber(y) ? y : 0;
    out[2] = isNumber(z) ? z : 0;
    out[3] = isNumber(w) ? w : 1;

    return out;
};

quat.copy = vec4.copy;

quat.clone = function(a) {
    var out = new quat.ArrayType(4);

    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];

    return out;
};

quat.identity = function(out) {

    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 1.0;

    return out;
};

quat.set = vec4.set;

quat.lengthSqValues = vec4.lengthSqValues;

quat.lengthValues = vec4.lengthValues;

quat.invLengthValues = vec4.invLengthValues;

quat.dot = vec4.dot;

quat.lengthSq = vec4.lengthSq;

quat.length = vec4.length;

quat.invLength = vec4.invLength;

quat.setLength = vec4.setLength;

quat.normalize = vec4.normalize;

quat.lerp = vec4.lerp;

quat.min = vec4.min;

quat.max = vec4.max;

quat.clamp = vec4.clamp;

quat.equal = vec4.equal;

quat.notEqual = vec4.notEqual;

quat.str = function(out) {
    return "Quat(" + out[0] + ", " + out[1] + ", " + out[2] + ", " + out[3] + ")";
};

quat.toString = quat.string = quat.str;

quat.mul = function(out, a, b) {
    var ax = a[0],
        ay = a[1],
        az = a[2],
        aw = a[3],
        bx = b[0],
        by = b[1],
        bz = b[2],
        bw = b[3];

    out[0] = ax * bw + aw * bx + ay * bz - az * by;
    out[1] = ay * bw + aw * by + az * bx - ax * bz;
    out[2] = az * bw + aw * bz + ax * by - ay * bx;
    out[3] = aw * bw - ax * bx - ay * by - az * bz;

    return out;
};

quat.div = function(out, a, b) {
    var ax = a[0],
        ay = a[1],
        az = a[2],
        aw = a[3],
        bx = -b[0],
        by = -b[1],
        bz = -b[2],
        bw = b[3];

    out[0] = ax * bw + aw * bx + ay * bz - az * by;
    out[1] = ay * bw + aw * by + az * bx - ax * bz;
    out[2] = az * bw + aw * bz + ax * by - ay * bx;
    out[3] = aw * bw - ax * bx - ay * by - az * bz;

    return out;
};

quat.inverse = function(out, a) {
    var d = quat.dot(a, a);

    if (d === 0) {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        out[3] = 0.0;

        return out;
    } else {
        d = 1 / d;

        out[0] = a[0] * -d;
        out[1] = a[1] * -d;
        out[2] = a[2] * -d;
        out[3] = a[3] * d;

        return out;
    }
};

quat.conjugate = function(out, a) {

    out[0] = -a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    out[3] = a[3];

    return out;
};

quat.calculateW = function(out, a) {
    var x = a[0],
        y = a[1],
        z = a[2];

    out[0] = x;
    out[1] = y;
    out[2] = z;
    out[3] = -mathf.sqrt(mathf.abs(1 - x * x - y * y - z * z));

    return out;
};

quat.nlerp = function(out, a, b, x) {
    return quat.normalize(quat.lerp(out, a, b, x));
};

quat.slerp = function(out, a, b, x) {
    var ax = a[0],
        ay = a[1],
        az = a[2],
        aw = a[3],
        bx = b[0],
        by = b[1],
        bz = b[2],
        bw = b[3],

        cosom = ax * bx + ay * by + az * bz + aw * bw,
        omega, sinom, scale0, scale1;

    if (cosom < 0.0) {
        cosom *= -1;
        bx *= -1;
        by *= -1;
        bz *= -1;
        bw *= -1;
    }

    if (1 - cosom > mathf.EPSILON) {
        omega = mathf.acos(cosom);

        sinom = mathf.sin(omega);
        sinom = sinom !== 0 ? 1 / sinom : sinom;

        scale0 = mathf.sin((1 - x) * omega) * sinom;
        scale1 = mathf.sin(x * omega) * sinom;
    } else {
        scale0 = 1 - x;
        scale1 = x;
    }

    out[0] = scale0 * ax + scale1 * bx;
    out[1] = scale0 * ay + scale1 * by;
    out[2] = scale0 * az + scale1 * bz;
    out[3] = scale0 * aw + scale1 * bw;

    return out;
};

quat.rotationX = function(out) {
    var z = out[2],
        w = out[3];

    return mathf.atan2(2 * out[0] * w + 2 * out[1] * z, 1 - 2 * (z * z + w * w));
};

quat.rotationY = function(out) {
    var theta = 2 * (out[0] * out[2] + out[3] * out[1]);

    return mathf.asin((theta < -1 ? -1 : theta > 1 ? 1 : theta));
};

quat.rotationZ = function(out) {
    var y = out[1],
        z = out[2];

    return mathf.atan2(2 * out[0] * y + 2 * z * out[3], 1 - 2 * (y * y + z * z));
};

quat.rotateX = function(out, a, angle) {
    var ax = a[0],
        ay = a[1],
        az = a[2],
        aw = a[3],
        halfAngle = angle * 0.5,
        bx = mathf.sin(halfAngle),
        bw = mathf.cos(halfAngle);

    out[0] = ax * bw + aw * bx;
    out[1] = ay * bw + az * bx;
    out[2] = az * bw - ay * bx;
    out[3] = aw * bw - ax * bx;

    return out;
};

quat.rotateY = function(out, a, angle) {
    var ax = a[0],
        ay = a[1],
        az = a[2],
        aw = a[3],
        halfAngle = angle * 0.5,
        by = Math.sin(halfAngle),
        bw = Math.cos(halfAngle);

    out[0] = ax * bw - az * by;
    out[1] = ay * bw + aw * by;
    out[2] = az * bw + ax * by;
    out[3] = aw * bw - ay * by;

    return out;
};

quat.rotateZ = function(out, a, angle) {
    var ax = a[0],
        ay = a[1],
        az = a[2],
        aw = a[3],
        halfAngle = angle * 0.5,
        bz = Math.sin(halfAngle),
        bw = Math.cos(halfAngle);

    out[0] = ax * bw + ay * bz;
    out[1] = ay * bw - ax * bz;
    out[2] = az * bw + aw * bz;
    out[3] = aw * bw - az * bz;

    return out;
};

quat.rotate = function(out, a, x, y, z) {

    if (isNumber(z)) {
        quat.rotateZ(out, a, z);
    }
    if (isNumber(x)) {
        quat.rotateX(out, a, x);
    }
    if (isNumber(y)) {
        quat.rotateY(out, a, y);
    }

    return out;
};

var lookRotation_up = vec3.create(0.0, 0.0, 1.0);
quat.lookRotation = function(out, forward, up) {
    var fx, fy, fz, ux, uy, uz, ax, ay, az, d, dsq, s;

    up = up || lookRotation_up;

    fx = forward[0];
    fy = forward[1];
    fz = forward[2];
    ux = up[0];
    uy = up[1];
    uz = up[2];

    ax = uy * fz - uz * fy;
    ay = uz * fx - ux * fz;
    az = ux * fy - uy * fx;

    d = (1.0 + ux * fx + uy * fy + uz * fz) * 2.0;
    dsq = d * d;
    s = dsq !== 0 ? 1.0 / dsq : dsq;

    out[0] = ax * s;
    out[1] = ay * s;
    out[2] = az * s;
    out[3] = dsq * 0.5;

    return out;
};

var rotationTo_tmp = vec3.create(),
    rotationTo_x = vec3.create(1.0, 0.0, 0.0),
    rotationTo_y = vec3.create(0.0, 1.0, 0.0);
quat.rotationTo = function(out, a, b) {
    var dot = vec3.dot(a, b),
        unitx = rotationTo_x,
        unity = rotationTo_y,
        tmp = rotationTo_tmp;

    if (dot < -0.999999) {
        vec3.cross(tmp, unitx, a);

        if (vec3.length(tmp) < 0.000001) {
            vec3.cross(tmp, unity, a);
        }

        vec3.normalize(tmp, tmp);
        quat.fromAxisAngle(out, tmp, Math.PI);

        return out;
    } else if (dot > 0.999999) {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        out[3] = 1.0;

        return out;
    } else {
        vec3.cross(tmp, a, b);

        out[0] = tmp[0];
        out[1] = tmp[1];
        out[2] = tmp[2];
        out[3] = 1.0 + dot;

        return quat.normalize(out, out);
    }
};

quat.getAxisAngle = function(out, q) {
    var angle = Math.acos(q[3]) * 2.0;
    s = Math.sin(angle / 2.0);

    if (s !== 0.0) {
        out[0] = q[0] / s;
        out[1] = q[1] / s;
        out[2] = q[2] / s;
    } else {
        out[0] = 0;
        out[1] = 0;
        out[2] = 1;
    }

    return angle;
};

quat.fromAxisAngle = function(out, axis, angle) {
    var halfAngle = angle * 0.5,
        s = mathf.sin(halfAngle);

    out[0] = axis[0] * s;
    out[1] = axis[1] * s;
    out[2] = axis[2] * s;
    out[3] = mathf.cos(halfAngle);

    return out;
};

quat.fromMat = function(
    out,
    m11, m12, m13,
    m21, m22, m23,
    m31, m32, m33
) {
    var trace = m11 + m22 + m33,
        s, invS;

    if (trace > 0.0) {
        s = 0.5 / mathf.sqrt(trace + 1);

        out[3] = 0.25 / s;
        out[0] = (m32 - m23) * s;
        out[1] = (m13 - m31) * s;
        out[2] = (m21 - m12) * s;
    } else if (m11 > m22 && m11 > m33) {
        s = 2 * mathf.sqrt(1 + m11 - m22 - m33);
        invS = 1.0 / s;

        out[3] = (m32 - m23) * invS;
        out[0] = 0.25 * s;
        out[1] = (m12 + m21) * invS;
        out[2] = (m13 + m31) * invS;
    } else if (m22 > m33) {
        s = 2.0 * mathf.sqrt(1.0 + m22 - m11 - m33);
        invS = 1.0 / s;

        out[3] = (m13 - m31) * invS;
        out[0] = (m12 + m21) * invS;
        out[1] = 0.25 * s;
        out[2] = (m23 + m32) * invS;
    } else {
        s = 2 * mathf.sqrt(1 + m33 - m11 - m22);
        invS = 1.0 / s;

        out[3] = (m21 - m12) * invS;
        out[0] = (m13 + m31) * invS;
        out[1] = (m23 + m32) * invS;
        out[2] = 0.25 * s;
    }

    return out;
};

quat.fromMat2 = function(out, m) {
    return quat.fromMat(
        out,
        m[0], m[2], 0.0,
        m[1], m[3], 0.0,
        0.0, 0.0, 1.0
    );
};

quat.fromMat32 = quat.fromMat2;

quat.fromMat3 = function(out, m) {
    return quat.fromMat(
        out,
        m[0], m[3], m[6],
        m[1], m[4], m[7],
        m[2], m[5], m[8]
    );
};

quat.fromMat4 = function(out, m) {
    return quat.fromMat(
        out,
        m[0], m[4], m[8],
        m[1], m[5], m[9],
        m[2], m[6], m[10]
    );
};
