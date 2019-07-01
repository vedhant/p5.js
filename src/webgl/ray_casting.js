'use strict';

var p5 = require('../core/main');

p5.RayCaster = function(_pInst) {
  this._pInst = _pInst;
  this.ray = {
    origin: new p5.Vector(),
    direction: new p5.Vector()
  };
};

p5.prototype.createRayCaster = function() {
  var rayCaster = new p5.RayCaster(this);
  return rayCaster;
};

p5.RayCaster.prototype.intersects = function() {
  // ray origin in View space is (0, 0, 0)
  this.ray.origin.x = 0;
  this.ray.origin.y = 0;
  this.ray.origin.z = 0;

  // Normalised Device Coordinates
  this.ray.direction.x = 2 * this._pInst.mouseX / this._pInst.canvas.width - 1;
  this.ray.direction.y = 1 - 2 * this._pInst.mouseY / this._pInst.canvas.height;
  this.ray.direction.z = 1;
  // homogeneous clip coordinates
  this.ray.direction.z = -1;
  // Eye (camera) coordinates
  var uPMatrixInverse = new p5.Matrix();
  uPMatrixInverse.invert(this._pInst._renderer.uPMatrix);
  uPMatrixInverse.transpose(uPMatrixInverse);
  uPMatrixInverse = uPMatrixInverse.mat4;
  var rayDirectionCopy = this.ray.direction.copy();
  this.ray.direction.x =
    uPMatrixInverse[0] * rayDirectionCopy.x +
    uPMatrixInverse[1] * rayDirectionCopy.y +
    uPMatrixInverse[2] * rayDirectionCopy.z +
    uPMatrixInverse[3];
  this.ray.direction.y =
    uPMatrixInverse[4] * rayDirectionCopy.x +
    uPMatrixInverse[5] * rayDirectionCopy.y +
    uPMatrixInverse[6] * rayDirectionCopy.z +
    uPMatrixInverse[7];

  this.ray.direction.normalize();

  for (var gId in this._pInst._renderer.gHash) {
    if (this.intersectsObject(this._pInst._renderer.gHash[gId].model)) {
      return true;
    }
  }
  return false;
};

p5.RayCaster.prototype.intersectsObject = function(object) {
  var intersects = false;
  for (var i = 0; i < object.faces.length; ++i) {
    // vertices in local space
    var vertices = [
      object.vertices[object.faces[i][0]].copy(),
      object.vertices[object.faces[i][1]].copy(),
      object.vertices[object.faces[i][2]].copy()
    ];
    // converting face vertices to view space
    var uMVMatrix = object.uMVMatrix.copy();
    uMVMatrix = uMVMatrix.transpose(uMVMatrix).mat4;
    for (var j = 0; j < 3; ++j) {
      var v = vertices[j].copy();
      vertices[j].x =
        uMVMatrix[0] * v.x +
        uMVMatrix[1] * v.y +
        uMVMatrix[2] * v.z +
        uMVMatrix[3];
      vertices[j].y =
        uMVMatrix[4] * v.x +
        uMVMatrix[5] * v.y +
        uMVMatrix[6] * v.z +
        uMVMatrix[7];
      vertices[j].z =
        uMVMatrix[8] * v.x +
        uMVMatrix[9] * v.y +
        uMVMatrix[10] * v.z +
        uMVMatrix[11];
    }
    var v1_v0 = new p5.Vector();
    var v2_v0 = new p5.Vector();
    p5.Vector.sub(vertices[1], vertices[0], v1_v0);
    p5.Vector.sub(vertices[2], vertices[0], v2_v0);
    // face normal in view space
    var faceNormal = p5.Vector.cross(v1_v0, v2_v0);
    faceNormal.normalize();

    var intersection = this.rayPlaneIntersectionPoint(vertices[0], faceNormal);
    // check if intersection point lies on the ray
    if (intersection !== null) {
      var intersectionOnRay =
        p5.Vector.dot(
          p5.Vector.sub(intersection, this.ray.origin),
          this.ray.direction
        ) > 0;
      if (
        intersectionOnRay &&
        this.pointInsideTriangle(intersection, vertices)
      ) {
        intersects = true;
      }
    }
  }
  return intersects;
};

p5.RayCaster.prototype.rayPlaneIntersectionPoint = function(
  planePoint,
  planeNormal
) {
  var w = p5.Vector.sub(planePoint, this.ray.origin);
  var d = p5.Vector.dot(this.ray.direction, planeNormal);
  if (d === 0) {
    return null;
  }
  var k = p5.Vector.dot(w, planeNormal) / d;
  return p5.Vector.add(this.ray.origin, this.ray.direction.copy().setMag(k));
};

p5.RayCaster.prototype.pointInsideTriangle = function(point, triangle) {
  // find barycentric coordinates
  var projABonCB = p5.Vector.sub(triangle[1], triangle[2]).normalize();
  projABonCB.setMag(
    p5.Vector.dot(p5.Vector.sub(triangle[1], triangle[0]), projABonCB)
  );
  var v = p5.Vector.sub(p5.Vector.sub(triangle[1], triangle[0]), projABonCB);

  var a =
    p5.Vector.dot(v, p5.Vector.sub(point, triangle[0])) /
    p5.Vector.dot(v, p5.Vector.sub(triangle[1], triangle[0]));
  a = 1 - a;

  var projBConAC = p5.Vector.sub(triangle[2], triangle[0]).normalize();
  projBConAC.setMag(
    p5.Vector.dot(p5.Vector.sub(triangle[2], triangle[1]), projBConAC)
  );
  var w = p5.Vector.sub(p5.Vector.sub(triangle[2], triangle[1]), projBConAC);

  var b =
    p5.Vector.dot(w, p5.Vector.sub(point, triangle[1])) /
    p5.Vector.dot(w, p5.Vector.sub(triangle[2], triangle[1]));
  b = 1 - b;
  var c = 1 - a - b;
  if (a < 0 || b < 0 || c < 0 || isNaN(a) || isNaN(b) || isNaN(c)) {
    return false;
  }
  return true;
};
