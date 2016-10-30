/*
typedef struct {
	int jointid;  // the joint the axis is attached to, from 0
	Vec3 axis;
	Vec2 limits;  // joint limits (radian) [lo, hi]
} CCD_axis;
*/

// assumption a function
// function fk (theta, joints)
// that does forward kinematics
//

var CCD_axis = function(axis, id) 
{
	this.jointid = id;
	this.axis = axis.clone();
	this.limits = new THREE.Vector2(-1e4,1e4); // default: no limits
}

function CLAMP(value, lo, hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

function proj2plane (p, n)
// p: the vector to be projected
// n: the normal defining the projection plane (unit vector)
{
	var nclone = n.clone();
	var pclone = p.clone();

	nclone.multiplyScalar (p.dot(n));   // (p.n)n;
	pclone.sub (nclone);
	return pclone;
}


function ik_ccd (target, theta)
{
	var end = new THREE.Vector3();
	var base = new THREE.Vector3();
	
	var njoints = axes[axes.length-1].jointid + 1;
	var joints=[];
	for (var i = 0; i <= njoints; i++) joints[i]=new THREE.Vector3();
	
	fk (theta, joints);
	end.copy (joints[njoints]);
	
	// convergence
	var eps = 1e-3, MAXITER = 10;
	
	var t_target = new THREE.Vector3();
	var t_end = new THREE.Vector3();
	var tmpV = new THREE.Vector3();

	// iteration
	
	for (var iter = 0; iter < MAXITER; iter++) {
		for (var i = axes.length-1; i >= 0; i--) {
			base.copy(joints[axes[i].jointid]);
			
			// this part is quite different from the C counterpart
			var axis = axes[i].axis.clone();
			for (var j = i-1; j >= 0; j--) 
				axis.applyMatrix4 (new THREE.Matrix4().makeRotationAxis(axes[j].axis, theta[j])); 
				
			tmpV.subVectors (target, base);	tmpV = proj2plane (tmpV, axis);
			t_target.copy(tmpV.normalize());
			
			tmpV.subVectors (end, base); tmpV = proj2plane (tmpV, axis);
			t_end.copy(tmpV.normalize());

			var dotV = t_end.dot(t_target);
			var angle = Math.acos (CLAMP(dotV, -1,1));
			tmpV.crossVectors (t_end, t_target);
			var sign =  tmpV.dot(axis) > 0 ? 1: -1;
			theta[i] += sign * angle;
			
			// joint limit [-2.4, -0.1]
			theta[i] = CLAMP (theta[i], axes[i].limits.x, axes[i].limits.y)
      
			fk (theta, joints);
			end.copy(joints[joints.length-1]);  // was a bug ... by setting joints[2]

			if (end.distanceTo (target) < eps) {
				return 1;
			}
		}
	}
	
	if (iter < MAXITER)
		return 1;
	else {
		console.log ("do not converge");
		return 0;
	}
}
