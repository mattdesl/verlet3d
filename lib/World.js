var Class = require('klasse');
var Vector3 = require('vecmath').Vector3;
var Vector2 = require('vecmath').Vector2;

var PointMass = require('./PointMass');
var Constraint = require('./Constraint');

var distSq = require('minimath').distanceSq;

var World = new Class({

	initialize: function(gravity) {
		this.gravity = gravity || new Vector3(0, 1200, 0);
		this.friction = 0.99;
		this.bounds = null;
		this.accuracy = 2;
		this.points = [];
		
		this.move = new PointMass(0,0,0);
		this.tear = new PointMass(0,0,0);
		this.tearInfluenceSq = 0;
		this.moveInfluenceSq = 0;

		this.tear3D = false;
		
		this._pinRemoveInfluenceSq = 0;
		this.moveScalar = 1.8; 
		this.maxMoveSq = 0.0;
		
		this.obstacles = [];
		
		this.groundFriction = 1.0;
		this.floor = null;
		this.bounds = null;
	},

	setPinRemoveInfluence: function(influence) {
		this._pinRemoveInfluenceSq = influence * influence;
	},
	
	/** Adds a single point mass to the world. */
	addPoint: function(pointMass) {
		this.points.push(pointMass);
	},
	
	/** Adds an array of point masses. */
	addPoints: function(pointsArray) {
		for (var i=0; i<pointsArray.length; i++) {
			this.points.push(pointsArray[i]);
		}
	},
	
	applyMotion: function(x, y, z, influence, scalar, maxDist) {
		this.moveInfluenceSq = influence * influence;
		this.moveScalar = scalar || 1.8;

		var mov = this.move.position;
		mov.x = x;
		mov.y = y;
		mov.z = z;
		
		if (maxDist === 0) {
			this.maxMoveSq = 0;
		} else {
			maxDist = +(maxDist || 100.0);
			this.maxMoveSq = maxDist * maxDist;
		}
	},
	
	applyTear: function(x, y, z, influence) {
		this.tearInfluenceSq = influence * influence;
		var tpos = this.tear.position;
		tpos.x = x;
		tpos.y = y;
		tpos.z = z;
	}, 

	_handleInteractions: function() {
		if (this.moveInfluenceSq!==0 || this.tearInfluenceSq!==0) {
			//interactions
			for (var i=0; i<this.points.length; i++) {
				var p = this.points[i];
				var movePos = this.move.position;
				var tearPos = this.tear.position;
				var otherPos = p.position;
				var mvec = movePos;
				var tvec = tearPos;
				var ovec = otherPos;

				var moveDistSq, tearDistSq;
				if (this.tear3D) {
					moveDistSq = mvec.distSq(ovec);
					tearDistSq = tvec.distSq(ovec);
				} else {
					moveDistSq = distSq(mvec.x, mvec.y, ovec.x, ovec.y);
					tearDistSq = distSq(tvec.x, tvec.y, ovec.x, ovec.y);
				}

					

				if (this.moveInfluenceSq!==0 && moveDistSq < this.moveInfluenceSq) {
					//new and old positions
					var op = this.move.lastPosition;
					
					var mAmtX = (mvec.x - op.x);
					var mAmtY = (mvec.y - op.y);
					var mAmtZ = (mvec.z - op.z);

					var max = 0;
					var lenSq = mAmtX*mAmtX + mAmtY*mAmtY + mAmtZ*mAmtZ;
					if (this.maxMoveSq === 0 || lenSq < this.maxMoveSq) {
						mAmtX *= this.moveScalar;
						mAmtY *= this.moveScalar;
						mAmtZ *= this.moveScalar;
						p.lastPosition.x = p.position.x - mAmtX;
						p.lastPosition.y = p.position.y - mAmtY;
						p.lastPosition.z = p.position.z - mAmtZ;
						
						if (p.isWeakPin) {
							p.unpin();
						}
					}
				} else if (this._pinRemoveInfluenceSq!==0 && moveDistSq < this._pinRemoveInfluenceSq) {
					if (p.isWeakPin)
						p.unpin();
				}
				if (this.tearInfluenceSq!==0 && tearDistSq < this.tearInfluenceSq) {
					p.constraints = [];
				}
			}
		}
	},

	_solveConstraints: function() {
		//given the degree of accuracy, we'll solve our constraints here...
		var i = this.accuracy;
		while (i--) {
			for (var j=0; j<this.points.length; j++) {
				this.points[j].solveConstraints(this);
			}
		}
	},

	_integrate: function(delta) {
		//now update with verlet integration
		for (var i=0; i<this.points.length; i++) {
			var p = this.points[i];
			
			//add world gravity
			p.addForce(this.gravity);

			var posvec = p.position;
			var lastvec = p.lastPosition;
			var accelvec = p.acceleration;

			//difference in positions
			var vx = posvec.x - lastvec.x;
			var vy = posvec.y - lastvec.y;
			var vz = posvec.z - lastvec.z;

			//dampen velocity
			vx *= this.friction;
			vy *= this.friction;
			vz *= this.friction;

			p.velocity.x = vx;
			p.velocity.y = vy;
			p.velocity.z = vz;

			//len2
			var len2 = vx * vx + vy * vy + vz * vz;
			
			if (this.floor!==null && p.position.y >= this.floor && len2 > 0.000001) {
				var m = Math.sqrt(len2);
				if (m!=0) {
					vx /= m;
					vy /= m;
				}
				vx *= m*this.groundFriction;
				vy *= m*this.groundFriction;
			}
			
			var tSqr = delta * delta;
			
			//verlet integration
			var nx = posvec.x + vx + 0.5 * accelvec.x * tSqr;
			var ny = posvec.y + vy + 0.5 * accelvec.y * tSqr;
			var nz = posvec.z + vz + 0.5 * accelvec.z * tSqr;

			//set last position & new position
			lastvec.x = posvec.x;
			lastvec.y = posvec.y;
			lastvec.z = posvec.z;

			posvec.x = nx;
			posvec.y = ny;
			posvec.z = nz;

			//reset acceleration
			accelvec.x = 0;
			accelvec.y = 0;
			accelvec.z = 0;
		}
	},

	_setPins: function() {
		for (var i=0; i<this.points.length; i++) {
			var p = this.points[i];
			if (p.pinned && p.pinPosition !== null) {
				p.position.x = p.pinPosition.x;
				p.position.y = p.pinPosition.y;
				p.position.z = p.pinPosition.z;
			}   
		}
	},

	step: function(delta) {		
		//use small methods in the hopes that V8 optimizes them a bit better
		this._solveConstraints();
		this._handleInteractions();
		this._integrate(delta);
		this._setPins();
		
		this.move.lastPosition.x = this.move.position.x;
		this.move.lastPosition.y = this.move.position.y;
		this.move.lastPosition.z = this.move.position.z;
		
		this.tear.lastPosition.x = this.tear.position.x;
		this.tear.lastPosition.y = this.tear.position.y;
		this.tear.lastPosition.z = this.tear.position.z;
		
		this.moveInfluenceSq = this.tearInfluenceSq = 0;
	}
});

module.exports = World;