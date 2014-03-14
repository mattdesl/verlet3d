var Class = require('klasse');
var Vector3 = require('vecmath').Vector3;
var Constraint = require('./Constraint');

var PointMass = new Class({
	initialize: function (x, y, z, mass) {
		this.position = new Vector3(x, y, z);
		this.lastPosition = new Vector3(x, y, z);
		this.acceleration = new Vector3(0.0, 0.0, 0.0);
		
		/** Changing this doesn't do anything. 
		This is only included so you can do something with the velocity. */
		this.velocity = new Vector3(0, 0, 0);

		this.pinPosition = null;
		this.pinned = false;
		this.isWeakPin = false;
		this.mass = mass || 1.0;
		
		this.constraints = [];
	},
	
	addForce: function(vec) {
		// acceleration = force / mass
		var a = this.acceleration; //vector components
		a.x += vec.x / this.mass;
		a.y += vec.y / this.mass;
		a.z += vec.z / this.mass;
	},
	
	attach: function(point, restingDistance, stiffness, tearDistance) {
		var c = new Constraint(this, point, restingDistance, stiffness, tearDistance);
		this.addConstraint(c);
		return c;
	},
	
	addConstraint: function(constraint) {
		this.constraints.push(constraint);  
	},
	
	removeConstraint: function(constraint) {
		var i = this.constraints.length;
		while (i--) {
			if (this.constraints[i] === constraint)
				this.constraints.splice(i, 1);
		}
	},
	
	pin: function(x, y, z, weak) {
		if (this.pinPosition === null)
			this.pinPosition = new Vector3();
		this.pinPosition.x = x;
		this.pinPosition.y = y;
		this.pinPosition.z = z;
		this.pinned = true;
		this.isWeakPin = !!weak;
	},
	
	unpin: function() {
		this.pinned = false;   
		this.isWeakPin = false;
	},
	
	solveConstraints: function(world) {
		//solve each constraint
		for (var i=0; i<this.constraints.length; i++) 
			this.constraints[i].solve();
		
		//force the constraint within the bounds of our window
		// if (world.bounds !== null) {
		// 	var bx = world.bounds.x + 1;
		// 	var by = world.bounds.y + 1;
		// 	var bw = world.bounds.width - 1;
		// 	var bh = world.bounds.height - 1;
			
		// 	var pvec = this.position;
		// 	var px = this.position.x;
		// 	var py = this.position.y;

		// 	if (py < by)
		// 		pvec.y = 2 * by - py;
		// 	if (py > bh)
		// 		pvec.y = 2 * bh - py;
		// 	if (px < bx)
		// 		pvec.x = 2 * bx - px;
		// 	if (px > bw)
		// 		pvec.x = 2 * bw - px;
		// }
		
		//TODO: solve before pinning?
//            if (this.pinned && this.pinPosition !== null) {
//                this.position.x = this.pinPosition.x;
//                this.position.y = this.pinPosition.y;
//            }
	}
});

module.exports = PointMass;