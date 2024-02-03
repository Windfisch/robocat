$fs=0.75;
$fa=3;

$fs=0.25;
$fa=2;

module servo_horn() {
	tol = 0.5;
	r1 = (7.5+tol)/2;
	r2 = (4.8+tol)/2;


	intersection() {
		linear_extrude(3.5) hull() {
			circle(r1);
			translate([21.5-r1-r2, 0]) circle(r2);
		}
		rotate([90,0,0])
		linear_extrude(100, center=true)
		polygon([
			[-99, 3.5],
			[r1, 3.5],
			[5.75, 2],
			[99, 2],
			[99, 0],
			[-99, 0],
		]);
	}
	translate([0,0,-2-tol])
	cylinder(5.5, r1,r1);
}

module servo_horn_slidein() {
	servo_horn();
	
	hull() {
		intersection() {
			servo_horn();
			cube([0.1, 99, 99], center=true);
		}
		
		translate([-99, 0, 0])
		servo_horn();
	}
}


module servo() {
	tol = 0.5;
	
	translate([-tol/2, -tol/2, -tol/2])
		cube([23.2+tol, 12.2+tol, 24.2+tol]);
	
	translate([-tol/2 - (32.3-23.2)/2,-tol/2, 17.3 - tol/2])
		cube([32.3 +tol, 12.2 +tol, 2.6 + tol]);
	
	translate([12.2/2, 12.2/2, 24.2])
		cylinder(4.2 + tol/2, d=12.2+tol);

	translate([12.2/2, 12.2/2, 24.2])
		cylinder(7.7 + tol/2, d=5+tol);

	translate([12.2/2 + 8.8 - 5/2, 12.2/2, 24.2])
		cylinder(4.2 + tol/2, d=5+tol);
	
	translate([-1, 12.2/2 - 5.5/2, 0])
		cube([1+tol, 5.5, 6]);
}

module servo_slidein() {
	servo();
	
	translate([23.2/2-27.3/2, 12.2/2, 17.3 - 5])
		cylinder(5+0.1, d=1.2);

	translate([23.2/2+27.3/2, 12.2/2, 17.3 - 5])
		cylinder(5+0.1, d=1.2);
}

//!servo_slidein();

module servo_at_horn() {
	translate([-12.2/2, -12.2/2,-31.5]) servo();
}

//!servo_horn();

difference() {
    union() {
        difference() {
            hull() {
                translate([-8.5 - 5.5, -15, 10])
                    rotate([0,90,0]) cylinder(5.5 + 2, d=12);
        //		translate([-8.5 - 5.5, 8, 10])
        //			rotate([0,90,0]) cylinder(5.5 + 2, d=12);
            translate([-8.5-5.5, -2, 0.01])
                cube([5.5+2, 12.2+2+2, 17.3+2.5]);

            }
            translate([-8.5, -15, 10])
                rotate([-90,0,90]) servo_horn_slidein();
        }

        difference() {
            translate([-5 - 2, -2, 0.01])
                cube([32.3+2+2, 12.2+2+2, 17.3+2.5]);
            servo_slidein();
        }
    }
    linear_extrude(999) polygon([
       [-19,0],
        [0, 19],
        [-99,99]
    ]);
}

/*color("blue", 0.2) servo();
translate([-8.5, -15, 10]) rotate([-90,0,90]) {
	color("white", 0.4) servo_horn();
	color("blue", 0.2) rotate([0,0,180-45]) servo_at_horn();
}
	*/


