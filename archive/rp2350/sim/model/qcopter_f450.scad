// Parametric, visual F450/Q450-style frame. Millimetres.
// Provisional geometry only: measure the actual clone before manufacturing.
$fn = 48;
motor_diagonal = 450;
motor_offset = motor_diagonal / 2 / sqrt(2);
prop_diameter = 9.0 * 25.4; // User's 9047 prop: 9.0 in diameter x 4.7 in pitch
arm_width = 22;
arm_height = 8;
plate_size = 110;
plate_gap = 18;
motor_z = 30;
show_props = true;

module rounded_box(size, radius) {
    minkowski() {
        cube([size[0]-2*radius, size[1]-2*radius, size[2]-2*radius], center=true);
        sphere(r=radius);
    }
}

module arm(to_x, to_y, color_name) {
    length = sqrt(to_x*to_x + to_y*to_y);
    angle = atan2(to_y, to_x);
    color(color_name)
        translate([to_x/2, to_y/2, 0])
            rotate([0, 0, angle])
                cube([length, arm_width, arm_height], center=true);
}

module motor_and_prop(x, y, color_name, direction=1) {
    translate([x, y, motor_z]) {
        color(color_name) cylinder(h=22, d=28, center=true);
        color([0.10,0.10,0.11]) translate([0,0,12]) cylinder(h=6, d=18, center=true);
        if (show_props) {
            color([0.1,0.1,0.1,0.36])
                translate([0,0,17]) rotate([0,0,direction*8])
                    linear_extrude(height=1.5, center=true)
                        hull() {
                            translate([0,0]) circle(d=12);
                            translate([prop_diameter*0.46,0]) circle(d=20);
                        }
        }
    }
}

// Center plates and flight-controller stack.
color([0.13,0.14,0.16]) {
    translate([0,0,-plate_gap/2]) rounded_box([plate_size,plate_size,3],6);
    translate([0,0, plate_gap/2]) rounded_box([plate_size,plate_size,3],6);
}
// Red front pair, white rear pair; confirm arm coloring/orientation on the purchased clone.
arm( motor_offset,  motor_offset, [0.78,0.06,0.04]);
arm( motor_offset, -motor_offset, [0.78,0.06,0.04]);
arm(-motor_offset, -motor_offset, [0.88,0.87,0.82]);
arm(-motor_offset,  motor_offset, [0.88,0.87,0.82]);

// Approximate two landing legs/skids across the airframe.
for (y=[-43,43]) color([0.10,0.10,0.11]) {
    translate([0,y,-42]) cube([250,10,8],center=true);
    for (x=[-95,95]) translate([x,y,-28]) cylinder(h=30,d=8);
}

// Board, battery and payload are placeholders; dimensions/mass need measurement.
color([0.05,0.32,0.18]) translate([0,0,8]) rounded_box([85,85,2],4);
color([0.18,0.18,0.20]) translate([-5,0,-23]) rounded_box([125,42,24],5);
color([0.22,0.24,0.27]) translate([0,0,27]) rounded_box([46,36,12],3);

motor_and_prop( motor_offset,  motor_offset, [0.78,0.06,0.04],  1); // FL
motor_and_prop( motor_offset, -motor_offset, [0.78,0.06,0.04], -1); // FR
motor_and_prop(-motor_offset, -motor_offset, [0.88,0.87,0.82],  1); // RR
motor_and_prop(-motor_offset,  motor_offset, [0.88,0.87,0.82], -1); // RL

// 3S battery visual block; actual pack size and CG are TBD.
color([0.08,0.08,0.09]) translate([-18,0,-8]) rounded_box([105,38,28],4);
