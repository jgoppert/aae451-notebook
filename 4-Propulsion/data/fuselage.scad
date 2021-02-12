$fn=100;

sheet_height=508;
sheet_width=762;

module line(points, thickness) {
    union() {
        for (i=[0:len(points)-2]) {
            hull() {
                translate(points[i]) circle(d=5);
                translate(points[i+1]) circle(d=5);
            }
        }
    }
}

h = 60;
w = 60;
l1 = 200;
l2 = 300;
t = 5;

module airfoil() {
    line([[0, 0], [20, 20], [100, 0]], 10);
}

module side() {
    difference() {
        polygon([[0, 0], [l1, 0], [l2, h], [0, h]]);
        translate([l1/6, h/3]) airfoil();
    }
}

module fuselage_2d() {
    d = sqrt(pow(l1 - l2, 2) + pow(h, 2));
    translate([0, h]) mirror([0, 1]) side();
    translate([0, h + t]) polygon([[0, 0], [l1 + d, 0], [l1 + d, w], [0, w]]);
    translate([0, 2*h + 2*t]) side();
    translate([0, 3*h + 3*t]) polygon([[0, 0], [l2, 0], [l2, w], [0, w]]);
}

fuselage_2d();

%square([sheet_width, sheet_height]);
