# %%

from build123d import *
from ocp_vscode import *

import ocp_vscode

Loc = Location
Rot = Rotation
Rect = Rectangle
RRect = RectangleRounded

def rot2d(angle):
    return Rotation(0,0,angle)

for x,xx in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
    for y,yy in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
        globals()[x+y] = (xx, yy)
        for z,zz in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
            globals()[x+y+z] = (xx, yy, zz)

set_defaults(black_edges=True, render_joints=True, render_edges=True, reset_camera=False, default_opacity=0.7)


# %%

servoconn_w, servoconn_h = 8.5, 4.5

def servo():
    body_xlen = 23
    body_ylen = 12.5
    body_zlen = 24
    
    hole_xdist = 27.7

    flange_xlen = 34
    flange_zlen = 2.5
    flange_zdist_bottom = 17.3

    axis_offcenter = 5.5

    result = extrude(Rect(body_xlen, body_ylen), body_zlen)
    flange = extrude(
        Rect(flange_xlen, body_ylen)
        - Loc((-hole_xdist/2,0)) * (Circle(3/2) + Rect(99, 1.2, align=HC))
        - Loc((+hole_xdist/2,0)) * (Circle(3/2) + Rect(99, 1.2, align=LC)),
        flange_zlen
    )
    flange = Loc((0,0,flange_zdist_bottom)) * flange
    result += flange

    topaxis = Plane(Loc((-axis_offcenter,0,0)) * result.faces().sort_by(Axis.Z)[-1].center_location)

    result += extrude( topaxis * Circle(11.15/2), 4)
    result += extrude( topaxis * Circle(5/2), 6.5)
    result += extrude( topaxis * Loc((11.15/2 + 3.35,0)) * Circle(2.75, align=HC), 4)

    RigidJoint("mount", result, flange.faces().sort_by(Axis.Z)[-1].center_location)
    RevoluteJoint("horn_master", result, Axis(result.faces().sort_by(Axis.Z)[-1].center_location.position, (0,0,1)))
    RigidJoint("horn_slave", result, result.faces().sort_by(Axis.Z)[-1].center_location)

    result.name="sg90 servo"
    result.color="#5599ffb0"

    return result

def servo_horn():
    xlen = 36

    end_r = 2
    mid_r = 7.5 / 2

    base = Circle(mid_r) + Loc((-xlen/2 +end_r,0)) * Circle(end_r) + Loc((xlen/2-end_r,0)) * Circle(end_r)
    base = make_hull(base.edges())
    
    hole_spacing = 2.5
    outer_hole_dist = 32
    hole_diam = 1.3
    n_holes = 5

    for i in range(n_holes):
        base -= Loc((outer_hole_dist/2 - i * hole_spacing,0)) * Circle(hole_diam/2)
        base -= Loc((-outer_hole_dist/2 + i * hole_spacing,0)) * Circle(hole_diam/2)
    
    base -= Circle(1.2)
    
    result = extrude(base, 2)
    pos = (0,0,1)
    RigidJoint('slave', result, Loc(pos, (0,0,1)))
    RevoluteJoint('master', result, Axis(pos, (0,0,1)))
    RigidJoint('mount', result, Loc((0,0,2), (0,0,0)))

    result.name = "servo horn"
    result.color = "#ffffffd0"

    return result


# %%

BURN_WIDTH=0.15
THICK = 3.1
servo_xlen = 23
servo_ylen = 12.0


# auto_finger_joint code taken from ZTKF
from enum import Enum, auto
class FingerType(Enum):
    ODD = auto()
    EVEN = auto()

from build123d import sqrt
from build123d import floor
from itertools import product
from build123d import sin
from build123d import pi
from numpy import linspace

def auto_finger_joint(
        a: Part,
        b: Part,
        min_finger_width: float,
        swap: bool = False,
        finger_type: FingerType = None
    ) -> tuple[Part, Part]:

    # We're operating on the intersection of the two parts
    inter = a.intersect(b)
    edges = inter.edges().copy()
    edges.sort(key=lambda e: e.length, reverse=True)

    # The operation will be along the shortest of the longest 4
    # edges in the direction of the edge
    edge = edges[0]
    z_dir = (edge @ 1 - edge @ 0).normalized()

    # Determine the number of fingers, one is added to the base
    # count since there is technically a 0th cut. That flips some
    # of the even/odd logic 
    n_fingers = floor(edge.length/min_finger_width) + 1
    if finger_type == FingerType.EVEN and not n_fingers & 1:
        n_fingers -= 1
    elif finger_type == FingerType.ODD and n_fingers & 1:
        n_fingers -= 1
    
    # These are the arrays we'll be filling
    fingers_a, fingers_b = [], []

    # We'll use linspace to evenly space the fingers, skip the
    # first and last because they're outside the intersection
    alternate = (fingers_a, fingers_b)
    to_div = inter

    # 1 is added here since 
    for x in linspace(0.0, 1.0, n_fingers)[1:-1]:

        # Split by our plane along the edge
        plane = Plane(origin=edge @ x, z_dir=z_dir)
        divs = [shape for shape in to_div.split(plane, Keep.BOTH)]

        # Select the correct bottom/top
        if plane.to_local_coords(divs[0]).center().Z >= 0:
            alternate[0].append(divs[1])
            to_div = divs[0]
        else:
            alternate[0].append(divs[0])
            to_div = divs[1]

        # Swap the arrays
        alternate = (alternate[1], alternate[0])

    # The remainder will be the last finger
    alternate[0].append(to_div)

    if swap:
        return (a - fingers_b, b - fingers_a)
    else:
        return (a - fingers_a, b - fingers_b)

def laserify(solid):
    if isinstance(solid, list):
        return [laserify(s) for s in solid]

    show(solid)

    faces = solid.faces().sort_by(SortBy.AREA)
    top = faces[-1]
    bot = faces[-2]
    thick = (top.center_location.inverse() * bot.center_location).position.Z
    sec = section(solid, section_by = Plane(top.center_location), height=thick/2)
    sec = top.center_location.inverse() * sec

    try:
        sec = offset(sec, amount=BURN_WIDTH/2)
    except:
        print("WARNING: Failed to laserify solid, continuing but the part might not fit.")
        sec.color="pink"

    return sec

def arrange1d(solids):
    x = 0
    result = []
    for s in solids:
        print(s)
        bb = s.bounding_box(1)
        result.append(Loc((x - bb.min.X, -bb.min.Y)) * s)
        x += bb.max.X - bb.min.X + 1
    return result


def make_horn_cutout():
    holes = Location((-8.5,0)) * SlotCenterToCenter(1.3, 2.1)
    holes += Location((8.5,0)) * SlotCenterToCenter(1.3, 2.1)
    #holes += Location((0,7)) * rot2d(90) * SlotCenterToCenter(1.3, 2.1)
    #holes += Location((0,-7)) * rot2d(90) * SlotCenterToCenter(1.3, 2.1)
    holes += RRect(10, 7.7, 1)
    return holes

def make_servo_cutout():
    screw_spacing = 28

    base = RRect(servo_xlen, servo_ylen, 0.5)
    base += Loc((-screw_spacing/2,0)) * Circle(1.5/2)
    base += Loc((+screw_spacing/2,0)) * Circle(1.5/2)
    return base


# %%

def make_body():
    LENGTH = 150
    LENGTH2 = LENGTH+80
    sec1a = Loc((30/2,0,0)) * (
        Box(THICK,LENGTH,40, align=CCH)
        - Loc((0,-LENGTH/2,0)) * Box(THICK,10,THICK, align=CLH)
        - Loc((0,LENGTH/2,0)) * Box(THICK,10,THICK, align=CHH)
    )
    sec1b = Loc((-30/2,0,0)) * (
        Box(THICK,LENGTH,40, align=CCH)
        - Loc((0,-LENGTH/2,0)) * Box(THICK,10,THICK, align=CLH)
        - Loc((0,LENGTH/2,0)) * Box(THICK,10,THICK, align=CHH)
    )
    top = (
        Box(120,LENGTH2,THICK, align=CCH)
        - Box(27,110,THICK, align=CCH)
        - Loc((120/2, LENGTH2/2, 0)) * Box(43,40,THICK, align=HHH)
        - Loc((120/2, -LENGTH2/2, 0)) * Box(43,40,THICK, align=HLH)
        - Loc((-120/2, LENGTH2/2, 0)) * Box(43,40,THICK, align=LHH)
        - Loc((-120/2, -LENGTH2/2, 0)) * Box(43,40,THICK, align=LLH)

        # holes for pcb
        - Loc((82/2, 62/2, 0)) * Cylinder(3.4/2, THICK, align=CCH)
        - Loc((-82/2, 62/2, 0)) * Cylinder(3.4/2, THICK, align=CCH)
        - Loc((82/2, -62/2, 0)) * Cylinder(3.4/2, THICK, align=CCH)
        - Loc((-82/2, -62/2, 0)) * Cylinder(3.4/2, THICK, align=CCH)

        # holes for servo cables from bottom to top (3 each)
        - Loc((55/2, 80/2, 0)) * Box(servoconn_w, servoconn_h, THICK, align=CCH)
        - Loc((-55/2, 80/2, 0)) * Box(servoconn_w, servoconn_h, THICK, align=CCH)
        - Loc((55/2, -80/2, 0)) * Box(servoconn_w, servoconn_h, THICK, align=CCH)
        - Loc((-55/2, -80/2, 0)) * Box(servoconn_w, servoconn_h, THICK, align=CCH)

        # holes for servo cables from top to bottom (2 each)
        - Loc((50/2, 110/2, 0)) * Box(servoconn_h, servoconn_w, THICK, align=CCH)
        - Loc((-50/2, 110/2, 0)) * Box(servoconn_h, servoconn_w, THICK, align=CCH)
        - Loc((50/2, -110/2, 0)) * Box(servoconn_h, servoconn_w, THICK, align=CCH)
        - Loc((-50/2, -110/2, 0)) * Box(servoconn_h, servoconn_w, THICK, align=CCH)
    )

    round_r = 20/3
    sec3 = Loc((0,-LENGTH/2,0)) * (
        #Box(120,THICK,25, align=CLH)
        Rot(90,0,0) * extrude(
            RRect(120,25, round_r, align=CH) + Rect(120,25/2, align=CH),
            -THICK)
        +Box(33,THICK,40, align=CLH)
        -Rot((90,0,0))*extrude(Loc((42,-12)) * make_servo_cutout(), -THICK)
        -Rot((90,0,0))*extrude(Loc((-42,-12)) * make_servo_cutout(), -THICK)
    )
    sec4 = Loc((0,LENGTH/2,0)) * (
        Rot(90,0,0) * extrude(
            RRect(120,25, round_r, align=CH) + Rect(120,25/2, align=CH),
            THICK)
        #Box(120,THICK,25, align=CHH)
        +Box(33,THICK,40, align=CHH)
        -Rot((90,0,0))*extrude(Loc((42,-12)) * make_servo_cutout(), THICK)
        -Rot((90,0,0))*extrude(Loc((-42,-12)) * make_servo_cutout(), THICK)
    )

    tri_sec3 = Loc((0, -LENGTH/2 + THICK, 0)) * Rot((0,90,0)) * extrude(
        Rect(20,40+THICK, align = LH) - Rect(THICK, THICK, align=LH),
        THICK/2,
        both = True
    )

    def support_plate_2d(tol):
        return (
            RRect(90, 20, 20/3, align=CH) -
            Loc((15,-12)) * Circle(3.4/2) -
            Loc((-15,-12)) * Circle(3.4/2) - 
            Loc((42-5.5, -12)) * Rect(THICK+tol, THICK+tol) -
            Loc((-42+5.5, -12)) * Rect(THICK+tol, THICK+tol)
        )


    plate_sec3 = Loc((0,-LENGTH/2-40, 0)) * Rot(90,0,0) * extrude(
        support_plate_2d(0.5),
        -THICK
    )
    plate2_sec3 = Loc((0,-LENGTH/2-40 - THICK-1, 0)) * Rot(90,0,0) * extrude(
        support_plate_2d(0),
        -THICK
    )


    tri_sec4 = Loc((0, LENGTH/2 - THICK, 0)) * Rot((0,90,0)) * extrude(
        Rect(20,40+THICK, align = LL) - Rect(THICK, THICK, align=LL),
        THICK/2,
        both = True
    )

    plate_sec4 = Loc((0,LENGTH/2+40, 0)) * Rot(90,0,0) * extrude(
        support_plate_2d(0.5),
        THICK
    )
    plate2_sec4 = Loc((0,LENGTH/2+40 + THICK+1, 0)) * Rot(90,0,0) * extrude(
        support_plate_2d(0),
        THICK
    )

    axes = [
        Loc((x*(42-5.5), y*( LENGTH/2+40 + THICK+1 ), -12)) * Rot(90,0,0) * extrude(
            Rect(THICK, THICK),
            18*y
        )
        for x,y in [(-1,-1),(-1,1),(1,-1),(1,1)]
    ]
    for a in axes: a.color="violet"

    #sec3 = Loc((0,-LENGTH/2,0)) * Box(120,THICK,40, align=CLH)
    #sec4 = Loc((0,LENGTH/2,0)) * Box(120,THICK,40, align=CHH)
    bottom = Loc((0,0,-40)) * Box(33, 80, 3, align=CCL)

    sec1a, top = auto_finger_joint(sec1a, top, 12)
    sec1a, sec3 = auto_finger_joint(sec1a, sec3, 5)
    sec1b, top = auto_finger_joint(sec1b, top, 12)
    sec1b, sec3 = auto_finger_joint(sec1b, sec3, 5)
    top, sec3 = auto_finger_joint(top, sec3, 12)
    sec1a, sec4 = auto_finger_joint(sec1a, sec4, 5)
    sec1b, sec4 = auto_finger_joint(sec1b, sec4, 5)
    top, sec4 = auto_finger_joint(top, sec4, 12)
    sec1a, bottom = auto_finger_joint(sec1a, bottom, 12)
    sec1b, bottom = auto_finger_joint(sec1b, bottom, 12)
    tri_sec3, sec3 = auto_finger_joint(tri_sec3, sec3, 5)
    tri_sec3, plate_sec3 = auto_finger_joint(tri_sec3, plate_sec3, 5)
    top, plate_sec3 = auto_finger_joint(top, plate_sec3, 5)
    top, tri_sec3 = auto_finger_joint(top, tri_sec3, 5)
    tri_sec4, sec4 = auto_finger_joint(tri_sec4, sec4, 5)
    tri_sec4, plate_sec4 = auto_finger_joint(tri_sec4, plate_sec4, 5)
    top, plate_sec4 = auto_finger_joint(top, plate_sec4, 5)
    top, tri_sec4 = auto_finger_joint(top, tri_sec4, 5)

    head=Loc((0,-LENGTH/2 - 40, 30))* Sphere(40)

    solids = [sec1a, sec1b, top, sec3, sec4, bottom, tri_sec3, plate_sec3, tri_sec4, plate_sec4, plate2_sec3, plate2_sec4] + axes


    joints = [
        RigidJoint("horn", sec3, Loc((42,-LENGTH/2+THICK, -12), (90,0,0))),
        RigidJoint("horn2", sec3, Loc((-42,-LENGTH/2+THICK, -12), (90,0,0))),
        RigidJoint("horn3", sec4, Loc((42,LENGTH/2-THICK, -12), (90,180,0))),
        RigidJoint("horn4", sec4, Loc((-42,LENGTH/2-THICK, -12), (90,180,0))),
    ]

    return solids, joints

body_solids, body_joints = make_body()

pcb = (
    Loc((0,0,5)) * extrude(RRect(90,70, 5, align=CC), 1.6)
    + Loc((0,0,-THICK-2)) * (
        Loc((82/2,62/2,0)) * Cylinder(2.5, 5+THICK+2, align=CCL) 
        + Loc((-82/2,62/2,0)) * Cylinder(2.5, 5+THICK+2, align=CCL) 
        + Loc((82/2,-62/2,0)) * Cylinder(2.5, 5+THICK+2, align=CCL) 
        + Loc((-82/2,-62/2,0)) * Cylinder(2.5, 5+THICK+2, align=CCL)
    )
)
pcb.color = "#8800ffb0"
pcb.name="pcb"

show(body_solids + [pcb])

# %%



def servo_horn_mount():
    a = Loc((-3, 0)) * Circle(13)
    b = Loc((16,0)) * Rectangle(24,13*2, align=LC)
    c = make_hull([a.edges(),b.edges()])


    holes = make_horn_cutout()
    r = c - holes

    r = extrude(r, THICK)

    joint_point = (
        r.faces().filter_by(Plane.XZ).sort_by(Axis.Y)[0]
        .edges().filter_by(Axis.X).sort_by(Axis.Y)[-1]
        .vertices().sort_by(Axis.X)[-1]
    )

    joint_point_bp = (
        r.faces().filter_by(Plane.XZ).sort_by(Axis.Y)[-1]
        .edges().filter_by(Axis.X).sort_by(Axis.Y)[-1]
        .vertices().sort_by(Axis.X)[-1]
    )

    print(joint_point)

    j1 = RevoluteJoint("knee_servo_horn", r, Axis((0,0,0), (0,0,1)))
    j2 = RigidJoint("hip_servo_mount", r, -Loc(joint_point, (0,0,180)))
    j3 = RigidJoint("servo_backplate", r, -Loc(joint_point_bp, (0,0,0)))

    return r

#show(servo_horn_mount())



def servo_hip_mount():
    d = 12
    round = 4

    yextra = 12

    
    base = Loc((-d,0)) * (RRect(d + servo_xlen + 5, servo_ylen + yextra, round, align = LC) + Rect(round + 0.1, servo_ylen + yextra, align = LC))
    base -= Loc((servo_xlen/2,0)) * make_horn_cutout()

    base = extrude(base, THICK)

    joint_loc = base.vertices().group_by(Axis.X)[0].group_by(Axis.Y)[0].sort_by(Axis.Z)[0]
    print(joint_loc)

    tri1_loc = Vector(joint_loc + (0,THICK/2,0))
    tri2_loc = tri1_loc.__copy__()
    tri2_loc.Y *= -1
    servo_loc = Vector(servo_xlen/2, 0, 0)

    RigidJoint("attach", base, Loc(joint_loc, (0,90,90)))
    RigidJoint("tri1", base, Loc(tri1_loc, (90,0,0)))
    RigidJoint("tri2", base, Loc(tri2_loc, (90,0,0)))
    RigidJoint("servo", base, Loc(servo_loc, (0,0,0)))

    return base

def tri():
    xlen = 30
    ylen = 23
    tol = 3
    
    tri = (
        Rect(xlen-tol, ylen-tol, align = LL)
        #Triangle(a=xlen-tol, c=ylen-tol, B = 90, align = LL)
        + Rect(xlen-tol, THICK+tol, align=LH)
        + Rect(THICK+tol, ylen-tol, align = HL)
        + Rect(tol, tol, align = HH)
        - Loc((xlen-2*tol,-tol)) * Rect(tol, THICK, align = LH)
        - Loc((-tol, ylen-2*tol)) * Rect(THICK, tol, align = HL)
    )
    tri = Loc((tol, tol)) * tri
    tri = tri - Loc((xlen/2+5.5, 6)) * Rect(servoconn_w, servoconn_h)

    tri = extrude(tri, THICK/2, both=True)
    j = RigidJoint("attach", tri, Loc((-THICK,-THICK,0), (0,0,0)))

    return tri


backplate = extrude(Rect(24,33, align = LL) - Loc((24/2,12 + servo_xlen/2)) * Circle(3.2 * sqrt(2) / 2 ) , 3) 
RigidJoint("attach", backplate, Loc((0,0,0),(90,180,0)))


hip_lower = servo_horn_mount()
hip_upper = Rot((90,0,90)) * servo_hip_mount()
hip_tri1 = tri()
hip_tri2 = tri()

servo1 = servo()
servo2 = servo()
horn1 = servo_horn()
horn_top = servo_horn()

knee_servo = servo()
knee_horn = servo_horn()

body_joints[0].connect_to(servo1.joints['mount'])
servo1.joints['horn_master'].connect_to(horn_top.joints['slave'], angle=180)
horn_top.joints['mount'].connect_to(hip_upper.joints['servo'])

hip_upper.joints['attach'].connect_to(hip_lower.joints['hip_servo_mount'])
hip_lower.joints['knee_servo_horn'].connect_to(horn1.joints['mount'], angle=0)

hip_lower.joints['servo_backplate'].connect_to(backplate.joints['attach'])

hip_upper.joints['tri1'].connect_to(hip_tri1.joints['attach'])
hip_upper.joints['tri2'].connect_to(hip_tri2.joints['attach'])

show([body_solids, horn_top, servo1, hip_tri1, hip_tri2, hip_upper, hip_lower, backplate, pcb])

# %%


def upper_leg():
    leg = SlotCenterToCenter(60, 20)
    leg -= Loc((-20, 0)) * make_servo_cutout()
    leg -= Loc((20, 0)) * make_servo_cutout()
    leg = extrude(leg, THICK)
    RigidJoint("upper", leg, Location((-20,0,0)))
    RigidJoint("lower", leg, Location((20,0,0), (0,0,180)))
    return leg

def lower_leg():
    leg = Loc((-60,0)) * Circle(15/2)
    leg += Loc((0,0)) * Circle(30/2)
    leg = make_hull(leg.edges())
    leg -= make_horn_cutout()

    leg = extrude(leg, THICK)

    RigidJoint("horn", leg, Loc((0,0,0)))

    return leg


uleg = upper_leg()
lleg = lower_leg()




horn1.joints['master'].connect_to(servo2.joints['horn_slave'], angle=180+45)
servo2.joints['mount'].connect_to(uleg.joints['upper'])
uleg.joints['lower'].connect_to(knee_servo.joints['mount'])
knee_servo.joints['horn_master'].connect_to(knee_horn.joints['slave'], angle = 360-90)
knee_horn.joints['mount'].connect_to(lleg.joints['horn'])

xtrans = hip_lower.location.inverse()
ytrans = hip_upper.location.inverse()
t1trans = hip_tri1.location.inverse()
t2trans = hip_tri2.location.inverse()


finger = auto_finger_joint

hip_lower, hip_upper = finger(hip_lower, hip_upper, 3)
hip_lower, hip_tri1 = finger(hip_lower, hip_tri1, 3, swap=True)
hip_lower, hip_tri2 = finger(hip_lower, hip_tri2, 3, swap=False)
hip_upper, hip_tri1 = finger(hip_upper, hip_tri1, 3, swap=True)
hip_upper, hip_tri2 = finger(hip_upper, hip_tri2, 3, swap=True)

hip_lower, backplate = finger(hip_lower, backplate, 3)
hip_tri1, backplate = finger(hip_tri1, backplate, 3)
hip_tri2, backplate = finger(hip_tri2, backplate, 3)

hip_lower.color='red'
hip_upper.color='blue'

hip_lower.name = 'leg_servo_horn'
hip_upper.name = 'hip_servo_mount'
hip_tri1.name = 'tri1'
hip_tri2.name = 'tri2'

leg_assembly = [horn_top,hip_lower,hip_upper,hip_tri1,hip_tri2,servo1, servo2, knee_servo, uleg, horn1, knee_horn, lleg, backplate]


def my_mirror(objs, plane):
    result = [mirror(o, plane) for o in objs]
    #result = mirror(objs, plane)
    for (orig, mirr) in zip(objs, result):
        mirr.color = orig.color
        #mirr.name = orig.name
    return result

leg_assembly2 = my_mirror(leg_assembly, Plane.XZ)
leg_assembly3 = my_mirror(leg_assembly, Plane.YZ)
leg_assembly4 = my_mirror(leg_assembly2, Plane.YZ)


show(body_solids, leg_assembly, leg_assembly2, leg_assembly3, leg_assembly4)

# %%

hip_parts = [hip_lower, hip_upper, hip_tri1, hip_tri2]

part2d = arrange1d(laserify(body_solids + hip_parts + [uleg, lleg]))

#show(part2d)

exporter = ExportSVG(scale=1)
exporter.add_layer("Visible")
exporter.add_shape(part2d, layer="Visible")
exporter.write("robocat_mk2.svg")


