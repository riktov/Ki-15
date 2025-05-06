# Simulate a double-hinged canopy
# We use x and y coordinates, as if looking forward from the cockpit. On the model x and y get mapped to y and z.

# hinge - fixed position on fusealage
# peg - sliding on the track
# apex - point where the two halves, one connected to the hinge and the other to the peg, of equal length, 
#        meet when the canopy is closed

var hinge = [0.25, 0.6] ;
var track_initial = [-0.25, 0.6] ;

var apex_initial = [0, 1.0] ;

var dx = hinge[0] - apex_initial[0] ;
var dy = hinge[1] - apex_initial[1] ;

var arm_length = math.sqrt((dx * dx) + (dy * dy)) ;

# angle of the line from peg to apex, measured from line connecting peg to hinge, when closed
var splay_angle_initial = -1 * math.atan(dy / dx) ;

# points along the track
var track = [
    track_initial, 
     [-0.24, 0.63], 
      [-0.235, 0.69], 
       [-0.22, 0.75], 
        [-0.20, 0.87], 
         [-0.15, 0.95], 
          apex_initial,  
         [0.15, 0.95], 
        [0.20, 0.87], 
       [0.22, 0.75], 
      [0.235, 0.69], 
     [0.24, 0.63], 
    [0.24, 0.62]
] ;

var canopy = aircraft.door.new("/controls/canopy/", 2) ;
print("Set Ki-15 double-hinged canopy.") ;

var inner_hinge_pos = func(peg) {
    var ddx = hinge[0] - peg[0] ; # hinge will always be farther right (positive) than peg
    var ddy = peg[1]- hinge[1] ;    # peg will always be same or higher than hinge

    # distance between the peg and the hinge. This serves as the x leg to calculate the angle of the arm
    var base_length = math.sqrt(ddx * ddx + ddy * ddy) ;
    
    # with canopy fully opened, peg is at hinge, angle could be anything, so set it to horizontal
    var base_angle = math.pi / 2 ;
    
    if(base_length > 0) {
        base_angle = math.atan(ddy / ddx) ;
     }
    
    splay_angle_displacement = splay_angle_initial - math.acos((base_length / 2 )  / arm_length)  ;
    
    var peg_angle   = base_angle + splay_angle_displacement ;
    var hinge_angle = base_angle - splay_angle_displacement ;

    hinge_angle_deg = int(hinge_angle * 180 / math.pi) ; 
    setprop("/sim/model/canopy/hinged/angle", hinge_angle_deg) ;

    peg_angle_deg = int(peg_angle * 180 / math.pi) ; 
    setprop("/sim/model/canopy/tracked/angle", peg_angle_deg) ;    

    setprop("/sim/model/canopy/tracked/x", peg[0] - track_initial[0]) ;
    setprop("/sim/model/canopy/tracked/y", peg[1] - track_initial[1]) ;
}


var canopy_listener = setlistener("/controls/canopy/position-norm",
    func(node) {
        var num_stops = size(track) ;
        var pos = int(node.getValue() * num_stops) ;
        if(pos < num_stops) {
            var trackpos = track[pos] ;
            inner_hinge_pos(track[pos]) ;
        }
    }, 1
) ;


