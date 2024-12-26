
(define (problem find_lowest_marker) 
(:domain maze)
(:requirements :typing)
(:objects 
    rob - robot 
    m1 m2 m3 m4 - marker
    fin_loc l1 l2 l3 l4 init_l - location
)

(:init
    (robot_on rob init_l)
    (marker_on m1 l1)
    (marker_on m2 l2)
    (marker_on m3 l3)
    (marker_on m4 l4)
    (unknown_ID m1)
    (unknown_ID m2)
    (unknown_ID m3)
    (unknown_ID m4)
    
    


)

(:goal (and
    (known_ID m1)
    (known_ID m2)
    (known_ID m3)
    (known_ID m4)
    (at_min_id_marker rob)
) 

)

)