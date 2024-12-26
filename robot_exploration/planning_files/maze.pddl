;Header and description

(define (domain maze)

;remove requirements that are not needed
(:requirements :strips :typing :conditional-effects :negative-preconditions :equality :universal-preconditions)

(:types 
    robot
    marker
    ;marker_known_id - marker
    ;mark_min_id - marker_known_id
    marker_set
    location

)

; un-comment following line if constants are needed
;(:constants )

(:predicates 
    (robot_on ?rob - robot ?loc - location)
    (marker_on ?marker - marker ?location - location)
    (known_ID ?marker - marker)
    (unknown_ID ?mark - marker)
    ;(has_minimum_id ?mark - marker)
    ;(has_marker ?ms - marker_set ?m - marker)
    ;(same_location ?rob - robot ?m - marker)
    (at_min_id_marker ?rob - robot)


)


(:functions ;todo: define numeric functions here
    
)



(:action robot_move
    :parameters (?rob - robot ?from - location ?to - location)
    :precondition (and 
        (robot_on ?rob ?from)
    )
    :effect (and 
        (not(robot_on ?rob ?from))   
        (robot_on ?rob ?to)
    )
)

(:action search_marker_id
    :parameters (?robot - robot ?marker - marker ?loc - location)
    :precondition (and
        (robot_on ?robot ?loc)
        (marker_on ?marker ?loc)
        (unknown_ID ?marker)
        ;(not (known_ID ?marker))
    )
    :effect (and 
        (known_ID ?marker)
        (not (unknown_ID ?marker))
        ;(marker_known_id ?marker)
    )
)
(:action reach_min_id_marker
    :parameters (?rob - robot)
    :precondition (and 
        (forall (?m - marker)  (and (known_ID ?m)))
        ;(not (at_min_id_marker ?rob))
    )
    :effect (and
        (at_min_id_marker ?rob)
    )
)
;(:action find_minimum_ID
;    :parameters (?ms - marker_set ?m1 ?m2 ?m3 ?m4 - marker)
;    :precondition (and 
;        ;(forall (?m - marker)  (and (known_ID ?m) (has_marker ?ms ?m)))
;        (known_ID ?m1)
;        (known_ID ?m2)
;        (known_ID ?m3)
;        (known_ID ?m4)
;    )
;    :effect (or
;        ;forall (?m - marker) (or (known_ID ?m))
;        (has_minimum_id ?m1)
;        (has_minimum_id ?m2)
;        (has_minimum_id ?m3)
;        (has_minimum_id ?m4)
;    )
;)

)



