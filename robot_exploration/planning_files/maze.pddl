;Header and description

(define (domain maze)

;remove requirements that are not needed
(:requirements :strips :typing :conditional-effects :negative-preconditions :equality :universal-preconditions :durative-actions)

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
    (known_id ?marker - marker)
    (unknown_id ?mark - marker)
    ;(has_minimum_id ?mark - marker)
    ;(has_marker ?ms - marker_set ?m - marker)
    ;(same_location ?rob - robot ?m - marker)
    (at_min_id_marker ?rob - robot)


)


(:functions ;todo: define numeric functions here
    
)

(:durative-action robot_move
    :parameters (?rob - robot ?from - location ?to - location)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robot_on ?rob ?from)
        ))
    )
    :effect (and 
        (at start (and 
            (not(robot_on ?rob ?from))  
        ))
        (at end (and 
            (robot_on ?rob ?to)
        ))
    )
)





(:durative-action search_marker_id
    :parameters (?robot - robot ?marker - marker ?loc - location)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robot_on ?robot ?loc)
            (marker_on ?marker ?loc)
            (unknown_id ?marker)
        ))
    )
    :effect (and 
        ;(at start (and
             
        ;))
        (at end (and 
            (known_id ?marker)
            (not (unknown_id ?marker))
        ))
    )
)




(:durative-action reach_min_id_marker
    :parameters (?rob - robot ?m1 -marker ?m2 - marker ?m3 - marker ?m4 - marker)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (forall (?m - marker)  (and (known_id ?m)))
            (known_id ?m1)
            (known_id ?m2)
            (known_id ?m3)
            (known_id ?m4)
            ;(not (= ?m1 ?m2))
            ;(not (= ?m1 ?m3))
            ;(not (= ?m1 ?m4))
            ;(not (= ?m2 ?m3))
            ;(not (= ?m2 ?m4))
            ;(not (= ?m3 ?m4))
        ))
    )
    :effect (and 
        (at end (and 
            (at_min_id_marker ?rob)
        ))
    )
)

;(:action find_minimum_id
;    :parameters (?ms - marker_set ?m1 ?m2 ?m3 ?m4 - marker)
;    :precondition (and 
;        ;(forall (?m - marker)  (and (known_id ?m) (has_marker ?ms ?m)))
;        (known_id ?m1)
;        (known_id ?m2)
;        (known_id ?m3)
;        (known_id ?m4)
;    )
;    :effect (or
;        ;forall (?m - marker) (or (known_id ?m))
;        (has_minimum_id ?m1)
;        (has_minimum_id ?m2)
;        (has_minimum_id ?m3)
;        (has_minimum_id ?m4)
;    )
;)

)



