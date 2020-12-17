 
(define (domain ur5_e)
;   negative preconditions not supported in rosplan
(:requirements :strips :typing :fluents :durative-actions :negative-preconditions)

(:types waypoint robot cube - object
	cubepos - waypoint)
  
(:predicates
    (robot_at ?v - robot ?wp - waypoint)
	(cube_at ?cu - cube ?cp - cubepos)
	(holding_cube ?v - robot ?c - cube)
	(holding ?v - robot)
	(not-holding ?v - robot)
	(moving ?v - robot)
	(not-moving ?v - robot)
)
  
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 20)
	:condition (and 
		(at start (robot_at ?v ?from))
		(at start (not-moving ?v))
		) ; add a can_move over all here
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at start (moving ?v))
		(at start (not (not-moving ?v)))
		(at end (robot_at ?v ?to))
		(at end (not (moving ?v)))
		(at end (not-moving ?v))
		)
)

(:durative-action pickup
	:parameters (?v - robot ?cu - cube ?cp - cubepos)
	:duration ( = ?duration 5)
	:condition (and 
		(at start (robot_at ?v ?cp))
		(at start (cube_at ?cu ?cp))
		(at start (not-holding ?v))
		)
	:effect (and
		(at end (holding ?v))
		(at end (not (not-holding ?v)))
		(at end (holding_cube ?v ?cu))
		)
)

(:durative-action drop
	:parameters (?v - robot ?cu - cube ?cp - cubepos)
	:duration ( = ?duration 5)
	:condition (and 
		(at start (robot_at ?v ?cp))
		(at start (holding ?v))
		(at start (holding_cube ?v ?cu))
		; add a can_move over all here
		)
	:effect (and
		(at end (not (holding ?v)))
		(at end (not-holding ?v))
		(at end (not (holding_cube ?v ?cu)))
		(at end (cube_at ?cu ?cp))
		)
)
)