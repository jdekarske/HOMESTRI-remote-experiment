 
(define (domain ur5_e)
(:requirements :strips :typing :fluents :durative-actions :timed-initial-literals)

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
	:duration ( = ?duration 10)
	:condition (and 
		(at start (robot_at ?v ?from))
		(at start (not-moving ?v))
		) ; add a can_move over all here
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at end (robot_at ?v ?to))
		
		(at start (moving ?v))
		(at start (not (not-moving ?v)))
		(at end (not (moving ?v)))
		(at end (not-moving ?v))
		)
)

(:durative-action pickup
	:parameters (?v - robot ?cu - cube ?cp - cubepos)
	:duration ( = ?duration 2)
	:condition (and 
		(at start (robot_at ?v ?cp))
		(over all(robot_at ?v ?cp))
		(at end (robot_at ?v ?cp))
		
		(at start (cube_at ?cu ?cp))
		
		(at start (not-moving ?v))
		(at start (not-holding ?v))
		)
	:effect (and
	    (at end (not (cube_at ?cu ?cp)))
	    
		(at end (holding_cube ?v ?cu))
		
		(at end (holding ?v))
		(at end (not (not-holding ?v)))
				)
)

(:durative-action drop
	:parameters (?v - robot ?cu - cube ?cp - cubepos)
	:duration ( = ?duration 1)
	:condition (and 
		(at start (robot_at ?v ?cp))
		(over all(robot_at ?v ?cp))
		(at end (robot_at ?v ?cp))
		
		(at start (not-moving ?v))
		(at start (holding ?v))
		(at start (holding_cube ?v ?cu))
		; add a can_move over all here
		)
	:effect (and
		(at end (cube_at ?cu ?cp))

		(at end (not (holding_cube ?v ?cu)))
		
		(at end (not (holding ?v)))
		(at end (not-holding ?v))
		)
)
)