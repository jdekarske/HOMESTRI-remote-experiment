 
(define (domain ur5_e)
;   negative preconditions not supported in rosplan
(:requirements :strips :typing :fluents :durative-actions :negative-preconditions)

(:types waypoint robot block - object
	blockpos - waypoint)
  
(:predicates
    (robot_at ?v - robot ?wp - waypoint)
	(block_at ?bl - block ?bp - blockpos)
	(holding_block ?v - robot ?b - block)
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
	:parameters (?v - robot ?bl - block ?bp - blockpos)
	:duration ( = ?duration 5)
	:condition (and 
		(at start (robot_at ?v ?bp))
		(at start (block_at ?bl ?bp))
		(at start (not-holding ?v))
		)
	:effect (and
		(at end (holding ?v))
		(at end (not (not-holding ?v)))
		(at end (holding_block ?v ?bl))
		)
)

(:durative-action drop
	:parameters (?v - robot ?bl - block ?bp - blockpos)
	:duration ( = ?duration 5)
	:condition (and 
		(at start (robot_at ?v ?bp))
		(at start (holding ?v))
		(at start (holding_block ?v ?bl))
		; add a can_move over all here
		)
	:effect (and
		(at end (not (holding ?v)))
		(at end (not-holding ?v))
		(at end (not (holding_block ?v ?bl)))
		(at end (block_at ?bl ?bp))
		)
)
)