(define (problem task)
(:domain ur5_e)
(:objects
    home input output - waypoint
    arm - robot
    c1 c2 c3 c4 - cube
    cin1 cin2 cin3 cin4 cout1 cout2 cout3 cout4 - cubepos
)
(:init
    (not-holding arm)
    (not-moving arm)
    (robot_at arm home)

    (cube_at c1 cin1)
    (cube_at c2 cin2)
    (cube_at c3 cin3)
    (cube_at c4 cin4)
)
(:goal (and
    (cube_at c1 cout1)
    (cube_at c2 cout2)
    (cube_at c3 cout3)
    (cube_at c4 cout4)
))
)
