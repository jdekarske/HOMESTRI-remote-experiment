(define (problem task)
(:domain ur5_e)
(:objects
    home input output - waypoint
    arm - robot
    b1 b2 b3 b4 b5 b6 - block
    bin1 bin2 bin3 bin4 bout1 bout2 bout3 bout4 - blockpos
)
(:init
    (not-holding arm)
    (not-moving arm)
    (robot_at arm home)

    (block_at b1 bin1)
    (block_at b2 bin2)
    (block_at b3 bin3)
    (block_at b4 bin4)
    (block_at b5 bout1)
    (block_at b6 bout2)
)
(:goal (and
    (block_at b1 bout1)
    (block_at b2 bout2)
    (block_at b3 bout3)
    (block_at b4 bout4)
    (block_at b5 bin1)
    (block_at b6 bin2)
))
)
