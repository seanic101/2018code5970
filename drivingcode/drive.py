"""
Will be called each time that the robot needs to move during teleop
"""

def splitArcade(steering,power,squared_input):

    if squared_input:
        steer_multiplier = 1
        throttle_multiplier = 1
        if steering < 0:
            steer_multiplier = -1
        if power < 0:
            throttle_multiplier = -1
        steering = steering*steering*steer_multiplier
        power = power*power*throttle_multiplier
    """"""
    left_power = power - steering
    right_power = power + steering
    left_power *= -1
    return [left_power,right_power]

def splitArcadeConstantRadius(steering,power,squared_input):

    if squared_input:
        steer_multiplier = 1
        throttle_multiplier = 1
        if steering < 0:
            steer_multiplier = -1
        if power < 0:
            throttle_multiplier = -1
        steering = steering*steering*steer_multiplier
        power = power*power*throttle_multiplier

    max_power = power

    if steering == 0:
        left_power = max_power
        right_power = max_power
    elif steering < 0:
        right_power = max_power
        left_power = max_power*(1 + steering)
    else:
        left_power = max_power
        right_power = max_power*(1 - steering)
    left_power *= -1

    return [left_power,right_power]

def tankdrive(left,right):
    left *= -1
    return [left, right]