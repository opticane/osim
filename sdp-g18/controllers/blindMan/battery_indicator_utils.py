def battery_to_haptic(battery, intensity=1.0):
    '''
    Translate a battery level from [0-100] to
    haptic feedback. The haptic feedback is a
    cumulative indicator, i.e. the possible
    levels of feedback are:
    [1, 0, 0, 0, 0],
    [1, 1, 0, 0, 0],
    [1, 1, 1, 0, 0],
    [1, 1, 1, 1, 0], and
    [1, 1, 1, 1, 1].
    '''

    feedback = [0, 0, 0, 0, 0]
    thresholds = [90, 70, 50, 30, 10]

    for i in range(len(thresholds)):
        if thresholds[i] >= battery:
            feedback[i] = intensity
        else:
            break

    return feedback