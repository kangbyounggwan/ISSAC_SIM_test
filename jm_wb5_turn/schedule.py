"""Pure tensor command schedule, testable without starting Isaac Sim."""


def scheduled_velocity(raw, group, elapsed):
    result = raw.clone()
    result[group == 0] = 0
    turning = (group == 1) | (group == 2)
    result[turning, :2] = 0
    result[(group == 1), 2] = raw[group == 1, 2].abs()
    result[(group == 2), 2] = -raw[group == 2, 2].abs()
    pause = turning & ((elapsed < 2.) | (elapsed >= 10.))
    result[pause] = 0
    return result
