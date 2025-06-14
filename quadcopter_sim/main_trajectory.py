import numpy as np

def get_main_trajectory():
    altitude = 6.0
    takeoff = [np.array([0, 0, z]) for z in np.linspace(0, altitude, 10)]
    hover = [np.array([0, 0, altitude])] * 100  # Shorter hover
    move1 = [np.array([x, x, altitude]) for x in np.linspace(0, 5, 30)]
    hover2 = [np.array([5, 5, altitude])] * 70
    move2 = [np.array([x, 5, altitude]) for x in np.linspace(5, -5, 30)]
    hover3 = [np.array([-5, 5, altitude])] * 70
    move3 = [np.array([x, y, altitude]) for x, y in zip(np.linspace(-5, 0, 30), np.linspace(5, 0, 30))]
    hover4 = [np.array([0, 0, altitude])] * 50
    wps = takeoff + hover + move1 + hover2 + move2 + hover3 + move3 + hover4
    return wps
