import matplotlib.pyplot as plt

points = [[63, 90], [71, 42], [59, 4], [78, 34], [63, 49], [40, 31], [33, 14], [90, 91], [13, 99], [73, 34],
          [47, 32], [40, 18], [44, 3], [57, 23], [21, 12], [68, 79], [70, 2], [12, 61], [24, 3], [14, 11],
          [97, 61], [38, 100], [89, 43], [37, 88], [30, 84], [17, 81], [67, 52], [6, 45], [82, 70], [98, 46],
          [25, 58], [48, 59], [22, 3], [30, 15], [28, 64], [0, 2], [50, 44], [75, 54], [36, 41], [59, 27],
          [44, 25], [64, 2], [99, 71], [38, 32], [62, 42], [34, 85], [10, 100], [67, 51], [96, 69], [67, 7],
          [54, 62], [15, 17], [39, 4], [79, 86], [100, 23], [50, 37], [81, 38], [63, 50], [67, 21], [8, 1], [79, 49],
          [19, 26], [22, 21], [97, 79], [32, 37], [93, 97], [47, 47], [59, 18], [9, 0], [85, 68], [16, 18], [80, 95],
          [68, 31], [13, 32], [61, 57], [89, 46], [57, 10], [11, 61], [29, 35], [70, 14], [98, 41], [26, 36], [63, 34],
          [81, 65], [85, 40], [48, 10], [17, 82], [96, 79], [91, 69], [10, 64], [94, 53], [51, 60], [83, 97], [70, 73],
          [22, 69], [68, 85], [11, 97], [15, 17], [51, 30], [68, 72], [94, 64], [21, 90], [59, 87], [6, 28], [52, 43],
          [72, 59], [9, 76], [13, 65], [21, 37], [100, 63], [36, 19], [5, 90], [97, 23], [71, 70], [82, 41], [23, 43],
          [93, 87], [34, 84], [49, 22], [41, 86]]


def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if x >= 100 or x <= 0 or y <= 0 or y >= 100:
        return False
    for point in points:
        if (x - point[0]) ** 2 + (y - point[1]) ** 2 < 16:
            return False
    return True


def isStateValid2(state):
    x = state[0]
    y = state[1]
    for point in points:
        if (x - point[0]) ** 2 + (y - point[1]) ** 2 < 16:
            return False
    return True


def paint_obs(LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
    for point in points:
        circl = plt.Circle(point, 3, color='k')
        plt.gcf().gca().add_artist(circl)


if __name__ == '__main__':
    paint_obs(0, 100)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
