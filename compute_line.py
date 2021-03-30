"""
Provides two raycasting implementations:
    1. Fast Voxel Traversal Algorithm (FVTA)
    2. Bresenham's Line Drawing Algorithm

    If I have time, I should figure out how FVTA works...still getting some
    nonideal lines.
"""
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_line_low(x0,y0,x1,y1):
    plotted_points = []
    dx = x1 - x0
    dy = y1 - y0
    yi = 1
    if dy < 0:
        yi = -1
        dy = -dy
    D = (2 * dy) - dx
    y = y0
    for x in range(x0,x1):
        plotted_points.append((x,y))
        if D > 0:
            y = y + yi
            D = D + (2 * (dy - dx))
        else:
            D = D + 2*dy
    return plotted_points

def plot_line_high(x0,y0,x1,y1):
    plotted_points = []
    dx = x1 - x0
    dy = y1 - y0
    xi = 1
    if dx < 0:
        xi = -1
        dx = -dx
    D = (2 * dx) - dy
    x = x0
    for y in range(y0,y1):
        plotted_points.append((x,y))
        if D > 0:
            x = x + xi
            D = D + (2 * (dx - dy))
        else:
            D = D + 2*dx
    return plotted_points

def compute_bresenham_line(x0, y0, x1, y1):
    if abs(y1 - y0) < abs(x1 - x0):
        if x0 > x1:
            plotted_points = plot_line_low(x1,y1,x0,y0)
        else:
            plotted_points = plot_line_low(x0,y0,x1,y1)
    else:
        if y0 > y1:
            plotted_points = plot_line_high(x1,y1,x0,y0)
        else:
            plotted_points = plot_line_high(x0,y0,x1,y1)
    plotted_points.append((x1,y1))
    return plotted_points

def sign(n):
    if n > 0:
        return 1
    elif n < 0:
        return -1
    else:
        return 0

def frac0(x):
    return (x - math.floor(x))

def frac1(x):
    return (1 - x + math.floor(x))

def compute_FVTA_line(x0,y0,x1,y1):
    points = []
    dx = sign(x1 - x0)
    if (dx != 0):
        tDeltaX = min(dx / (x1 - x0), 10000000.0)
    else:
        tDeltaX = 10000000.0
    if (dx > 0):
        tMaxX = tDeltaX * frac1(x0)
    else:
        tMaxX = tDeltaX * frac0(x0)
    x = int(round(x0))

    dy = sign(y1 - y0)
    if (dy != 0):
        tDeltaY = min(dy / (y1 - y0), 10000000.0)
    else:
        tDeltaY = 10000000.0
    if (dy > 0):
        tMaxY = tDeltaY * frac1(y0)
    else:
        tMaxY = tDeltaY * frac0(y0)
    y = int(round(y0))

    while(True):
        points.append((x,y))
        if (tMaxX < tMaxY):
            tMaxX = tMaxX + tDeltaX
            x = x + dx
        else:
            tMaxY = tMaxY + tDeltaY
            y = y + dy
        if (tMaxX > 1 and tMaxY > 1):
            points.append((x,y)) # Without this one of the vehicles did not draw completely
            break
    return points


"""Visualize Line Drawing"""
if __name__ == '__main__':
    x0, y0 = 1, 1
    x1, y1 = 5, 7

    pts = compute_FVTA_line(x0,y0,x1,y1)
    print(pts)

    width = 15
    length = 15
    grid_size = 1

    def visualize_line(x0,y0,x1,y1, points, width, length, grid_size):
        fig, ax = plt.subplots()
        ax.plot([x0,x1], [y0,y1], color='green', linewidth=1, markersize=12)
        for point in points:
            plotted_grid_square = patches.Rectangle((point[0], point[1]), \
                                width=grid_size, height=grid_size, linewidth=1,
                                edgecolor='b', facecolor='blue', alpha=0.2)
            ax.add_patch(plotted_grid_square)
        plt.xlim([0, width])
        plt.ylim([0, length])
        plt.show()

    visualize_line(x0,y0,x1,y1, pts, width,length,grid_size)
