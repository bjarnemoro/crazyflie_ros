import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def plot_barrier_regions(B_list, labels=None, ax=None):
    """
    Plot regions defined by Ax <= B with
    A = [[1,0],[-1,0],[0,1],[0,-1]]

    B_list : list of iterable of length 4
    labels : optional list of strings
    ax     : optional matplotlib axis
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 6))

    for i, B in enumerate(B_list):
        b1, b2, b3, b4 = B

        xmin = -b2
        xmax =  b1
        ymin = -b4
        ymax =  b3

        width  = xmax - xmin
        height = ymax - ymin

        rect = Rectangle(
            (xmin, ymin),
            width,
            height,
            fill=False,
            linewidth=2
        )
        ax.add_patch(rect)

        if labels:
            ax.text(
                xmin + width / 2,
                ymin + height / 2,
                labels[i],
                ha='center',
                va='center'
            )

    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)

    plt.show()


B_vectors = [
    [1.0, 1.0, 0.33333333360571693, 1.666666666394283],
    [1.0, 1.0, 0.33333333358944783, 1.6666666664105523],
    [1.0, 1.0, 0.33333333383118524, 1.6666666661688148],
]

plot_barrier_regions(B_vectors)