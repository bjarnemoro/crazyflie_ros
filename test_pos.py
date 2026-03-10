import matplotlib.pyplot as plt

agents = {
    1: [1.6, 0.0, 0.3],
    2: [1.23, 1.03, 0.3],
    3: [0.28, 1.58, 0.3],
    4: [-0.80, 1.39, 0.3],
    5: [-1.50, 0.55, 0.3],
    6: [-1.50, -0.55, 0.3],
    7: [-0.80, -1.39, 0.3],
    8: [0.28, -1.58, 0.3],
    9: [1.23, -1.03, 0.3],
}

x = [v[0] for v in agents.values()]
y = [v[1] for v in agents.values()]

plt.figure(figsize=(6,6))
plt.scatter(x, y, s=100)

for i, (px, py, _) in agents.items():
    plt.text(px, py, f"{i}", fontsize=12, ha="center", va="bottom")

plt.axhline(0, linewidth=0.8)
plt.axvline(0, linewidth=0.8)

plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Agent Positions")

plt.show()