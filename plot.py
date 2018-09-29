from matplotlib import pyplot as plt

fig = plt.figure(figsize=(12, 8))


def parse_tuple(t):
    t = t.split(',')
    t = (float(t[0].replace('(', '')), float(t[1].replace(')', '')))
    return t


def remove_braces(t):
    return t.replace('(', '').replace(')', '')


raw_lines = []
path_data = []

with open("graph.txt", "r") as f:

    lines = f.readlines()

    data_found = False

    for line in lines:
        if line.startswith('XXXX'):
            data_found = not data_found
            continue
        line = line.replace('\n', '')
        if data_found:
            raw_lines += [line]
        else:
            path_data += [line]

raw_nodes = raw_lines[0].replace('NODELIST: ', '').replace(
    '[', '').replace(']', '').split(',')[:-1]
raw_nodes = [v.replace('(', '').replace(')', '') for v in raw_nodes]

nodes = []
for idx in range(0, len(raw_nodes), 2):
    nodes += [(float(raw_nodes[idx]), float(raw_nodes[idx+1]))]

x_vals = [v[0] for v in nodes]
y_vals = [v[1] for v in nodes]

plt.scatter(x_vals, y_vals, marker='x')

raw_edges = raw_lines[2:]
edges = []
for edge in raw_edges:
    source, destinations = edge.split(' ')
    if destinations != '-' and not source.startswith('OBSTACLES'):
        source = parse_tuple(source)
        destinations = destinations.replace(
            '[', '').replace(']', '').replace('),(', ')|(')
        destinations = [parse_tuple(t) for t in destinations.split('|')]
        for d in destinations:
            edges += [source] + [d]

for idx in range(0, len(edges), 2):
    x, dx = edges[idx][0], edges[idx+1][0] - edges[idx][0]
    y, dy = edges[idx][1], edges[idx+1][1] - edges[idx][1]
    plt.arrow(x, y, dx, dy, width=0.17)
    #  x1, x2 = edges[idx][0], edges[idx+1][0]
    #  y1, y2 = edges[idx][1], edges[idx+1][1]
    #  plt.plot([x1, x2], [y1, y2], color="black")

raw_circles = raw_lines[-1].replace('[', '').replace(']', '')
raw_circles = raw_circles.split(' ')[1].split(',')[:-1]

circles = []
for idx in range(0, len(raw_circles), 3):
    c1 = float(remove_braces(raw_circles[idx]))
    c2 = float(remove_braces(raw_circles[idx+1]))
    c3 = float(remove_braces(raw_circles[idx+2]))

    circles += [(c1, c2, c3)]

ax = plt.gca()
for c in circles:
    ax.add_artist(plt.Circle((c[0], c[1]), c[2], fill=False, color='red'))
    plt.scatter(c[0], c[1], marker='o', color='black')

path_data = path_data[1:]
path_data = [parse_tuple(p) for p in path_data]
x_vals = [p[0] for p in path_data]
y_vals = [p[1] for p in path_data]
plt.plot(x_vals, y_vals, linewidth=4, color='green', label='Optimal path')

plt.plot([50], [50], 'ro', label='Start')
plt.plot([-50], [-50], 'bo', label='Goal')
plt.legend()

plt.savefig("out.png", dpi=400)
plt.show()
