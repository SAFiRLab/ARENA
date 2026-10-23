"""OcTree binary files (.bt, OcTree::writeBinary format) of the real flight maps: reading and writing the leaves.
Shared by crop_octomap.py (moved from the removed scaffold map generator scripts)."""

import os

DEMOS = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..'))
RESSOURCES = os.path.join(DEMOS, 'ressources')
TREE_DEPTH = 16
TREE_MAX_KEY = 1 << (TREE_DEPTH - 1)  # Key of the center of the octree (octomap::OcTreeKey)


def read_bt(path):
    """Leaves of an OcTree binary file: [(min key (3,), size in keys, occupied)] and the resolution."""
    with open(path, 'rb') as f:
        resolution = None
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('res'):
                resolution = float(line.split()[1])
            if line == 'data':
                break
        data = f.read()
    leaves, position = [], [0]

    def read_node(min_key, size):
        bits = data[position[0]] | (data[position[0] + 1] << 8)
        position[0] += 2
        half = size // 2
        inner = []
        for i in range(8):
            code = (bits >> (2 * i)) & 3
            child_min = (min_key[0] + (i & 1) * half, min_key[1] + ((i >> 1) & 1) * half, min_key[2] + ((i >> 2) & 1) * half)
            if code in (1, 2):
                leaves.append((child_min, half, code == 2))
            elif code == 3:
                inner.append(child_min)
        for child_min in inner:
            read_node(child_min, half)

    read_node((0, 0, 0), 1 << TREE_DEPTH)
    return leaves, resolution


def write_bt(path, occupied, free, resolution):
    """OcTree::writeBinary format: header, then every node as 2 bytes (2 bits per child: 00 unknown, 01 free leaf,
    10 occupied leaf, 11 inner node, children 0-3 in the first byte), inner children written depth first."""
    nodes = []  # Bytes of the nodes in the writing order
    count = [1]

    def write_node(keys_occupied, keys_free, depth):
        bit = TREE_DEPTH - 1 - depth
        children = [([], []) for _ in range(8)]
        for index, keys in enumerate((keys_occupied, keys_free)):
            for key in keys:
                child = ((key[0] >> bit) & 1) | (((key[1] >> bit) & 1) << 1) | (((key[2] >> bit) & 1) << 2)
                children[child][index].append(key)
        bits, inner = 0, []
        for child, (child_occupied, child_free) in enumerate(children):
            if not child_occupied and not child_free:
                continue
            count[0] += 1
            if depth == TREE_DEPTH - 1:
                code = 2 if child_occupied else 1  # Leaf at the resolution
            else:
                code = 3
                inner.append((child_occupied, child_free))
            bits |= code << (2 * child)
        nodes.append(bytes((bits & 0xFF, bits >> 8)))
        for child_occupied, child_free in inner:
            write_node(child_occupied, child_free, depth + 1)

    write_node(list(occupied), list(free), 0)
    with open(path, 'wb') as f:
        f.write('# Octomap OcTree binary file\nid OcTree\nsize {}\nres {}\ndata\n'.format(count[0], resolution).encode())
        f.write(b''.join(nodes))
