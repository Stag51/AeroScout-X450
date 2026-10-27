import struct, sys, os

def bounds(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    pts = []
    if 84 + n * 50 == len(data):                       # binary STL
        for i in range(n):
            off = 84 + i * 50 + 12
            for v in range(3):
                pts.append(struct.unpack('<3f', data[off + v * 12: off + v * 12 + 12]))
    else:                                              # ASCII STL
        for line in data.decode(errors='ignore').splitlines():
            p = line.split()
            if len(p) == 4 and p[0] == 'vertex':
                pts.append(tuple(float(x) for x in p[1:]))
    return [(min(c), max(c)) for c in zip(*pts)]

d = os.path.expanduser('~/aeroscout_meshes')
for name in sys.argv[1:]:
    b = bounds(os.path.join(d, name))
    print(name, ' X %.1f..%.1f  Y %.1f..%.1f  Z %.1f..%.1f' % (b[0] + b[1] + b[2]))
