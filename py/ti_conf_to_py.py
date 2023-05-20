import sys
fn = sys.argv[1]
a = open(fn)
t = a.read()
d = {}
for l in t.splitlines():
	r = l.split("0x")[-1]
	a = int(r[0:2],16)
	v = int(r[2:], 16)
	d[a]=v
	#self.writeRegister(a, v)
print('conf =', d)
