"""Small dependency-free 2D copper connectivity checks, units millimetres.
No joins are inferred from net names, module internals, jumper pads or models.
Native KiCad DRC remains an independent mandatory gate.
"""
import math
EPS = 1e-6

def distance_point_segment(q, a, b):
    x, y = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((q[0]-a[0])*x+(q[1]-a[1])*y)/(x*x+y*y))) if x*x+y*y else 0.
    return math.hypot(q[0]-a[0]-t*x, q[1]-a[1]-t*y)

def orient(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

def edges(ring):
    return zip(ring, ring[1:]+ring[:1])

def segments_cross(a, b, c, d):
    o1, o2, o3, o4 = orient(a,b,c), orient(a,b,d), orient(c,d,a), orient(c,d,b)
    if o1*o2 < -EPS and o3*o4 < -EPS:
        return True
    return min(distance_point_segment(a,c,d),distance_point_segment(b,c,d),
               distance_point_segment(c,a,b),distance_point_segment(d,a,b)) <= EPS

def segment_distance(a, b, c, d):
    if segments_cross(a,b,c,d):
        return 0.
    return min(distance_point_segment(a,c,d),distance_point_segment(b,c,d),
               distance_point_segment(c,a,b),distance_point_segment(d,a,b))

def inside_ring(q, ring):
    yes = False
    for a,b in edges(ring):
        if distance_point_segment(q,a,b) <= EPS:
            return True
        if (a[1]>q[1]) != (b[1]>q[1]) and q[0] < (b[0]-a[0])*(q[1]-a[1])/(b[1]-a[1])+a[0]:
            yes = not yes
    return yes

def polygon_contains(q, shape):
    return inside_ring(q,shape['outer']) and not any(inside_ring(q,h) for h in shape.get('holes',[]))

def rings(shape):
    return [shape['outer']]+shape.get('holes',[])

def bounds(shape):
    if shape['kind']=='circle':
        x,y=shape['point'];r=shape['radius'];return x-r,y-r,x+r,y+r
    if shape['kind']=='segment':
        a,b,r=shape['a'],shape['b'],shape['radius'];return min(a[0],b[0])-r,min(a[1],b[1])-r,max(a[0],b[0])+r,max(a[1],b[1])+r
    q=shape['outer'];return min(x for x,y in q),min(y for x,y in q),max(x for x,y in q),max(y for x,y in q)

def boxes_overlap(a,b):
    return a[0]<=b[2]+EPS and b[0]<=a[2]+EPS and a[1]<=b[3]+EPS and b[1]<=a[3]+EPS

def touches(a,b):
    if not boxes_overlap(bounds(a),bounds(b)):
        return False
    if a['kind']=='polygon' and b['kind']!='polygon':
        return touches(b,a)
    if a['kind']=='circle':
        if b['kind']=='circle':return math.dist(a['point'],b['point']) <= a['radius']+b['radius']+EPS
        if b['kind']=='segment':return distance_point_segment(a['point'],b['a'],b['b']) <= a['radius']+b['radius']+EPS
        return polygon_contains(a['point'],b) or any(distance_point_segment(a['point'],c,d)<=a['radius']+EPS for ring in rings(b) for c,d in edges(ring))
    if a['kind']=='segment':
        if b['kind']=='circle':return touches(b,a)
        if b['kind']=='segment':return segment_distance(a['a'],a['b'],b['a'],b['b']) <= a['radius']+b['radius']+EPS
        return polygon_contains(a['a'],b) or polygon_contains(a['b'],b) or any(segment_distance(a['a'],a['b'],c,d)<=a['radius']+EPS for ring in rings(b) for c,d in edges(ring))
    if any(polygon_contains(q,b) for q in a['outer']) or any(polygon_contains(q,a) for q in b['outer']):return True
    return any(segments_cross(c,d,e,f) for ra in rings(a) for rb in rings(b) for c,d in edges(ra) for e,f in edges(rb))

def component_labels(shapes):
    parent=list(range(len(shapes)))
    def root(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,a in enumerate(shapes):
        for j in range(i):
            if touches(a,shapes[j]):parent[root(i)]=root(j)
    return [root(i) for i in range(len(shapes))]
