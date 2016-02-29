import bpy
import math
# from mathutils.geometry import intersect_line_line
import mathutils

tau = 2 * math.pi

# ϕ = (1 + √5) / 2 = 1.61803398875
phi = 1.61803398875

def dupObjectTo(i, location):
    origObj = bpy.context.scene.objects.active
    bpy.ops.object.duplicate(linked=False, mode='TRANSLATION')
    dupObj = bpy.context.scene.objects.active
#    dupObj.name = origObj.name + '.' + str(i)
    dupObj.location = location
    dupObj.select = False
    bpy.context.scene.objects.active = origObj
    origObj.select = True

# duplicate the active element and orient it so that it centered between two points
def dupBetween (location1, location2):
    origObj = bpy.context.scene.objects.active
    bpy.ops.object.duplicate(linked=False, mode='TRANSLATION')
    dupObj = bpy.context.scene.objects.active

    # move center between to points
    midpoint = ((location1[0]+location2[0]) / 2, # x
                (location1[1]+location2[1]) / 2, # y
                (location1[2]+location2[2]) / 2) # z
    dupObj.location = midpoint

    # http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    vector1 = mathutils.Vector((1,0,0))
    vector2 = mathutils.Vector((location2[0]-location1[0], location2[1]-location1[1], location2[2]-location1[2]))
    vector2.normalize()
    cos_theta = vector1.dot(vector2)
    half_cos = math.sqrt(0.5 * (1 + cos_theta))
    half_sin = math.sqrt(0.5 * (1 - cos_theta))
    w        = vector1.cross(vector2)
    w.normalize()
    oldRotMode = dupObj.rotation_mode
    dupObj.rotation_mode = 'QUATERNION'
    dupObj.rotation_quaternion = (half_cos, half_sin * w[0], half_sin * w[1], half_sin * w[2])
    dupObj.rotation_mode = oldRotMode

    dupObj.select = False
    bpy.context.scene.objects.active = origObj
    origObj.select = True

def calcVerts(a):
    s = a/2
    vertex = []
    # (±3ϕ, 0, ±1)
    vertex.append(((3*phi*s), 0, s))
    vertex.append(((3*phi*s), 0, -s))
    vertex.append((-(3*phi*s), 0, s))
    vertex.append((-(3*phi*s), 0, -s))

    # (±(1+2ϕ), ±ϕ, ±2)
    vertex.append(( (1+2*phi)*s,   phi*s,   2*s))
    vertex.append(( (1+2*phi)*s,   phi*s,  -2*s))
    vertex.append(( (1+2*phi)*s, -(phi*s),  2*s))
    vertex.append(( (1+2*phi)*s, -(phi*s), -2*s))
    vertex.append((-(1+2*phi)*s,   phi*s,   2*s))
    vertex.append((-(1+2*phi)*s,   phi*s,  -2*s))
    vertex.append((-(1+2*phi)*s, -(phi*s),  2*s))
    vertex.append((-(1+2*phi)*s, -(phi*s), -2*s))

    # (±(2+ϕ), ±2ϕ, ±1)
    vertex.append(( (2+phi)*s, 2*phi*s,  s))
    vertex.append(( (2+phi)*s, 2*phi*s, -s))
    vertex.append(( (2+phi)*s,-2*phi*s,  s))
    vertex.append(( (2+phi)*s,-2*phi*s, -s))
    vertex.append((-(2+phi)*s, 2*phi*s,  s))
    vertex.append((-(2+phi)*s, 2*phi*s, -s))
    vertex.append((-(2+phi)*s,-2*phi*s,  s))
    vertex.append((-(2+phi)*s,-2*phi*s, -s))


    # (±1, ±3ϕ, 0)
    vertex.append(( s,  (3*phi*s), 0))
    vertex.append(( s, -(3*phi*s), 0))
    vertex.append((-s,  (3*phi*s), 0))
    vertex.append((-s, -(3*phi*s), 0))

    # (±2, ±(1+2ϕ), ±ϕ)
    vertex.append(( 2*s, (1+2*phi)*s,   phi*s))
    vertex.append(( 2*s, (1+2*phi)*s,  -phi*s))
    vertex.append(( 2*s, -(1+2*phi)*s,  phi*s))
    vertex.append(( 2*s, -(1+2*phi)*s, -phi*s))
    vertex.append((-2*s, (1+2*phi)*s,   phi*s))
    vertex.append((-2*s, (1+2*phi)*s,  -phi*s))
    vertex.append((-2*s, -(1+2*phi)*s,  phi*s))
    vertex.append((-2*s, -(1+2*phi)*s, -phi*s))

    # (±1, ±(2+ϕ), ±2ϕ)
    vertex.append(( s, (2+phi)*s,  2*phi*s))
    vertex.append(( s, (2+phi)*s, -2*phi*s))
    vertex.append(( s,-(2+phi)*s,  2*phi*s))
    vertex.append(( s,-(2+phi)*s, -2*phi*s))
    vertex.append((-s, (2+phi)*s,  2*phi*s))
    vertex.append((-s, (2+phi)*s, -2*phi*s))
    vertex.append((-s,-(2+phi)*s,  2*phi*s))
    vertex.append((-s,-(2+phi)*s, -2*phi*s))

    # (0, ±1, ±3ϕ)
    vertex.append((0,  s,  3*phi*s))
    vertex.append((0,  s, -3*phi*s))
    vertex.append((0, -s,  3*phi*s))
    vertex.append((0, -s, -3*phi*s))

    # (±ϕ, ±2, ±(1+2ϕ))
    vertex.append(( phi*s,  2*s,  (1+2*phi)*s))
    vertex.append(( phi*s,  2*s, -(1+2*phi)*s))
    vertex.append(( phi*s, -2*s,  (1+2*phi)*s))
    vertex.append(( phi*s, -2*s, -(1+2*phi)*s))
    vertex.append((-phi*s,  2*s,  (1+2*phi)*s))
    vertex.append((-phi*s,  2*s, -(1+2*phi)*s))
    vertex.append((-phi*s, -2*s,  (1+2*phi)*s))
    vertex.append((-phi*s, -2*s, -(1+2*phi)*s))

    # (±2ϕ, ±1, ±(2+ϕ))
    vertex.append(( 2*phi*s,  s,  (2+phi)*s))
    vertex.append(( 2*phi*s,  s, -(2+phi)*s))
    vertex.append(( 2*phi*s, -s,  (2+phi)*s))
    vertex.append(( 2*phi*s, -s, -(2+phi)*s))
    vertex.append((-2*phi*s,  s,  (2+phi)*s))
    vertex.append((-2*phi*s,  s, -(2+phi)*s))
    vertex.append((-2*phi*s, -s,  (2+phi)*s))
    vertex.append((-2*phi*s, -s, -(2+phi)*s))

    return vertex

def domeVerts(vertex):
    vertex2 = [ vertex[42], vertex[46], vertex[34], vertex[38], vertex[50],
                vertex[40], vertex[54], vertex[26], vertex[30], vertex[58],
                vertex[44], vertex[52], vertex[ 6], vertex[14], vertex[21], vertex[23], vertex[18], vertex[10], vertex[56], vertex[48],
                vertex[32], vertex[ 4], vertex[ 0], vertex[15], vertex[27], vertex[31], vertex[19], vertex[ 2], vertex[ 8], vertex[36],
                vertex[24], vertex[12], vertex[ 1], vertex[ 7], vertex[35], vertex[39], vertex[11], vertex[ 4], vertex[16], vertex[28],
                vertex[20], vertex[13], vertex[ 5], vertex[55], vertex[47], vertex[51], vertex[59], vertex[ 9], vertex[17], vertex[22],
                vertex[25], vertex[53], vertex[43], vertex[57], vertex[29],
                vertex[33], vertex[45], vertex[41], vertex[49], vertex[37]
              ]
    return vertex2

def dupVerts(verts):
#    origVerts = calcVerts(edgeLength/3.28084)
#    verts = domeVerts(origVerts)
#    verts = origVerts
    for i in range(0, len(verts)):
        dupObjectTo(i, verts[i])

def dupEdges(verts):
    # face 1
    dupBetween(verts[0], verts[1])
    dupBetween(verts[1], verts[2])
    dupBetween(verts[2], verts[3])
    dupBetween(verts[3], verts[4])
    dupBetween(verts[4], verts[0])

def genMagetarium():
    # settings
    edgeLength = 1 # in feet
    verts = domeVerts(calcVerts(edgeLength/3.28084))
    # dupVerts(verts)
    dupEdges(verts)

if __name__ == "__main__":
    dupObject()

# r = 2.47801866 * a

# ϕ = (1 + √5) / 2 = 1.61803398875
# (0, ±1, ±3ϕ)
# (±2, ±(1+2ϕ), ±ϕ)
# (±1, ±(2+ϕ), ±2ϕ)

# X axis
# (±3ϕ, 0, ±1)      -- 4 ?
# (±(1+2ϕ), ±ϕ, ±2) -- 8 ?
# (±(2+ϕ), ±2ϕ, ±1) -- 8 ?

# Y axis
# (±1, ±3ϕ, 0)
# (±2, ±(1+2ϕ), ±ϕ)
# (±1, ±(2+ϕ), ±2ϕ)

# Z axis
# (0, ±1, ±3ϕ)
# (±ϕ, ±2, ±(1+2ϕ))
# (±2ϕ, ±1, ±(2+ϕ))
