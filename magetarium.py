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
                vertex[32], vertex[ 3], vertex[ 0], vertex[15], vertex[27], vertex[31], vertex[19], vertex[ 2], vertex[ 8], vertex[36],
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

    # face 2
    dupBetween(verts[ 0], verts[ 5])
    dupBetween(verts[ 5], verts[10])
    dupBetween(verts[10], verts[11])
    dupBetween(verts[11], verts[ 6])
    dupBetween(verts[ 6], verts[ 1])

    # face 3
    dupBetween(verts[ 6], verts[12])
    dupBetween(verts[12], verts[13])
    dupBetween(verts[13], verts[ 7])
    dupBetween(verts[ 7], verts[ 2])

    # face 4
    dupBetween(verts[ 7], verts[14])
    dupBetween(verts[14], verts[15])
    dupBetween(verts[15], verts[ 8])
    dupBetween(verts[ 8], verts[ 3])

    # face 5
    dupBetween(verts[ 8], verts[16])
    dupBetween(verts[16], verts[17])
    dupBetween(verts[17], verts[ 9])
    dupBetween(verts[ 9], verts[ 4])

    # face 6
    dupBetween(verts[ 9], verts[18])
    dupBetween(verts[18], verts[19])
    dupBetween(verts[19], verts[ 5])

    # face 7
    dupBetween(verts[19], verts[29])
    dupBetween(verts[29], verts[20])
    dupBetween(verts[20], verts[10])

    # face 8
    dupBetween(verts[20], verts[30])
    dupBetween(verts[30], verts[31])
    dupBetween(verts[31], verts[37])
    dupBetween(verts[37], verts[11])

    # face 9
    dupBetween(verts[37], verts[22])
    dupBetween(verts[22], verts[12])

    # face 10
    dupBetween(verts[22], verts[32])
    dupBetween(verts[32], verts[33])
    dupBetween(verts[33], verts[23])
    dupBetween(verts[23], verts[13])

    # face 11
    dupBetween(verts[23], verts[24])
    dupBetween(verts[24], verts[14])

    # face 12
    dupBetween(verts[24], verts[34])
    dupBetween(verts[34], verts[35])
    dupBetween(verts[35], verts[25])
    dupBetween(verts[25], verts[15])

    # face 13
    dupBetween(verts[25], verts[26])
    dupBetween(verts[26], verts[16])

    # face 14
    dupBetween(verts[26], verts[36])
    dupBetween(verts[36], verts[21])
    dupBetween(verts[21], verts[27])
    dupBetween(verts[27], verts[17])

    # face 15
    dupBetween(verts[27], verts[28])
    dupBetween(verts[28], verts[18])

    # face 16
    dupBetween(verts[28], verts[38])
    dupBetween(verts[38], verts[39])
    dupBetween(verts[39], verts[29])

    # face 17
    dupBetween(verts[39], verts[49])
    dupBetween(verts[49], verts[40])
    dupBetween(verts[40], verts[30])

    # face 18
    dupBetween(verts[40], verts[50])
    dupBetween(verts[50], verts[41])
    dupBetween(verts[41], verts[31])

    # face 19
    dupBetween(verts[41], verts[42])
    dupBetween(verts[42], verts[32])

    # face 20
    dupBetween(verts[42], verts[51])
    dupBetween(verts[51], verts[43])
    dupBetween(verts[43], verts[33])

    # face 21
    dupBetween(verts[43], verts[44])
    dupBetween(verts[44], verts[34])

    # face 22
    dupBetween(verts[44], verts[52])
    dupBetween(verts[52], verts[45])
    dupBetween(verts[45], verts[35])

    # face 23
    dupBetween(verts[45], verts[46])
    dupBetween(verts[46], verts[36])

    # face 24
    dupBetween(verts[46], verts[53])
    dupBetween(verts[53], verts[47])
    dupBetween(verts[47], verts[21])

    # face 25
    dupBetween(verts[47], verts[48])
    dupBetween(verts[48], verts[38])

    # face 26
    dupBetween(verts[48], verts[54])
    dupBetween(verts[54], verts[49])

    # face 27
    dupBetween(verts[54], verts[59])
    dupBetween(verts[59], verts[55])
    dupBetween(verts[55], verts[50])

    # face 28
    dupBetween(verts[55], verts[56])
    dupBetween(verts[56], verts[51])

    # face 29
    dupBetween(verts[56], verts[57])
    dupBetween(verts[57], verts[52])

    # face 30
    dupBetween(verts[57], verts[58])
    dupBetween(verts[58], verts[53])

    # face 31
    dupBetween(verts[58], verts[59])

    # face 32

def genMagetarium():
    # settings
    edgeLength = 1 # in feet
    verts = domeVerts(calcVerts(edgeLength/3.28084))
    #dupVerts(verts)
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
