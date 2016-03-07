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

# reorder vertexs to a more sensible numbering
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

def domeVerts_ccw(vertex):
    vertex2 = [ vertex[42], vertex[50], vertex[38], vertex[34], vertex[46],
                vertex[40], vertex[58], vertex[30], vertex[26], vertex[54],
                vertex[44], vertex[48], vertex[56], vertex[10], vertex[18], vertex[23], vertex[21], vertex[14], vertex[ 6], vertex[52],
                vertex[32], vertex[36], vertex[ 8], vertex[ 2], vertex[19], vertex[31], vertex[27], vertex[15], vertex[ 0], vertex[ 4],
                vertex[24], vertex[28], vertex[16], vertex[ 3], vertex[11], vertex[39], vertex[35], vertex[ 7], vertex[ 1], vertex[12],
                vertex[20], vertex[22], vertex[17], vertex[ 9], vertex[59], vertex[51], vertex[47], vertex[55], vertex[ 5], vertex[13],
                vertex[25], vertex[29], vertex[57], vertex[43], vertex[53],
                vertex[33], vertex[37], vertex[49], vertex[41], vertex[45],
              ]
    return vertex2

def dupVerts(verts):
#    origVerts = calcVerts(edgeLength/3.28084)
#    verts = domeVerts(origVerts)
#    verts = origVerts
    for i in range(0, len(verts)):
        dupObjectTo(i, verts[i])

# list of vertexs that form the edges
def faceList():
              # face 1
    faces = [ [ (0, 1),
                (1, 2),
                (2, 3),
                (3, 4),
                (4, 0)],
              # face 2
              [ (0, 5),
                (5, 11),
                (11, 12),
                (12, 6),
                (6,  1),
                (1,  0)],
              # face 3
              [ (1, 6),
                (6, 13),
                (13, 14),
                (14, 7),
                (7, 2),
                (2, 1)],
              # face 4
              [ (2, 7),
                (7, 15),
                (15, 16),
                (16, 8),
                (8, 3),
                (3, 2),
                (2, 7)],
              # face 5
              [ (3, 8),
                (8, 17),
                (17, 18),
                (18, 9),
                (9, 4),
                (4, 3)],
              # face 6
              [ (4, 9),
                (9, 19),
                (19, 10),
                (10, 5),
                (5, 0),
                (0, 4)],
              # face 7
              [ (5, 10),
                (10, 20),
                (20, 21),
                (21, 11),
                (11, 5)],
              # face 8
              [ (6, 12),
                (12, 22),
                (22, 23),
                (23, 13),
                (13, 6)],
              # face 9
              [ (7, 14),
                (14, 24),
                (24, 25),
                (25, 15),
                (15, 7)],
              # face 10
              [ (8, 16),
                (16, 26),
                (26, 27),
                (27, 17),
                (17, 8)],
              # face 11
              [ (9, 18),
                (18, 28),
                (28, 33),
                (33, 19),
                (19, 9)],
              # face 12
              [ (11, 21),
                (21, 31),
                (31, 32),
                (32, 22),
                (22, 12)],
              # face 13
              [ (13, 23),
                (23, 29),
                (29, 34),
                (34, 24),
                (24, 14),
                (14, 13)],
              # face 14
              [ (15, 25),
                (25, 35),
                (35, 36),
                (36, 26),
                (26, 16),
                (16, 15)],
              # face 15
              [ (17, 27),
                (27, 37),
                (37, 38),
                (38, 28),
                (28, 18),
                (18, 17)],
              # face 16
              [ (19, 33),
                (33, 39),
                (39, 30),
                (30, 20),
                (20, 10),
                (10, 19)],
              # face 17
              [ (20, 30),
                (30, 40),
                (40, 41),
                (41, 31),
                (31, 21),
                (21, 20)],
              # face 18
              [ (22, 32),
                (32, 42),
                (42, 43),
                (43, 29),
                (29, 23),
                (23, 22)],
              # face 19
              [ (24, 34),
                (34, 44),
                (44, 45),
                (45, 35),
                (35, 25),
                (25, 24)],
              # face 20
              [ (26, 36),
                (36, 46),
                (46, 47),
                (47, 37),
                (37, 27),
                (27, 26) ],
              # face 21
              [ (28, 38),
                (38, 48),
                (48, 49),
                (49, 39),
                (39, 33),
                (33, 28) ]

              # face 22

              ]
    return faces


def edgeList():
              # pentagram top
    edges = [ (0, 1),
              (1, 2),
              (2, 3),
              (3, 4),
              (4, 0),
              # top 5 verts
              (0, 5),
              (1, 6),
              (2, 7),
              (3, 8),
              (4, 9),
              # hexagon tops
              (5, 10),
              (5, 11),
              (6, 12),
              (6, 13),
              (7, 14),
              (7, 15),
              (8, 16),
              (8, 17),
              (9, 18),
              (9, 19),
              # hexagon bottoms
              (11, 12),
              (13, 14),
              (15, 16),
              (17, 18),
              (19, 10),
              # pentagon bottom sides
              (10, 20),
              (11, 21),
              (12, 22),
              (13, 23),
              (14, 24),
              (15, 25),
              (16, 26),
              (17, 27),
              (18, 28),
              (19, 29),
              # pentagon bottoms
              (20, 21),
              (22, 23),
              (24, 25),
              (26, 27),
              (28, 29),
              # hexagon side bottoms
              (20, 30),
              (21, 31),
              (22, 32),
              (23, 33),
              (24, 34),
              (25, 35),
              (26, 36),
              (27, 37),
              (28, 38),
              (29, 39),
              # hexagon bottoms
              (31, 32),
              (33, 34),
              (35, 36),
              (37, 38),
              (39, 30),
              # hexagon bottom sides
              (30, 40),
              (31, 41),
              (32, 42),
              (33, 43),
              (34, 44),
              (35, 45),
              (36, 46),
              (37, 47),
              (38, 48),
              (39, 49),
              # hexagon bottoms
              (40, 41),
              (42, 43),
              (44, 45),
              (46, 47),
              (48, 49)
    ]
    return edges

def dupEdges(verts, edges):
    for edge in edges:
        dupBetween(verts[edge[0]], verts[edge[1]])

def makeEdgesFromFaces (verts, faces):
    for face in faces:
        for edge in face:
            dupBetween(verts[edge[0]], verts[edge[1]])

def dupEdges_old(verts):
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
    edgeLength = 3 # in feet
    verts = domeVerts_ccw(calcVerts(edgeLength/3.28084))[:50]
    faces = faceList()
    edges = edgeList()

#    dupVerts(verts)
    dupEdges(verts, edges)
#    edgeList(verts, faces)

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
