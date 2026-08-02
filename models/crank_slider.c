#include "harness.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Units and frame
//
// Every constant below is in metres. Vert() scales by UNIT, so one world unit is 100 mm and a grid square reads as 10 cm.
// y runs up from the bench the rig stands on, x along the crankshaft, z across it. The crank axis is at y = AXIS_Y so the baseplate sits on the grid rather than through it.
//
// This is a slider-crank demonstration rig, not a copy of a particular engine, so there is no reference photograph to be measured against: the dimensions below define the mechanism rather than record one.
// What is falsifiable here is the kinematics, and that is checked by construction in CheckJoints().
// ---------------------------------------------------------------------------

#define UNIT     10.0f

#define AXIS_Y   0.100f    // crank axis height above the baseplate underside
#define CRANK_R  0.0425f   // crank throw: half of an 85 mm stroke
#define ROD_L    0.1440f   // connecting rod, big-end centre to small-end centre
#define PISTON_R 0.0430f   // piston outside radius, an 86 mm bore
#define CYCLE    2.4f      // seconds per crankshaft revolution

// ---------------------------------------------------------------------------
// Mesh builder
// ---------------------------------------------------------------------------

typedef struct {
    float *vertices;
    float *normals;
    float *texcoords;
    unsigned short *indices;
    int vertexCount;
    int triangleCount;
    int vertexCap;
    int triangleCap;
} Builder;

static void Reserve(Builder *b, int verts, int tris)
{
    if (b->vertexCount + verts > b->vertexCap) {
        int cap = (b->vertexCap > 0) ? b->vertexCap : 256;
        while (cap < b->vertexCount + verts) cap *= 2;
        b->vertices = (float *)MemRealloc(b->vertices, (size_t)cap * 3 * sizeof(float));
        b->normals = (float *)MemRealloc(b->normals, (size_t)cap * 3 * sizeof(float));
        b->texcoords = (float *)MemRealloc(b->texcoords, (size_t)cap * 2 * sizeof(float));
        b->vertexCap = cap;
    }
    if (b->triangleCount + tris > b->triangleCap) {
        int cap = (b->triangleCap > 0) ? b->triangleCap : 512;
        while (cap < b->triangleCount + tris) cap *= 2;
        b->indices = (unsigned short *)MemRealloc(b->indices, (size_t)cap * 3 * sizeof(unsigned short));
        b->triangleCap = cap;
    }
}

static int Vert(Builder *b, Vector3 p, Vector3 n)
{
    Reserve(b, 1, 0);
    int i = b->vertexCount++;
    b->vertices[i * 3 + 0] = p.x * UNIT;
    b->vertices[i * 3 + 1] = p.y * UNIT;
    b->vertices[i * 3 + 2] = p.z * UNIT;
    b->normals[i * 3 + 0] = n.x;
    b->normals[i * 3 + 1] = n.y;
    b->normals[i * 3 + 2] = n.z;
    b->texcoords[i * 2 + 0] = 0.0f;
    b->texcoords[i * 2 + 1] = 0.0f;
    return i;
}

static void Tri(Builder *b, int a, int c, int d)
{
    Reserve(b, 0, 1);
    int i = b->triangleCount++;
    b->indices[i * 3 + 0] = (unsigned short)a;
    b->indices[i * 3 + 1] = (unsigned short)c;
    b->indices[i * 3 + 2] = (unsigned short)d;
}

static void QuadN(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,
                  Vector3 n0, Vector3 n1, Vector3 n2, Vector3 n3)
{
    int a = Vert(b, p0, n0), c = Vert(b, p1, n1), d = Vert(b, p2, n2), e = Vert(b, p3, n3);
    Tri(b, a, c, d);
    Tri(b, a, d, e);
}

// Flat-shaded quad. Winding p0-p1-p2-p3 must be counter-clockwise seen from outside.
static void Quad(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(p1, p0), Vector3Subtract(p2, p0)));
    QuadN(b, p0, p1, p2, p3, n, n, n, n);
}

// c[0..3] bottom face in (-x-z, +x-z, +x+z, -x+z) order, c[4..7] the matching top.
static void Hex(Builder *b, const Vector3 c[8])
{
    Quad(b, c[0], c[1], c[2], c[3]);
    Quad(b, c[7], c[6], c[5], c[4]);
    Quad(b, c[0], c[4], c[5], c[1]);
    Quad(b, c[1], c[5], c[6], c[2]);
    Quad(b, c[2], c[6], c[7], c[3]);
    Quad(b, c[3], c[7], c[4], c[0]);
}

static void Box(Builder *b, float x0, float x1, float y0, float y1, float z0, float z1)
{
    Vector3 c[8] = {
        { x0, y0, z0 }, { x1, y0, z0 }, { x1, y0, z1 }, { x0, y0, z1 },
        { x0, y1, z0 }, { x1, y1, z0 }, { x1, y1, z1 }, { x0, y1, z1 },
    };
    Hex(b, c);
}

// Box whose cross-section tapers linearly from one end to the other along y.
static void Taper(Builder *b, float y0, float y1, float hx0, float hz0, float hx1, float hz1)
{
    Vector3 c[8] = {
        { -hx0, y0, -hz0 }, { hx0, y0, -hz0 }, { hx0, y0, hz0 }, { -hx0, y0, hz0 },
        { -hx1, y1, -hz1 }, { hx1, y1, -hz1 }, { hx1, y1, hz1 }, { -hx1, y1, hz1 },
    };
    Hex(b, c);
}

// Circular tube or cone frustum from a to b. Winding follows the sweep so front faces point outwards.
static void Tube(Builder *bd, Vector3 a, Vector3 b, float r0, float r1, int sides, bool capA, bool capB)
{
    Vector3 dir = Vector3Subtract(b, a);
    float len = Vector3Length(dir);
    if (len < 1e-6f) return;
    dir = Vector3Scale(dir, 1.0f / len);

    Vector3 ref = (fabsf(dir.y) < 0.9f) ? (Vector3){ 0, 1, 0 } : (Vector3){ 1, 0, 0 };
    Vector3 u = Vector3Normalize(Vector3CrossProduct(ref, dir));
    Vector3 v = Vector3CrossProduct(dir, u);
    float slant = (r0 - r1) / len;

    for (int j = 0; j < sides; j++) {
        float t0 = 2.0f * PI * (float)j / (float)sides;
        float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
        Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
        Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
        Vector3 n0 = Vector3Normalize(Vector3Add(d0, Vector3Scale(dir, slant)));
        Vector3 n1 = Vector3Normalize(Vector3Add(d1, Vector3Scale(dir, slant)));

        QuadN(bd,
              Vector3Add(a, Vector3Scale(d0, r0)), Vector3Add(a, Vector3Scale(d1, r0)),
              Vector3Add(b, Vector3Scale(d1, r1)), Vector3Add(b, Vector3Scale(d0, r1)),
              n0, n1, n1, n0);
    }

    if (capA && r0 > 1e-6f) {
        Vector3 n = Vector3Negate(dir);
        int centre = Vert(bd, a, n);
        for (int j = 0; j < sides; j++) {
            float t0 = 2.0f * PI * (float)j / (float)sides;
            float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
            Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
            Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
            int p1 = Vert(bd, Vector3Add(a, Vector3Scale(d1, r0)), n);
            int p0 = Vert(bd, Vector3Add(a, Vector3Scale(d0, r0)), n);
            Tri(bd, centre, p1, p0);
        }
    }
    if (capB && r1 > 1e-6f) {
        int centre = Vert(bd, b, dir);
        for (int j = 0; j < sides; j++) {
            float t0 = 2.0f * PI * (float)j / (float)sides;
            float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
            Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
            Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
            int p0 = Vert(bd, Vector3Add(b, Vector3Scale(d0, r1)), dir);
            int p1 = Vert(bd, Vector3Add(b, Vector3Scale(d1, r1)), dir);
            Tri(bd, centre, p0, p1);
        }
    }
}

// Surface of revolution about the Y axis. Each profile point is (height, radius) and the profile is walked in order, so a closed outline comes out solid: normals are taken from the profile tangent and therefore flip with it, which is what lets one profile carry the outside, the top, the bore and the underside.
// The ring is swept from +z toward +x, the right-handed sense about +y, so the winding matches Tube's.
static void RevolveY(Builder *b, const Vector2 *prof, int n, int segments)
{
    for (int i = 0; i < n - 1; i++) {
        Vector2 p0 = prof[i], p1 = prof[i + 1];
        Vector2 t = { p1.x - p0.x, p1.y - p0.y };
        if (fabsf(t.x) < 1e-9f && fabsf(t.y) < 1e-9f) continue;
        Vector2 nl = Vector2Normalize((Vector2){ -t.y, t.x });

        for (int j = 0; j < segments; j++) {
            float a0 = 2.0f * PI * (float)j / (float)segments;
            float a1 = 2.0f * PI * (float)(j + 1) / (float)segments;
            float s0 = sinf(a0), c0 = cosf(a0), s1 = sinf(a1), c1 = cosf(a1);

            Vector3 v00 = { p0.y * s0, p0.x, p0.y * c0 };
            Vector3 v01 = { p0.y * s1, p0.x, p0.y * c1 };
            Vector3 v11 = { p1.y * s1, p1.x, p1.y * c1 };
            Vector3 v10 = { p1.y * s0, p1.x, p1.y * c0 };

            Vector3 n00 = { nl.y * s0, nl.x, nl.y * c0 };
            Vector3 n01 = { nl.y * s1, nl.x, nl.y * c1 };

            QuadN(b, v00, v01, v11, v10, n00, n01, n01, n00);
        }
    }
}

// ---------------------------------------------------------------------------
// Materials and nodes
//
// A node is one rigid body: its geometry is authored in its OWN frame, about its own pivot, and its pose is a matrix.
// Nothing here is baked into world space, so the same mesh is correct at every crank angle and the pose lives in exactly one place.
// lighting.fs gamma-corrects with pow(c, 1/2.2), so these read considerably lighter than the raw values once shaded.
// ---------------------------------------------------------------------------

typedef enum { MAT_CASE, MAT_STEEL, MAT_BRIGHT, MAT_MARK, MAT_COUNT } MatId;

static const Color MAT_COLOR[MAT_COUNT] = {
    [MAT_CASE]   = {  38,  41,  46, 255 },
    [MAT_STEEL]  = {  74,  78,  84, 255 },
    [MAT_BRIGHT] = { 116, 121, 128, 255 },
    [MAT_MARK]   = { 138,  30,  26, 255 },
};

typedef struct {
    const char *name;
    Builder b[MAT_COUNT];
    Model model[MAT_COUNT];
    bool has[MAT_COUNT];
    BoundingBox local;   // in the node's own frame, from the built mesh
    BoundingBox swept;   // the union of local over every pose in one cycle, in world space
    Matrix xform;        // current pose, rebuilt every frame by Update
} Node;

static Node gFrame, gCrank, gFlywheel, gRod, gPiston;

static void NodeFinish(Node *n)
{
    bool any = false;
    for (int m = 0; m < MAT_COUNT; m++) {
        Builder *b = &n->b[m];
        if (b->vertexCount == 0) continue;
        if (b->vertexCount > 65535) {
            TraceLog(LOG_ERROR, "%s: material %d has %d vertices, over the 65535 index limit",
                     n->name, m, b->vertexCount);
        }

        Mesh mesh = { 0 };
        mesh.vertexCount = b->vertexCount;
        mesh.triangleCount = b->triangleCount;
        mesh.vertices = b->vertices;
        mesh.normals = b->normals;
        mesh.texcoords = b->texcoords;
        mesh.indices = b->indices;
        UploadMesh(&mesh, false);

        n->model[m] = LoadModelFromMesh(mesh);
        n->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].color = MAT_COLOR[m];
        HarnessApplyLighting(&n->model[m]);
        n->has[m] = true;

        BoundingBox bb = GetModelBoundingBox(n->model[m]);
        if (!any) { n->local = bb; any = true; }
        else {
            n->local.min = Vector3Min(n->local.min, bb.min);
            n->local.max = Vector3Max(n->local.max, bb.max);
        }
    }
    n->xform = MatrixIdentity();
}

static void NodeDraw(Node *n)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (!n->has[m]) continue;
        n->model[m].transform = n->xform;
        DrawModel(n->model[m], Vector3Zero(), 1.0f, WHITE);
    }
}

static void NodeUnload(Node *n)
{
    for (int m = 0; m < MAT_COUNT; m++) if (n->has[m]) UnloadModel(n->model[m]);
}

// ---------------------------------------------------------------------------
// Frame: baseplate, two standards, the main bearing bosses, the guide cage and the top deck.
// The only node whose own frame is the world frame, because it never moves.
// ---------------------------------------------------------------------------

#define STAND_IN   0.062f
#define STAND_OUT  0.080f
#define DECK_Y0    0.325f
#define GUIDE_R    0.0475f  // guide-cage bore, 4.5 mm clear of the piston

static void BuildFrame(void)
{
    Node *n = &gFrame;
    Builder *cast = &n->b[MAT_CASE];
    Builder *steel = &n->b[MAT_STEEL];

    Box(cast, -0.100f, 0.140f, 0.000f, 0.018f, -0.070f, 0.070f);

    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        float xi = sign * STAND_IN, xo = sign * STAND_OUT;
        Box(cast, fminf(xi, xo), fmaxf(xi, xo), 0.018f, DECK_Y0 + 0.020f, -0.050f, 0.050f);
        // Main bearing boss, buried 18 mm into the standard so the two never meet on a coincident plane.
        Tube(cast, (Vector3){ sign * 0.044f, AXIS_Y, 0.0f }, (Vector3){ sign * 0.080f, AXIS_Y, 0.0f },
             0.032f, 0.032f, 24, true, false);
    }

    Box(cast, -STAND_OUT, STAND_OUT, DECK_Y0, DECK_Y0 + 0.020f, -0.050f, 0.050f);

    // Guide cage: two rings on four bars.
    // The piston is 62 mm long against an 85 mm stroke, so no single ring can stay engaged over the whole cycle; two rings 65 mm apart keep at least one engaged at both dead centres, and leaving the sides open is what lets the piston be seen at all.
    const Vector2 ringLo[] = {
        { 0.215f, 0.062f }, { 0.235f, 0.062f }, { 0.235f, GUIDE_R }, { 0.215f, GUIDE_R }, { 0.215f, 0.062f },
    };
    const Vector2 ringHi[] = {
        { 0.280f, 0.062f }, { 0.300f, 0.062f }, { 0.300f, GUIDE_R }, { 0.280f, GUIDE_R }, { 0.280f, 0.062f },
    };
    RevolveY(steel, ringLo, 5, 32);
    RevolveY(steel, ringHi, 5, 32);

    for (int j = 0; j < 4; j++) {
        float a = PI * 0.25f + PI * 0.5f * (float)j;
        float bx = 0.0545f * sinf(a), bz = 0.0545f * cosf(a);
        Tube(steel, (Vector3){ bx, 0.215f, bz }, (Vector3){ bx, 0.300f, bz }, 0.006f, 0.006f, 10, true, true);
    }

    // Cage supports: the lower ring is tied to both standards, the upper ring up to the deck.
    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        Box(steel, fminf(sign * 0.052f, sign * 0.072f), fmaxf(sign * 0.052f, sign * 0.072f),
            0.219f, 0.231f, -0.014f, 0.014f);
        Box(steel, fminf(sign * 0.052f, sign * 0.062f), fmaxf(sign * 0.052f, sign * 0.062f),
            0.295f, DECK_Y0 + 0.010f, -0.010f, 0.010f);
    }
}

// ---------------------------------------------------------------------------
// Crankshaft: local origin ON the axis of rotation, so the pose is a bare rotation about local x.
// The crankpin sits at local (0, CRANK_R, 0), which is the definition the rod's placement reads back.
// ---------------------------------------------------------------------------

static void BuildCrank(void)
{
    Node *n = &gCrank;
    Builder *steel = &n->b[MAT_STEEL];

    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        Tube(steel, (Vector3){ sign * 0.032f, 0.0f, 0.0f }, (Vector3){ sign * 0.090f, 0.0f, 0.0f },
             0.024f, 0.024f, 24, false, true);
        // Web carrying the pin above the axis and the counterweight below it.
        Box(steel, fminf(sign * 0.032f, sign * 0.050f), fmaxf(sign * 0.032f, sign * 0.050f),
            -0.058f, 0.062f, -0.026f, 0.026f);
        Box(steel, fminf(sign * 0.032f, sign * 0.050f), fmaxf(sign * 0.032f, sign * 0.050f),
            -0.058f, -0.012f, -0.044f, 0.044f);
    }

    Tube(steel, (Vector3){ -0.036f, CRANK_R, 0.0f }, (Vector3){ 0.036f, CRANK_R, 0.0f },
         0.020f, 0.020f, 24, false, false);
}

// The flywheel is a second node on the same pivot as the crank: it proves that a shared parent pose is one matrix reused, not a second copy of the placement written out by hand.
static void BuildFlywheel(void)
{
    Node *n = &gFlywheel;
    Builder *steel = &n->b[MAT_STEEL];
    Builder *mark = &n->b[MAT_MARK];

    Tube(steel, (Vector3){ 0.078f, 0.0f, 0.0f }, (Vector3){ 0.104f, 0.0f, 0.0f }, 0.030f, 0.030f, 24, false, true);
    Tube(steel, (Vector3){ 0.090f, 0.0f, 0.0f }, (Vector3){ 0.104f, 0.0f, 0.0f }, 0.074f, 0.074f, 40, true, true);
    Tube(steel, (Vector3){ 0.086f, 0.0f, 0.0f }, (Vector3){ 0.108f, 0.0f, 0.0f }, 0.074f, 0.074f, 40, true, true);

    // Timing mark.
    // A plain disc turning about its own axis is invisible in a still; this is the only thing in the model that tells a reviewer which frame is which.
    Box(mark, 0.086f, 0.108f, 0.058f, 0.078f, -0.007f, 0.007f);
}

// ---------------------------------------------------------------------------
// Connecting rod: local origin at the BIG-END centre, shank along local +y, small end at local (0, ROD_L, 0).
// Both joints are therefore points the pose code can name, which is what makes the constraint checkable.
// ---------------------------------------------------------------------------

static void BuildRod(void)
{
    Node *n = &gRod;
    Builder *bright = &n->b[MAT_BRIGHT];
    Builder *steel = &n->b[MAT_STEEL];

    Tube(bright, (Vector3){ -0.024f, 0.0f, 0.0f }, (Vector3){ 0.024f, 0.0f, 0.0f }, 0.030f, 0.030f, 28, true, true);
    Tube(bright, (Vector3){ -0.018f, ROD_L, 0.0f }, (Vector3){ 0.018f, ROD_L, 0.0f }, 0.018f, 0.018f, 24, true, true);
    Taper(bright, 0.022f, ROD_L - 0.014f, 0.016f, 0.013f, 0.011f, 0.009f);

    // Big-end cap bolts, across the split line the cap would be parted on.
    for (int s = 0; s < 2; s++) {
        float z = (s == 0) ? -0.021f : 0.021f;
        Tube(steel, (Vector3){ -0.026f, -0.014f, z }, (Vector3){ 0.026f, -0.014f, z }, 0.005f, 0.005f, 8, true, true);
    }
}

// ---------------------------------------------------------------------------
// Piston: local origin at the GUDGEON-PIN centre, which is the joint it shares with the rod's small end.
// The skirt is one closed profile revolved about local y, so it has a real wall thickness and is not an open tube seen from inside at bottom dead centre.
// ---------------------------------------------------------------------------

static void BuildPiston(void)
{
    Node *n = &gPiston;
    Builder *bright = &n->b[MAT_BRIGHT];
    Builder *steel = &n->b[MAT_STEEL];

    const float R = PISTON_R, G = PISTON_R - 0.0025f;
    const Vector2 prof[] = {
        { -0.034f, R },      { 0.012f, R },
        { 0.0125f, G },      { 0.0155f, G },   { 0.016f, R },
        { 0.020f, R },
        { 0.0205f, G },      { 0.0235f, G },   { 0.024f, R },
        { 0.028f, R },       { 0.028f, 0.0f },
        { 0.020f, 0.0f },    { 0.020f, 0.036f },
        { -0.034f, 0.036f }, { -0.034f, R },
    };
    RevolveY(bright, prof, (int)(sizeof(prof) / sizeof(prof[0])), 40);

    // Gudgeon pin, deliberately left proud of the skirt with a collar at each end: on a real piston it is hidden inside the bosses, and hiding it here would hide the one joint this model exists to show.
    Tube(steel, (Vector3){ -0.049f, 0.0f, 0.0f }, (Vector3){ 0.049f, 0.0f, 0.0f }, 0.011f, 0.011f, 20, true, true);
    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        Tube(steel, (Vector3){ sign * 0.046f, 0.0f, 0.0f }, (Vector3){ sign * 0.053f, 0.0f, 0.0f },
             0.015f, 0.015f, 16, true, true);
    }
}

// ---------------------------------------------------------------------------
// Pose
//
// One crank angle drives everything. The piston pin height comes from the loop-closure equation
//     y(a) = R cos a + sqrt(L*L - R*R sin(a)*sin(a))
// and the rod angle from the same triangle, so the rod's ends land exactly on the crankpin and the piston pin at every angle rather than approximately.
// CheckJoints() measures that rather than asserting it.
// ---------------------------------------------------------------------------

static float gTheta;

static void Update(float t)
{
    float theta = 2.0f * PI * t / CYCLE;
    gTheta = theta;

    float rs = CRANK_R * sinf(theta);
    float rc = CRANK_R * cosf(theta);
    float axial = sqrtf(ROD_L * ROD_L - rs * rs);   // rod's component along the bore axis
    float yp = rc + axial;                          // piston pin height above the crank axis
    float phi = atan2f(-rs, axial);                 // rod tilt, positive about local x

    gFrame.xform = MatrixIdentity();
    gCrank.xform = MatrixMultiply(MatrixRotateX(theta), MatrixTranslate(0.0f, AXIS_Y * UNIT, 0.0f));
    gFlywheel.xform = gCrank.xform;
    gRod.xform = MatrixMultiply(MatrixRotateX(phi),
                                MatrixTranslate(0.0f, (AXIS_Y + rc) * UNIT, rs * UNIT));
    gPiston.xform = MatrixTranslate(0.0f, (AXIS_Y + yp) * UNIT, 0.0f);
}

// The two revolute joints are the only claims this model makes that can be wrong without looking wrong, so they are measured over a full revolution at build time instead of being argued for in a comment.
static void CheckJoints(void)
{
    const int steps = 720;
    float worstBig = 0.0f, worstSmall = 0.0f;

    for (int i = 0; i < steps; i++) {
        Update(CYCLE * (float)i / (float)steps);

        Vector3 crankpin = Vector3Transform((Vector3){ 0.0f, CRANK_R * UNIT, 0.0f }, gCrank.xform);
        Vector3 bigEnd = Vector3Transform(Vector3Zero(), gRod.xform);
        Vector3 smallEnd = Vector3Transform((Vector3){ 0.0f, ROD_L * UNIT, 0.0f }, gRod.xform);
        Vector3 pistonPin = Vector3Transform(Vector3Zero(), gPiston.xform);

        float eb = Vector3Distance(crankpin, bigEnd);
        float es = Vector3Distance(smallEnd, pistonPin);
        if (eb > worstBig) worstBig = eb;
        if (es > worstSmall) worstSmall = es;
    }

    // World units are 100 mm, so this reports the joint slack in millimetres.
    float bigMm = worstBig * 100.0f, smallMm = worstSmall * 100.0f;
    if (bigMm > 0.01f || smallMm > 0.01f) {
        TraceLog(LOG_WARNING, "crank_slider: joints separate by up to %.4f mm (big end) and %.4f mm (small end)",
                 bigMm, smallMm);
    } else {
        TraceLog(LOG_INFO, "crank_slider: joints hold to %.5f mm (big end) and %.5f mm (small end) over %d steps",
                 bigMm, smallMm, steps);
    }
}

// A part that moves has no single bounding box, so --part frames the union over the whole cycle.
// Sampling the pose beats asking raylib: GetModelBoundingBox transforms only the box's min and max corner, which is wrong under a rotation (vendor/raylib/src/rmodels.c, "does not support rotation transformations").
static void SweepBounds(void)
{
    Node *nodes[] = { &gFrame, &gCrank, &gFlywheel, &gRod, &gPiston };
    const int steps = 64;

    for (size_t k = 0; k < sizeof(nodes) / sizeof(nodes[0]); k++) nodes[k]->swept = (BoundingBox){ 0 };

    for (int i = 0; i <= steps; i++) {
        Update(CYCLE * (float)i / (float)steps);
        for (size_t k = 0; k < sizeof(nodes) / sizeof(nodes[0]); k++) {
            Node *n = nodes[k];
            BoundingBox l = n->local;
            for (int c = 0; c < 8; c++) {
                Vector3 corner = {
                    (c & 1) ? l.max.x : l.min.x,
                    (c & 2) ? l.max.y : l.min.y,
                    (c & 4) ? l.max.z : l.min.z,
                };
                Vector3 w = Vector3Transform(corner, n->xform);
                if (i == 0 && c == 0) { n->swept.min = w; n->swept.max = w; }
                else {
                    n->swept.min = Vector3Min(n->swept.min, w);
                    n->swept.max = Vector3Max(n->swept.max, w);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

static void Init(void)
{
    gFrame.name = "frame";
    gCrank.name = "crankshaft";
    gFlywheel.name = "flywheel";
    gRod.name = "conrod";
    gPiston.name = "piston";

    BuildFrame();
    BuildCrank();
    BuildFlywheel();
    BuildRod();
    BuildPiston();

    NodeFinish(&gFrame);
    NodeFinish(&gCrank);
    NodeFinish(&gFlywheel);
    NodeFinish(&gRod);
    NodeFinish(&gPiston);

    CheckJoints();
    SweepBounds();
    Update(0.0f);
}

static void Unload(void)
{
    NodeUnload(&gFrame);
    NodeUnload(&gCrank);
    NodeUnload(&gFlywheel);
    NodeUnload(&gRod);
    NodeUnload(&gPiston);
}

static void DrawFrame(void) { NodeDraw(&gFrame); }
static void DrawCrank(void) { NodeDraw(&gCrank); }
static void DrawFlywheel(void) { NodeDraw(&gFlywheel); }
static void DrawRod(void) { NodeDraw(&gRod); }
static void DrawPiston(void) { NodeDraw(&gPiston); }

static BoundingBox FrameBounds_(void) { return gFrame.swept; }
static BoundingBox CrankBounds(void) { return gCrank.swept; }
static BoundingBox FlywheelBounds(void) { return gFlywheel.swept; }
static BoundingBox RodBounds(void) { return gRod.swept; }
static BoundingBox PistonBounds(void) { return gPiston.swept; }

static const Part PARTS[] = {
    { .name = "frame", .draw = DrawFrame, .bounds = FrameBounds_ },
    { .name = "crankshaft", .draw = DrawCrank, .bounds = CrankBounds },
    { .name = "flywheel", .draw = DrawFlywheel, .bounds = FlywheelBounds },
    { .name = "conrod", .draw = DrawRod, .bounds = RodBounds },
    { .name = "piston", .draw = DrawPiston, .bounds = PistonBounds },
};

const Scene SCENE = {
    .name = "crank_slider",
    .description =
        "A slider-crank demonstration rig: an 85 mm stroke driving an 86 mm piston through a 144 mm connecting rod, on an open frame 240 mm long and 345 mm tall. Dimensions are in metres; one world unit is 100 mm, so a grid square is 10 cm.\n"
        "\n"
        "It is not a copy of a particular engine, so there is no reference photograph it can be measured against: the numbers define the mechanism rather than record one. What it does claim is kinematic, and that claim is checked rather than asserted.\n"
        "\n"
        "Every part is authored in its own frame about its own pivot and posed by a matrix, so nothing is baked into world coordinates:\n"
        "frame: baseplate 240 x 140 x 18, two standards at x 62 to 80 carrying 32 mm main bearing bosses buried 18 mm into them, a top deck at y 325, and a guide cage of two 47.5 mm rings at y 215 and y 280 on four 6 mm bars. The piston is 62 mm long against an 85 mm stroke, so no single ring can stay engaged over the whole cycle; two rings 65 mm apart keep at least one engaged at both dead centres, and the open sides are what let the piston be seen at all. Its own frame is the world frame, because it never moves.\n"
        "crankshaft: local origin on the axis of rotation. Main journals of 24 mm radius, two webs at x 32 to 50 carrying a 20 mm crankpin at local y 42.5 and a counterweight below the axis, reaching 72.8 mm from the axis against 82 mm of clearance to the baseplate. Posed by a bare rotation about local x.\n"
        "flywheel: a 74 mm disc with a 22 mm rim and a red radial timing mark, on the SAME pose matrix as the crankshaft rather than a second copy of that placement. A plain disc turning about its own axis is invisible in a still; the mark is the only thing that tells a reviewer which frame is which.\n"
        "conrod: local origin at the big-end centre, shank along local +y, small end at local y 144. Both joints are named points in its own frame, which is what makes the constraint measurable.\n"
        "piston: local origin at the gudgeon-pin centre, the joint it shares with the rod's small end. The skirt is one closed profile revolved about local y, so it has a real 7 mm wall and two ring grooves rather than being an open tube. The gudgeon pin is deliberately left proud of the skirt with a collar at each end: on a real piston it is hidden inside the bosses, and hiding it here would hide the one joint this model exists to show.\n"
        "\n"
        "The pose comes from the loop-closure equation y(theta) = R cos theta + sqrt(L^2 - R^2 sin^2 theta), with the rod tilt taken from the same triangle, so the rod's ends land exactly on the crankpin and the piston pin at every angle. CheckJoints() measures that over 720 steps at build time and warns if either joint separates by more than 0.01 mm.\n"
        "\n"
        "One revolution takes 2.4 s. Rendered with --anim the camera holds still and the frames step through one cycle; rendered without it the frames are a turntable of the top-dead-centre pose.\n",
    .init = Init,
    .unload = Unload,
    .update = Update,
    .duration = CYCLE,
    .animYaw = 90.0f,   // the frame is open along z and solid along x, so the linkage only reads from the side
    .parts = PARTS,
    .partCount = (int)(sizeof(PARTS) / sizeof(PARTS[0])),
    .target = { 0.0f, 1.7f, 0.0f },
    .orbitRadius = 4.6f,
    .orbitHeight = 1.1f,
};
