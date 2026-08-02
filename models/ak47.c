#include "harness.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Units and frame
//
// Every constant below is in millimetres, taken from a measured side elevation (references/ak47/ref_01.png, a 2450x950 background-free photograph of a Type 2 AK-47 whose silhouette was sampled column by column at 0.3915 mm/px).
// z runs aft from the muzzle face, y up from the magazine floorplate, x across.
// Vert() is the single place where that turns into world space: it scales by UNIT and slides the origin to mid-length, so one world unit is 100 mm and the grid squares read as 10 cm.
// ---------------------------------------------------------------------------

#define UNIT      0.01f
#define Z_ORIGIN  440.0f

#define BORE      204.4f   // barrel axis height
#define ROD_Y     191.0f   // cleaning rod axis
#define GAS_Y     228.8f   // gas tube axis

// ---------------------------------------------------------------------------
// Mesh builder
//
// Surfaces are hexahedra (eight freely placed corners), surfaces of revolution about an arbitrary segment, or sections swept along a frame list.
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
    b->vertices[i * 3 + 2] = (p.z - Z_ORIGIN) * UNIT;
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

// Flat-shaded quad. Winding p0-p1-p2-p3 must be counter-clockwise seen from outside.
static void Quad(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(p1, p0), Vector3Subtract(p2, p0)));
    int a = Vert(b, p0, n), c = Vert(b, p1, n), d = Vert(b, p2, n), e = Vert(b, p3, n);
    Tri(b, a, c, d);
    Tri(b, a, d, e);
}

static void QuadN(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,
                  Vector3 n0, Vector3 n1, Vector3 n2, Vector3 n3)
{
    int a = Vert(b, p0, n0), c = Vert(b, p1, n1), d = Vert(b, p2, n2), e = Vert(b, p3, n3);
    Tri(b, a, c, d);
    Tri(b, a, d, e);
}

// c[0..3] low-y face in (-x-z, +x-z, +x+z, -x+z) order, c[4..7] the matching high-y face.
static void Hex(Builder *b, const Vector3 c[8])
{
    Quad(b, c[0], c[1], c[2], c[3]);
    Quad(b, c[7], c[6], c[5], c[4]);
    Quad(b, c[0], c[4], c[5], c[1]);
    Quad(b, c[1], c[5], c[6], c[2]);
    Quad(b, c[2], c[6], c[7], c[3]);
    Quad(b, c[3], c[7], c[4], c[0]);
}

// Slab spanning x0..x1 and z0..z1 whose floor and ceiling each run from one height at z0 to another at z1.
static void Prism(Builder *b, float x0, float x1, float z0, float z1,
                  float yb0, float yb1, float yt0, float yt1)
{
    Vector3 c[8] = {
        { x0, yb0, z0 }, { x1, yb0, z0 }, { x1, yb1, z1 }, { x0, yb1, z1 },
        { x0, yt0, z0 }, { x1, yt0, z0 }, { x1, yt1, z1 }, { x0, yt1, z1 },
    };
    Hex(b, c);
}

static void Box(Builder *b, float x0, float x1, float y0, float y1, float z0, float z1)
{
    Prism(b, x0, x1, z0, z1, y0, y0, y1, y1);
}

typedef struct {
    int vert;
    int tri;
} Mark;

// Duplicate everything added since the mark, reflected through x = 0, with the winding reversed.
static void MirrorX(Builder *b, Mark m)
{
    int verts = b->vertexCount - m.vert;
    int tris = b->triangleCount - m.tri;
    if (verts <= 0 || tris <= 0) return;
    Reserve(b, verts, tris);

    int base = b->vertexCount;
    for (int i = 0; i < verts; i++) {
        int s = (m.vert + i) * 3, d = (base + i) * 3;
        b->vertices[d + 0] = -b->vertices[s + 0];
        b->vertices[d + 1] = b->vertices[s + 1];
        b->vertices[d + 2] = b->vertices[s + 2];
        b->normals[d + 0] = -b->normals[s + 0];
        b->normals[d + 1] = b->normals[s + 1];
        b->normals[d + 2] = b->normals[s + 2];
        b->texcoords[(base + i) * 2 + 0] = b->texcoords[(m.vert + i) * 2 + 0];
        b->texcoords[(base + i) * 2 + 1] = b->texcoords[(m.vert + i) * 2 + 1];
    }
    b->vertexCount += verts;

    int tbase = b->triangleCount;
    for (int i = 0; i < tris; i++) {
        int s = (m.tri + i) * 3, d = (tbase + i) * 3;
        int off = base - m.vert;
        b->indices[d + 0] = (unsigned short)(b->indices[s + 2] + off);
        b->indices[d + 1] = (unsigned short)(b->indices[s + 1] + off);
        b->indices[d + 2] = (unsigned short)(b->indices[s + 0] + off);
    }
    b->triangleCount += tris;
}

// Cylinder or cone frustum from a to b, smooth around its circumference.
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

// ---------------------------------------------------------------------------
// Swept sections
//
// A Frame places a normalised section: the section point (u, v) lands at c + ax*u + ay*v.
// ax cross ay points along the sweep, which is what makes the side quads wind outwards for a counter-clockwise section.
// The same machinery carries the wooden furniture along z, the pistol grip along a tilted axis and the magazine around its arc.
// ---------------------------------------------------------------------------

#define SECT_MAX 40

typedef struct {
    Vector3 c;
    Vector3 ax;
    Vector3 ay;
} Frame;

typedef struct {
    float z, y, hw, hh;
} Node;

// Counter-clockwise rounded rectangle inscribed in |u|,|v| <= 1, corner radius r.
static int RoundRect(Vector2 *out, float r, int cs)
{
    static const float base[4] = { -90.0f, 0.0f, 90.0f, 180.0f };
    const float cu[4] = { 1.0f - r, 1.0f - r, -1.0f + r, -1.0f + r };
    const float cv[4] = { -1.0f + r, 1.0f - r, 1.0f - r, -1.0f + r };
    int n = 0;
    for (int k = 0; k < 4; k++) {
        for (int i = 0; i <= cs; i++) {
            float a = (base[k] + 90.0f * (float)i / (float)cs) * DEG2RAD;
            out[n++] = (Vector2){ cu[k] + r * cosf(a), cv[k] + r * sinf(a) };
        }
    }
    return n;
}

// Flat floor with a stamped crown over it, for the receiver dust cover.
static int DomeSect(Vector2 *out, int segs)
{
    int n = 0;
    out[n++] = (Vector2){ -1.0f, -1.0f };
    out[n++] = (Vector2){ 1.0f, -1.0f };
    for (int i = 0; i <= segs; i++) {
        float a = PI * (float)i / (float)segs;
        out[n++] = (Vector2){ cosf(a), -0.25f + 1.25f * sinf(a) };
    }
    return n;
}

// Lower handguard: flat cheeks carrying the finger groove, full width at the top, narrowed round the bottom.
static const Vector2 HG_LOWER[] = {
    { -0.62f, -1.00f }, {  0.62f, -1.00f }, {  0.88f, -0.74f }, {  0.98f, -0.32f },
    {  0.94f, -0.10f }, {  0.94f,  0.20f }, {  1.00f,  0.38f }, {  1.00f,  0.82f },
    {  0.80f,  1.00f }, { -0.80f,  1.00f }, { -1.00f,  0.80f }, { -1.00f,  0.34f },
    { -0.94f,  0.20f }, { -0.94f, -0.10f }, { -0.98f, -0.32f }, { -0.88f, -0.74f },
};

// Upper handguard: a rounded trapezoid, widest at the seam and drawn in toward the crown.
static const Vector2 HG_UPPER[] = {
    { -0.70f, -1.00f }, {  0.70f, -1.00f }, {  1.00f, -0.60f }, {  1.00f,  0.10f },
    {  0.72f,  0.80f }, {  0.45f,  1.00f }, { -0.45f,  1.00f }, { -0.72f,  0.80f },
    { -1.00f,  0.10f }, { -1.00f, -0.60f },
};

static void Sweep(Builder *b, const Vector2 *sect, int n, const Frame *fr, int nf, bool capA, bool capB)
{
    Vector2 sn[SECT_MAX];
    for (int j = 0; j < n; j++) {
        Vector2 a = sect[(j + n - 1) % n], c = sect[j], d = sect[(j + 1) % n];
        Vector2 e0 = Vector2Normalize((Vector2){ c.x - a.x, c.y - a.y });
        Vector2 e1 = Vector2Normalize((Vector2){ d.x - c.x, d.y - c.y });
        sn[j] = Vector2Normalize((Vector2){ e0.y + e1.y, -e0.x - e1.x });
    }

    Vector3 p[SECT_MAX], q[SECT_MAX], np[SECT_MAX], nq[SECT_MAX];
    for (int i = 0; i < nf; i++) {
        Vector3 *pos = (i == 0) ? p : q;
        Vector3 *nrm = (i == 0) ? np : nq;
        float sx = Vector3Length(fr[i].ax), sy = Vector3Length(fr[i].ay);
        Vector3 ux = Vector3Scale(fr[i].ax, 1.0f / sx);
        Vector3 uy = Vector3Scale(fr[i].ay, 1.0f / sy);
        for (int j = 0; j < n; j++) {
            pos[j] = Vector3Add(fr[i].c,
                                Vector3Add(Vector3Scale(fr[i].ax, sect[j].x), Vector3Scale(fr[i].ay, sect[j].y)));
            nrm[j] = Vector3Normalize(Vector3Add(Vector3Scale(ux, sn[j].x / sx), Vector3Scale(uy, sn[j].y / sy)));
        }
        if (i == 0) continue;
        for (int j = 0; j < n; j++) {
            int k = (j + 1) % n;
            QuadN(b, p[j], p[k], q[k], q[j], np[j], np[k], nq[k], nq[j]);
        }
        for (int j = 0; j < n; j++) { p[j] = q[j]; np[j] = nq[j]; }
    }

    if (capA) {
        Vector3 nA = Vector3Normalize(Vector3Negate(Vector3CrossProduct(fr[0].ax, fr[0].ay)));
        int centre = Vert(b, fr[0].c, nA);
        for (int j = 0; j < n; j++) {
            int k = (j + 1) % n;
            Vector3 a = Vector3Add(fr[0].c, Vector3Add(Vector3Scale(fr[0].ax, sect[j].x), Vector3Scale(fr[0].ay, sect[j].y)));
            Vector3 c = Vector3Add(fr[0].c, Vector3Add(Vector3Scale(fr[0].ax, sect[k].x), Vector3Scale(fr[0].ay, sect[k].y)));
            int ia = Vert(b, c, nA), ib = Vert(b, a, nA);
            Tri(b, centre, ia, ib);
        }
    }
    if (capB) {
        int last = nf - 1;
        Vector3 nB = Vector3Normalize(Vector3CrossProduct(fr[last].ax, fr[last].ay));
        int centre = Vert(b, fr[last].c, nB);
        for (int j = 0; j < n; j++) {
            int k = (j + 1) % n;
            Vector3 a = Vector3Add(fr[last].c, Vector3Add(Vector3Scale(fr[last].ax, sect[j].x), Vector3Scale(fr[last].ay, sect[j].y)));
            Vector3 c = Vector3Add(fr[last].c, Vector3Add(Vector3Scale(fr[last].ax, sect[k].x), Vector3Scale(fr[last].ay, sect[k].y)));
            int ia = Vert(b, a, nB), ib = Vert(b, c, nB);
            Tri(b, centre, ia, ib);
        }
    }
}

// Frames from a centreline in the z-y plane: the section plane stays square to the local path direction.
static void NodeFrames(Frame *fr, const Node *nd, int n)
{
    for (int i = 0; i < n; i++) {
        int a = (i > 0) ? i - 1 : 0;
        int c = (i < n - 1) ? i + 1 : n - 1;
        float dz = nd[c].z - nd[a].z, dy = nd[c].y - nd[a].y;
        float len = sqrtf(dz * dz + dy * dy);
        if (len < 1e-6f) { dz = 1.0f; dy = 0.0f; len = 1.0f; }
        fr[i].c = (Vector3){ 0.0f, nd[i].y, nd[i].z };
        fr[i].ax = (Vector3){ nd[i].hw, 0.0f, 0.0f };
        fr[i].ay = (Vector3){ 0.0f, nd[i].hh * dz / len, -nd[i].hh * dy / len };
    }
}

// Wire loop around an ellipse in the plane x = px, built from straight segments.
static void WireLoop(Builder *b, float px, float cz, float cy, float rz, float ry, float wire, int segs)
{
    for (int i = 0; i < segs; i++) {
        float a0 = 2.0f * PI * (float)i / (float)segs;
        float a1 = 2.0f * PI * (float)(i + 1) / (float)segs;
        Vector3 p0 = { px, cy + ry * sinf(a0), cz + rz * cosf(a0) };
        Vector3 p1 = { px, cy + ry * sinf(a1), cz + rz * cosf(a1) };
        Tube(b, p0, p1, wire, wire, 10, true, true);
    }
}

// ---------------------------------------------------------------------------
// Materials and groups
// ---------------------------------------------------------------------------

typedef enum {
    MAT_WOOD, MAT_STEEL, MAT_GREY, MAT_BLUED, MAT_COUNT
} MatId;

// lighting.fs gamma-corrects with pow(c, 1/2.2), so these read considerably lighter than the raw values once shaded.
static const Color MAT_COLOR[MAT_COUNT] = {
    [MAT_WOOD]  = { 112,  44,  22, 255 },
    [MAT_STEEL] = { 140, 144, 152, 255 },
    [MAT_GREY]  = {  62,  65,  70, 255 },
    [MAT_BLUED] = {  20,  21,  23, 255 },
};

typedef struct {
    Builder b[MAT_COUNT];
    Model model[MAT_COUNT];
    bool has[MAT_COUNT];
    BoundingBox bounds;
} Group;

typedef struct {
    Mark m[MAT_COUNT];
} GroupMark;

static GroupMark GroupMarkNow(Group *g)
{
    GroupMark gm;
    for (int i = 0; i < MAT_COUNT; i++) {
        gm.m[i].vert = g->b[i].vertexCount;
        gm.m[i].tri = g->b[i].triangleCount;
    }
    return gm;
}

static void GroupMirrorX(Group *g, GroupMark gm)
{
    for (int i = 0; i < MAT_COUNT; i++) MirrorX(&g->b[i], gm.m[i]);
}

static void GroupFinish(Group *g, const char *name)
{
    bool any = false;
    for (int m = 0; m < MAT_COUNT; m++) {
        Builder *b = &g->b[m];
        if (b->vertexCount == 0) continue;
        if (b->vertexCount > 65535) {
            TraceLog(LOG_ERROR, "%s: material %d has %d vertices, over the 65535 index limit",
                     name, m, b->vertexCount);
        }

        Mesh mesh = { 0 };
        mesh.vertexCount = b->vertexCount;
        mesh.triangleCount = b->triangleCount;
        mesh.vertices = b->vertices;
        mesh.normals = b->normals;
        mesh.texcoords = b->texcoords;
        mesh.indices = b->indices;
        UploadMesh(&mesh, false);

        g->model[m] = LoadModelFromMesh(mesh);
        g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].color = MAT_COLOR[m];
        HarnessApplyLighting(&g->model[m]);
        g->has[m] = true;

        BoundingBox bb = GetModelBoundingBox(g->model[m]);
        if (!any) {
            g->bounds = bb;
            any = true;
        } else {
            g->bounds.min = Vector3Min(g->bounds.min, bb.min);
            g->bounds.max = Vector3Max(g->bounds.max, bb.max);
        }
    }
}

static void GroupDraw(const Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (g->has[m]) DrawModel(g->model[m], Vector3Zero(), 1.0f, WHITE);
    }
}

static void GroupUnload(Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (g->has[m]) UnloadModel(g->model[m]);
    }
}

static Group gSight, gBarrel, gWood, gRecv, gMag, gFire, gStock;

// ---------------------------------------------------------------------------
// Front sight assembly, z 0 to 46
// ---------------------------------------------------------------------------

static void BuildSight(void)
{
    Builder *s = &gSight.b[MAT_BLUED];

    Tube(s, (Vector3){ 0, BORE, 0.0f }, (Vector3){ 0, BORE, 13.0f }, 7.5f, 7.5f, 30, true, true);
    Tube(s, (Vector3){ 0, BORE, 13.0f }, (Vector3){ 0, BORE, 46.0f }, 6.85f, 6.85f, 30, false, true);

    Vector3 collar[8] = {
        { -9.0f, 187.0f, 14.0f }, { 9.0f, 187.0f, 14.0f }, { 9.0f, 187.0f, 34.5f }, { -9.0f, 187.0f, 34.5f },
        { -7.5f, 219.0f, 14.0f }, { 7.5f, 219.0f, 14.0f }, { 7.5f, 219.0f, 36.5f }, { -7.5f, 219.0f, 36.5f },
    };
    Hex(s, collar);

    // The window measures z 17 to 36.6 by y 218.4 to 229.1 on the reference, so the tower is a
    // front post, a rear buttress and a bridge over the top rather than a narrow slot.
    Box(s, -5.5f, 5.5f, 218.0f, 229.5f, 14.0f, 18.0f);
    Vector3 buttress[8] = {
        { -5.5f, 212.0f, 34.0f }, { 5.5f, 212.0f, 34.0f }, { 5.5f, 212.0f, 46.0f }, { -5.5f, 212.0f, 46.0f },
        { -5.5f, 229.0f, 31.0f }, { 5.5f, 229.0f, 31.0f }, { 5.5f, 229.0f, 39.5f }, { -5.5f, 229.0f, 39.5f },
    };
    Hex(s, buttress);
    Vector3 bridge[8] = {
        { -5.5f, 229.0f, 14.0f }, { 5.5f, 229.0f, 14.0f }, { 5.5f, 229.0f, 39.5f }, { -5.5f, 229.0f, 39.5f },
        { -4.4f, 246.0f, 15.0f }, { 4.4f, 246.0f, 15.0f }, { 4.4f, 246.0f, 33.0f }, { -4.4f, 246.0f, 33.0f },
    };
    Hex(s, bridge);

    Tube(s, (Vector3){ 0, 250.1f, 13.6f }, (Vector3){ 0, 250.1f, 33.2f }, 5.1f, 5.1f, 26, true, true);
    Tube(s, (Vector3){ 0, 246.0f, 23.4f }, (Vector3){ 0, 256.0f, 23.4f }, 1.2f, 1.0f, 14, false, true);
}

// ---------------------------------------------------------------------------
// Barrel, gas block, gas tube, cleaning rod: z 12 to 330
// ---------------------------------------------------------------------------

static void BuildBarrel(void)
{
    Builder *s = &gBarrel.b[MAT_BLUED];
    Builder *m = &gBarrel.b[MAT_GREY];

    Tube(s, (Vector3){ 0, BORE, 12.0f }, (Vector3){ 0, BORE, 104.0f }, 6.85f, 6.85f, 30, false, false);
    Tube(s, (Vector3){ 0, BORE, 104.0f }, (Vector3){ 0, BORE, 330.0f }, 7.65f, 7.65f, 30, true, true);
    Tube(s, (Vector3){ 0, ROD_Y, 16.0f }, (Vector3){ 0, ROD_Y, 340.0f }, 2.0f, 2.0f, 14, true, true);

    Vector3 block[8] = {
        { -7.6f, 188.0f, 105.0f }, { 7.6f, 188.0f, 105.0f }, { 7.6f, 188.0f, 143.0f }, { -7.6f, 188.0f, 143.0f },
        { -8.5f, 219.0f, 105.0f }, { 8.5f, 219.0f, 105.0f }, { 8.5f, 219.0f, 143.0f }, { -8.5f, 219.0f, 143.0f },
    };
    Hex(s, block);
    Tube(s, (Vector3){ 0, BORE, 101.0f }, (Vector3){ 0, BORE, 109.0f }, 9.6f, 9.6f, 26, true, false);
    Tube(s, (Vector3){ 0, BORE, 139.0f }, (Vector3){ 0, BORE, 147.0f }, 9.6f, 9.6f, 26, false, true);
    Box(s, -4.0f, 4.0f, 184.0f, 190.0f, 126.0f, 138.0f);
    Vector3 riser[8] = {
        { -8.5f, 213.0f, 105.0f }, { 8.5f, 213.0f, 105.0f }, { 8.5f, 213.0f, 143.0f }, { -8.5f, 213.0f, 143.0f },
        { -6.6f, 238.0f, 130.0f }, { 6.6f, 238.0f, 130.0f }, { 6.6f, 238.0f, 143.0f }, { -6.6f, 238.0f, 143.0f },
    };
    Hex(s, riser);

    Tube(s, (Vector3){ 0, GAS_Y, 141.0f }, (Vector3){ 0, GAS_Y, 152.0f }, 9.2f, 9.2f, 28, true, false);
    Tube(s, (Vector3){ 0, GAS_Y, 152.0f }, (Vector3){ 0, GAS_Y, 206.0f }, 8.0f, 8.0f, 28, false, false);
    Tube(s, (Vector3){ 0, GAS_Y, 206.0f }, (Vector3){ 0, GAS_Y, 326.0f }, 9.2f, 9.2f, 28, false, true);

    // Slotted panel on top of the gas tube: a dark floor framed by two rails and five cross ribs, leaving the four ports the reference shows.
    Box(s, -5.0f, 5.0f, 234.0f, 236.4f, 152.0f, 206.0f);
    Box(m, -5.0f, -3.8f, 234.0f, 238.0f, 152.0f, 206.0f);
    Box(m, 3.8f, 5.0f, 234.0f, 238.0f, 152.0f, 206.0f);
    for (int i = 0; i < 5; i++) {
        float z0 = 152.0f + 12.0f * (float)i;
        Box(m, -3.8f, 3.8f, 234.0f, 238.0f, z0, z0 + 6.0f);
    }

    Box(s, -9.0f, 9.0f, 193.0f, 217.0f, 196.0f, 214.0f);
    Tube(s, (Vector3){ -9.5f, 205.0f, 205.0f }, (Vector3){ 9.5f, 205.0f, 205.0f }, 1.8f, 1.8f, 8, true, true);
}

// ---------------------------------------------------------------------------
// Wooden handguards, z 196 to 372
// ---------------------------------------------------------------------------

static void BuildWood(void)
{
    Builder *w = &gWood.b[MAT_WOOD];
    Builder *s = &gWood.b[MAT_BLUED];

    Frame fr[10];

    static const Node lower[] = {
        { 210.0f, 200.3f, 18.0f, 16.6f },
        { 226.0f, 199.9f, 21.0f, 17.0f },
        { 260.0f, 197.5f, 21.0f, 17.8f },
        { 290.0f, 195.7f, 21.0f, 18.0f },
        { 312.0f, 195.9f, 20.5f, 19.0f },
        { 340.0f, 196.0f, 19.0f, 19.0f },
    };
    NodeFrames(fr, lower, 6);
    Sweep(w, HG_LOWER, (int)(sizeof(HG_LOWER) / sizeof(HG_LOWER[0])), fr, 6, true, true);

    static const Node upper[] = {
        { 214.0f, 231.0f, 13.5f, 12.0f },
        { 226.0f, 231.6f, 16.0f, 13.1f },
        { 270.0f, 231.9f, 16.5f, 13.5f },
        { 310.0f, 232.9f, 16.0f, 13.7f },
        { 326.0f, 233.0f, 14.5f, 13.0f },
    };
    NodeFrames(fr, upper, 5);
    Sweep(w, HG_UPPER, (int)(sizeof(HG_UPPER) / sizeof(HG_UPPER[0])), fr, 5, true, true);

    Box(s, -16.5f, 16.5f, 217.0f, 244.0f, 209.0f, 219.0f);
    Box(s, -19.5f, 19.5f, 175.0f, 217.0f, 338.0f, 352.0f);
}

// ---------------------------------------------------------------------------
// Receiver, z 322 to 640
//
// The milled lightening cut and the right-hand ejection port are real openings: the receiver core stops at x = 15.5 and the side plates out to 17.2 are laid down as separate panels around each window, so the core shows through as the recess floor.
// ---------------------------------------------------------------------------

#define RCV_CORE   14.0f
#define RCV_SIDE   17.2f
#define CUT_Z0     380.0f
#define CUT_Z1     458.0f
#define CUT_Y0     179.0f
#define CUT_Y1     196.0f
#define PORT_Z0    395.0f
#define PORT_Z1    455.0f
#define TRACK_Z1   540.0f
#define TRACK_Y0   218.0f
#define TRACK_Y1   225.0f
#define PORT_Y0    211.0f
#define RCV_TOP    228.0f
#define RCV_Z0     372.0f
#define RCV_Z1     620.0f

// Receiver floor line, measured off the elevation: it drops fastest over the magazine well.
static float RecvFloor(float z)
{
    if (z <= 420.0f) return 175.0f + (z - 372.0f) * (173.4f - 175.0f) / 48.0f;
    if (z <= 510.0f) return 173.4f + (z - 420.0f) * (166.4f - 173.4f) / 90.0f;
    return 166.4f + (z - 510.0f) * (165.0f - 166.4f) / 110.0f;
}

// One face of a window bevel, wound so it faces out of the pocket on whichever flank sgn selects.
static void BevQuad(Builder *b, float sgn, Vector3 a, Vector3 c, Vector3 d, Vector3 e)
{
    a.x *= sgn; c.x *= sgn; d.x *= sgn; e.x *= sgn;
    if (sgn > 0.0f) Quad(b, a, c, d, e);
    else Quad(b, e, d, c, a);
}

// 45-degree rim round a window: the opening at the outer face xout is r larger all round than the pocket mouth at xin.
// Depth alone cannot define a 3.2 mm pocket in a 34.4 mm receiver; the rim highlight is what makes it read as milled.
static void BevelWindow(Builder *b, float sgn, float xin, float xout,
                        float z0, float z1, float y0, float y1, float r)
{
    Vector3 i0 = { xin, y0, z0 }, i1 = { xin, y0, z1 }, i2 = { xin, y1, z1 }, i3 = { xin, y1, z0 };
    Vector3 o0 = { xout, y0 - r, z0 - r }, o1 = { xout, y0 - r, z1 + r };
    Vector3 o2 = { xout, y1 + r, z1 + r }, o3 = { xout, y1 + r, z0 - r };
    BevQuad(b, sgn, i0, i1, o1, o0);
    BevQuad(b, sgn, i2, i3, o3, o2);
    BevQuad(b, sgn, i3, i0, o0, o3);
    BevQuad(b, sgn, i1, i2, o2, o1);
}

// Fill the four corners of a window with 45-degree wedges: at render scale that reads as the rounded end of a milled pocket.
static void WindowFillets(Builder *b, float x0, float x1, float z0, float z1, float y0, float y1, float r)
{
    Prism(b, x0, x1, z0, z0 + r, y1 - r, y1, y1, y1);
    Prism(b, x0, x1, z1 - r, z1, y1, y1 - r, y1, y1);
    Prism(b, x0, x1, z0, z0 + r, y0, y0, y0 + r, y0);
    Prism(b, x0, x1, z1 - r, z1, y0, y0, y0, y0 + r);
}

static void SidePlate(Builder *b, float x0, float x1, bool withPort)
{
    Prism(b, x0, x1, RCV_Z0, CUT_Z0, RecvFloor(RCV_Z0), RecvFloor(CUT_Z0), RCV_TOP, RCV_TOP);
    Prism(b, x0, x1, CUT_Z0, CUT_Z1, RecvFloor(CUT_Z0), RecvFloor(CUT_Z1), CUT_Y0, CUT_Y0);

    float sgn = (x0 < 0.0f) ? -1.0f : 1.0f;
    WindowFillets(b, x0, x1, CUT_Z0, CUT_Z1, CUT_Y0, CUT_Y1, 5.0f);
    BevelWindow(b, sgn, RCV_CORE, RCV_SIDE, CUT_Z0 + 2.5f, CUT_Z1 - 2.5f, CUT_Y0 + 2.5f, CUT_Y1 - 2.5f, 2.5f);

    if (withPort) {
        WindowFillets(b, x0, x1, PORT_Z0, PORT_Z1, PORT_Y0, RCV_TOP, 5.0f);
        BevelWindow(b, sgn, RCV_CORE, RCV_SIDE, PORT_Z0 + 2.5f, PORT_Z1 - 2.5f, PORT_Y0 + 2.5f, RCV_TOP, 2.5f);
        Box(b, x0, x1, CUT_Y1, RCV_TOP, CUT_Z0, PORT_Z0);
        Box(b, x0, x1, CUT_Y1, PORT_Y0, PORT_Z0, PORT_Z1);
        Box(b, x0, x1, CUT_Y1, TRACK_Y0, PORT_Z1, TRACK_Z1);
        Box(b, x0, x1, TRACK_Y1, RCV_TOP, PORT_Z1, TRACK_Z1);
        Prism(b, x0, x1, CUT_Z1, TRACK_Z1, RecvFloor(CUT_Z1), RecvFloor(TRACK_Z1), CUT_Y1, CUT_Y1);
        Prism(b, x0, x1, TRACK_Z1, RCV_Z1, RecvFloor(TRACK_Z1), RecvFloor(RCV_Z1), RCV_TOP, RCV_TOP);
    } else {
        Box(b, x0, x1, CUT_Y1, RCV_TOP, CUT_Z0, CUT_Z1);
        Prism(b, x0, x1, CUT_Z1, RCV_Z1, RecvFloor(CUT_Z1), RecvFloor(RCV_Z1), RCV_TOP, RCV_TOP);
    }
}

static void BuildRecv(void)
{
    Builder *m = &gRecv.b[MAT_STEEL];
    Builder *s = &gRecv.b[MAT_GREY];

    Prism(m, -RCV_CORE, RCV_CORE, RCV_Z0, PORT_Z0, RecvFloor(RCV_Z0), RecvFloor(PORT_Z0), RCV_TOP, RCV_TOP);
    Prism(m, -RCV_CORE, RCV_CORE, PORT_Z0, PORT_Z1, RecvFloor(PORT_Z0), RecvFloor(PORT_Z1), PORT_Y0, PORT_Y0);
    Prism(m, -RCV_CORE, RCV_CORE, PORT_Z1, RCV_Z1, RecvFloor(PORT_Z1), RecvFloor(RCV_Z1), RCV_TOP, RCV_TOP);
    Box(&gRecv.b[MAT_BLUED], -9.5f, 9.5f, PORT_Y0, RCV_TOP, PORT_Z0, PORT_Z1);
    Prism(m, -RCV_SIDE, RCV_SIDE, 350.0f, 380.0f, 171.0f, 174.0f, 232.0f, 232.0f);
    SidePlate(m, RCV_CORE, RCV_SIDE, true);
    SidePlate(m, -RCV_SIDE, -RCV_CORE, false);

    Vector2 dome[SECT_MAX];
    int dn = DomeSect(dome, 10);
    Frame cover[2];
    cover[0].c = (Vector3){ 0.0f, 233.5f, 374.0f };
    cover[0].ax = (Vector3){ 17.8f, 0.0f, 0.0f };
    cover[0].ay = (Vector3){ 0.0f, 6.5f, 0.0f };
    cover[1].c = (Vector3){ 0.0f, 232.8f, 612.0f };
    cover[1].ax = (Vector3){ 17.8f, 0.0f, 0.0f };
    cover[1].ay = (Vector3){ 0.0f, 5.8f, 0.0f };
    Sweep(m, dome, dn, cover, 2, true, true);
    Vector3 tang[8] = {
        { -16.5f, 196.0f, 606.0f }, { 16.5f, 196.0f, 606.0f }, { 16.5f, 196.0f, 634.0f }, { -16.5f, 196.0f, 634.0f },
        { -16.5f, 238.6f, 606.0f }, { 16.5f, 238.6f, 606.0f }, { 16.5f, 218.0f, 634.0f }, { -16.5f, 218.0f, 634.0f },
    };
    Hex(m, tang);
    Prism(m, -RCV_SIDE, RCV_SIDE, RCV_Z1, 634.0f, RecvFloor(RCV_Z1), 160.0f, 196.0f, 196.0f);

    // Rear sight: base block, leaf with raised side rails, slider, tangent lever.
    Vector3 sightBase[8] = {
        { -16.0f, 222.0f, 328.0f }, { 16.0f, 222.0f, 328.0f }, { 16.0f, 222.0f, 380.0f }, { -16.0f, 222.0f, 380.0f },
        { -11.5f, 240.0f, 331.0f }, { 11.5f, 240.0f, 331.0f }, { 11.5f, 247.0f, 378.0f }, { -11.5f, 247.0f, 378.0f },
    };
    Hex(s, sightBase);
    Box(s, -8.0f, 8.0f, 247.0f, 250.0f, 334.0f, 374.0f);
    Box(s, -8.0f, -6.0f, 247.0f, 252.5f, 334.0f, 374.0f);
    Box(s, 6.0f, 8.0f, 247.0f, 252.5f, 334.0f, 374.0f);
    Box(s, -9.0f, 9.0f, 245.0f, 253.0f, 352.0f, 362.0f);
    Box(s, -4.0f, 4.0f, 240.0f, 251.0f, 378.0f, 392.0f);
    Box(s, -2.0f, 2.0f, 251.0f, 253.5f, 393.0f, 400.0f);

    // Right side only: selector lever with its pivot boss, and the charging handle.
    Vector3 lever[8] = {
        { RCV_SIDE, 173.5f, 478.0f }, { 20.5f, 174.5f, 478.0f }, { 20.5f, 189.0f, 555.0f }, { RCV_SIDE, 188.0f, 555.0f },
        { RCV_SIDE, 180.0f, 478.0f }, { 20.5f, 179.0f, 478.0f }, { 20.5f, 204.0f, 555.0f }, { RCV_SIDE, 206.0f, 555.0f },
    };
    Hex(m, lever);
    Tube(m, (Vector3){ RCV_SIDE, 197.5f, 555.0f }, (Vector3){ 21.5f, 197.5f, 555.0f }, 6.5f, 5.0f, 16, false, true);
    Box(m, RCV_SIDE, 20.5f, 197.5f, 214.0f, 555.0f, 567.0f);

    Box(&gRecv.b[MAT_BLUED], 12.0f, 19.0f, 214.0f, 227.0f, 382.0f, 394.0f);
    Tube(&gRecv.b[MAT_BLUED], (Vector3){ 17.0f, 221.0f, 388.0f }, (Vector3){ 30.0f, 221.0f, 388.0f }, 4.8f, 4.0f, 20, false, true);

    GroupMark gm = GroupMarkNow(&gRecv);
    Tube(m, (Vector3){ RCV_SIDE, 177.0f, 491.0f }, (Vector3){ 18.4f, 177.0f, 491.0f }, 2.2f, 1.8f, 10, false, true);
    Tube(m, (Vector3){ RCV_SIDE, 177.0f, 502.0f }, (Vector3){ 18.4f, 177.0f, 502.0f }, 2.2f, 1.8f, 10, false, true);
    Tube(m, (Vector3){ RCV_SIDE, 214.0f, 596.0f }, (Vector3){ 18.4f, 214.0f, 596.0f }, 2.2f, 1.8f, 10, false, true);
    GroupMirrorX(&gRecv, gm);
}

// ---------------------------------------------------------------------------
// Magazine, z 340 to 505
//
// Both walls of the 30-round magazine fall on circles about a common centre on the bore line: fitting the measured elevation gives centre (z 248, y 202) with the front wall at radius 177.1 and the rear at 236.5, to better than 0.5 mm.
// ---------------------------------------------------------------------------

#define MAG_CZ    248.0f
#define MAG_CY    202.0f
#define MAG_RMID  206.8f
#define MAG_HT    29.7f
#define MAG_HW    14.0f
#define MAG_TH0   -8.0f
#define MAG_TH1   -57.5f

static void MagFrames(Frame *fr, int n, float th0, float th1, float rMid, float ht, float hw)
{
    for (int i = 0; i < n; i++) {
        float th = (th0 + (th1 - th0) * (float)i / (float)(n - 1)) * DEG2RAD;
        fr[i].c = (Vector3){ 0.0f, MAG_CY + rMid * sinf(th), MAG_CZ + rMid * cosf(th) };
        fr[i].ax = (Vector3){ hw, 0.0f, 0.0f };
        fr[i].ay = (Vector3){ 0.0f, ht * sinf(th), ht * cosf(th) };
    }
}

static void BuildMag(void)
{
    Builder *s = &gMag.b[MAT_BLUED];

    Vector2 sect[SECT_MAX];
    Frame fr[16];

    int n = RoundRect(sect, 0.30f, 3);
    MagFrames(fr, 14, MAG_TH0, MAG_TH1, MAG_RMID, MAG_HT, MAG_HW);
    fr[13].ay = Vector3Scale(fr[13].ay, 0.96f);
    Sweep(s, sect, n, fr, 14, true, true);

    // Five ribs pressed along each side, clustered toward the spine as the photograph shows.
    int rn = RoundRect(sect, 0.5f, 2);
    static const float ribV[5] = { -0.05f, 0.17f, 0.39f, 0.61f, 0.83f };
    for (int k = 0; k < 5; k++) {
        MagFrames(fr, 12, -10.5f, -55.0f, MAG_RMID + ribV[k] * MAG_HT, 5.0f, MAG_HW + 0.3f);
        fr[0].ax = Vector3Scale(Vector3Normalize(fr[0].ax), MAG_HW - 0.4f);
        fr[11].ax = Vector3Scale(Vector3Normalize(fr[11].ax), MAG_HW - 0.4f);
        fr[0].ay = Vector3Scale(fr[0].ay, 0.15f);
        fr[11].ay = Vector3Scale(fr[11].ay, 0.15f);
        Sweep(s, sect, rn, fr, 12, false, false);
    }

    MagFrames(fr, 2, MAG_TH1, MAG_TH1 - 1.1f, MAG_RMID, MAG_HT * 0.99f, MAG_HW + 0.8f);
    Sweep(s, sect, n, fr, 2, true, true);
}

// ---------------------------------------------------------------------------
// Trigger group and pistol grip, z 492 to 645
// ---------------------------------------------------------------------------

static void BuildFire(void)
{
    Builder *s = &gFire.b[MAT_GREY];
    Builder *w = &gFire.b[MAT_WOOD];

    // Measured bow line: 135.8 at z 510, 133.1 at 528, 132.3 at 550, 133.4 at 562. Swept, so the contour stays continuous.
    Vector2 bowSect[SECT_MAX];
    Frame bowFr[6];
    static const Node bow[] = {
        { 506.0f, 139.0f, 8.0f, 3.1f }, { 518.0f, 136.5f, 8.0f, 2.9f }, { 530.0f, 135.4f, 8.0f, 2.8f },
        { 542.0f, 134.9f, 8.0f, 2.8f }, { 554.0f, 135.0f, 8.0f, 2.9f }, { 566.0f, 136.4f, 8.0f, 3.1f },
    };
    int bn = RoundRect(bowSect, 0.55f, 3);
    NodeFrames(bowFr, bow, 6);
    Sweep(s, bowSect, bn, bowFr, 6, true, true);
    Prism(s, -8.0f, 8.0f, 502.0f, 514.0f, 135.8f, 135.8f, 172.0f, 141.0f);
    Prism(s, -8.0f, 8.0f, 558.0f, 570.0f, 133.4f, 133.4f, 139.0f, 172.0f);
    Box(s, -6.0f, 6.0f, 130.0f, 152.0f, 492.0f, 502.0f);

    Vector3 trig[8] = {
        { -2.5f, 140.0f, 531.0f }, { 2.5f, 140.0f, 531.0f }, { 2.5f, 152.0f, 549.0f }, { -2.5f, 152.0f, 549.0f },
        { -2.5f, 150.0f, 531.0f }, { 2.5f, 150.0f, 531.0f }, { 2.5f, 172.0f, 549.0f }, { -2.5f, 172.0f, 549.0f },
    };
    Hex(s, trig);

    Vector2 sect[SECT_MAX];
    Frame fr[6];
    static const Node grip[] = {
        { 588.0f, 172.0f, 14.2f, 31.0f },
        { 601.0f, 134.0f, 14.6f, 27.5f },
        { 610.0f, 105.0f, 15.2f, 24.5f },
        { 617.0f,  84.0f, 15.5f, 22.0f },
        { 620.0f,  71.0f, 14.4f, 19.5f },
    };
    int n = RoundRect(sect, 0.24f, 3);
    NodeFrames(fr, grip, 5);
    Sweep(w, sect, n, fr, 5, true, true);
}

// ---------------------------------------------------------------------------
// Buttstock, z 606 to 880
// ---------------------------------------------------------------------------

static void BuildStock(void)
{
    Builder *w = &gStock.b[MAT_WOOD];
    Builder *m = &gStock.b[MAT_GREY];
    Builder *s = &gStock.b[MAT_GREY];

    Vector3 wrist[8] = {
        { -16.5f, 160.0f, 628.0f }, { 16.5f, 160.0f, 628.0f }, { 14.4f, 163.0f, 658.0f }, { -14.4f, 163.0f, 658.0f },
        { -16.5f, 214.0f, 628.0f }, { 16.5f, 214.0f, 628.0f }, { 14.4f, 206.0f, 658.0f }, { -14.4f, 206.0f, 658.0f },
    };
    Hex(m, wrist);

    Vector2 sect[SECT_MAX];
    Frame fr[20];
    static const Node butt[] = {
        { 640.0f, 183.2f, 13.6f, 21.9f },
        { 656.0f, 179.5f, 13.7f, 23.3f },
        { 672.0f, 174.8f, 13.9f, 24.9f },
        { 688.0f, 169.9f, 14.2f, 26.6f },
        { 704.0f, 165.8f, 14.6f, 28.4f },
        { 712.0f, 163.3f, 14.9f, 29.8f },
        { 720.0f, 162.1f, 15.3f, 32.1f },
        { 728.0f, 162.3f, 15.7f, 36.3f },
        { 736.0f, 161.3f, 16.2f, 38.8f },
        { 752.0f, 156.2f, 16.9f, 41.1f },
        { 768.0f, 151.3f, 17.6f, 43.3f },
        { 784.0f, 146.4f, 18.2f, 45.4f },
        { 800.0f, 141.2f, 18.8f, 48.0f },
        { 816.0f, 135.6f, 19.2f, 50.7f },
        { 832.0f, 130.6f, 19.6f, 53.5f },
        { 856.0f, 123.5f, 19.9f, 56.2f },
        { 867.5f, 121.0f, 19.6f, 54.8f },
    };
    int n = RoundRect(sect, 0.10f, 3);
    NodeFrames(fr, butt, 17);

    // The butt face is raked: toe at z 858, heel at z 877.
    // Tilting the last section plane onto that face is what puts the rake in the wood itself rather than leaving a square cut behind the buttplate.
    float fz = 19.0f, fy = 108.0f, fl = sqrtf(fz * fz + fy * fy);
    fr[16].ay = (Vector3){ 0.0f, 54.8f * fy / fl, 54.8f * fz / fl };
    Sweep(w, sect, n, fr, 17, true, true);

    Frame plate[2];
    plate[0] = fr[16];
    plate[1].c = (Vector3){ 0.0f, 121.0f - 1.2f * fz / fl, 867.5f + 1.2f * fy / fl };
    plate[1].ax = (Vector3){ 19.3f, 0.0f, 0.0f };
    plate[1].ay = (Vector3){ 0.0f, 54.3f * fy / fl, 54.3f * fz / fl };
    Sweep(s, sect, n, plate, 2, false, true);

    // Sling swivel on the left of the wrist.
    Box(s, -19.5f, -16.0f, 160.0f, 176.0f, 692.0f, 712.0f);
    WireLoop(s, -18.5f, 678.0f, 168.0f, 11.0f, 14.0f, 1.8f, 18);
    Box(s, -19.0f, -16.5f, 164.0f, 172.0f, 688.0f, 698.0f);
    Tube(m, (Vector3){ -20.0f, 168.0f, 706.0f }, (Vector3){ -14.0f, 168.0f, 706.0f }, 2.6f, 2.0f, 12, false, true);
}

// ---------------------------------------------------------------------------

static void Init(void)
{
    BuildSight();
    BuildBarrel();
    BuildWood();
    BuildRecv();
    BuildMag();
    BuildFire();
    BuildStock();

    GroupFinish(&gSight, "front_sight");
    GroupFinish(&gBarrel, "barrel");
    GroupFinish(&gWood, "handguards");
    GroupFinish(&gRecv, "receiver");
    GroupFinish(&gMag, "magazine");
    GroupFinish(&gFire, "fire_control");
    GroupFinish(&gStock, "stock");
}

static void Unload(void)
{
    GroupUnload(&gSight);
    GroupUnload(&gBarrel);
    GroupUnload(&gWood);
    GroupUnload(&gRecv);
    GroupUnload(&gMag);
    GroupUnload(&gFire);
    GroupUnload(&gStock);
}

static void DrawSight(void) { GroupDraw(&gSight); }
static void DrawBarrel(void) { GroupDraw(&gBarrel); }
static void DrawWood(void) { GroupDraw(&gWood); }
static void DrawRecv(void) { GroupDraw(&gRecv); }
static void DrawMag(void) { GroupDraw(&gMag); }
static void DrawFire(void) { GroupDraw(&gFire); }
static void DrawStock(void) { GroupDraw(&gStock); }

static BoundingBox SightBounds(void) { return gSight.bounds; }
static BoundingBox BarrelBounds(void) { return gBarrel.bounds; }
static BoundingBox WoodBounds(void) { return gWood.bounds; }
static BoundingBox RecvBounds(void) { return gRecv.bounds; }
static BoundingBox MagBounds(void) { return gMag.bounds; }
static BoundingBox FireBounds(void) { return gFire.bounds; }
static BoundingBox StockBounds(void) { return gStock.bounds; }

static const Part PARTS[] = {
    { .name = "front_sight", .draw = DrawSight, .bounds = SightBounds },
    { .name = "barrel", .draw = DrawBarrel, .bounds = BarrelBounds },
    { .name = "handguards", .draw = DrawWood, .bounds = WoodBounds },
    { .name = "receiver", .draw = DrawRecv, .bounds = RecvBounds },
    { .name = "magazine", .draw = DrawMag, .bounds = MagBounds },
    { .name = "fire_control", .draw = DrawFire, .bounds = FireBounds },
    { .name = "stock", .draw = DrawStock, .bounds = StockBounds },
};

const Scene SCENE = {
    .name = "ak47",
    .description =
        "AK-47 with a Type 2 milled receiver and fixed wooden furniture, the configuration in references/ak47/ref_01.png.\n"
        "\n"
        "One world unit is 100 mm, so a grid square is 10 cm. Source constants are in millimetres: z runs aft from the muzzle face, y up from the magazine floorplate, x across; Vert() scales and recentres so the model sits centred on the grid.\n"
        "880 overall, bore axis at y 204.4, cleaning rod axis 191.0, gas tube axis 228.8, overall height 256.\n"
        "\n"
        "The side elevation is measured, not estimated. ref_01 was thresholded to a silhouette and sampled column by column at 0.3915 mm/px, and every station below comes from that table:\n"
        "front sight block z 14 to 46, gas block 105 to 143, exposed gas tube 143 to 212, handguards 210 to 336, receiver 372 to 620 with its floor dropping 175 to 165 and its cover crown at 240, magazine 340 to 505, trigger guard 508 to 562, pistol grip 588 to 640, buttstock 640 to 880 with the toe at z 858 and the heel at 877.\n"
        "\n"
        "Seven parts.\n"
        "front_sight: muzzle nut, tapered base collar, and a tower built as a front post, a rear buttress and a bridge over the top, so the window between them measures z 18 to 34 by y 218 to 229 as the reference does; cylindrical post housing at y 250 with the post reaching 256.\n"
        "barrel: 13.7 diameter ahead of the gas block and 15.3 behind, cleaning rod carried aft to z 340, gas block with a chamfered crown, a barrel collar and a diagonal riser, gas tube waisted to 16 diameter over the slotted section where a dark floor framed by two rails and five cross ribs leaves four ports, handguard retainer.\n"
        "handguards: upper and lower wooden shells swept along authored sections rather than plain rounded rectangles, the lower one with flat cheeks carrying a finger groove and a narrowed underside, the upper a rounded trapezoid drawn in toward its crown; they meet at a 1.5 seam that hides the barrel exactly as the reference does, and end in a 14 wide steel retaining band at z 338 to 352, aft of which the receiver front carries the junction down to y 171.\n"
        "receiver: core at half-width 14.0 with side plates out to 17.2 laid down as panels around two real openings, the milled lightening cut z 380 to 458 on both flanks and, on the right only, the ejection port z 395 to 455 opening into a cavity that shows the bolt carrier plus the separate 7 mm charging-handle track z 455 to 540 cut only through the 3.2 wall; both windows get 5 mm corner fillets and a 2.5 mm 45-degree rim bevel, since 3.2 mm of depth in a 34.4 wide receiver cannot define a pocket on its own. Dust cover swept on a domed section, near-vertical sided with the crown over the top third. Rear sight base tapering in plan with leaf, rails, slider and tangent lever; right-hand selector lever standing 3.3 proud on its pivot boss, charging handle, rivets.\n"
        "magazine: both walls are arcs about a common centre at (z 248, y 202) on the bore line, radius 177.1 front and 236.5 rear, swept from -8 to -57.5 degrees, with five pressed ribs a side and a floorplate.\n"
        "fire_control: trigger guard bow 6.2 deep and 16 wide, swept through six frames so it follows the measured sag from y 135.8 at z 510 to 132.3 at 550 as one continuous contour rather than running as a straight bar, its two webs, trigger, magazine catch, wooden pistol grip swept along an axis raked 17 degrees with a palm swell at the heel.\n"
        "stock: wrist ferrule tapering into the wood, wooden butt swept through seventeen flat-cheeked sections so the comb rise at z 720 to 736 comes out as a curve rather than a step, the last section plane tilted onto the raked butt face so the rake is in the wood and not just the buttplate; left-side sling swivel loop.\n"
        "\n"
        "Widths are the weakest numbers here: no plan or front elevation was found, so only the receiver is measured (34.4 across, from the assembled rear receiver view in references/ak47/ref_06.jpg). Handguard 42, magazine 28, grip 31, and a buttstock tapering from 27 at the wrist to 40 at the butt, are all inferred from the receiver and should be treated as the least trustworthy dimensions in the model.\n"
        "Known simplifications, all repeatedly flagged in renders/ak47/v*/critique.md and all consequences of building from hexahedra and swept sections: the magazine ribs are swept strips standing 0.3 proud that fade out at both ends rather than true pressings; the gas block, front sight base and handguard band are chamfered prisms rather than forgings flowing into rounded barrel collars; and the receiver flanks are flat plates with milled pockets rather than a body that changes section along its length.\n"
        "The magazine ribs were queried in all four rounds as looking additive; references/ak47/ref_01.png shows the AK-47 steel magazine carrying raised longitudinal pressings, not recessed channels, so they stay raised and were only reduced in projection.",
    .init = Init,
    .unload = Unload,
    .parts = PARTS,
    .partCount = 7,
    .target = { 0.0f, 1.30f, 0.0f },
    .orbitRadius = 10.5f,
    .orbitHeight = 3.4f,
};
