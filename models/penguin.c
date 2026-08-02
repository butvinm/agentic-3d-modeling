#include "harness.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Units and frame
//
// Every constant below is in metres, measured off two references of the same species.
// references/penguin/ref_02.jpg is a front elevation of an upright adult: crown at y 205 px, feet on the snow at y 1360 px, so 1155 px is the standing height and one pixel is 0.866 mm at 1.000 m tall.
// references/penguin/ref_01.jpg is a true side elevation of a bird with a bowed neck; its torso is upright, so it supplies fore-aft depth only. It is 1.446x the front view's scale, fixed by matching the fattest station in both.
// Standing height is set to 1.000 m against Prevost's measured maximum of 1.08 m for 86 wild birds; Wikipedia's "110-120 cm" is body length lying flat, not height.
// y runs up from the snow, z forward toward the bill, x across. Vert() scales by UNIT, so one world unit is 100 mm and a grid square reads as 10 cm.
// ---------------------------------------------------------------------------

#define UNIT 10.0f

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

static void Quad(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(p1, p0), Vector3Subtract(p2, p0)));
    QuadN(b, p0, p1, p2, p3, n, n, n, n);
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
// Materials and groups
//
// lighting.fs gamma-corrects with pow(c, 1/2.2), so these read considerably lighter than the raw values once shaded.
// ---------------------------------------------------------------------------

typedef enum {
    MAT_BLACK, MAT_BACK, MAT_WHITE, MAT_CREAM, MAT_CREAM_MID, MAT_CREAM_DEEP,
    MAT_YELLOW, MAT_ORANGE, MAT_FOOT, MAT_COUNT
} MatId;

static const Color MAT_COLOR[MAT_COUNT] = {
    [MAT_BLACK]      = {  14,  14,  17, 255 },
    [MAT_BACK]       = {  26,  26,  27, 255 },
    [MAT_WHITE]      = { 236, 238, 240, 255 },
    [MAT_CREAM]      = { 233, 231, 216, 255 },
    [MAT_CREAM_MID]  = { 230, 222, 186, 255 },
    [MAT_CREAM_DEEP] = { 226, 206, 140, 255 },
    [MAT_YELLOW]     = { 242, 186,  40, 255 },
    [MAT_ORANGE]     = { 206,  96,  40, 255 },
    [MAT_FOOT]       = {  24,  22,  21, 255 },
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

static Group gBody, gBill, gFlippers, gTail, gFeet;

// ---------------------------------------------------------------------------
// Lofting
//
// A Section is a station on a spine: a centre, a half-extent ra across x, and a half-extent rb along the spine's in-plane normal.
// The ring is c + ay*cos(a) + ax*sin(a), so a = 0 lands on the ventral side of the body, on the trailing edge of a flipper, and on the underside of the bill; a = PI lands on the opposite one.
// ay x ax points along the sweep, which is what makes the quads wind outwards.
//
// A whole surface is one continuous ring grid, so plumage colour cannot come from splitting the mesh: instead each quad asks a Zone callback which material its centre falls in.
// The callback receives the surface point in metres, the ring angle, and the normalised arc position along the spine.
// ---------------------------------------------------------------------------

typedef struct {
    float x, y, z;
    float ra, rb;
} Section;

typedef struct {
    Vector3 c;
    Vector3 ax;
    Vector3 ay;
} Frame;

typedef int (*Zone)(Vector3 p, float a, float t);

#define FRAME_MAX 160

static float CatmullRom(float p0, float p1, float p2, float p3, float s)
{
    float s2 = s * s, s3 = s2 * s;
    return 0.5f * ((2.0f * p1) + (-p0 + p2) * s +
                   (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * s2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * s3);
}

static Section SectionAt(const Section *s, int n, float u)
{
    int i = (int)u;
    if (i > n - 2) i = n - 2;
    if (i < 0) i = 0;
    float f = u - (float)i;
    int a = (i > 0) ? i - 1 : 0;
    int d = (i + 2 < n) ? i + 2 : n - 1;
    Section out;
    out.x = CatmullRom(s[a].x, s[i].x, s[i + 1].x, s[d].x, f);
    out.y = CatmullRom(s[a].y, s[i].y, s[i + 1].y, s[d].y, f);
    out.z = CatmullRom(s[a].z, s[i].z, s[i + 1].z, s[d].z, f);
    out.ra = CatmullRom(s[a].ra, s[i].ra, s[i + 1].ra, s[d].ra, f);
    out.rb = CatmullRom(s[a].rb, s[i].rb, s[i + 1].rb, s[d].rb, f);
    if (out.ra < 0.0015f) out.ra = 0.0015f;
    if (out.rb < 0.0015f) out.rb = 0.0015f;
    return out;
}

// Subdivide an authored station list and turn it into frames whose in-plane normal follows the spine's tangent projected on the y-z plane.
static int Frames(Frame *out, const Section *s, int n, int sub)
{
    int nf = (n - 1) * sub + 1;
    if (nf > FRAME_MAX) nf = FRAME_MAX;

    Section fine[FRAME_MAX];
    for (int i = 0; i < nf; i++) {
        fine[i] = SectionAt(s, n, (float)(n - 1) * (float)i / (float)(nf - 1));
    }

    for (int i = 0; i < nf; i++) {
        int a = (i > 0) ? i - 1 : i;
        int b = (i < nf - 1) ? i + 1 : i;
        float ty = fine[b].y - fine[a].y;
        float tz = fine[b].z - fine[a].z;
        float len = sqrtf(ty * ty + tz * tz);
        if (len < 1e-9f) { ty = 1.0f; tz = 0.0f; len = 1.0f; }
        ty /= len;
        tz /= len;
        out[i].c = (Vector3){ fine[i].x, fine[i].y, fine[i].z };
        out[i].ax = (Vector3){ fine[i].ra, 0.0f, 0.0f };
        out[i].ay = (Vector3){ 0.0f, -tz * fine[i].rb, ty * fine[i].rb };
    }
    return nf;
}

// Normals come from finite differences of the ring grid itself, so a station that tapers gets a normal tilted along the spine without any extra bookkeeping.
static void Loft(Group *g, const Frame *fr, int nf, int sides, Zone zone, bool capA, bool capB)
{
    int count = nf * sides;
    Vector3 *p = (Vector3 *)MemAlloc((unsigned int)count * sizeof(Vector3));
    Vector3 *nrm = (Vector3 *)MemAlloc((unsigned int)count * sizeof(Vector3));

    for (int i = 0; i < nf; i++) {
        for (int j = 0; j < sides; j++) {
            float a = 2.0f * PI * (float)j / (float)sides;
            p[i * sides + j] = Vector3Add(fr[i].c,
                Vector3Add(Vector3Scale(fr[i].ay, cosf(a)), Vector3Scale(fr[i].ax, sinf(a))));
        }
    }

    for (int i = 0; i < nf; i++) {
        for (int j = 0; j < sides; j++) {
            int jp = (j + 1) % sides, jm = (j + sides - 1) % sides;
            int ip = (i < nf - 1) ? i + 1 : i;
            int im = (i > 0) ? i - 1 : i;
            Vector3 da = Vector3Subtract(p[i * sides + jp], p[i * sides + jm]);
            Vector3 dt = Vector3Subtract(p[ip * sides + j], p[im * sides + j]);
            Vector3 n = Vector3CrossProduct(da, dt);
            if (Vector3LengthSqr(n) < 1e-16f) {
                n = Vector3Normalize(Vector3Add(Vector3Scale(fr[i].ay, cosf(2.0f * PI * (float)j / (float)sides)),
                                                Vector3Scale(fr[i].ax, sinf(2.0f * PI * (float)j / (float)sides))));
            }
            nrm[i * sides + j] = Vector3Normalize(n);
        }
    }

    // A quad whose four corners fall in different zones is split SPLIT x SPLIT and each piece placed on its own, because a plumage boundary assigned a whole quad at a time stair-steps one station per step and that is the most visible artefact on the whole model. Only boundary quads pay for it.
    const int SPLIT = 6;
    for (int i = 0; i < nf - 1; i++) {
        float da = 2.0f * PI / (float)sides;
        float t0 = (float)i / (float)(nf - 1), t1 = (float)(i + 1) / (float)(nf - 1);
        for (int j = 0; j < sides; j++) {
            int jp = (j + 1) % sides;
            Vector3 q[4] = { p[i * sides + j], p[i * sides + jp],
                             p[(i + 1) * sides + jp], p[(i + 1) * sides + j] };
            Vector3 n[4] = { nrm[i * sides + j], nrm[i * sides + jp],
                             nrm[(i + 1) * sides + jp], nrm[(i + 1) * sides + j] };
            float a0 = da * (float)j, a1 = da * (float)(j + 1);

            int m0 = zone(q[0], a0, t0), m1 = zone(q[1], a1, t0);
            int m2 = zone(q[2], a1, t1), m3 = zone(q[3], a0, t1);

            if (m0 == m1 && m1 == m2 && m2 == m3) {
                Vector3 mid = Vector3Scale(Vector3Add(Vector3Add(q[0], q[1]), Vector3Add(q[2], q[3])), 0.25f);
                QuadN(&g->b[zone(mid, a0 + 0.5f * da, 0.5f * (t0 + t1))], q[0], q[1], q[2], q[3],
                      n[0], n[1], n[2], n[3]);
                continue;
            }

            for (int l = 0; l < SPLIT; l++) {
                for (int k = 0; k < SPLIT; k++) {
                    float u[2] = { (float)k / SPLIT, (float)(k + 1) / SPLIT };
                    float v[2] = { (float)l / SPLIT, (float)(l + 1) / SPLIT };
                    Vector3 sp[4], sn[4];
                    const int cu[4] = { 0, 1, 1, 0 }, cv[4] = { 0, 0, 1, 1 };
                    for (int c = 0; c < 4; c++) {
                        float uu = u[cu[c]], vv = v[cv[c]];
                        Vector3 top = Vector3Add(Vector3Scale(q[0], (1.0f - uu)), Vector3Scale(q[1], uu));
                        Vector3 bot = Vector3Add(Vector3Scale(q[3], (1.0f - uu)), Vector3Scale(q[2], uu));
                        sp[c] = Vector3Add(Vector3Scale(top, 1.0f - vv), Vector3Scale(bot, vv));
                        Vector3 tn = Vector3Add(Vector3Scale(n[0], (1.0f - uu)), Vector3Scale(n[1], uu));
                        Vector3 bn = Vector3Add(Vector3Scale(n[3], (1.0f - uu)), Vector3Scale(n[2], uu));
                        sn[c] = Vector3Normalize(Vector3Add(Vector3Scale(tn, 1.0f - vv), Vector3Scale(bn, vv)));
                    }
                    Vector3 mid = Vector3Scale(Vector3Add(Vector3Add(sp[0], sp[1]), Vector3Add(sp[2], sp[3])), 0.25f);
                    float ma = a0 + da * 0.5f * (u[0] + u[1]);
                    float mt = t0 + (t1 - t0) * 0.5f * (v[0] + v[1]);
                    QuadN(&g->b[zone(mid, ma, mt)], sp[0], sp[1], sp[2], sp[3], sn[0], sn[1], sn[2], sn[3]);
                }
            }
        }
    }

    if (capA) {
        Vector3 n = Vector3Normalize(Vector3Negate(Vector3CrossProduct(fr[0].ay, fr[0].ax)));
        Builder *b = &g->b[zone(fr[0].c, 0.0f, 0.0f)];
        int centre = Vert(b, fr[0].c, n);
        for (int j = 0; j < sides; j++) {
            int jp = (j + 1) % sides;
            int i1 = Vert(b, p[jp], n), i0 = Vert(b, p[j], n);
            Tri(b, centre, i1, i0);
        }
    }
    if (capB) {
        int last = nf - 1;
        Vector3 n = Vector3Normalize(Vector3CrossProduct(fr[last].ay, fr[last].ax));
        Builder *b = &g->b[zone(fr[last].c, 0.0f, 1.0f)];
        int centre = Vert(b, fr[last].c, n);
        for (int j = 0; j < sides; j++) {
            int jp = (j + 1) % sides;
            int i0 = Vert(b, p[last * sides + j], n), i1 = Vert(b, p[last * sides + jp], n);
            Tri(b, centre, i0, i1);
        }
    }

    MemFree(p);
    MemFree(nrm);
}

// ---------------------------------------------------------------------------
// Plumage zones
//
// The pattern is one continuous map over the body surface, so it is described the same way: a ventral half-angle that widens toward the feet, a bib line that the black head stops at, and an ear patch straddling both.
// ---------------------------------------------------------------------------

// Height at which the black chin gives way to the pale throat. ref_11 puts that boundary at y 355 px, 0.874 of the way up the bird.
#define H_BIB 0.874f

static float Table(const float *xs, const float *ys, int n, float x)
{
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    for (int i = 1; i < n; i++) {
        if (x <= xs[i]) {
            float f = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + f * (ys[i] - ys[i - 1]);
        }
    }
    return ys[n - 1];
}

// The white wraps well past the widest line, which is why ref_02 reads as an almost entirely white bird seen head on with only a thin dark flank showing.
// Two independent readings agree: on ref_02 the boundary projects at 0.95 of the half-width at mid-body, and on ref_11 the same boundary sits 0.065 m behind the axis on a 0.200 m half-depth, which is 109 degrees. The front view alone cannot tell 71 degrees from 109; the side view resolves it.
// Over the neck the two views disagree, because ref_11's bird is turned a few degrees off a true profile and the boundary there runs almost along the line of sight. ref_02 is a square front elevation, so the neck is taken from it: at y 380 px the pale throat covers 175 px of a 225 px neck, which is 48 degrees, and the black comes down each side of the neck to meet the flanks. That dark band either side of the throat is what stops the head reading as a cap floating on a pale tube.
static float VentralAngle(float h)
{
    static const float hh[] = { 0.05f, 0.15f, 0.30f, 0.45f, 0.60f, 0.72f, 0.78f, 0.82f, 0.86f };
    static const float aa[] = { 2.40f, 2.20f, 2.05f, 1.95f, 1.88f, 1.70f, 1.45f, 1.15f, 0.92f };
    return Table(hh, aa, 9, h);
}

// The bib is not a level line: seen head on it dips at the sides, so the black cheek reaches lower than the black chin.
static float BibHeight(float d)
{
    return H_BIB - 0.030f * (d / PI);
}

static int BodyZone(Vector3 p, float a, float t)
{
    (void)t;
    float d = (a > PI) ? (2.0f * PI - a) : a;
    float h = p.y;

    // Ear patch: ref_11 puts it over y 280 to 400 px and 0.030 to 0.077 m behind the axis, so it straddles the bib line and sits just aft of the side, behind the eye rather than on the crown.
    // It is a comma, not a badge: broad and well aft at the top, sweeping down and forward until its lower end runs into the pale throat. The centre angle and the width both ride on the vertical coordinate, and the sweep is large enough that the bottom of the patch crosses the ventral boundary instead of sitting marooned in black.
    // A wider, paler field of the same shape surrounds it, because the references show cream beginning high and aft and only becoming yellow near the crown.
    float u = (h - 0.884f) / 0.052f;
    float cd = 1.80f + 0.55f * u;
    float wd = 0.46f * (0.55f + 0.225f * (u + 1.0f));
    float v = (d - cd) / wd;
    float r2 = u * u + v * v;
    if (r2 < 1.0f) return MAT_YELLOW;
    if (r2 < 1.9f) return MAT_CREAM_DEEP;

    if (h > BibHeight(d)) return MAT_BLACK;
    // The dorsum is one tone all the way down. An earlier version switched from head black to the greyer back at 0.800 m, which drew a hard level line right across the back at the same height the flipper roots emerge and made the shoulders look like a separate cap fitted onto the torso. The only tonal break left is the bib, where the bird has one.
    if (d > VentralAngle(h)) return MAT_BACK;

    // The breast fades from the bib down to white. Four steps rather than two: the gradient a texture would carry has to be spent on more, smaller bands or it reads as painted stripes.
    if (h > 0.815f) return MAT_CREAM_DEEP;
    if (h > 0.775f) return MAT_CREAM_MID;
    if (h > 0.730f) return MAT_CREAM;
    return MAT_WHITE;
}

static int BillZone(Vector3 p, float a, float t)
{
    (void)p;
    float d = (a > PI) ? (2.0f * PI - a) : a;
    if (t > 0.12f && t < 0.84f && d > 0.55f && d < 1.15f) return MAT_ORANGE;
    return MAT_BLACK;
}

static int FlipperZone(Vector3 p, float a, float t)
{
    (void)p;
    (void)t;
    // The white underside wraps round the leading edge as a narrow stripe. Every side elevation shows that pale line, and it is the only thing that separates the flipper from the flank behind it, which is black on black: ref_11 puts the blade's leading edge exactly on the plumage boundary.
    if (a > PI - 0.11f && a < PI + 0.11f) return MAT_WHITE;
    if (a <= PI) return MAT_BLACK;                      // outer face
    if (a > 2.0f * PI - 0.45f) return MAT_BLACK;        // trailing margin wrapping onto the inner face
    return MAT_WHITE;
}

static int TailZone(Vector3 p, float a, float t)
{
    (void)p;
    (void)a;
    (void)t;
    return MAT_BLACK;
}

// ---------------------------------------------------------------------------
// Body: a vertical spine from the underside to the crown
//
// Half-widths come from ref_02 column by column, half-depths from ref_11 the same way. The body is 0.366 m wide and 0.404 m deep at its fattest, so it is 12 per cent deeper fore and aft than it is across, not a solid of revolution.
// All three side elevations put the centre of the head over the centre of the body to within 3 mm: ref_11 reads 675 px for the body axis at its fattest and 677 px for the head at mid-height. The neck does not lean forward at all, so the spine stays on z = 0 and only the bill projects. An earlier version raked the spine forward through the neck and the bird came out permanently stooped.
// ---------------------------------------------------------------------------

static const Section BODY[] = {
    { 0.0f, 0.002f, 0.012f, 0.070f, 0.064f },
    { 0.0f, 0.008f, 0.012f, 0.094f, 0.086f },
    { 0.0f, 0.018f, 0.010f, 0.102f, 0.094f },
    { 0.0f, 0.040f, 0.008f, 0.110f, 0.108f },
    { 0.0f, 0.075f, 0.005f, 0.119f, 0.126f },
    { 0.0f, 0.108f, 0.003f, 0.127f, 0.138f },
    { 0.0f, 0.175f, 0.002f, 0.147f, 0.155f },
    { 0.0f, 0.242f, 0.001f, 0.163f, 0.170f },
    { 0.0f, 0.309f, 0.000f, 0.172f, 0.187f },
    { 0.0f, 0.376f, 0.000f, 0.178f, 0.198f },
    { 0.0f, 0.444f, 0.000f, 0.181f, 0.202f },
    { 0.0f, 0.511f, 0.000f, 0.182f, 0.200f },
    { 0.0f, 0.578f, 0.000f, 0.178f, 0.194f },
    { 0.0f, 0.646f, 0.000f, 0.167f, 0.183f },
    { 0.0f, 0.713f, 0.000f, 0.156f, 0.168f },
    { 0.0f, 0.780f, 0.000f, 0.139f, 0.138f },
    { 0.0f, 0.800f, 0.000f, 0.137f, 0.132f },
    { 0.0f, 0.820f, 0.000f, 0.131f, 0.124f },
    { 0.0f, 0.840f, 0.000f, 0.120f, 0.115f },
    { 0.0f, 0.860f, 0.000f, 0.104f, 0.105f },
    { 0.0f, 0.880f, 0.000f, 0.086f, 0.098f },
    { 0.0f, 0.900f, 0.000f, 0.074f, 0.093f },
    { 0.0f, 0.914f, 0.000f, 0.067f, 0.090f },
    { 0.0f, 0.940f, 0.000f, 0.064f, 0.082f },
    { 0.0f, 0.958f, 0.000f, 0.059f, 0.076f },
    { 0.0f, 0.972f, 0.000f, 0.052f, 0.065f },
    { 0.0f, 0.983f, 0.000f, 0.042f, 0.053f },
    { 0.0f, 0.991f, 0.000f, 0.032f, 0.040f },
    { 0.0f, 0.996f, 0.000f, 0.021f, 0.026f },
    { 0.0f, 0.999f, 0.000f, 0.011f, 0.014f },
    { 0.0f, 1.000f, 0.000f, 0.003f, 0.004f },
};

// Bill: 126 mm of sweep, of which 0.090 to 0.216 in z is the 0.126 m outside the feathers; ref_11 measures 178 px from the feather line to the tip, which is 0.120 m.
// 30 by 35 mm where it leaves the head, taken from ref_11's y 225 to 272 px at the feather line.
// It rises 12 mm over the first four fifths and turns down again at the tip, which is the shape all three upright references show; only ref_01, whose bird has its neck bowed, has the bill pointing down.
static const Section BILL[] = {
    { 0.0f, 0.939f, 0.010f, 0.034f, 0.0460f },
    { 0.0f, 0.941f, 0.032f, 0.028f, 0.0380f },
    { 0.0f, 0.944f, 0.060f, 0.024f, 0.0320f },
    { 0.0f, 0.947f, 0.090f, 0.020f, 0.0260f },
    { 0.0f, 0.951f, 0.125f, 0.015f, 0.0190f },
    { 0.0f, 0.955f, 0.158f, 0.013f, 0.0160f },
    { 0.0f, 0.956f, 0.185f, 0.010f, 0.0120f },
    { 0.0f, 0.953f, 0.205f, 0.005f, 0.0065f },
    { 0.0f, 0.948f, 0.216f, 0.001f, 0.0015f },
};

// Flipper stations: height, azimuth round the body, and how far the blade's mid-plane stands off the body surface at that azimuth.
// ref_11 puts the blade's chord centre 0.101 m behind the axis on a 0.200 m half-depth, so it hangs at 120 degrees round from the breast rather than on the widest line. That is also where the plumage boundary runs, which is why the flipper's black merges into the flank's.
// A negative standoff buries the station inside the body, so the root cap never shows and the blade grows out of the shoulder.
typedef struct {
    float h, deg, gap, halfThick, halfChord;
} FlipperStation;

// Half-chords follow ref_11 station by station: 70 px at y 500, 92 at 700, 110 at 900, 94 at 1100, 60 at 1250. The blade is widest at 0.51 m off the snow and narrows both ways from there, rather than holding one width down most of its length.
static const FlipperStation FLIPPER[] = {
    { 0.870f, 120.0f, -0.036f, 0.011f, 0.016f },
    { 0.840f, 120.0f, -0.018f, 0.011f, 0.028f },
    { 0.790f, 120.0f, -0.004f, 0.011f, 0.036f },
    { 0.700f, 120.0f,  0.001f, 0.011f, 0.041f },
    { 0.610f, 120.0f,  0.003f, 0.010f, 0.0435f },
    { 0.510f, 120.0f,  0.005f, 0.009f, 0.044f },
    { 0.420f, 120.0f,  0.007f, 0.008f, 0.040f },
    { 0.340f, 119.0f,  0.010f, 0.007f, 0.032f },
    { 0.280f, 118.0f,  0.014f, 0.006f, 0.021f },
    { 0.240f, 117.0f,  0.017f, 0.004f, 0.010f },
    { 0.215f, 116.0f,  0.018f, 0.002f, 0.003f },
};

// Tail: a short stiff wedge, broad and flat rather than conical, trailing back and down out of the rear of the body until it rests on the snow, as the left-hand bird in ref_06 does.
static const Section TAIL[] = {
    { 0.0f, 0.140f, -0.045f, 0.095f, 0.030f },
    { 0.0f, 0.120f, -0.090f, 0.090f, 0.020f },
    { 0.0f, 0.092f, -0.135f, 0.081f, 0.015f },
    { 0.0f, 0.060f, -0.180f, 0.068f, 0.011f },
    { 0.0f, 0.032f, -0.216f, 0.052f, 0.008f },
    { 0.0f, 0.016f, -0.245f, 0.036f, 0.006f },
    { 0.0f, 0.008f, -0.268f, 0.019f, 0.004f },
    { 0.0f, 0.006f, -0.286f, 0.004f, 0.002f },
};

static void BuildBody(void)
{
    Frame fr[FRAME_MAX];
    int nf = Frames(fr, BODY, (int)(sizeof(BODY) / sizeof(BODY[0])), 7);
    Loft(&gBody, fr, nf, 96, BodyZone, true, true);

    // Feathered tarsus: the belly plumage carries on down over the ankle, so the foot is not left hanging under a gap. It ends inside the ankle lump so its cap never shows.
    GroupMark gm = GroupMarkNow(&gBody);
    Tube(&gBody.b[MAT_WHITE], (Vector3){ 0.072f, 0.160f, -0.005f }, (Vector3){ 0.072f, 0.022f, 0.016f },
         0.055f, 0.038f, 20, false, true);
    GroupMirrorX(&gBody, gm);
}

static void BuildBill(void)
{
    Frame fr[FRAME_MAX];
    int nf = Frames(fr, BILL, (int)(sizeof(BILL) / sizeof(BILL[0])), 6);
    Loft(&gBill, fr, nf, 28, BillZone, false, true);
}

// Half-width and half-depth of the body at a given height, so the flipper can be hung off the body's own surface instead of a second set of hand-copied numbers that would drift out of step with it.
static void BodyEllipse(float h, float *rx, float *rz)
{
    int n = (int)(sizeof(BODY) / sizeof(BODY[0]));
    if (h <= BODY[0].y) { *rx = BODY[0].ra; *rz = BODY[0].rb; return; }
    for (int i = 1; i < n; i++) {
        if (h <= BODY[i].y) {
            float f = (h - BODY[i - 1].y) / (BODY[i].y - BODY[i - 1].y);
            *rx = BODY[i - 1].ra + f * (BODY[i].ra - BODY[i - 1].ra);
            *rz = BODY[i - 1].rb + f * (BODY[i].rb - BODY[i - 1].rb);
            return;
        }
    }
    *rx = BODY[n - 1].ra;
    *rz = BODY[n - 1].rb;
}

// The blade lies flat on the flank, so its thickness runs along the body's outward normal at that azimuth and its chord along the horizontal tangent. Building the frame from the ellipse normal rather than from world x is what keeps a blade hung at 120 degrees from cutting into the body on one edge and lifting off it on the other.
static int FlipperFrames(Frame *out, const FlipperStation *s, int n, int sub)
{
    Section spine[FRAME_MAX];
    for (int i = 0; i < n; i++) {
        float rx, rz;
        BodyEllipse(s[i].h, &rx, &rz);
        float a = s[i].deg * DEG2RAD;
        Vector3 nrm = Vector3Normalize((Vector3){ sinf(a) / rx, 0.0f, cosf(a) / rz });
        spine[i].x = rx * sinf(a) + nrm.x * s[i].gap;
        spine[i].y = s[i].h;
        spine[i].z = rz * cosf(a) + nrm.z * s[i].gap;
        spine[i].ra = s[i].halfThick;
        spine[i].rb = s[i].halfChord;
    }

    int nf = (n - 1) * sub + 1;
    if (nf > FRAME_MAX) nf = FRAME_MAX;
    for (int i = 0; i < nf; i++) {
        float u = (float)(n - 1) * (float)i / (float)(nf - 1);
        Section c = SectionAt(spine, n, u);

        int k = (int)u;
        if (k > n - 2) k = n - 2;
        float f = u - (float)k;
        float deg = s[k].deg + f * (s[k + 1].deg - s[k].deg);
        float rx, rz;
        BodyEllipse(c.y, &rx, &rz);
        float a = deg * DEG2RAD;
        Vector3 nrm = Vector3Normalize((Vector3){ sinf(a) / rx, 0.0f, cosf(a) / rz });
        Vector3 tan = { nrm.z, 0.0f, -nrm.x };

        out[i].c = (Vector3){ c.x, c.y, c.z };
        out[i].ax = Vector3Scale(nrm, c.ra);
        out[i].ay = Vector3Scale(tan, c.rb);
    }
    return nf;
}

static void BuildFlippers(void)
{
    Frame fr[FRAME_MAX];
    int nf = FlipperFrames(fr, FLIPPER, (int)(sizeof(FLIPPER) / sizeof(FLIPPER[0])), 5);
    GroupMark gm = GroupMarkNow(&gFlippers);
    Loft(&gFlippers, fr, nf, 32, FlipperZone, false, true);
    GroupMirrorX(&gFlippers, gm);
}

static void BuildTail(void)
{
    Frame fr[FRAME_MAX];
    int nf = Frames(fr, TAIL, (int)(sizeof(TAIL) / sizeof(TAIL[0])), 5);
    Loft(&gTail, fr, nf, 24, TailZone, false, true);
}

// ---------------------------------------------------------------------------
// Feet
//
// Three forward toes joined by webbing, each ending in a claw, on an ankle lump that the tarsus plumage covers.
// The toes reach 0.10 m in front of the lowest belly, which is what ref_01 measures once its scale is divided out.
// ---------------------------------------------------------------------------

#define FOOT_X 0.072f

static const float TOE_DIR[3][2] = { { -0.34f, 0.940f }, { 0.03f, 0.9995f }, { 0.37f, 0.929f } };
static const float TOE_LEN[3] = { 0.072f, 0.078f, 0.070f };

static Vector3 ToePoint(int i, float s)
{
    return (Vector3){
        FOOT_X + TOE_DIR[i][0] * TOE_LEN[i] * s,
        0.013f,
        0.047f + TOE_DIR[i][1] * TOE_LEN[i] * s,
    };
}

static void BuildWeb(Builder *b, int i0, int i1)
{
    const int steps = 6;
    const float reach = 0.92f;
    for (int k = 0; k < steps; k++) {
        float s0 = reach * (float)k / (float)steps;
        float s1 = reach * (float)(k + 1) / (float)steps;
        Vector3 a0 = ToePoint(i0, s0), a1 = ToePoint(i0, s1);
        Vector3 b0 = ToePoint(i1, s0), b1 = ToePoint(i1, s1);

        Vector3 top[4] = {
            { a0.x, 0.0235f, a0.z }, { a1.x, 0.0235f, a1.z }, { b1.x, 0.0235f, b1.z }, { b0.x, 0.0235f, b0.z },
        };
        Quad(b, top[0], top[1], top[2], top[3]);

        Vector3 bot[4] = {
            { a0.x, 0.0025f, a0.z }, { b0.x, 0.0025f, b0.z }, { b1.x, 0.0025f, b1.z }, { a1.x, 0.0025f, a1.z },
        };
        Quad(b, bot[0], bot[1], bot[2], bot[3]);
    }
}

static int FootZone(Vector3 p, float a, float t)
{
    (void)p;
    (void)a;
    (void)t;
    return MAT_FOOT;
}

// Toes are lofted on a flattened section, 38 mm across by 22 mm deep at the base, rather than swept as round tubes. A round 40 mm toe on 75 mm of length reads as a rod, and the pair of feet came out looking taloned; the references show two low, wide, dark masses in which the toe divisions are a subordinate detail.
static void BuildToe(int i)
{
    static const float ss[5] = { -0.20f, 0.10f, 0.45f, 0.78f, 1.00f };
    static const float ra[5] = { 0.019f, 0.019f, 0.017f, 0.014f, 0.010f };
    static const float rb[5] = { 0.011f, 0.011f, 0.010f, 0.009f, 0.007f };

    Section st[5];
    for (int k = 0; k < 5; k++) {
        Vector3 p = ToePoint(i, ss[k]);
        st[k] = (Section){ p.x, p.y, p.z, ra[k], rb[k] };
    }

    Frame fr[FRAME_MAX];
    int nf = Frames(fr, st, 5, 5);
    Loft(&gFeet, fr, nf, 16, FootZone, true, true);
}

static void BuildFeet(void)
{
    GroupMark gm = GroupMarkNow(&gFeet);
    Builder *foot = &gFeet.b[MAT_FOOT];

    Tube(foot, (Vector3){ FOOT_X, 0.040f, 0.002f }, (Vector3){ FOOT_X, 0.016f, 0.055f },
         0.038f, 0.026f, 20, true, true);

    for (int i = 0; i < 3; i++) {
        BuildToe(i);

        Vector3 tip = ToePoint(i, 1.0f);
        Vector3 claw = {
            tip.x + TOE_DIR[i][0] * 0.009f,
            tip.y - 0.002f,
            tip.z + TOE_DIR[i][1] * 0.009f,
        };
        Tube(&gFeet.b[MAT_BLACK], tip, claw, 0.008f, 0.0020f, 12, false, true);
    }

    BuildWeb(foot, 0, 1);
    BuildWeb(foot, 1, 2);

    GroupMirrorX(&gFeet, gm);
}

// ---------------------------------------------------------------------------

static void Init(void)
{
    BuildBody();
    BuildBill();
    BuildFlippers();
    BuildTail();
    BuildFeet();

    GroupFinish(&gBody, "body");
    GroupFinish(&gBill, "bill");
    GroupFinish(&gFlippers, "flippers");
    GroupFinish(&gTail, "tail");
    GroupFinish(&gFeet, "feet");
}

static void Unload(void)
{
    GroupUnload(&gBody);
    GroupUnload(&gBill);
    GroupUnload(&gFlippers);
    GroupUnload(&gTail);
    GroupUnload(&gFeet);
}

static void DrawBody(void) { GroupDraw(&gBody); }
static void DrawBill(void) { GroupDraw(&gBill); }
static void DrawFlippers(void) { GroupDraw(&gFlippers); }
static void DrawTail(void) { GroupDraw(&gTail); }
static void DrawFeet(void) { GroupDraw(&gFeet); }

static BoundingBox BodyBounds(void) { return gBody.bounds; }
static BoundingBox BillBounds(void) { return gBill.bounds; }
static BoundingBox FlipperBounds(void) { return gFlippers.bounds; }
static BoundingBox TailBounds(void) { return gTail.bounds; }
static BoundingBox FeetBounds(void) { return gFeet.bounds; }

static const Part PARTS[] = {
    { .name = "body", .draw = DrawBody, .bounds = BodyBounds },
    { .name = "bill", .draw = DrawBill, .bounds = BillBounds },
    { .name = "flippers", .draw = DrawFlippers, .bounds = FlipperBounds },
    { .name = "tail", .draw = DrawTail, .bounds = TailBounds },
    { .name = "feet", .draw = DrawFeet, .bounds = FeetBounds },
};

const Scene SCENE = {
    .name = "penguin",
    .description =
        "Adult emperor penguin (Aptenodytes forsteri) standing upright, the pose of references/penguin/ref_02.jpg and ref_11.jpg.\n"
        "\n"
        "One world unit is 100 mm, so a grid square is 10 cm. Source constants are in metres: y up from the snow, z forward toward the bill, x across.\n"
        "Standing height 1.000 m. Body 0.366 m wide at 0.51 m off the snow and 0.404 m deep at 0.44 m, so it is 12 per cent deeper fore and aft than across. Head 0.134 wide by 0.166 deep by 0.126 tall. Bill 0.126 m of sweep out of the head. Flipper 0.64 m root to tip. Foot 0.16 m including claws, the pair 0.144 m apart. Tail reaches 0.15 m behind the body.\n"
        "\n"
        "The proportions are measured, not estimated, off three elevations of three different birds.\n"
        "ref_02 is a square front elevation: crown at y 205 px, feet at y 1360 px, so 1155 px is the standing height and one pixel is 0.866 mm. Every half-width comes from it.\n"
        "ref_11 is an upright side elevation: crown at y 172 px, snow at y 1660 px, 1488 px and 0.672 mm per pixel. Every half-depth comes from it, and so does the head, the neck and the bill.\n"
        "ref_10 is a second upright side elevation used to check ref_11. It gives a maximum depth of 0.46 rather than 0.40 of standing height; that bird is simply fatter, and ref_11 was kept because its flipper is separable from its flank and ref_10's is not.\n"
        "ref_01 is a true side elevation but its bird has its neck bowed, so it is used only for the feet, which is the one place it is unambiguous.\n"
        "Height is set to 1.000 m against Prevost 1961's measured maximum of 1.08 m over 86 wild birds. The often-quoted 110-120 cm is body length lying flat and would have made the model a fifth too big.\n"
        "\n"
        "Five parts.\n"
        "body: one continuous loft of 31 authored stations subdivided to 155, 96 sides, from the closed underside up the torso and neck to the crown. The belly holds 0.100 m of half-width down to 0.02 m off the snow and closes on a disc that only the ground sees, so the plumage skirt covers the ankles; an earlier version tapered it to a point from 0.10 m up and left the two tarsus tubes standing out like pegs. The spine is vertical throughout. All three side elevations put the centre of the head over the centre of the body to within 3 mm, so the neck does not lean forward at all and only the bill projects; an earlier version raked the spine forward through the neck and the bird came out permanently stooped. The crown is a dome whose slope steepens toward the apex rather than the cone a linear taper gives. The shoulder-to-neck reduction is spread over seven stations from 0.780 to 0.914 m; taking it in three left a bottle-shaped ledge under the head. A feathered tarsus tube carries the belly plumage down over each ankle.\n"
        "bill: separate loft of 10 stations. It starts as a 68 by 92 mm wedge buried at the centre of the head and is down to 40 by 52 mm by the time it clears the feathers at z 0.075, so the face flows into it instead of the bill appearing to sprout from a pinched waist. From there it is 40 by 52 mm, holding depth through the middle and taking most of its taper in the last fifth. It rises 12 mm over the first four fifths and turns down again at the tip. Every upright reference shows that shape; only ref_01, whose bird is bowed, has the bill pointing down.\n"
        "flippers: a blade of 11 stations hung at 120 degrees round from the breast rather than on the widest line, which is where ref_11 puts its chord centre, 0.101 m behind the axis on a 0.200 m half-depth. Each station's frame is built from the body's own ellipse normal at that azimuth, so the blade lies flat on the flank instead of cutting in on one edge and lifting off on the other. The chord follows ref_11 station by station, 70 px at y 500 through 110 px at y 900 to 60 px at y 1250, so it is widest at 0.51 m off the snow with an 88 mm full chord and narrows both ways, taking most of its taper in the last third rather than running parallel-sided to a long point; the first two stations sit inside the body so the root cap never shows.\n"
        "tail: a wedge of 8 stations, 0.190 m across the root and 60 mm thick there thinning to 26 mm as soon as it leaves the rump, so it grows out of the plumage instead of slicing through it. It drops steeply for its first half and then turns onto the snow, the last third running almost level to a sharp point 0.29 m behind the axis, as the left-hand bird in ref_06 does. It is a flat blade, not a paddle: the references show the tail broad and thin and visually part of the rump.\n"
        "feet: ankle lump, three forward toes splayed at -20, +2 and +22 degrees, webbing over the first 92 per cent of their length, and a short claw on each. Each toe is lofted on a flattened section, 38 mm across by 22 mm deep at the base over 72 to 78 mm of length, not swept as a round tube: a round 40 mm toe on that length reads as a rod and the pair came out looking taloned, where the references show two low, wide, dark masses whose toe divisions are a subordinate detail. The toes clear the lowest belly by 0.03 m; ref_01 puts that at 0.10 m and ref_11 at zero, the two birds stand differently, and the model splits them.\n"
        "\n"
        "Plumage colour is a map over the surface, not a mesh split: every quad of a loft asks a zone callback which material its centre falls in, keyed on the angle round the ring and the height. A quad whose four corners disagree is split six by six and each piece placed separately, because a boundary assigned a whole quad at a time stair-steps one station per step and that was the most visible artefact on the model. Only boundary quads pay for it, so the cost is proportional to the length of the boundary rather than to the area of the bird.\n"
        "The dorsum is one tone from the bib to the tail. An earlier version switched from head black to a greyer back at 0.800 m and that drew a hard level line across the back at exactly the height the flipper roots emerge, which made the shoulders read as a separate cap fitted onto the torso. The only tonal break on the dorsal side is now the bib, where the bird has one.\n"
        "The ventral white wraps further round the flanks the lower it gets, from 53 degrees off the breast centreline at the throat to 137 at the belly. That single number is what makes ref_02 read as an almost entirely white bird head on while ref_11 reads as an almost entirely black one in profile. The front view alone cannot tell 71 degrees from 109 because both project to the same place; the side view resolves it.\n"
        "The black head stops at a bib line that dips 30 mm lower at the sides than at the chin; the breast fades from deep cream to white in four steps; and the flipper carries a white stripe round its leading edge, which is the only thing separating it from the flank behind it, that being black on black.\n"
        "The ear patch is a comma rather than a badge: its centre angle and its width both ride on height, so it is broad and well aft at the top and sweeps down and forward until its lower end crosses the ventral boundary and runs into the pale throat. A wider field of the same shape surrounds it in deep cream, because the references show cream beginning high and aft and only becoming yellow near the crown. Left as a plain ellipse it read as a disc stuck on the side of the head.\n"
        "\n"
        "The maximum forward projection of the breast is at 0.44 of standing height, not high on the torso: ref_11's front contour reads x 420 px at y 600, 375 px at y 1000 and 430 px at y 1400, so the belly's most forward point is 42 per cent of the way up. A review round called this out as too low and it was checked and kept.\n"
        "\n"
        "Known simplifications. The eye is not modelled: it is dark brown on jet black plumage and would not read. The plumage gradients are discrete bands rather than continuous. Feather texture is absent entirely, so the body reads smoother than a real bird and its lower belly is a clean surface where a real one has a ragged plumage skirt. The bill sweep measures 0.126 m against the 8 cm exposed culmen usually quoted, because the culmen is measured from further forward than the feather line this sweep starts at; ref_11 measures 178 px from feather line to tip, which is 0.120 m, and ref_02 agrees.",
    .init = Init,
    .unload = Unload,
    .parts = PARTS,
    .partCount = 5,
    .target = { 0.0f, 5.0f, 0.0f },
    .orbitRadius = 15.0f,
    .orbitHeight = 4.5f,
};
