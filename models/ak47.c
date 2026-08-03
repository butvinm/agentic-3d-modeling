#include "harness.h"
#include "raymath.h"
#include "rlgl.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Units and frame
//
// Every constant below is in millimetres, taken from a measured side elevation (references/ak47/ref_04.png, a 2450x950 background-free photograph of a Type 2 AK-47 whose silhouette was sampled column by column at 0.3915 mm/px).
// z runs aft from the muzzle face, y up from the magazine floorplate, x across.
// Vert() is the single place where that turns into world space: it scales by UNIT and slides the origin to mid-length, so one world unit is 100 mm and the grid squares read as 10 cm.
//
// The rifle is posed: four bodies move, and each is authored about its own pivot instead of being baked into the model frame. gOrigin is subtracted as every vertex is emitted, so a group's vertices come out relative to the point it turns or slides about and its rest pose is the translation back to that point. A static group leaves gOrigin at the model origin and builds in the model frame exactly as it would with no posing at all, which is what lets the seven static parts stay written in the rifle's own measured millimetres.
// ---------------------------------------------------------------------------

#define UNIT      0.01f
#define Z_ORIGIN  440.0f

#define BORE      204.4f   // barrel axis height
#define ROD_Y     191.0f   // cleaning rod axis
#define GAS_Y     228.8f   // gas tube axis

// The gas tube's two ends, named because the piston has to stay between them and reads its rest position off the front one.
#define GASTUBE_Z0 141.0f
#define GASTUBE_Z1 326.0f

static Vector3 gOrigin = { 0.0f, 0.0f, Z_ORIGIN };

// Model millimetres to world units. The only place the two frames meet, so a pivot written in millimetres below turns into the translation that places its body.
static Vector3 ToWorld(Vector3 p)
{
    return (Vector3){ p.x * UNIT, p.y * UNIT, (p.z - Z_ORIGIN) * UNIT };
}

// ---------------------------------------------------------------------------
// Mesh builder
//
// Surfaces are hexahedra (eight freely placed corners), surfaces of revolution about an arbitrary segment, or sections swept along a frame list.
// ---------------------------------------------------------------------------

typedef struct {
    float *vertices;
    float *normals;
    float *texcoords;
    unsigned char *colors;
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
        b->colors = (unsigned char *)MemRealloc(b->colors, (size_t)cap * 4 * sizeof(unsigned char));
        b->vertexCap = cap;
    }
    if (b->triangleCount + tris > b->triangleCap) {
        int cap = (b->triangleCap > 0) ? b->triangleCap : 512;
        while (cap < b->triangleCount + tris) cap *= 2;
        b->indices = (unsigned short *)MemRealloc(b->indices, (size_t)cap * 3 * sizeof(unsigned short));
        b->triangleCap = cap;
    }
}

static int Vert(Builder *b, Vector3 p, Vector3 n, Vector2 uv)
{
    Reserve(b, 1, 0);
    int i = b->vertexCount++;
    b->vertices[i * 3 + 0] = (p.x - gOrigin.x) * UNIT;
    b->vertices[i * 3 + 1] = (p.y - gOrigin.y) * UNIT;
    b->vertices[i * 3 + 2] = (p.z - gOrigin.z) * UNIT;
    b->normals[i * 3 + 0] = n.x;
    b->normals[i * 3 + 1] = n.y;
    b->normals[i * 3 + 2] = n.z;
    b->texcoords[i * 2 + 0] = uv.x;
    b->texcoords[i * 2 + 1] = uv.y;
    // White until something says otherwise, so every mesh built without a colour ramp is unchanged: both shaders multiply by the vertex colour, and white is the identity.
    b->colors[i * 4 + 0] = 255;
    b->colors[i * 4 + 1] = 255;
    b->colors[i * 4 + 2] = 255;
    b->colors[i * 4 + 3] = 255;
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

// ---------------------------------------------------------------------------
// Texture coordinates
//
// Carried in millimetres and divided by the material's repeat length in GroupFinish, so one texture repeat covers a fixed physical distance wherever it lands and the same map serves a 6 mm pin and a 240 mm buttstock.
// u runs along the part and v across it: for a flat face that is (z, y) or (z, x), for a tube the axial distance and the circumferential arc, for a swept section the path length and the section perimeter. Wood grain is drawn along u, so it follows the barrel axis on the handguards and the comb line on the stock.
// The projection is chosen per face rather than per vertex: a dominant-axis rule evaluated at each vertex flips part way round a cylinder and seams every barrel.
// ---------------------------------------------------------------------------

static Vector2 PlanarUV(Vector3 p, Vector3 n)
{
    float ax = fabsf(n.x), ay = fabsf(n.y), az = fabsf(n.z);
    if (az >= ax && az >= ay) return (Vector2){ p.x, p.y };
    if (ay >= ax) return (Vector2){ p.z, p.x };
    return (Vector2){ p.z, p.y };
}

static Vector2 PlaneUV(Vector3 p, Vector3 c, Vector3 ux, Vector3 uy, float uOff)
{
    Vector3 d = Vector3Subtract(p, c);
    return (Vector2){ uOff + Vector3DotProduct(d, ux), Vector3DotProduct(d, uy) };
}

// Flat-shaded quad. Winding p0-p1-p2-p3 must be counter-clockwise seen from outside.
static void Quad(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(p1, p0), Vector3Subtract(p2, p0)));
    int a = Vert(b, p0, n, PlanarUV(p0, n)), c = Vert(b, p1, n, PlanarUV(p1, n));
    int d = Vert(b, p2, n, PlanarUV(p2, n)), e = Vert(b, p3, n, PlanarUV(p3, n));
    Tri(b, a, c, d);
    Tri(b, a, d, e);
}

static void QuadUV(Builder *b, const Vector3 p[4], const Vector3 n[4], const Vector2 t[4])
{
    int a = Vert(b, p[0], n[0], t[0]), c = Vert(b, p[1], n[1], t[1]);
    int d = Vert(b, p[2], n[2], t[2]), e = Vert(b, p[3], n[3], t[3]);
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
        for (int c = 0; c < 4; c++) b->colors[(base + i) * 4 + c] = b->colors[(m.vert + i) * 4 + c];
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

// Ramp the vertex colour of everything added since `m` across one axis, between the extremes that range actually reaches.
// This is how the muzzle flash goes white at the muzzle and orange at its tips, and how a smoke puff is lighter on top, one mesh each. Neither can get that from the lighting shader: the flash is self-luminous and the smoke is transparent, so both are drawn unlit and this is the only shading they have.
static void ShadeRange(Builder *b, Mark m, int comp, Color lo, Color hi)
{
    if (b->vertexCount <= m.vert) return;

    float vmin = b->vertices[m.vert * 3 + comp], vmax = vmin;
    for (int i = m.vert; i < b->vertexCount; i++) {
        float v = b->vertices[i * 3 + comp];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    float span = vmax - vmin;
    if (span < 1e-9f) span = 1.0f;

    for (int i = m.vert; i < b->vertexCount; i++) {
        float t = (b->vertices[i * 3 + comp] - vmin) / span;
        b->colors[i * 4 + 0] = (unsigned char)((float)lo.r + ((float)hi.r - (float)lo.r) * t);
        b->colors[i * 4 + 1] = (unsigned char)((float)lo.g + ((float)hi.g - (float)lo.g) * t);
        b->colors[i * 4 + 2] = (unsigned char)((float)lo.b + ((float)hi.b - (float)lo.b) * t);
        b->colors[i * 4 + 3] = (unsigned char)((float)lo.a + ((float)hi.a - (float)lo.a) * t);
    }
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

    // Axial coordinate measured from the world origin along the tube's own axis, so segments butted end to end continue one pattern instead of each restarting it: the barrel is three tubes and would otherwise show a texture step at every junction.
    float uA = Vector3DotProduct(a, dir);
    float uB = uA + sqrtf(len * len + (r0 - r1) * (r0 - r1));
    float rm = 0.5f * (r0 + r1);

    for (int j = 0; j < sides; j++) {
        float t0 = 2.0f * PI * (float)j / (float)sides;
        float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
        Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
        Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
        Vector3 n0 = Vector3Normalize(Vector3Add(d0, Vector3Scale(dir, slant)));
        Vector3 n1 = Vector3Normalize(Vector3Add(d1, Vector3Scale(dir, slant)));

        Vector3 P[4] = {
            Vector3Add(a, Vector3Scale(d0, r0)), Vector3Add(a, Vector3Scale(d1, r0)),
            Vector3Add(b, Vector3Scale(d1, r1)), Vector3Add(b, Vector3Scale(d0, r1)),
        };
        Vector3 N[4] = { n0, n1, n1, n0 };
        Vector2 T[4] = { { uA, rm * t0 }, { uA, rm * t1 }, { uB, rm * t1 }, { uB, rm * t0 } };
        QuadUV(bd, P, N, T);
    }

    if (capA && r0 > 1e-6f) {
        Vector3 n = Vector3Negate(dir);
        int centre = Vert(bd, a, n, PlaneUV(a, a, u, v, uA));
        for (int j = 0; j < sides; j++) {
            float t0 = 2.0f * PI * (float)j / (float)sides;
            float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
            Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
            Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
            Vector3 q1 = Vector3Add(a, Vector3Scale(d1, r0)), q0 = Vector3Add(a, Vector3Scale(d0, r0));
            int p1 = Vert(bd, q1, n, PlaneUV(q1, a, u, v, uA));
            int p0 = Vert(bd, q0, n, PlaneUV(q0, a, u, v, uA));
            Tri(bd, centre, p1, p0);
        }
    }
    if (capB && r1 > 1e-6f) {
        int centre = Vert(bd, b, dir, PlaneUV(b, b, u, v, uB));
        for (int j = 0; j < sides; j++) {
            float t0 = 2.0f * PI * (float)j / (float)sides;
            float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
            Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
            Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
            Vector3 q0 = Vector3Add(b, Vector3Scale(d0, r1)), q1 = Vector3Add(b, Vector3Scale(d1, r1));
            int p0 = Vert(bd, q0, dir, PlaneUV(q0, b, u, v, uB));
            int p1 = Vert(bd, q1, dir, PlaneUV(q1, b, u, v, uB));
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

    // v is the perimeter walked in world space at each frame and u the distance travelled along the path, anchored at the first frame's z so a swept part and an abutting flat one carry the grain at the same rate.
    Vector3 p[SECT_MAX], q[SECT_MAX], np[SECT_MAX], nq[SECT_MAX];
    float vp[SECT_MAX + 1], vq[SECT_MAX + 1];
    float up = fr[0].c.z, uq = up;
    for (int i = 0; i < nf; i++) {
        Vector3 *pos = (i == 0) ? p : q;
        Vector3 *nrm = (i == 0) ? np : nq;
        float *vs = (i == 0) ? vp : vq;
        float sx = Vector3Length(fr[i].ax), sy = Vector3Length(fr[i].ay);
        Vector3 ux = Vector3Scale(fr[i].ax, 1.0f / sx);
        Vector3 uy = Vector3Scale(fr[i].ay, 1.0f / sy);
        for (int j = 0; j < n; j++) {
            pos[j] = Vector3Add(fr[i].c,
                                Vector3Add(Vector3Scale(fr[i].ax, sect[j].x), Vector3Scale(fr[i].ay, sect[j].y)));
            nrm[j] = Vector3Normalize(Vector3Add(Vector3Scale(ux, sn[j].x / sx), Vector3Scale(uy, sn[j].y / sy)));
        }
        vs[0] = 0.0f;
        for (int j = 1; j <= n; j++) vs[j] = vs[j - 1] + Vector3Distance(pos[j % n], pos[j - 1]);
        if (i == 0) continue;
        uq = up + Vector3Distance(fr[i].c, fr[i - 1].c);
        for (int j = 0; j < n; j++) {
            int k = (j + 1) % n;
            Vector3 P[4] = { p[j], p[k], q[k], q[j] };
            Vector3 N[4] = { np[j], np[k], nq[k], nq[j] };
            Vector2 T[4] = { { up, vp[j] }, { up, vp[j + 1] }, { uq, vq[j + 1] }, { uq, vq[j] } };
            QuadUV(b, P, N, T);
        }
        for (int j = 0; j < n; j++) { p[j] = q[j]; np[j] = nq[j]; }
        for (int j = 0; j <= n; j++) vp[j] = vq[j];
        up = uq;
    }

    if (capA) {
        Vector3 nA = Vector3Normalize(Vector3Negate(Vector3CrossProduct(fr[0].ax, fr[0].ay)));
        Vector3 ux = Vector3Normalize(fr[0].ax), uy = Vector3Normalize(fr[0].ay);
        float off = fr[0].c.z;
        int centre = Vert(b, fr[0].c, nA, PlaneUV(fr[0].c, fr[0].c, ux, uy, off));
        for (int j = 0; j < n; j++) {
            int k = (j + 1) % n;
            Vector3 a = Vector3Add(fr[0].c, Vector3Add(Vector3Scale(fr[0].ax, sect[j].x), Vector3Scale(fr[0].ay, sect[j].y)));
            Vector3 c = Vector3Add(fr[0].c, Vector3Add(Vector3Scale(fr[0].ax, sect[k].x), Vector3Scale(fr[0].ay, sect[k].y)));
            int ia = Vert(b, c, nA, PlaneUV(c, fr[0].c, ux, uy, off));
            int ib = Vert(b, a, nA, PlaneUV(a, fr[0].c, ux, uy, off));
            Tri(b, centre, ia, ib);
        }
    }
    if (capB) {
        int last = nf - 1;
        Vector3 nB = Vector3Normalize(Vector3CrossProduct(fr[last].ax, fr[last].ay));
        Vector3 ux = Vector3Normalize(fr[last].ax), uy = Vector3Normalize(fr[last].ay);
        float off = fr[last].c.z;
        int centre = Vert(b, fr[last].c, nB, PlaneUV(fr[last].c, fr[last].c, ux, uy, off));
        for (int j = 0; j < n; j++) {
            int k = (j + 1) % n;
            Vector3 a = Vector3Add(fr[last].c, Vector3Add(Vector3Scale(fr[last].ax, sect[j].x), Vector3Scale(fr[last].ay, sect[j].y)));
            Vector3 c = Vector3Add(fr[last].c, Vector3Add(Vector3Scale(fr[last].ax, sect[k].x), Vector3Scale(fr[last].ay, sect[k].y)));
            int ia = Vert(b, a, nB, PlaneUV(a, fr[last].c, ux, uy, off));
            int ib = Vert(b, c, nB, PlaneUV(c, fr[last].c, ux, uy, off));
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

// MAT_MILLED and MAT_STEEL share the one milled-steel map and differ only in colour: the receiver is that surface blued, the bolt and piston are it bare.
typedef enum {
    MAT_WOOD, MAT_STEEL, MAT_MILLED, MAT_GREY, MAT_BLUED, MAT_BRASS, MAT_COUNT
} MatId;

// lighting.fs gamma-corrects with pow(c, 1/2.2), so these read considerably lighter than the raw values once shaded.
// Each entry is the material at its cleanest and brightest, not its average: the diffuse maps below are eight-bit and multiply the colour, so they can only ever subtract from it. The average surface is the colour times the map's mean, and BuildTextures measures those means rather than trusting this comment: 0.655 wood, 0.557 steel and milled, 0.715 grey, 0.366 blued, 0.755 brass. The 0.60 previously quoted for steel was 8 percent high, which is exactly the sort of drift the check is there to catch.
// EVERY METAL FINISH ON THIS RIFLE IS ONE COLOUR, and the map means are what make that awkward to write down. ref_04 shows barrel, receiver, sights, trigger group and fittings all at one dark value, so the target is one average SURFACE value, not one material colour: blued's 56 through a mean of 0.366 gives 20.9, and every other metal carries 20.9 divided by its own mean. Milled 20.9/0.557 = 37.5, grey 20.9/0.715 = 29.2.
// Setting them to one COLOUR instead is the trap, and it is what this model did: identical colours through maps averaging 0.366 and 0.715 come out two to one apart, which is how the phosphate parts, at 56 against blued's 21, ended up the brightest metal on a rifle whose reference has them the same dark as the barrel. The receiver was worse again at 79, four times the barrel bolted to it.
// The one metal deliberately off the common value is MAT_STEEL, bare bright steel: the bolt, the gas piston and the wear face on the carrier's flank, and that wear face is the only thing the ejection port has to show moving against a dark surround.
static const Color MAT_COLOR[MAT_COUNT] = {
    [MAT_WOOD]   = { 138,  54,  27, 255 },
    [MAT_STEEL]  = { 140, 144, 152, 255 },
    [MAT_MILLED] = {  38,  37,  38, 255 },
    [MAT_GREY]   = {  29,  29,  30, 255 },
    [MAT_BLUED]  = {  56,  56,  60, 255 },
    [MAT_BRASS]  = { 168, 128,  48, 255 },
};

// Millimetres covered by one texture repeat, along the part (u) and across it (v).
// Wood is the anisotropic one, and the v figure is the number that matters: on a swept part v is the section perimeter, so the handguard's 130 mm girth wraps the map most of the way round. At 145 mm the 22 latewood lines the map carries land about 6.6 mm apart and about eighteen of them go round the handguard, which is what references/ak47/ref_05.jpg shows. An earlier 78 mm put nearly forty round it and the wood read as corrugation.
static const Vector2 MAT_REPEAT[MAT_COUNT] = {
    [MAT_WOOD]   = { 320.0f, 145.0f },
    [MAT_STEEL]  = { 165.0f,  95.0f },
    [MAT_MILLED] = { 165.0f,  95.0f },
    [MAT_GREY]   = {  62.0f,  62.0f },
    [MAT_BLUED]  = { 128.0f,  96.0f },
    [MAT_BRASS]  = {  26.0f,  18.0f },
};

// ---------------------------------------------------------------------------
// Procedural surface maps
//
// lighting.fs:73 evaluates texelColor*(tint + specular)*lightDot, so a diffuse texture multiplies the material colour rather than replacing it. An eight-bit map cannot exceed 1.0, so it can only darken: every map here is written as a pure darkener with a mean well below 1.0, and MAT_COLOR carries the clean state each material darkens away from. Writing them centred on 1.0 instead is what a first pass did, and it saturated a third of the blued map to solid white, which expressed nothing at all.
//
// The noise is value noise on an integer lattice evaluated modulo its period, so every map tiles seamlessly. That is the whole reason it is written out here instead of calling GenImagePerlinNoise: stb's Perlin does not tile, and a barrel 318 mm long crosses two and a half repeats of the blued map.
// ---------------------------------------------------------------------------

#define TEX_SIZE 512

static float Hash2(int x, int y, int seed)
{
    unsigned int h = (unsigned int)x * 374761393u + (unsigned int)y * 668265263u + (unsigned int)seed * 1442695041u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (float)((h ^ (h >> 16)) & 0xffffffu) / (float)0xffffffu;
}

static float Smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }

// Bilinear value noise on a pu x pv lattice. Sampling u, v over [0,1) wraps exactly.
static float Noise(float u, float v, int pu, int pv, int seed)
{
    float x = u * (float)pu, y = v * (float)pv;
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    float fx = Smoothstep01(x - (float)x0), fy = Smoothstep01(y - (float)y0);
    int xa = ((x0 % pu) + pu) % pu, xb = (xa + 1) % pu;
    int ya = ((y0 % pv) + pv) % pv, yb = (ya + 1) % pv;
    float n00 = Hash2(xa, ya, seed), n10 = Hash2(xb, ya, seed);
    float n01 = Hash2(xa, yb, seed), n11 = Hash2(xb, yb, seed);
    return (n00 * (1.0f - fx) + n10 * fx) * (1.0f - fy) + (n01 * (1.0f - fx) + n11 * fx) * fy;
}

// Octaves double both periods, so each one still divides the map and the sum keeps tiling.
// pu < pv stretches the features along u, which is how the grain lines and the machining marks are made directional.
static float Fbm(float u, float v, int pu, int pv, int seed, int octaves)
{
    float sum = 0.0f, amp = 1.0f, norm = 0.0f;
    for (int i = 0; i < octaves; i++) {
        sum += amp * Noise(u, v, pu << i, pv << i, seed + i * 71);
        norm += amp;
        amp *= 0.5f;
    }
    return sum / norm;
}

static float Sstep(float e0, float e1, float x)
{
    return Smoothstep01(Clamp((x - e0) / (e1 - e0), 0.0f, 1.0f));
}

static unsigned char Chan(float v)
{
    int i = (int)(v * 255.0f + 0.5f);
    return (unsigned char)((i < 0) ? 0 : (i > 255) ? 255 : i);
}

// Darkening a wood multiplier has to pull green and blue down faster than red, or the grain reads as soot on the surface instead of denser latewood underneath it.
// The shift is measured from the map's mean rather than from 1.0, so only wood darker than average turns redder and the palest earlywood keeps the authored hue.
#define WOOD_MEAN 0.66f

static Color WoodPixel(float m)
{
    float d = WOOD_MEAN - m;
    return (Color){ Chan(m), Chan(m * (1.0f - 0.55f * d)), Chan(m * (1.0f - 0.95f * d)), 255 };
}

// Beech stock and handguards: long latewood lines running along u, broad tonal blotches, and the occasional near-black mineral streak that both ref_04 and ref_07 show on the butt.
static Image WoodImage(void)
{
    Image img = GenImageColor(TEX_SIZE, TEX_SIZE, WHITE);
    Color *px = (Color *)img.data;
    for (int y = 0; y < TEX_SIZE; y++) {
        float v = (float)y / (float)TEX_SIZE;
        for (int x = 0; x < TEX_SIZE; x++) {
            float u = (float)x / (float)TEX_SIZE;

            // Evenly spaced bands read as corduroy, so the band coordinate is warped: a slow term across v that stretches and compresses the spacing, and a faster one that makes each line wander along its length. The warp is deliberately small next to the 22 bands, because a large one merges them into a handful of wide swirls that read as cartoon woodgrain.
            // What matters more than either is that no line survives the whole length. A band that runs unbroken from wrist to buttplate makes the stock read as plywood or a contour map however faint it is, so `present` gates each line on a noise field that varies along u and barely at all across v: whole lines fade out together over stretches, and broad tone rather than banding carries most of the variation.
            float warp = 1.4f * Fbm(u, v, 1, 2, 71, 2) + 1.0f * Fbm(u, v, 3, 6, 5, 4) + 0.5f * Fbm(u, v, 2, 2, 23, 3);
            float band = 0.5f - 0.5f * cosf(2.0f * PI * (v * 22.0f + warp));
            float b2 = band * band, b4 = b2 * b2;
            float present = Sstep(0.30f, 0.70f, Fbm(u, v, 6, 2, 83, 4));
            float line = b4 * b4 * present;

            float fibre = Fbm(u, v, 2, 20, 31, 5);
            float blotch = Fbm(u, v, 2, 2, 47, 4);
            float streak = Fbm(u, v, 1, 6, 91, 4);
            float mineral = Sstep(0.66f, 0.88f, Fbm(u, v, 1, 12, 59, 4));

            float m = 0.70f
                    - 0.17f * line
                    - 0.12f * (fibre - 0.5f)
                    - 0.38f * (blotch - 0.5f)
                    - 0.16f * (streak - 0.5f)
                    - 0.34f * mineral;
            px[y * TEX_SIZE + x] = WoodPixel(m);
        }
    }
    return img;
}

// Milled steel: draw-marks along the length, a broad polish sweep on the high spots and a cooler patina in the hollows. Loaded twice, as MAT_MILLED for the blued receiver and as MAT_STEEL for the bare bolt and piston, which differ in colour and not in surface.
// The receiver keeps this map rather than moving to the blued one along with its colour. The blued map's wear term ramps a tenth of its area up to warm bare steel in blobs three by seven per 128 by 96 mm repeat, which is sized for a magazine rib and reads as wear there. Across the receiver's 248 by 34 flank the same blobs come out as six tan smears the size of a thumb, which is what the first attempt at this looked like: see renders/ak47/v12.
static Image SteelImage(void)
{
    Image img = GenImageColor(TEX_SIZE, TEX_SIZE, WHITE);
    Color *px = (Color *)img.data;
    for (int y = 0; y < TEX_SIZE; y++) {
        float v = (float)y / (float)TEX_SIZE;
        for (int x = 0; x < TEX_SIZE; x++) {
            float u = (float)x / (float)TEX_SIZE;

            float mill = Fbm(u, v, 2, 40, 307, 4);
            float sweep = Fbm(u, v, 3, 10, 311, 4);
            float patina = Fbm(u, v, 2, 2, 313, 4);
            float speck = Sstep(0.76f, 0.94f, Fbm(u, v, 12, 12, 317, 3));

            float m = 0.55f
                    + 0.16f * (mill - 0.5f)
                    + 0.40f * (sweep - 0.5f)
                    - 0.32f * (patina - 0.5f)
                    - 0.34f * speck;
            float warm = 0.09f * (patina - 0.5f);
            px[y * TEX_SIZE + x] = (Color){ Chan(m + warm), Chan(m), Chan(m - warm * 0.6f), 255 };
        }
    }
    return img;
}

// Phosphated small parts: fine granular tooth, matte, very little large-scale variation.
static Image GreyImage(void)
{
    Image img = GenImageColor(TEX_SIZE, TEX_SIZE, WHITE);
    Color *px = (Color *)img.data;
    for (int y = 0; y < TEX_SIZE; y++) {
        float v = (float)y / (float)TEX_SIZE;
        for (int x = 0; x < TEX_SIZE; x++) {
            float u = (float)x / (float)TEX_SIZE;

            float tooth = Fbm(u, v, 24, 24, 211, 4);
            float blotch = Fbm(u, v, 3, 3, 223, 4);
            float rub = Sstep(0.70f, 0.94f, Fbm(u, v, 4, 8, 227, 4));

            float m = 0.72f + 0.22f * (tooth - 0.5f) + 0.18f * (blotch - 0.5f) + 0.20f * rub;
            px[y * TEX_SIZE + x] = (Color){ Chan(m), Chan(m), Chan(m), 255 };
        }
    }
    return img;
}

// Blued barrel and magazine: a smooth dark finish with faint polishing streaks, plus the plum-brown thinning along the high spots that ref_04 and ref_05 both show on the magazine ribs.
static Image BluedImage(void)
{
    Image img = GenImageColor(TEX_SIZE, TEX_SIZE, WHITE);
    Color *px = (Color *)img.data;
    for (int y = 0; y < TEX_SIZE; y++) {
        float v = (float)y / (float)TEX_SIZE;
        for (int x = 0; x < TEX_SIZE; x++) {
            float u = (float)x / (float)TEX_SIZE;

            // This is the widest map of the four: the finish itself is very dark but the spots it has worn through are bare steel, so the darkener has to span nearly its whole range for the two to read as different surfaces at all.
            float polish = Fbm(u, v, 2, 24, 101, 5);
            float cloud = Fbm(u, v, 3, 3, 113, 4);
            float thin = Sstep(0.52f, 0.88f, Fbm(u, v, 3, 7, 127, 4));

            float m = 0.30f + 0.10f * (polish - 0.5f) + 0.17f * (cloud - 0.5f) + 0.62f * thin;
            px[y * TEX_SIZE + x] = (Color){ Chan(m), Chan(m * (1.0f - 0.11f * thin)), Chan(m * (1.0f - 0.30f * thin)), 255 };
        }
    }
    return img;
}

// Fired cartridge case: drawn brass, so the marks run around the case rather than along it. The repeat is the smallest of the five at 26 by 18 mm, because the case is 38.7 mm long and a map sized for a buttstock would put a quarter of one repeat on the whole part and read as a flat colour.
static Image BrassImage(void)
{
    Image img = GenImageColor(TEX_SIZE, TEX_SIZE, WHITE);
    Color *px = (Color *)img.data;
    for (int y = 0; y < TEX_SIZE; y++) {
        float v = (float)y / (float)TEX_SIZE;
        for (int x = 0; x < TEX_SIZE; x++) {
            float u = (float)x / (float)TEX_SIZE;

            float draw = Fbm(u, v, 30, 3, 401, 4);
            float tarnish = Fbm(u, v, 3, 3, 409, 4);
            float soot = Sstep(0.62f, 0.92f, Fbm(u, v, 5, 5, 419, 4));

            float m = 0.78f + 0.16f * (draw - 0.5f) + 0.20f * (tarnish - 0.5f) - 0.30f * soot;
            px[y * TEX_SIZE + x] = (Color){ Chan(m), Chan(m * (1.0f - 0.10f * soot)), Chan(m * (1.0f - 0.26f * soot)), 255 };
        }
    }
    return img;
}

static Texture2D gTex[MAT_COUNT];
static bool gTexReady;

static void BuildTextures(void)
{
    Image img[MAT_COUNT];
    img[MAT_WOOD] = WoodImage();
    img[MAT_STEEL] = SteelImage();
    img[MAT_MILLED] = SteelImage();
    img[MAT_GREY] = GreyImage();
    img[MAT_BLUED] = BluedImage();
    img[MAT_BRASS] = BrassImage();

    // Every metal but the bare steel inside the action is meant to land on one average surface value, and a map is free to be edited without anyone noticing that its mean moved and took its material's colour off that value with it. So the means are measured here rather than remembered: the figures quoted against MAT_COLOR come from this, and it reports the spread so a drift shows up as a number instead of as a reviewer saying one part looks lighter than another. It is the same build-time check CheckAction is, applied to colour instead of to clearances.
    float surf[MAT_COUNT];
    for (int m = 0; m < MAT_COUNT; m++) {
        double sr = 0.0, sg = 0.0, sb = 0.0;
        const Color *px = (const Color *)img[m].data;
        for (int i = 0; i < TEX_SIZE * TEX_SIZE; i++) { sr += px[i].r; sg += px[i].g; sb += px[i].b; }
        double n = (double)TEX_SIZE * TEX_SIZE * 255.0;
        surf[m] = (float)((MAT_COLOR[m].r * sr + MAT_COLOR[m].g * sg + MAT_COLOR[m].b * sb) / (3.0 * n));
        TraceLog(LOG_INFO, "ak47: material %d map mean %.3f, average surface %.1f", m, (sr + sg + sb) / (3.0 * n), surf[m]);
    }
    const MatId metal[] = { MAT_MILLED, MAT_GREY, MAT_BLUED };
    float lo = surf[metal[0]], hi = surf[metal[0]];
    for (int i = 1; i < (int)(sizeof(metal) / sizeof(metal[0])); i++) {
        if (surf[metal[i]] < lo) lo = surf[metal[i]];
        if (surf[metal[i]] > hi) hi = surf[metal[i]];
    }
    TraceLog(lo > 0.0f && hi / lo > 1.15f ? LOG_WARNING : LOG_INFO,
             "ak47: the finished metals run %.1f to %.1f in average surface value, a spread of %.0f percent; they are meant to be one colour",
             lo, hi, lo > 0.0f ? (hi / lo - 1.0f) * 100.0f : 0.0f);

    for (int m = 0; m < MAT_COUNT; m++) {
        gTex[m] = LoadTextureFromImage(img[m]);
        GenTextureMipmaps(&gTex[m]);
        SetTextureFilter(gTex[m], TEXTURE_FILTER_TRILINEAR);
        SetTextureWrap(gTex[m], TEXTURE_WRAP_REPEAT);
        UnloadImage(img[m]);
    }
    gTexReady = true;
}

// UnloadModel frees a material's map array but deliberately leaves its textures alone (vendor/raylib/src/rmodels.c:1199), so one texture per material is shared across all seven groups and released here.
static void UnloadTextures(void)
{
    if (!gTexReady) return;
    for (int m = 0; m < MAT_COUNT; m++) UnloadTexture(gTex[m]);
    gTexReady = false;
}

// A group is one rigid body. `pivot` is the point in model millimetres its geometry is authored about, and `xform` is where that body currently is; the four that move rebuild xform every frame in Update and nothing about them is baked into world coordinates.
typedef struct {
    const char *name;
    Vector3 pivot;
    Builder b[MAT_COUNT];
    Model model[MAT_COUNT];
    bool has[MAT_COUNT];
    BoundingBox local;   // in the group's own frame, from the built mesh
    BoundingBox swept;   // the union of local over one cycle, in world space
    Matrix xform;
} Group;

// Everything emitted after this call is authored about `pivot`, until the next call names a different one.
static void GroupOrigin(Group *g, Vector3 pivot)
{
    g->pivot = pivot;
    gOrigin = pivot;
}

// The pose that puts a group exactly where the model frame draws it, which is its rest pose.
static Matrix GroupRest(const Group *g)
{
    Vector3 w = ToWorld(g->pivot);
    return MatrixTranslate(w.x, w.y, w.z);
}

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
    g->name = name;
    for (int m = 0; m < MAT_COUNT; m++) {
        Builder *b = &g->b[m];
        if (b->vertexCount == 0) continue;
        if (b->vertexCount > 65535) {
            TraceLog(LOG_ERROR, "%s: material %d has %d vertices, over the 65535 index limit",
                     name, m, b->vertexCount);
        }

        // Vert() left the texture coordinates in millimetres; one repeat is a fixed physical size, so this is where a material's tiling rate is applied.
        for (int i = 0; i < b->vertexCount; i++) {
            b->texcoords[i * 2 + 0] /= MAT_REPEAT[m].x;
            b->texcoords[i * 2 + 1] /= MAT_REPEAT[m].y;
        }

        Mesh mesh = { 0 };
        mesh.vertexCount = b->vertexCount;
        mesh.triangleCount = b->triangleCount;
        mesh.vertices = b->vertices;
        mesh.normals = b->normals;
        mesh.texcoords = b->texcoords;
        mesh.colors = b->colors;
        mesh.indices = b->indices;
        UploadMesh(&mesh, false);

        g->model[m] = LoadModelFromMesh(mesh);
        g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].color = MAT_COLOR[m];
        if (gTexReady) g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = gTex[m];
        HarnessApplyLighting(&g->model[m]);
        g->has[m] = true;

        BoundingBox bb = GetModelBoundingBox(g->model[m]);
        if (!any) {
            g->local = bb;
            any = true;
        } else {
            g->local.min = Vector3Min(g->local.min, bb.min);
            g->local.max = Vector3Max(g->local.max, bb.max);
        }
    }
    g->xform = GroupRest(g);
    g->swept = g->local;
}

// A model that is deliberately NOT given the lighting shader, and therefore renders flat at exactly the colour it carries.
//
// CLAUDE.md warns that forgetting HarnessApplyLighting leaves a mesh as a flat unlit silhouette and makes the review loop worthless. That is a warning about doing it by accident. Doing it on purpose is the only way this file has to express two things the lighting shader cannot:
//   self-luminous. A muzzle flash emits; nothing in lighting.fs adds emission, so a lit flash is a yellow plastic cone that gets darker away from the key light.
//   transparent. lighting.fs:73 computes finalColor as texelColor*((tint + vec4(specular, 1.0))*vec4(lightDot, 1.0)), whose alpha is texelColor.a*(tint.a + 1.0). That is at or above 1.0 for every tint alpha, so the lighting shader cannot carry transparency at all. raylib's default shader is finalColor = texelColor*colDiffuse*fragColor, which does.
// Both shaders declare vertexColor and pass it to fragColor, so a single unlit mesh can still vary across itself; ShadeRange is what puts the shading in.
static Model FinishUnlit(Builder *b, const char *name)
{
    if (b->vertexCount > 65535) {
        TraceLog(LOG_ERROR, "%s: %d vertices, over the 65535 index limit", name, b->vertexCount);
    }

    Mesh mesh = { 0 };
    mesh.vertexCount = b->vertexCount;
    mesh.triangleCount = b->triangleCount;
    mesh.vertices = b->vertices;
    mesh.normals = b->normals;
    mesh.texcoords = b->texcoords;
    mesh.colors = b->colors;
    mesh.indices = b->indices;
    UploadMesh(&mesh, false);

    return LoadModelFromMesh(mesh);
}

// DrawModel composes an identity onto model.transform, so the pose passes through untouched.
static void GroupDraw(Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (!g->has[m]) continue;
        g->model[m].transform = g->xform;
        DrawModel(g->model[m], Vector3Zero(), 1.0f, WHITE);
    }
}

static void GroupUnload(Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (g->has[m]) UnloadModel(g->model[m]);
    }
}

static Group gSight, gBarrel, gWood, gRecv, gMag, gFire, gStock;
static Group gCarrier, gTrigger, gCase;

// The seven groups above never move, so their own frame is the model frame.
#define STATIC_ORIGIN ((Vector3){ 0.0f, 0.0f, Z_ORIGIN })

// ---------------------------------------------------------------------------
// Front sight assembly, z 0 to 46
// ---------------------------------------------------------------------------

static void BuildSight(void)
{
    GroupOrigin(&gSight, STATIC_ORIGIN);
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
    GroupOrigin(&gBarrel, STATIC_ORIGIN);
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

    Tube(s, (Vector3){ 0, GAS_Y, GASTUBE_Z0 }, (Vector3){ 0, GAS_Y, 152.0f }, 9.2f, 9.2f, 28, true, false);
    Tube(s, (Vector3){ 0, GAS_Y, 152.0f }, (Vector3){ 0, GAS_Y, 206.0f }, 8.0f, 8.0f, 28, false, false);
    Tube(s, (Vector3){ 0, GAS_Y, 206.0f }, (Vector3){ 0, GAS_Y, GASTUBE_Z1 }, 9.2f, 9.2f, 28, false, true);

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
    GroupOrigin(&gWood, STATIC_ORIGIN);
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

// A receiver that is a solid billet with two windows milled into it is enough while nothing behind them moves. Here the ejection port has to open into somewhere, so the core is cut back to a channel: a base slab under it, a full-height wall on each flank, and a rear wall the carrier stops against. The channel is what the port and the charging-handle slot now look into.
#define CHAN_HW    11.5f    // channel half-width, 0.5 mm clear of the carrier
#define CHAN_Y0    196.0f   // channel floor, 1.0 mm under the bolt
#define CHAN_Z1    600.0f   // channel rear wall, 6 mm ahead of the stock tang

// ---------------------------------------------------------------------------
// The four pivots
//
// Every posed body reads its pivot from here and nowhere else, and the parts that have to agree with a pivot (the trigger pin head on the receiver flank, the charging handle's rest position against its slot) read it from here too. That is the one rule this project has paid for four times over: a child written as an absolute coordinate stops agreeing with its parent the moment anyone edits the parent.
// ---------------------------------------------------------------------------

#define CAR_Z0     380.0f   // bolt carrier front face, in battery
#define CAR_L      140.0f   // carrier body length
#define CAR_HW     11.0f
#define CAR_Y0     211.0f   // carrier body underside, on top of the bolt
#define CAR_Y1     226.0f   // carrier body top, 1 mm under the dust cover floor
#define BOLT_R       7.4f
#define HAND_Z0    383.0f   // charging handle stem, 3 mm behind the slot's front stop
#define HAND_Z1    393.0f

#define TRIG_PIVOT_Y 174.0f
#define TRIG_PIVOT_Z 550.0f
#define TRIG_PULL     9.0f  // degrees; about 5 mm at the shoe

// ---------------------------------------------------------------------------
// Recoil
//
// Derived, not styled. The AK-47 specification gives a 7.9 g M43 bullet at 715 m/s and a 4.39 kg rifle with a loaded 30-round magazine, which is the configuration this model shows. Conservation of momentum fixes the recoil velocity at 7.9e-3 * 715 / 4.39 = 1.29 m/s.
// The propellant gas is left out. It adds roughly a third again to the impulse in ordinary free-recoil practice, but no reference consulted gives a charge mass for 7.62x39, and inventing one to make the recoil larger is exactly the move this project has been caught making before.
//
// Muzzle rise falls out of the same impulse. The recoil acts along the bore axis at y 204.4 and is resisted at the shoulder, which the buttplate centre puts at y 121.0: an 83.4 mm moment arm. The angular impulse about that point is 5.65 * 0.0834 = 0.471 N m s, against an inertia of about 0.81 kg m^2 for a 4.39 kg rifle whose centre of mass sits roughly 350 mm ahead of the butt, giving 0.58 rad/s.
//
// The shoulder is the one quantity here that is chosen rather than derived: it is modelled as absorbing the recoil and returning the rifle to battery over RECOIL_T, as a half sine whose initial slope is the velocity above. Peak travel is therefore v * RECOIL_T / pi, and CheckAction reports what that comes to.
//
// The result is small, and that is the answer rather than a shortfall: one shot from a 4.4 kg rifle shifts it about 25 mm and turns it about two thirds of a degree within a cycle. It reads by comparing the muzzle between frames, not at a glance. Anything larger would be a decision to exaggerate, and this file does not take it.
// ---------------------------------------------------------------------------

#define BULLET_M     0.0079f   // kg
#define BULLET_V     715.0f    // m/s
#define RIFLE_M      4.39f     // kg, loaded
#define RIFLE_I      0.81f     // kg m^2 about the shoulder
#define SHOULDER_Y   121.0f    // buttplate centre, from the last stock section
#define SHOULDER_Z   867.5f

// ---------------------------------------------------------------------------
// The burst
//
// Three rounds at the AK-47's 600 rpm cyclic rate: one action cycle is 0.100 s of real time and the burst is 0.300 s. The cycle runs five shot periods so the last round has somewhere to finish and the loop has a pause in it, and every ignition therefore falls on a fifth of the cycle, at phases 0, 0.2 and 0.4.
// That is not tidiness. A muzzle flash lasts about 3 ms, under one percent of this cycle, so no evenly spaced set of frames catches one by luck: any --frames that is a multiple of 5 lands on all three ignitions, --frames 15 is the useful default, and --frames 8 will show only the first.
// ---------------------------------------------------------------------------

#define SHOTS        3
#define SHOT_T       0.100f    // real seconds for one action cycle, at 600 rpm
#define BURST_T      (SHOTS * SHOT_T)
#define TOTAL_T      0.500f    // five shot periods: three firing, two of pause
// SLOWDOWN is the ONLY thing in this file that is about playback rather than about the rifle. Everything else is computed in real seconds, so this scales what the interactive window and SCENE.duration do and nothing else: --anim samples N evenly spaced phases of the cycle, so the rendered frames come out identical whatever this is set to. To play those frames back at real speed, run them at N / TOTAL_T fps, which is 30 fps for --frames 15.
// At 16 the burst read as a slow-motion replay, and at 4 and at 2.5 it still did. At 1 it is real time: a whole shot is six frames at 60 Hz and the carrier's stroke is two of them, which is the point, since nothing slower is what the rifle does.
#define SLOWDOWN     1.0f
#define CYCLE        (TOTAL_T * SLOWDOWN)   // 0.5 s of playback for the whole burst

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
        // With a fixed handle this stretch of plate stays solid and the charging handle sits on top of it. The handle now travels, so the slot has to run from the receiver's front face all the way aft: this is the span forward of the ejection port, split above and below the slot.
        Box(b, x0, x1, CUT_Y1, TRACK_Y0, CUT_Z0, PORT_Z0);
        Box(b, x0, x1, TRACK_Y1, RCV_TOP, CUT_Z0, PORT_Z0);
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
    GroupOrigin(&gRecv, STATIC_ORIGIN);
    // Blued, at the same surface value as the barrel and the magazine, and keeping the milled map because a 248 by 34 flank is not what the blued map's wear blobs are sized for. It was bare bright steel until now, on the strength of ref_04 showing the receiver lighter than the barrel: it is lighter there, but by a little, and MAT_STEEL against MAT_BLUED is four times the surface value, which made the receiver read as a different metal from the rest of the rifle rather than as the same metal worn brighter.
    Builder *m = &gRecv.b[MAT_MILLED];
    Builder *s = &gRecv.b[MAT_GREY];

    // Base slab under the carrier channel.
    Prism(m, -CHAN_HW, CHAN_HW, RCV_Z0, RCV_Z1, RecvFloor(RCV_Z0), RecvFloor(RCV_Z1), CHAN_Y0, CHAN_Y0);
    // Flank walls, taken to full height so the milled lightening cut on either side still has a floor to be milled into.
    Prism(m, -RCV_CORE, -CHAN_HW, RCV_Z0, RCV_Z1, RecvFloor(RCV_Z0), RecvFloor(RCV_Z1), RCV_TOP, RCV_TOP);
    // Right wall. The charging-handle slot is cut through this as well as through the outer plate: the stem crosses 2.5 mm of core before it reaches the plate, and cutting only the plate leaves it travelling through solid steel.
    Prism(m, CHAN_HW, RCV_CORE, RCV_Z0, CUT_Z0, RecvFloor(RCV_Z0), RecvFloor(CUT_Z0), RCV_TOP, RCV_TOP);
    Prism(m, CHAN_HW, RCV_CORE, CUT_Z0, PORT_Z0, RecvFloor(CUT_Z0), RecvFloor(PORT_Z0), TRACK_Y0, TRACK_Y0);
    Box(m, CHAN_HW, RCV_CORE, TRACK_Y1, RCV_TOP, CUT_Z0, PORT_Z0);
    // Over the ejection port the wall drops to the sill, which is what turns the port from a recess into a hole through into the channel.
    Prism(m, CHAN_HW, RCV_CORE, PORT_Z0, PORT_Z1, RecvFloor(PORT_Z0), RecvFloor(PORT_Z1), PORT_Y0, PORT_Y0);
    Prism(m, CHAN_HW, RCV_CORE, PORT_Z1, TRACK_Z1, RecvFloor(PORT_Z1), RecvFloor(TRACK_Z1), TRACK_Y0, TRACK_Y0);
    Box(m, CHAN_HW, RCV_CORE, TRACK_Y1, RCV_TOP, PORT_Z1, TRACK_Z1);
    Prism(m, CHAN_HW, RCV_CORE, TRACK_Z1, RCV_Z1, RecvFloor(TRACK_Z1), RecvFloor(RCV_Z1), RCV_TOP, RCV_TOP);
    // Rear wall: the surface the carrier runs back against, and therefore the thing that sets the travel.
    Box(m, -CHAN_HW, CHAN_HW, CHAN_Y0, RCV_TOP, CHAN_Z1, RCV_Z1);

    // The channel interior has to stay dark, and now it is dark for free. Four review rounds running reported the ejection port as a barely changing recess, because bare milled walls, a blued carrier and a bright wear patch on it are all mid greys at this camera distance and nothing in the opening changed value as the carrier ran. What fixed it was a dark background behind a bright moving face, and while the receiver was bare steel that took three blued liner boxes buried into the left wall, the floor and the rear wall. A receiver blued throughout is that background already, so the liners sat at the same surface value as the walls they lined, invisible, and are gone. Anything that puts bare steel back into this channel has to put them back with it: the bright wear face on the carrier is the only thing in the port that moves visibly, and it needs something dark to move against.
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

    // The charging handle has moved to the bolt carrier group, which is the body it is actually part of.

    GroupMark gm = GroupMarkNow(&gRecv);
    Tube(m, (Vector3){ RCV_SIDE, 177.0f, 491.0f }, (Vector3){ 18.4f, 177.0f, 491.0f }, 2.2f, 1.8f, 10, false, true);
    Tube(m, (Vector3){ RCV_SIDE, 177.0f, 502.0f }, (Vector3){ 18.4f, 177.0f, 502.0f }, 2.2f, 1.8f, 10, false, true);
    Tube(m, (Vector3){ RCV_SIDE, 214.0f, 596.0f }, (Vector3){ 18.4f, 214.0f, 596.0f }, 2.2f, 1.8f, 10, false, true);
    // Trigger pin head. A fixed trigger has no reason to show where it hangs from; a swinging one does, and the head is drawn from TRIG_PIVOT rather than from a copy of its coordinates.
    Tube(m, (Vector3){ RCV_SIDE, TRIG_PIVOT_Y, TRIG_PIVOT_Z }, (Vector3){ 18.6f, TRIG_PIVOT_Y, TRIG_PIVOT_Z }, 3.0f, 2.6f, 12, false, true);
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
    GroupOrigin(&gMag, STATIC_ORIGIN);
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
    GroupOrigin(&gFire, STATIC_ORIGIN);
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

    // The trigger has moved to its own group: it swings, so it cannot share a mesh with the guard it swings inside.

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
    GroupOrigin(&gStock, STATIC_ORIGIN);
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
// Bolt carrier group: carrier body, bolt, gas piston and charging handle
//
// One rigid body and therefore one mesh: the charging handle is part of the carrier on the real rifle, and making it a separate group would let the two disagree. Its local origin is the carrier's front face on the gas-tube axis, so the pose is a bare translation aft and the travel is exactly the number CheckAction reads back.
//
// The piston rod runs forward at GAS_Y and so passes through the rear sight base, the receiver front and the upper handguard, none of which is bored. That is the convention the static geometry already works to: the gas tube sits wholly inside the solid swept handguard wood, because the wood is a solid section and not a shell. Nothing here is visible from outside, and the alternative is boring four measured solids to hide a part no assembled view ever shows.
// ---------------------------------------------------------------------------

static void BuildCarrier(void)
{
    GroupOrigin(&gCarrier, (Vector3){ 0.0f, GAS_Y, CAR_Z0 });
    Builder *s = &gCarrier.b[MAT_BLUED];
    Builder *b = &gCarrier.b[MAT_STEEL];

    // The carrier is blued, as references/ak47/ref_01.jpg and ref_03.jpg both show it, and so now is the channel it runs in. The one thing the ejection port has to show is the bright wear face below travelling against that dark surround: a bright carrier in a bright channel changes nothing visible as it travels, which is what a first attempt at this looked like.
    // Carrier body, and a skirt dropping alongside the bolt over the rear third, which is the stepped profile ref_03 shows.
    Box(s, -CAR_HW, CAR_HW, CAR_Y0, CAR_Y1, CAR_Z0, CAR_Z0 + CAR_L);
    Box(s, -CAR_HW, CAR_HW, BORE - BOLT_R - 0.4f, CAR_Y0, CAR_Z0 + 90.0f, CAR_Z0 + CAR_L);

    // Bolt, protruding forward of the carrier: in battery that puts its head in the chamber, and at full travel it is the part the ejection port shows moving.
    Tube(b, (Vector3){ 0.0f, BORE, CAR_Z0 - 14.0f }, (Vector3){ 0.0f, BORE, CAR_Z0 + 92.0f }, BOLT_R, BOLT_R, 26, true, true);
    for (int k = 0; k < 2; k++) {
        float a = (k == 0) ? 40.0f : -40.0f;
        Vector3 lug = { BOLT_R * cosf(a * DEG2RAD), BORE + BOLT_R * sinf(a * DEG2RAD), CAR_Z0 - 10.0f };
        Tube(b, lug, (Vector3){ lug.x * 1.28f, BORE + (lug.y - BORE) * 1.28f, CAR_Z0 - 2.0f }, 3.0f, 3.0f, 10, false, true);
    }

    // A bright bare-steel wear patch on the carrier's right flank, standing 0.15 proud of it and 0.35 clear of the channel wall.
    // The carrier, the channel it runs in and the slot above it are all dark and all much the same dark, so the carrier body is hard to follow through the port however far it travels: three review rounds reported the port as an unchanging recess. A moving body needs a feature that changes value as it travels, or the motion is invisible however far it goes: a uniform surface sliding behind a uniform opening tells a reviewer nothing about which frame is which. It is also where a carrier actually wears bright, on the flank that rides the rail.
    // Taken as tall as the port opening and as far outboard as the channel allows, because a smaller one did not carry: at 11 mm tall and 0.65 proud it was still read as an unchanging dark recess. The carrier only ever slides along z, so the 0.3 mm side clearance never closes.
    Box(b, CAR_HW - 0.5f, CAR_HW + 0.2f, PORT_Y0 + 0.5f, CAR_Y1 - 0.8f, CAR_Z0 + 4.0f, CAR_Z0 + 56.0f);

    // Charging handle: a stem sized to the slot it runs in, then the knob outboard of the plate.
    Box(s, 6.0f, RCV_SIDE + 0.4f, TRACK_Y0 + 0.5f, TRACK_Y1 - 0.5f, HAND_Z0, HAND_Z1);
    Box(s, RCV_SIDE + 0.4f, 21.5f, 214.0f, 227.0f, HAND_Z0 - 2.0f, HAND_Z1 + 2.0f);
    Tube(s, (Vector3){ 21.5f, 220.5f, 388.0f }, (Vector3){ 30.0f, 220.5f, 388.0f }, 4.8f, 4.0f, 20, false, true);

    // Gas piston. The head has to stay inside the gas tube at BOTH ends of the travel, and its rest position is set from the tube's front cap rather than typed: at z 118 to 142 it stood 23 mm out in the open ahead of the tube whenever the carrier was forward, covered only by a gas-block riser that is both shorter and narrower than it, and it withdrew out of sight as the carrier ran back. On the real rifle the head sits in the gas block's cylinder at rest, but this model's gas block is not bored, so the nearest honest place for it is just inside the tube's mouth.
    Tube(b, (Vector3){ 0.0f, GAS_Y, GASTUBE_Z0 + 29.0f }, (Vector3){ 0.0f, GAS_Y, CAR_Z0 + 8.0f }, 4.0f, 4.0f, 16, false, false);
    Tube(b, (Vector3){ 0.0f, GAS_Y, GASTUBE_Z0 + 5.0f }, (Vector3){ 0.0f, GAS_Y, GASTUBE_Z0 + 29.0f }, 7.5f, 7.5f, 22, true, true);
}

// ---------------------------------------------------------------------------
// Trigger: local origin ON the pin it swings about, so the pose is a bare rotation about local x and the pin head on the receiver flank is drawn from the same two numbers.
// ---------------------------------------------------------------------------

static void BuildTrigger(void)
{
    GroupOrigin(&gTrigger, (Vector3){ 0.0f, TRIG_PIVOT_Y, TRIG_PIVOT_Z });
    Builder *s = &gTrigger.b[MAT_GREY];

    Vector3 trig[8] = {
        { -2.5f, 140.0f, 531.0f }, { 2.5f, 140.0f, 531.0f }, { 2.5f, 152.0f, 549.0f }, { -2.5f, 152.0f, 549.0f },
        { -2.5f, 150.0f, 531.0f }, { 2.5f, 150.0f, 531.0f }, { 2.5f, 172.0f, 549.0f }, { -2.5f, 172.0f, 549.0f },
    };
    Hex(s, trig);

    // Body up to the pin, so the blade hangs off something rather than floating below its own pivot.
    Vector3 body[8] = {
        { -2.5f, 152.0f, 545.0f }, { 2.5f, 152.0f, 545.0f }, { 2.5f, 152.0f, 556.0f }, { -2.5f, 152.0f, 556.0f },
        { -2.5f, TRIG_PIVOT_Y + 5.0f, 545.0f }, { 2.5f, TRIG_PIVOT_Y + 5.0f, 545.0f },
        { 2.5f, TRIG_PIVOT_Y + 5.0f, 556.0f }, { -2.5f, TRIG_PIVOT_Y + 5.0f, 556.0f },
    };
    Hex(s, body);
    Tube(s, (Vector3){ -3.6f, TRIG_PIVOT_Y, TRIG_PIVOT_Z }, (Vector3){ 3.6f, TRIG_PIVOT_Y, TRIG_PIVOT_Z }, 2.6f, 2.6f, 14, true, true);
}

// ---------------------------------------------------------------------------
// Fired cartridge case, 7.62x39
//
// Case length 38.70, rim and base diameter 11.35, shoulder 10.07, neck 8.60, from the cartridge drawing on the 7.62x39mm Wikipedia article. Overall cartridge length 56.00, which is the figure the carrier's travel has to beat for a fresh round to have anywhere to go, and CheckAction reports that margin.
//
// Its local origin is the case's own centre, so it can tumble about itself and be flung along a trajectory by one matrix.
// ---------------------------------------------------------------------------

#define CASE_L      38.70f
#define CASE_R       5.675f
#define CASE_NECK_R  4.30f
#define CASE_Z      (CAR_Z0 - 14.0f)   // the case sits on the bolt face, and the bolt face is where the bolt says it is

static void BuildCase(void)
{
    GroupOrigin(&gCase, (Vector3){ 0.0f, BORE, CASE_Z - CASE_L * 0.5f });
    Builder *s = &gCase.b[MAT_BRASS];

    const float y = BORE, z0 = CASE_Z - CASE_L;
    Tube(s, (Vector3){ 0, y, z0 + 38.70f }, (Vector3){ 0, y, z0 + 33.00f }, CASE_NECK_R, CASE_NECK_R, 20, true, false);
    Tube(s, (Vector3){ 0, y, z0 + 33.00f }, (Vector3){ 0, y, z0 + 29.00f }, CASE_NECK_R, 5.035f, 20, false, false);
    Tube(s, (Vector3){ 0, y, z0 + 29.00f }, (Vector3){ 0, y, z0 + 6.00f }, 5.035f, CASE_R, 20, false, false);
    Tube(s, (Vector3){ 0, y, z0 + 6.00f }, (Vector3){ 0, y, z0 + 4.60f }, CASE_R, 4.85f, 20, false, false);
    Tube(s, (Vector3){ 0, y, z0 + 4.60f }, (Vector3){ 0, y, z0 + 2.40f }, 4.85f, 4.85f, 20, false, false);
    Tube(s, (Vector3){ 0, y, z0 + 2.40f }, (Vector3){ 0, y, z0 + 1.00f }, 4.85f, CASE_R, 20, false, false);
    Tube(s, (Vector3){ 0, y, z0 + 1.00f }, (Vector3){ 0, y, z0 }, CASE_R, CASE_R, 20, false, true);
}

// ---------------------------------------------------------------------------
// Muzzle flash
//
// One unlit mesh at the muzzle: a spindle along the bore and six radial spikes, white-hot at the muzzle face and dropping to orange at the tips through vertex colour, since it is one mesh and the material carries one colour.
// The AK-47 has no flash hider, so the flash is a broad open star rather than the narrow petals a pronged device makes. Size is judged, not measured: roughly 140 mm along the bore and 190 mm across.
// ---------------------------------------------------------------------------

static Builder gFlashB;
static Model gFlashM;
static BoundingBox gFlashLocal;
static Matrix gFlashX;
static bool gFlashOn;

static void BuildFlash(void)
{
    Builder *f = &gFlashB;
    gOrigin = (Vector3){ 0.0f, BORE, 0.0f };
    Mark m = { f->vertexCount, f->triangleCount };

    // Forward is -z: z runs aft from the muzzle face, which is z 0.
    Tube(f, (Vector3){ 0, BORE, 6.0f }, (Vector3){ 0, BORE, -22.0f }, 13.0f, 31.0f, 18, false, false);
    Tube(f, (Vector3){ 0, BORE, -22.0f }, (Vector3){ 0, BORE, -140.0f }, 31.0f, 2.5f, 18, false, true);

    // Seven rays at irregular angles, lengths and thicknesses. Six evenly spaced rays of two alternating lengths is a graphic star, which is what the first version drew: the eye reads the symmetry before it reads the flash.
    for (int k = 0; k < 7; k++) {
        float a = (360.0f * (float)k / 7.0f + 62.0f * Hash2(k, 1, 907)) * DEG2RAD;
        float reach = 42.0f + 66.0f * Hash2(k, 2, 907);
        float root = 8.0f + 6.0f * Hash2(k, 3, 907);
        float lean = 0.45f + 0.75f * Hash2(k, 4, 907);
        Vector3 a0 = { 0.0f, BORE, -12.0f };
        Vector3 a1 = { reach * cosf(a), BORE + reach * sinf(a), -12.0f - reach * lean };
        Tube(f, a0, a1, root, 1.2f, 10, false, true);
    }

    // Ramped along z, and forward is -z, so the low end of the range is the tips.
    ShadeRange(f, m, 2, (Color){ 226, 104, 30, 255 }, (Color){ 255, 250, 226, 255 });
    gOrigin = STATIC_ORIGIN;
}

// ---------------------------------------------------------------------------
// Smoke
//
// One lumpy unit blob, drawn once per puff with a scale and a translation, because a puff differs from its neighbours only in where it is, how big it has grown and how faded it is. Building eight meshes to say that would be eight copies of the same statement.
//
// Transparent, and therefore unlit: lighting.fs cannot carry alpha at all (see FinishUnlit), so the only shading a puff has is the vertical ramp ShadeRange bakes into it. Depth writes are turned off while they draw and the smoke is drawn last, so puffs blend with each other and with the rifle instead of punching holes in one another in whatever order the array happens to be in.
//
// Smoke that behaved honestly would still be hanging there at the end of the cycle, and a repeating cycle would silt up with it. Each puff therefore fades to nothing by the end, which is a concession to looping and not a claim about smoke.
// ---------------------------------------------------------------------------

typedef struct {
    Vector3 from;      // model mm, where it leaves the rifle
    float t0;          // phase at which it appears
    float r0, grow;    // mm, and mm per real second
    Vector3 drift;     // mm per real second
    Vector3 rot;       // fixed orientation, so one mesh does not read as one shape repeated
    float peak;        // alpha at its strongest
} PuffSpec;

#define MUZZLE_PUFFS 22
#define PORT_PUFFS    6
#define PUFF_COUNT   (MUZZLE_PUFFS + PORT_PUFFS)

static PuffSpec gPuffSpec[PUFF_COUNT];

// One instance per puff per round: each shot lays down its own plume and they pile up, which is what a burst looks like.
#define PUFF_INSTANCES (SHOTS * PUFF_COUNT)
#define PUFF_LIFE      0.26f   // real seconds a puff lasts; the last round's must clear before the cycle repeats
#define PUFF_DRAG      0.060f  // real seconds for a puff to shed its jet momentum

// Many faint puffs, not a few solid ones. A single blob at the alpha needed to be seen at all is a grey rock with a polygonal edge, however finely it is subdivided, because a closed opaque-ish surface has a hard silhouette wherever you put it. Overlapping seventeen of them at about a sixth of that alpha each puts the density in the middle and lets the edge fall off, which is the only soft edge available without a per-fragment falloff the fixed shaders cannot express.
// The variation is deterministic: Hash2 keyed on the puff index, the same lattice hash the diffuse maps are built from, so the plume is identical on every run and a critique of frame 3 still refers to the same frame 3 tomorrow.
static void MakePuffs(void)
{
    for (int i = 0; i < MUZZLE_PUFFS; i++) {
        PuffSpec *s = &gPuffSpec[i];
        float u = (float)i / (float)(MUZZLE_PUFFS - 1);      // 0 at the muzzle, 1 at the head of the plume
        // The size spread is deliberately wide. Puffs within a narrow size band read as a row of repeated shells however many there are, because each one contributes a silhouette of about the same radius; a spread of four to one breaks that up.
        float big = Hash2(i, 1, 31) * Hash2(i, 8, 31);
        s->from = (Vector3){ (Hash2(i, 9, 31) - 0.5f) * 26.0f, BORE + (Hash2(i, 10, 31) - 0.5f) * 22.0f, -4.0f - 52.0f * u };
        s->t0 = 0.0004f * (float)i;   // real seconds after this round's ignition
        s->r0 = 4.0f + 9.0f * big;
        s->grow = 170.0f + 420.0f * big;
        s->drift = (Vector3){ (Hash2(i, 3, 31) - 0.5f) * 520.0f,
                              70.0f + 400.0f * Hash2(i, 4, 31),
                              -600.0f - 2500.0f * u };
        s->rot = (Vector3){ Hash2(i, 5, 31) * 2.0f * PI, Hash2(i, 6, 31) * 2.0f * PI, Hash2(i, 7, 31) * 2.0f * PI };
        s->peak = 0.135f - 0.065f * u;
    }

    // The port only smokes once the action has opened it, so these start after the carrier has begun to travel and go up and outboard.
    // They are deliberately much fainter than the muzzle plume. At the muzzle's density they drift between the camera and the receiver and veil the whole flank, and the ejection port is the one thing in this model a reviewer most needs to see into.
    for (int i = 0; i < PORT_PUFFS; i++) {
        PuffSpec *s = &gPuffSpec[MUZZLE_PUFFS + i];
        float u = (float)i / (float)(PORT_PUFFS - 1);
        s->from = (Vector3){ 14.0f, 220.0f, 415.0f + 30.0f * u };
        s->t0 = 0.019f + 0.006f * (float)i;
        s->r0 = 8.0f + 6.0f * Hash2(i, 11, 57);
        s->grow = 300.0f + 200.0f * Hash2(i, 12, 57);
        s->drift = (Vector3){ 190.0f + 170.0f * Hash2(i, 13, 57),
                              430.0f + 380.0f * Hash2(i, 14, 57),
                              (Hash2(i, 15, 57) - 0.5f) * 260.0f };
        s->rot = (Vector3){ Hash2(i, 16, 57) * 2.0f * PI, Hash2(i, 17, 57) * 2.0f * PI, Hash2(i, 18, 57) * 2.0f * PI };
        s->peak = 0.10f - 0.04f * u;
    }
}

static Builder gPuffB;
static Model gPuffM;
static BoundingBox gPuffLocal;
static Matrix gPuffX[PUFF_INSTANCES];
static float gPuffA[PUFF_INSTANCES];

// A sphere of unit world radius, with its radius pushed about by a few low-order harmonics so a puff is not a billiard ball. Emitted at 100 mm because Vert scales by UNIT, and 100 mm is one world unit.
// Normals are radial and approximate; nothing that draws this is lit, so they are never read.
static void Blob(Builder *b, int seed, int rings, int segs)
{
    float s = (float)seed * 1.7f;
    for (int i = 0; i < rings; i++) {
        for (int j = 0; j < segs; j++) {
            Vector3 q[4];
            for (int c = 0; c < 4; c++) {
                float phi = PI * (float)(i + ((c == 2 || c == 3) ? 1 : 0)) / (float)rings;
                float th = 2.0f * PI * (float)(j + ((c == 1 || c == 2) ? 1 : 0)) / (float)segs;
                float r = 100.0f * (1.0f + 0.26f * sinf(3.0f * th + s) * sinf(2.0f * phi)
                                         + 0.17f * cosf(5.0f * phi + s * 1.3f)
                                         + 0.14f * sinf(4.0f * th - 3.0f * phi + s)
                                         + 0.09f * cosf(7.0f * th + 2.0f * phi + s * 2.1f));
                q[c] = (Vector3){ r * sinf(phi) * sinf(th), r * cosf(phi), r * sinf(phi) * cosf(th) };
            }
            Quad(b, q[0], q[1], q[2], q[3]);
        }
    }
}

static void BuildPuff(void)
{
    gOrigin = (Vector3){ 0.0f, 0.0f, 0.0f };
    Mark m = { gPuffB.vertexCount, gPuffB.triangleCount };
    Blob(&gPuffB, 3, 18, 26);
    // Shaded down the y axis: smoke is lit from above by the same key light as everything else, and unlit geometry has to be told so.
    ShadeRange(&gPuffB, m, 1, (Color){ 92, 94, 102, 255 }, (Color){ 214, 214, 220, 255 });
    gOrigin = STATIC_ORIGIN;
}

// ---------------------------------------------------------------------------
// Pose
//
// A three-round burst, shown at one sixteenth speed. The AK-47's cyclic rate is 600 rounds per minute, so one action cycle is 0.100 s of real time and the burst is 0.300 s; the cycle runs five shot periods so that the last round has somewhere to finish and the loop has a pause in it. Every motion below is computed in real seconds and played back over CYCLE, which is what lets three ballistic cases, three smoke plumes and the recoil of three shots all share one clock instead of being animated by eye.
//
// Ignition of each round falls on a fifth of the cycle: phases 0, 0.2 and 0.4. That is deliberate and it is the only way the muzzle flashes can be rendered at all, since a flash lasts about 3 ms, under one percent of this cycle. Any --frames that is a multiple of 5 lands a frame on all three ignitions; --frames 15 is the useful default and --frames 8 will catch only the first.
//
// The travel is NOT a measured figure. No reference found gives one, so it is the largest travel this receiver admits: the carrier stops against the channel's rear wall. CheckAction measures the margin at both ends and reports which limit binds.
// ---------------------------------------------------------------------------

#define TRAVEL      72.0f     // carrier travel, mm

// Fractions of ONE shot, not of the cycle.
#define T_FLASH     0.030f   // flash gone by here: 3 ms of real time

// A flash is 3 ms. At sixteen times slow that was 48 ms of playback, about three frames of a 60 Hz window; at real time it is 3 ms, under a fifth of one frame, so the window would drop it on most cycles and the rounds would go off invisibly. This floor holds a flash on for FLASH_MIN_PLAYBACK of PLAYBACK time however fast the playback is set, so it bites only when the playback is fast: at the old sixteen times it is shorter than the real duration and does nothing at all.
// It is the one place in this file that looks at playback rather than at the rifle, and it is a concession to the frame rate rather than a claim about muzzle flash.
// It DOES reach a rendered sequence whenever the interval between frames is shorter than the floor, which an earlier note here wrongly denied. At real time the floor is the full 34 ms, against 33 ms between frames at --frames 15 and 20 ms at --frames 25, so a flash now occupies two frames rather than one at any useful frame count. That is the price of playing the burst at the rate the rifle fires it: a 3 ms event cannot be both real-time and one frame long at 60 Hz, and a flash the window drops entirely is worse than one held a frame too long.
#define FLASH_MIN_PLAYBACK 0.034f
#define T_UNLOCK    0.010f   // carrier starts moving
#define T_REAR      0.300f   // fraction of a shot at which it reaches full travel
#define T_DWELL     0.360f
#define T_HOME      0.940f   // back in battery just as the next round fires, which is what a cyclic rate means
// The case leaves at the end of the rearward stroke, not partway through it, and that is forced rather than chosen. Its centre sits 19.35 mm from each end, so it is only clear of the port's front edge at z 395 once the carrier has drawn it back past z 414.35, which takes essentially the whole 72 mm of travel. Ejecting at 0.18 put 29 mm of its forward end through the receiver wall ahead of the port, which is what the z half of the crossing check caught.
#define T_EJECT     0.310f   // fraction of a shot at which the case leaves the extractor, during the dwell at full travel

#define GRAVITY    9810.0f   // mm/s^2
#define CASE_VX    5000.0f   // ejection velocity, mm/s: right, up, and slightly forward. Faster than it needs to look, because the crossing time is what sets how far the case drifts vertically while it is in the opening
#define CASE_VY    1250.0f
#define CASE_VZ    -300.0f
#define CASE_LIFT    12.1f   // the ejector kicks the case up off the bolt face, and it has to: a case centred on the bore at 204.4 does not line up with an opening that starts at 211, so it leaves all but sitting on the sill

// The case does not begin to tumble until it is clear of the receiver wall, and that is physics rather than convenience: while it is in the port it is still boxed in by the opening it is going through. A 17 mm port passing an 11.35 mm case leaves 5.65 mm of slack in total, so a case tumbling on the way out simply does not fit, and CheckAction proves it does not: sampling the case's surface rather than its axis, a free tumble from the moment of extraction put brass at y 208.0 to 230.2 through a port of 211 to 228.
// Held straight until it is out, the same case clears with about a millimetre at each edge. That is a tight fit, and it should be: a real ejection port is a tight fit.
#define CASE_SPIN    12.0f   // turns a second, once it is clear

static float Ramp(float t, float t0, float t1)
{
    if (t <= t0) return 0.0f;
    if (t >= t1) return 1.0f;
    return Smoothstep01((t - t0) / (t1 - t0));
}

// One action cycle, u running 0 to 1 across it.
static float CarrierStroke(float u)
{
    if (u < T_REAR) return TRAVEL * Ramp(u, T_UNLOCK, T_REAR);
    if (u < T_DWELL) return TRAVEL;
    return TRAVEL * (1.0f - Ramp(u, T_DWELL, T_HOME));
}

// The carrier runs one stroke per round and then stops in battery for the rest of the cycle.
static float CarrierTravelAt(float tr)
{
    if (tr < 0.0f || tr >= BURST_T) return 0.0f;
    return CarrierStroke(fmodf(tr, SHOT_T) / SHOT_T);
}

static float TriggerAngle(float tr)
{
    // Held down through the whole burst, which is what makes it a burst rather than three shots, then released and pulled again before the cycle repeats so the end meets the start with no step in it.
    // Negative about local x swings the shoe aft: the shoe sits below and forward of the pin, and z' = y sin a + z cos a with a negative local y.
    float pull = 1.0f - Ramp(tr, BURST_T + 0.02f, BURST_T + 0.06f) + Ramp(tr, TOTAL_T - 0.05f, TOTAL_T);
    return -TRIG_PULL * DEG2RAD * pull;
}

// The frame the whole rifle sits in. Every body on the rifle is posed inside it; a case once it has left the extractor, and smoke once it has left the muzzle, are not, because they are no longer attached to anything that recoils.
//
// Each round contributes its own impulse and they are summed, which is where a burst stops being three separate shots. The two responses have different time constants ON PURPOSE, and that is the whole of why a burst climbs:
//   the shoulder absorbs the rearward push and gives it back inside RECOIL_T = 60 ms, which is less than the 100 ms between rounds, so the linear recoil is fully spent before the next round fires and does not accumulate. Every shot moves the rifle back the same 24.6 mm.
//   the arms let the muzzle rise and bring it down over RISE_T = 220 ms, which is longer than the gap, so the second round's rotation lands on top of the first's and the third on top of both. Peak climb is over three times a single shot's.
// Both constants are chosen rather than derived, as the single-shot shoulder always was; what is derived is the impulse each round delivers.
#define RECOIL_T     0.060f   // real seconds for the shoulder to absorb one round and return
#define RISE_T       0.220f   // real seconds for the arms to bring the muzzle back down

static Matrix RifleAt(float tr, float *outAft, float *outRise)
{
    float v = BULLET_M * BULLET_V / RIFLE_M * 1000.0f;                              // mm/s
    float w = BULLET_M * BULLET_V * ((BORE - SHOULDER_Y) * 0.001f) / RIFLE_I;       // rad/s

    float aft = 0.0f, rise = 0.0f;
    for (int k = 0; k < SHOTS; k++) {
        float dt = tr - (float)k * SHOT_T;
        if (dt > 0.0f && dt < RECOIL_T) aft += v * RECOIL_T / PI * sinf(PI * dt / RECOIL_T);
        if (dt > 0.0f && dt < RISE_T) rise += w * RISE_T / PI * sinf(PI * dt / RISE_T);
    }
    if (outAft) *outAft = aft;
    if (outRise) *outRise = rise;

    // Positive about x lifts the muzzle: the muzzle is at negative z from the shoulder, and y' = y cos a - z sin a.
    Vector3 s0 = ToWorld((Vector3){ 0.0f, SHOULDER_Y, SHOULDER_Z });
    return MatrixMultiply(MatrixTranslate(-s0.x, -s0.y, -s0.z),
                          MatrixMultiply(MatrixRotateX(rise),
                                         MatrixTranslate(s0.x, s0.y, s0.z + aft * UNIT)));
}

static float gRecoilAft, gRecoilRise;
static Matrix gCaseX[SHOTS];
static bool gCaseOn[SHOTS];

// Where round k's case is. Before its own round fires it is still in the magazine and is not drawn at all; between ignition and extraction it rides that round's bolt face; after that it is a free body thrown from wherever the rifle was when it let go.
static void PoseCase(int k, float tr, float rifleTravel, Matrix rifle)
{
    float dt = tr - (float)k * SHOT_T;
    gCaseOn[k] = (dt >= 0.0f);
    if (!gCaseOn[k]) return;

    Vector3 c = ToWorld(gCase.pivot);
    if (dt < T_EJECT * SHOT_T) {
        gCaseX[k] = MatrixMultiply(MatrixTranslate(c.x, c.y, c.z + rifleTravel * UNIT), rifle);
        return;
    }

    float ft = dt - T_EJECT * SHOT_T;             // real seconds since it left the extractor
    float x = CASE_VX * ft;
    float y = CASE_VY * ft - 0.5f * GRAVITY * ft * ft;
    float z = CASE_VZ * ft;
    // Free to tumble only once its whole width is past the outer face of the receiver plate.
    float clear = (RCV_SIDE + CASE_R) / CASE_VX;
    float spin = 2.0f * PI * CASE_SPIN * fmaxf(0.0f, ft - clear);

    float ejectAt = (float)k * SHOT_T + T_EJECT * SHOT_T;
    Vector3 origin = Vector3Transform((Vector3){ c.x, c.y + CASE_LIFT * UNIT,
                                                 c.z + CarrierStroke(T_EJECT) * UNIT },
                                      RifleAt(ejectAt, NULL, NULL));
    gCaseX[k] = MatrixMultiply(MatrixRotateX(spin),
                               MatrixTranslate(origin.x + x * UNIT,
                                               origin.y + y * UNIT,
                                               origin.z + z * UNIT));
}

static void Update(float t)
{
    float phase = t / CYCLE;
    phase -= floorf(phase);
    float tr = phase * TOTAL_T;              // real seconds into the burst

    Matrix rifle = RifleAt(tr, &gRecoilAft, &gRecoilRise);

    gSight.xform = MatrixMultiply(GroupRest(&gSight), rifle);
    gBarrel.xform = MatrixMultiply(GroupRest(&gBarrel), rifle);
    gWood.xform = MatrixMultiply(GroupRest(&gWood), rifle);
    gRecv.xform = MatrixMultiply(GroupRest(&gRecv), rifle);
    gMag.xform = MatrixMultiply(GroupRest(&gMag), rifle);
    gFire.xform = MatrixMultiply(GroupRest(&gFire), rifle);
    gStock.xform = MatrixMultiply(GroupRest(&gStock), rifle);

    float travel = CarrierTravelAt(tr);
    Vector3 c = ToWorld(gCarrier.pivot);
    gCarrier.xform = MatrixMultiply(MatrixTranslate(c.x, c.y, c.z + travel * UNIT), rifle);

    Vector3 p = ToWorld(gTrigger.pivot);
    gTrigger.xform = MatrixMultiply(MatrixMultiply(MatrixRotateX(TriggerAngle(tr)),
                                                   MatrixTranslate(p.x, p.y, p.z)), rifle);

    // The flash is gas still leaving the barrel, so it belongs to the rifle and moves with it. Over its three milliseconds the rifle has barely started to move anyway: the flash is over long before the recoil is.
    Vector3 fm = ToWorld((Vector3){ 0.0f, BORE, 0.0f });
    gFlashOn = false;
    float flashT = fmaxf(T_FLASH * SHOT_T, FLASH_MIN_PLAYBACK / SLOWDOWN);
    for (int k = 0; k < SHOTS; k++) {
        float dt = tr - (float)k * SHOT_T;
        if (dt >= 0.0f && dt < flashT) gFlashOn = true;
    }
    gFlashX = MatrixMultiply(MatrixTranslate(fm.x, fm.y, fm.z), rifle);

    for (int k = 0; k < SHOTS; k++) PoseCase(k, tr, travel, rifle);

    for (int k = 0; k < SHOTS; k++) {
        for (int i = 0; i < PUFF_COUNT; i++) {
            const PuffSpec *s = &gPuffSpec[i];
            int n = k * PUFF_COUNT + i;
            float born = (float)k * SHOT_T + s->t0;
            float age = tr - born;
            if (age < 0.0f || age > PUFF_LIFE) { gPuffA[n] = 0.0f; continue; }

            // Smoke leaves the muzzle or the port and is then free of the rifle, so its start point is where that opening was at the moment it left.
            Vector3 origin = Vector3Transform(ToWorld(s->from), RifleAt(born, NULL, NULL));
            float r = (s->r0 + s->grow * age) * UNIT;

            // Drag, not constant velocity. A puff carried at its jet speed for the whole of a burst ends up 800 mm downrange and out of frame; real smoke sheds its momentum in a few tens of milliseconds and then hangs. This is that, as a first-order decay to a terminal displacement of drift * PUFF_DRAG.
            float carried = PUFF_DRAG * (1.0f - expf(-age / PUFF_DRAG));

            gPuffX[n] = MatrixMultiply(MatrixMultiply(MatrixScale(r, r, r), MatrixRotateXYZ(s->rot)),
                                       MatrixTranslate(origin.x + s->drift.x * carried * UNIT,
                                                       origin.y + s->drift.y * carried * UNIT,
                                                       origin.z + s->drift.z * carried * UNIT));
            gPuffA[n] = s->peak * Clamp(age / 0.004f, 0.0f, 1.0f) * (1.0f - age / PUFF_LIFE);
        }
    }
}

// ---------------------------------------------------------------------------
// The claims a pose can get wrong without looking wrong in any one frame, measured over the cycle rather than argued for.
// ---------------------------------------------------------------------------

static void CheckAction(void)
{
    // Five times the real time of the single-shot cycle, so the step count goes up five times with it: the case crosses the 5.7 mm of receiver wall in about 5 ms, and a sampling interval coarser than that cannot support the millimetre margins reported below.
    const int steps = 3600;
    float handRear = -1e9f, carRear = -1e9f, headAft = -1e9f, headFwd = 1e9f;
    float shoeAft = -1e9f, shoeLow = 1e9f;
    float portLoY = 1e9f, portHiY = -1e9f;
    float portLoZ = 1e9f, portHiZ = -1e9f;
    bool portSeen = false;

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / (float)steps;
        float tr = phase * TOTAL_T;
        Update(CYCLE * phase);
        float travel = CarrierTravelAt(tr);

        if (HAND_Z1 + travel > handRear) handRear = HAND_Z1 + travel;
        if (CAR_Z0 + CAR_L + travel > carRear) carRear = CAR_Z0 + CAR_L + travel;
        if (GASTUBE_Z0 + 29.0f + travel > headAft) headAft = GASTUBE_Z0 + 29.0f + travel;
        if (GASTUBE_Z0 + 5.0f + travel < headFwd) headFwd = GASTUBE_Z0 + 5.0f + travel;

        // Everything the trigger and the case are checked against is on the rifle and recoils with it, so both are measured in the rifle's frame. Measuring them in the world instead compares a recoiled part against an un-recoiled limit, which is what the first version of this did: it reported the trigger 21 mm through a guard it never touches.
        Matrix rifleInv = MatrixInvert(RifleAt(tr, NULL, NULL));

        // Trigger shoe, transformed by the pose rather than predicted from it.
        Vector3 shoe = Vector3Transform((Vector3){ 0.0f, (140.0f - TRIG_PIVOT_Y) * UNIT, (549.0f - TRIG_PIVOT_Z) * UNIT },
                                        MatrixMultiply(gTrigger.xform, rifleInv));
        float shoeZ = shoe.z / UNIT + Z_ORIGIN, shoeY = shoe.y / UNIT;
        if (shoeZ > shoeAft) shoeAft = shoeZ;
        if (shoeY < shoeLow) shoeLow = shoeY;

        // Where the case crosses the thickness of the receiver plate: that is the moment it is either going through the ejection port or through the side of the rifle. Both ends and the middle are sampled, because a tumbling case presents anything between its 11.35 mm diameter and its 38.7 mm length to the opening.
        // Every round's case, not just one: three of them go through the same opening and only the check knows whether all three make it. A case still riding the bolt never enters the x window below, so no separate test for "has it been ejected yet" is needed.
        for (int k = 0; k < SHOTS; k++) {
            if (!gCaseOn[k]) continue;
            // The case's SURFACE, not its axis. Sampling the centreline says nothing about a body 11.35 mm thick: an axis 4.8 mm below the top edge of the port still has 0.9 mm of brass through the plate. Five stations along the case by sixteen around it, at the full base radius, so the neck end is checked conservatively.
            Matrix toRifle = MatrixMultiply(gCaseX[k], rifleInv);
            for (int e = 0; e <= 4; e++) {
                for (int a = 0; a < 16; a++) {
                    float ang = 2.0f * PI * (float)a / 16.0f;
                    Vector3 local = { CASE_R * cosf(ang) * UNIT, CASE_R * sinf(ang) * UNIT,
                                      ((float)e / 4.0f - 0.5f) * CASE_L * UNIT };
                    Vector3 q = Vector3Transform(local, toRifle);
                    float qx = q.x / UNIT, qy = q.y / UNIT;
                    if (qx >= CHAN_HW && qx <= RCV_SIDE) {
                        // Height alone does not prove the case got out: brass at the right height but forward of the port's front edge is going through the plate just as surely. The opening is bounded both ways and so is this.
                        float qz = q.z / UNIT + Z_ORIGIN;
                        portSeen = true;
                        if (qy < portLoY) portLoY = qy;
                        if (qy > portHiY) portHiY = qy;
                        if (qz < portLoZ) portLoZ = qz;
                        if (qz > portHiZ) portHiZ = qz;
                    }
                }
            }
        }
    }

    // Recoil, reported as what the momentum actually comes to rather than as the numbers that were typed in.
    {
        float v = BULLET_M * BULLET_V / RIFLE_M * 1000.0f;
        float w = BULLET_M * BULLET_V * ((BORE - SHOULDER_Y) * 0.001f) / RIFLE_I;
        float aftPeak = 0.0f, risePeak = 0.0f, aftEnd = 0.0f;
        for (int i = 0; i <= steps; i++) {
            float a, r;
            RifleAt((float)i / (float)steps * TOTAL_T, &a, &r);
            if (a > aftPeak) aftPeak = a;
            if (r > risePeak) risePeak = r;
            aftEnd = a;
        }
        // A moment arm this long turns into rise at the muzzle 880 mm from the shoulder.
        float muzzleUp = risePeak * (SHOULDER_Z - 0.0f);
        TraceLog(LOG_INFO, "ak47: recoil velocity %.0f mm/s and %.3f rad/s from a %.1f g bullet at %.0f m/s into %.2f kg on an %.1f mm moment arm",
                 v, w, BULLET_M * 1000.0f, BULLET_V, RIFLE_M, BORE - SHOULDER_Y);
        // One shot on its own gives 24.6 mm and 0.64 degrees. The aft figure should match it, because the shoulder is done inside 60 ms and the next round is 100 ms away; the rise figure should be several times it, because the arms are not, and that difference IS the burst.
        TraceLog(LOG_INFO, "ak47: %d rounds; peak recoil %.1f mm aft (one shot alone gives %.1f) and %.2f degrees of climb (one shot alone gives %.2f), lifting the muzzle %.1f mm; %.3f mm left at the end of the cycle",
                 SHOTS, aftPeak, v * RECOIL_T / PI, risePeak * RAD2DEG, w * RISE_T / PI * RAD2DEG, muzzleUp, aftEnd);
        if (aftEnd > 0.05f) {
            TraceLog(LOG_WARNING, "ak47: the rifle is still %.2f mm out of battery when the cycle repeats", aftEnd);
        }
    }

    TraceLog(LOG_INFO, "ak47: travel %.1f mm; carrier rear reaches %.1f against the channel wall at %.1f (%.1f mm spare)",
             TRAVEL, carRear, CHAN_Z1, CHAN_Z1 - carRear);
    TraceLog(LOG_INFO, "ak47: charging handle reaches %.1f against the slot end at %.1f (%.1f mm spare); the channel wall is the binding limit",
             handRear, TRACK_Z1, TRACK_Z1 - handRear);
    // BOTH ends. Testing only the aft one reported 112 mm of spare while 23 mm of the head stood out in the open ahead of the tube at every pose where the carrier was forward, which is the failure this file's own notes describe: a bound checked in one axis and not the other.
    TraceLog(LOG_INFO, "ak47: piston head runs %.1f to %.1f inside a gas tube of %.1f to %.1f (%.1f mm spare at the front, %.1f at the rear)",
             headFwd, headAft, GASTUBE_Z0, GASTUBE_Z1, headFwd - GASTUBE_Z0, GASTUBE_Z1 - headAft);
    TraceLog(LOG_INFO, "ak47: travel %.1f mm against a 56.0 mm cartridge, %.1f mm of clearance for the next round",
             TRAVEL, TRAVEL - 56.0f);
    TraceLog(LOG_INFO, "ak47: trigger shoe reaches z %.1f, guard rear web at 558.0 (%.1f mm spare), and drops to y %.1f over the bow at 133.4 (%.1f mm spare)",
             shoeAft, 558.0f - shoeAft, shoeLow, shoeLow - 133.4f);

    if (carRear > CHAN_Z1) TraceLog(LOG_WARNING, "ak47: carrier passes through the channel rear wall by %.2f mm", carRear - CHAN_Z1);
    if (handRear > TRACK_Z1) TraceLog(LOG_WARNING, "ak47: charging handle passes the end of its slot by %.2f mm", handRear - TRACK_Z1);
    if (headAft > GASTUBE_Z1) TraceLog(LOG_WARNING, "ak47: piston head leaves the back of the gas tube by %.2f mm", headAft - GASTUBE_Z1);
    if (headFwd < GASTUBE_Z0) TraceLog(LOG_WARNING, "ak47: piston head stands %.2f mm out of the front of the gas tube", GASTUBE_Z0 - headFwd);
    if (TRAVEL < 56.0f) TraceLog(LOG_WARNING, "ak47: travel %.1f mm is under the 56.0 mm cartridge length", TRAVEL);

    if (!portSeen) {
        TraceLog(LOG_WARNING, "ak47: no case crosses the receiver wall; the ejection is not being sampled");
    } else if (portLoY < PORT_Y0 || portHiY > RCV_TOP || portLoZ < PORT_Z0 || portHiZ > PORT_Z1) {
        TraceLog(LOG_WARNING, "ak47: the case crosses the receiver wall at y %.1f to %.1f and z %.1f to %.1f, outside the ejection port's y %.1f to %.1f by z %.1f to %.1f",
                 portLoY, portHiY, portLoZ, portHiZ, PORT_Y0, RCV_TOP, PORT_Z0, PORT_Z1);
    } else {
        TraceLog(LOG_INFO, "ak47: the case crosses the receiver wall at y %.1f to %.1f and z %.1f to %.1f, inside the ejection port's y %.1f to %.1f by z %.1f to %.1f",
                 portLoY, portHiY, portLoZ, portHiZ, PORT_Y0, RCV_TOP, PORT_Z0, PORT_Z1);
    }
}

// A part that moves has no single bounding box, so --part frames the union over the cycle.
// GetModelBoundingBox will not do the job: it transforms only the box's own min and max corner and carries the warning "does not support rotation transformations" (vendor/raylib/src/rmodels.c:1243), which is wrong for the trigger and badly wrong for the tumbling case.
static void SweepBounds(Group **groups, int n)
{
    // Five shot periods, and the dwell at full travel is 6 ms of them, so the sampling has to be fine enough to land on it.
    const int steps = 240;
    bool seen[16] = { false };

    for (int i = 0; i <= steps; i++) {
        Update(CYCLE * (float)i / (float)steps);
        for (int g = 0; g < n; g++) {
            BoundingBox l = groups[g]->local;
            for (int c = 0; c < 8; c++) {
                Vector3 corner = {
                    (c & 1) ? l.max.x : l.min.x,
                    (c & 2) ? l.max.y : l.min.y,
                    (c & 4) ? l.max.z : l.min.z,
                };
                Vector3 w = Vector3Transform(corner, groups[g]->xform);
                if (!seen[g]) { groups[g]->swept.min = w; groups[g]->swept.max = w; seen[g] = true; }
                else {
                    groups[g]->swept.min = Vector3Min(groups[g]->swept.min, w);
                    groups[g]->swept.max = Vector3Max(groups[g]->swept.max, w);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------

static void Init(void)
{
    BuildTextures();

    BuildSight();
    BuildBarrel();
    BuildWood();
    BuildRecv();
    BuildMag();
    BuildFire();
    BuildStock();
    BuildCarrier();
    BuildTrigger();
    BuildCase();

    GroupFinish(&gSight, "front_sight");
    GroupFinish(&gBarrel, "barrel");
    GroupFinish(&gWood, "handguards");
    GroupFinish(&gRecv, "receiver");
    GroupFinish(&gMag, "magazine");
    GroupFinish(&gFire, "fire_control");
    GroupFinish(&gStock, "stock");
    GroupFinish(&gCarrier, "bolt_carrier");
    GroupFinish(&gTrigger, "trigger");
    GroupFinish(&gCase, "case");

    BuildFlash();
    MakePuffs();
    BuildPuff();
    gFlashM = FinishUnlit(&gFlashB, "muzzle_flash");
    gPuffM = FinishUnlit(&gPuffB, "smoke");
    gFlashLocal = GetModelBoundingBox(gFlashM);
    gPuffLocal = GetModelBoundingBox(gPuffM);

    CheckAction();

    Group *moving[] = { &gCarrier, &gTrigger };
    SweepBounds(moving, 2);
    Update(0.0f);
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
    GroupUnload(&gCarrier);
    GroupUnload(&gTrigger);
    GroupUnload(&gCase);
    UnloadModel(gFlashM);
    UnloadModel(gPuffM);
    UnloadTextures();
}

static void DrawSight(void) { GroupDraw(&gSight); }
static void DrawBarrel(void) { GroupDraw(&gBarrel); }
static void DrawWood(void) { GroupDraw(&gWood); }
static void DrawRecv(void) { GroupDraw(&gRecv); }
static void DrawMag(void) { GroupDraw(&gMag); }
static void DrawFire(void) { GroupDraw(&gFire); }
static void DrawStock(void) { GroupDraw(&gStock); }
static void DrawCarrier(void) { GroupDraw(&gCarrier); }
static void DrawTrigger(void) { GroupDraw(&gTrigger); }
// Three rounds, three cases. Each is the same mesh drawn where its own round put it, and a round that has not been fired yet is not drawn at all: it is still in the magazine.
static void DrawCase(void)
{
    for (int k = 0; k < SHOTS; k++) {
        if (!gCaseOn[k]) continue;
        gCase.xform = gCaseX[k];
        GroupDraw(&gCase);
    }
}

static void DrawFlash(void)
{
    if (!gFlashOn) return;
    gFlashM.transform = gFlashX;
    DrawModel(gFlashM, Vector3Zero(), 1.0f, WHITE);
}

// Depth writes off, and last in the part list, so the puffs blend with each other and with the rifle instead of the nearest-drawn one punching a hole in the ones behind it. They still depth-test, so smoke behind the receiver stays behind it.
static void DrawSmoke(void)
{
    rlDisableDepthMask();
    for (int i = 0; i < PUFF_INSTANCES; i++) {
        if (gPuffA[i] <= 0.004f) continue;
        gPuffM.transform = gPuffX[i];
        DrawModel(gPuffM, Vector3Zero(), 1.0f,
                  (Color){ 255, 255, 255, (unsigned char)(gPuffA[i] * 255.0f) });
    }
    rlEnableDepthMask();
}

static BoundingBox SightBounds(void) { return gSight.local; }
static BoundingBox BarrelBounds(void) { return gBarrel.local; }
static BoundingBox WoodBounds(void) { return gWood.local; }
static BoundingBox RecvBounds(void) { return gRecv.local; }
static BoundingBox MagBounds(void) { return gMag.local; }
static BoundingBox FireBounds(void) { return gFire.local; }
static BoundingBox StockBounds(void) { return gStock.local; }
static BoundingBox CarrierBounds(void) { return gCarrier.swept; }
static BoundingBox TriggerBounds(void) { return gTrigger.swept; }

// The case is the one part framed to itself rather than to its sweep. The swept rule exists so that a body which turns or slides still frames to something a viewer can see; a body thrown 120 mm clear of the rifle frames to a box in which it is a speck. What an isolated view of a cartridge case is for is judging the case, and its arc is judged in the assembled --anim frames where the port it came out of is also in shot.
static BoundingBox CaseBounds(void)
{
    Vector3 c = ToWorld(gCase.pivot);
    BoundingBox b = gCase.local;
    b.min = Vector3Add(b.min, c);
    b.max = Vector3Add(b.max, c);
    return b;
}

static BoundingBox FlashBounds(void)
{
    Vector3 c = ToWorld((Vector3){ 0.0f, BORE, 0.0f });
    BoundingBox b = gFlashLocal;
    b.min = Vector3Add(b.min, c);
    b.max = Vector3Add(b.max, c);
    return b;
}

// The smoke's box IS its sweep: the puffs are what the part is, and where they get to is the thing worth looking at.
static BoundingBox SmokeBounds(void)
{
    BoundingBox out = { 0 };
    bool any = false;
    for (int i = 0; i <= 120; i++) {
        Update(CYCLE * (float)i / 120.0f);
        for (int k = 0; k < PUFF_INSTANCES; k++) {
            if (gPuffA[k] <= 0.004f) continue;
            for (int c = 0; c < 8; c++) {
                Vector3 corner = {
                    (c & 1) ? gPuffLocal.max.x : gPuffLocal.min.x,
                    (c & 2) ? gPuffLocal.max.y : gPuffLocal.min.y,
                    (c & 4) ? gPuffLocal.max.z : gPuffLocal.min.z,
                };
                Vector3 w = Vector3Transform(corner, gPuffX[k]);
                if (!any) { out.min = w; out.max = w; any = true; }
                else { out.min = Vector3Min(out.min, w); out.max = Vector3Max(out.max, w); }
            }
        }
    }
    Update(0.0f);
    return out;
}

static const Part PARTS[] = {
    { .name = "front_sight", .draw = DrawSight, .bounds = SightBounds },
    { .name = "barrel", .draw = DrawBarrel, .bounds = BarrelBounds },
    { .name = "handguards", .draw = DrawWood, .bounds = WoodBounds },
    { .name = "receiver", .draw = DrawRecv, .bounds = RecvBounds },
    { .name = "magazine", .draw = DrawMag, .bounds = MagBounds },
    { .name = "fire_control", .draw = DrawFire, .bounds = FireBounds },
    { .name = "stock", .draw = DrawStock, .bounds = StockBounds },
    { .name = "bolt_carrier", .draw = DrawCarrier, .bounds = CarrierBounds },
    { .name = "trigger", .draw = DrawTrigger, .bounds = TriggerBounds },
    { .name = "case", .draw = DrawCase, .bounds = CaseBounds },
    { .name = "muzzle_flash", .draw = DrawFlash, .bounds = FlashBounds },
    // Last, and it has to stay last: it is the only transparent thing here and it is drawn with depth writes off.
    { .name = "smoke", .draw = DrawSmoke, .bounds = SmokeBounds },
};

const Scene SCENE = {
    .name = "ak47",
    .description =
        "AK-47 with a Type 2 milled receiver and fixed wooden furniture, the configuration in references/ak47/ref_04.png, posed through a three-round burst. Four bodies are lifted out of the static groups into their own frames; the seven static parts below are the rifle's measured geometry and are unaffected by the posing.\n"
        "\n"
        "WHAT MOVES, AND WHAT IS CLAIMED\n"
        "Three rounds, played back at real time. The AK-47's cyclic rate is 600 rounds a minute, so one action cycle is 0.100 s of real time and the burst is 0.300 s; the cycle runs five shot periods, three firing and two of pause, so the last round has somewhere to finish and the loop has a gap in it. That is 0.500 s of real time over 0.500 s of playback. Every motion is computed in real seconds and played back on that one clock, which is what lets three ballistic cases, three smoke plumes and the recoil of three rounds share a clock with the carrier that threw them rather than being animated by eye.\n"
        "The playback rate is the ONE quantity in this model that is about playback rather than about the rifle, and it scales only what the interactive window and SCENE.duration do. It cannot change a rendered sequence: --anim samples N evenly spaced phases of the cycle, so the frames come out identical whatever it is set to, which was checked rather than assumed by rendering fifteen frames at one sixteenth speed and again at one quarter and finding all fifteen byte for byte the same. If a rendered sequence looks slow, the number to change is the frame rate it is played at, and real speed is N / 0.5 fps: 30 fps for --frames 15, 60 for --frames 30.\n"
        "One consequence is worth naming. A flash is 3 ms, which at one sixteenth speed was three frames of a 60 Hz window and at real time is under a fifth of one, so the window would drop it and rounds would go off invisibly. A flash is therefore held for a minimum of 34 ms of PLAYBACK, which bites only when the playback is fast and does nothing at the old rate. That is a concession to the frame rate rather than a claim about muzzle flash, and at real time it DOES reach the renders: the floor is 34 ms against 33 ms between frames at --frames 15 and 20 ms at --frames 25, so a flash occupies two frames rather than one. A 3 ms event cannot be both real-time and one frame long at 60 Hz, and a dropped flash is worse than a long one.\n"
        "Ignition falls on a fifth of the cycle: phases 0, 0.2 and 0.4. That is not tidiness, it is the only way the flashes can be rendered at all. A flash lasts about 3 ms, under one percent of this cycle, so no evenly spaced set of frames catches one by luck. Any --frames that is a multiple of 5 lands a frame on all three ignitions; --frames 15 is the useful default, and --frames 8 will show the first flash and miss the other two.\n"
        "\n"
        "recoil: derived, not styled, and applied as a parent frame that every body on the rifle is posed inside. A 7.9 g M43 bullet at 715 m/s into a 4.39 kg rifle with a loaded magazine gives 1.29 m/s by conservation of momentum. The bore axis at y 204.4 is resisted at the shoulder, which the buttplate centre puts at y 121.0, an 83.4 mm moment arm; against about 0.81 kg m^2 that is 0.58 rad/s. Each round contributes that impulse and they are summed, which is where a burst stops being three separate shots.\n"
        "The two responses have DIFFERENT time constants on purpose, and that is the whole of why a burst climbs. The shoulder absorbs the rearward push and gives it back inside 60 ms, which is less than the 100 ms between rounds, so the linear recoil is spent before the next round fires and does not accumulate: every shot moves the rifle back the same 24.6 mm. The arms let the muzzle rise and bring it down over 220 ms, which is longer than the gap, so the second round lands on top of the first and the third on top of both. Peak climb is 3.53 degrees against the 2.33 a single round gives, lifting the muzzle 53 mm rather than 10.\n"
        "Both time constants are chosen rather than derived, as the single-shot shoulder always was; what is derived is the impulse each round delivers. CheckAction reports both figures beside their single-shot values, so the accumulation is visible as a number and not just asserted. The rifle is back to battery, level, at the end of the cycle. The propellant gas is left out of the impulse, which would add roughly a third again, because no reference consulted gives a charge mass for 7.62x39.\n"
        "\n"
        "muzzle_flash: one unlit mesh at the muzzle, a spindle along the bore and six radial spikes about 140 mm long and 190 mm across, white-hot at the muzzle face and falling to orange at the tips through vertex colour. The AK-47 has no flash hider, so it is a broad open star rather than the narrow petals a pronged device makes. The size is judged, not measured. It is deliberately NOT given the lighting shader: nothing in lighting.fs adds emission, so a lit flash is a yellow plastic cone that gets darker away from the key light.\n"
        "smoke: one lumpy unit blob drawn twenty-eight times per round, eighty-four times in all, with a scale, a fixed orientation and a translation, since a puff differs from its neighbours only in where it is, how far it has grown and how faded it is. Twenty-two puffs ride the gas jet forward from the muzzle at up to 3 m/s, six much fainter ones leave the ejection port once the action has opened it. Transparent and therefore also unlit: lighting.fs:73 computes alpha as texelColor.a*(tint.a + 1.0), which is at or above 1.0 for every tint, so that shader cannot carry transparency at all. Depth writes are off while the puffs draw and the smoke is drawn last, so they blend with each other and with the rifle instead of punching holes in one another.\n"
        "Many faint puffs rather than a few solid ones, because a closed surface has a hard silhouette at any subdivision: at the alpha needed to be seen at all, one blob is a grey rock. Overlapping twenty-eight at about a sixth of that alpha, over a four-to-one spread of sizes, puts density in the middle and lets the edge fall off. That is the only soft edge available: a genuinely continuous plume needs a per-fragment falloff, which means a shader, and shaders belong to the harness rather than to any one model. Review rounds three and four both reported the puffs as still individually readable, which is a fair description of what this technique can do rather than a defect that more tuning will remove. The port puffs are much fainter than the muzzle ones on purpose: at the muzzle's density they drift between the camera and the receiver and veil the flank, and the ejection port is the thing a reviewer most needs to see into.\n"
        "Each round lays down its own plume and they pile up, which is what a burst looks like. A puff lives 0.26 s, so the last round's has cleared before the cycle repeats; that fade is a concession to looping and not a claim about smoke, which in reality would still be hanging there. Drift is first-order drag to a terminal displacement rather than constant velocity: carried at its jet speed for a whole burst a puff ends up 800 mm downrange and out of frame, where real smoke sheds its momentum in a few tens of milliseconds and then hangs.\n"
        "The case and the smoke are the two things NOT posed inside the rifle's recoil frame. Both are free bodies once they have left, so each is launched from where its opening was at the instant it let go and is independent of the rifle after that.\n"
        "bolt_carrier: carrier body 140 long by 22 wide by 15 deep, a 14.8 diameter bolt protruding 14 forward of it with two locking lugs, a gas piston on the gas-tube axis with a 15 diameter head, and the charging handle. The head's rest position is read off the gas tube's front cap rather than typed: sited at z 118 to 142 it stood 23 mm proud of the tube in the open air ahead of the gas block whenever the carrier was forward, and withdrew out of sight as the carrier ran back. On the rifle it would sit in the gas block's cylinder, but this model's gas block is not bored, so just inside the tube's mouth is the nearest honest place for it. One mesh, because on the rifle they are one part: the handle is the only externally visible token of where the carrier is, and letting it be a second body would let the two disagree. Local origin on the carrier's front face at the gas-tube axis, so the pose is a bare translation aft.\n"
        "trigger: local origin ON the pin at (y 174, z 550) that it swings about, so the pose is a bare rotation about local x, 9 degrees, about 5 mm at the shoe. The pin head now shown on both receiver flanks is drawn from those same two numbers rather than from a copy of them.\n"
        "case: three of them, one per round, the same mesh drawn where its own round put it. A fired 7.62x39 case is 38.70 long on an 11.35 rim with a 10.07 shoulder and an 8.60 neck, from the cartridge drawing on the 7.62x39mm Wikipedia article. Round k's case is not drawn at all until round k fires, because until then it is a live cartridge in the magazine. It then rides that round's bolt face to the end of the rearward stroke, is kicked 12.1 up off it by the ejector, and is thrown at 5.0 m/s right, 1.25 up and 0.3 forward, tumbling 12 turns a second under gravity once it is clear.\n"
        "Almost every number in that sentence was forced by the port rather than chosen, and the check is what forced them. A 17 mm high opening passing an 11.35 mm thick case leaves 5.65 mm of slack in total, so:\n"
        "  it is ejected at the END of the stroke, not partway through, because its centre sits 19.35 from each end and it is only clear of the port's front edge at z 395 once the carrier has drawn it back past z 414.35. Ejecting it at 18 percent of a shot put 29 mm of its forward end through the receiver wall ahead of the port.\n"
        "  it does not begin to tumble until its whole width is past the outer face of the plate, because while it is in the port it is still boxed in by the opening. A free tumble from the moment of extraction put brass at y 208.0 to 230.2 through a port of 211 to 228.\n"
        "  it is kicked 9.7 up rather than leaving on the bore axis, because a case centred on the bore at y 204.4 does not line up with an opening that starts at 211.\n"
        "  it leaves at 5.0 m/s, faster than it needs to look, because the crossing time is what sets how far it drifts vertically while it is in the opening. All three are checked, not one: they leave with the rifle at three different attitudes, since the climb has accumulated by the second and third.\n"
        "They cross at y 212.2 to 227.0 and z 398.2 to 437.8, against an opening of y 211 to 228 by z 395 to 455: about a millimetre at each edge, sampled at 0.17 mm. That is a tight fit and it should be, because a real ejection port is a tight fit.\n"
        "The check samples the case's SURFACE, sixteen points around it at each of five stations, not its centreline. Sampling the axis says nothing about a body 11.35 thick and it hid all three of the defects above: it reported the ejection as clearing while brass was going through the plate.\n"
        "The case is never hidden. A first version switched it off at 85 percent of the cycle and the review in renders/ak47/v1 caught it vanishing while still a grid square from the rifle and well inside the frame. Flying it out of shot instead is not available: on one time scale for the whole model it has 0.076 s of real flight in a cycle, which carries it 152 mm, against a rifle 880 mm long. So it stays in flight to the end, and the case in the last frame and the case in the first are different rounds, because a repeating cycle fires a fresh one each time.\n"
        "receiver: unchanged outside, cut away inside. A solid billet with two windows milled into it is enough while nothing behind them moves, and that is what this receiver was before the action was posed. Here the core is cut back to a real channel 23 wide with its floor at y 196, a full-height wall on each flank so the milled lightening cut still has a floor, and a rear wall at z 600. The right wall drops to the port sill over the ejection port, so the port is now a hole into the channel instead of a shallow recess, and the charging-handle slot is opened forward from z 380 so the handle has somewhere to sit and somewhere to run.\n"
        "The channel interior is dark and the carrier carries a bright 52 by 14 wear face on its right flank. Both are true of a receiver and a carrier that has cycled, and the pairing is here because four review rounds running reported the ejection port as a barely changing recess: bare milled walls, a blued carrier and a bright patch on it are all mid greys at this camera distance, so nothing in the opening changed value as the carrier ran. Making the patch bigger did not fix that and was not the problem. A dark background behind a bright moving face was. While the receiver was bare bright steel the dark background was three blued liner boxes buried into the left wall, the floor and the rear wall; the receiver is blued throughout now, so the channel is that background on its own and the liners have gone.\n"
        "\n"
        "THE TRAVEL IS NOT A MEASURED FIGURE, AND THE MODEL IS INCONSISTENT ABOUT IT\n"
        "No reference found gives a bolt carrier travel for the AK-47, so 72 mm is not measured: it is the largest travel this receiver admits, the carrier stopping 8 mm short of the channel's rear wall. CheckAction measures that at build time over 360 steps and reports it, together with the margin at the charging handle's slot, the piston head against BOTH ends of the gas tube, the trigger shoe against the guard's rear web, the recoil the momentum comes to and whether it is back in battery when the cycle repeats, and the height at which the case crosses the receiver wall against the ejection port's opening.\n"
        "The trigger and the case are checked in the RIFLE's frame, not the world's, because everything they are checked against is bolted to a rifle that now recoils. Measuring them in the world compares a recoiled part against a limit that moved with it, which is what the first version of that check did: it reported the trigger 21 mm through a guard it never touches.\n"
        "That last set of numbers exposes a real inconsistency in the geometry, and it is recorded rather than hidden: the charging-handle slot measured off ref_04 runs z 380 to 540, which with a 10 mm handle implies about 147 mm of travel, but the receiver's internal length between the front face at z 372 and the stock tang at z 606 leaves only 72 mm once a 140 mm carrier is inside it. Either the slot is longer than it should be, or the carrier is too long for it, or the receiver's internal length is short. The travel is set to the smaller of the two limits, so about 75 mm of the slot is never used, and a reviewer should read that unused slot as an open question rather than as a modelling slip.\n"
        "Beating the 56.00 mm cartridge overall length is the one thing the travel has to do for a fresh round to have anywhere to go, and at 72 mm it does, by 16 mm.\n"
        "\n"
        "The internal action beyond this is not modelled and nothing here claims it: there is no hammer, no disconnector, no recoil spring, no chamber and no feed. The receiver is a billet with a channel cut in it, the magazine is a closed shell, and the round the trigger releases is not represented. What the model claims is the externally visible motion, which is the part a turntable of the rifle at rest cannot show.\n"
        "\n"
        "Some hidden geometry sits inside other solids: the piston rod runs at the gas-tube axis and so passes through the rear sight base, the receiver front block and the upper handguard, and the bolt in battery sits inside the barrel where the chamber would be. That is the convention the static geometry already works to, where the gas tube sits wholly inside the solid swept handguard wood because the wood is a solid section and not a shell. None of it is visible from outside, and the alternative is boring four measured solids to hide parts no assembled view ever shows.\n"
        "\n"
        "--anim holds the camera at yaw 20, a right-flank elevation turned twenty degrees toward the butt: the ejection port, the charging-handle slot and the trigger are all on that side, and the obliquity is what lets the case's outward travel read at all. Rendered without --anim the frames are a turntable of the first round's ignition: in battery, flash at full, no recoil yet.\n"
        "\n"
        "References attached to this model: ref_01 the bolt carrier, bolt and gas piston; ref_02 a trigger group; ref_03 a field-stripped AKM; ref_04 the measured Type 2 side elevation every dimension in this file comes from; ref_05 a Type 2 right-side photograph; ref_06 a receiver fabrication drawing; ref_07 two AKs in a case, the beech and laminate stocks side by side.\n"
        "\n"
        "--- the static geometry underneath the pose ---\n"
        "\n"
        "AK-47 with a Type 2 milled receiver and fixed wooden furniture, the configuration in references/ak47/ref_04.png.\n"
        "\n"
        "One world unit is 100 mm, so a grid square is 10 cm. Source constants are in millimetres: z runs aft from the muzzle face, y up from the magazine floorplate, x across; Vert() scales and recentres so the model sits centred on the grid.\n"
        "880 overall, bore axis at y 204.4, cleaning rod axis 191.0, gas tube axis 228.8, overall height 256.\n"
        "\n"
        "The side elevation is measured, not estimated. ref_04 was thresholded to a silhouette and sampled column by column at 0.3915 mm/px, and every station below comes from that table:\n"
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
        "Surfaces carry procedural diffuse maps, 512x512 images generated at init and shared by every part: wood, milled steel, phosphate grey, blued and brass. Milled steel is generated twice and carried as two materials, MAT_MILLED for the blued receiver and MAT_STEEL for the bare bolt and piston, which differ in colour and not in surface.\n"
        "EVERY METAL FINISH IS ONE COLOUR, which ref_04 shows and which the maps make awkward to write down. Each map is a darkener with a different mean, so equal material colours do not give equal surfaces: the target is one average surface value, blued's 56 through a mean of 0.366 giving 20.9, and each other metal carries 20.9 divided by its own mean, milled 37.5 at 0.557 and phosphate grey 29.2 at 0.715. Equal colours through those means came out two to one apart, which is how the phosphate parts, the sights and trigger group and fittings, ended up the brightest metal on a rifle whose reference has them the same dark as the barrel. BuildTextures measures the means at init rather than trusting a number in a comment, and warns if the finished metals drift more than 15 percent apart; they currently run 20.9 to 21.0.\n"
        "The receiver is the part this was noticed on. It was bare bright steel at four times the surface value of the barrel bolted to it, reading as a different metal rather than the same one worn brighter. It is now blued to the common value but keeps the milled map rather than moving to the blued one: the blued map's wear term ramps a tenth of its area to warm bare steel in blobs sized for a magazine rib, and across a 248 by 34 receiver flank those come out as six thumb-sized tan smears. renders/ak47/v12 is that intermediate state, kept because it is the argument for the split.\n"
        "The only metal deliberately off the common value is bare steel inside the action: the bolt, the gas piston and the wear face on the carrier's flank, which is the one bright thing the ejection port has to show moving.\n"
        "lighting.fs multiplies the map by the material colour and an eight-bit map cannot exceed 1.0, so each map is a pure darkener and MAT_COLOR is now the clean bright state of the material rather than its average; measured mean map values are 0.655 wood, 0.557 steel and milled, 0.715 grey, 0.366 blued and 0.755 brass. The 0.60 this line used to quote for steel was 8 percent high, which is why the means are now measured rather than remembered.\n"
        "wood: 22 warped latewood lines per repeat, each gated by a noise field that varies along the grain so no line survives the whole length of a part, over broad tonal blotches, longitudinal figure and occasional near-black mineral streaks; broad tone carries most of the variation and the banding only a sixth of it, and darker wood is shifted redder, since latewood is denser rather than sooty. milled steel: draw marks along the length, a polish sweep on the high spots, cooler patina in the hollows and dark pitting. grey: fine granular phosphate tooth. blued: the widest map of the four, a dark finish worn through to warm bare steel over roughly a tenth of its area.\n"
        "Texture coordinates are in millimetres and divided by a per-material repeat length, so one repeat is a fixed physical size on every part: 320 by 145 for wood, 165 by 95 steel and milled, 62 square grey, 128 by 96 blued, 26 by 18 brass. u runs along the part and v across it, so wood grain follows the barrel axis on the handguards and the comb line on the stock. Flat faces project planar by dominant face normal, tubes map axially and circumferentially with the axial coordinate taken from the world origin so the three barrel segments continue one pattern, and swept sections map by path length and section perimeter. Noise is periodic value noise so every map tiles.\n"
        "\n"
        "Widths are the weakest numbers here: no plan or front elevation was found, so only the receiver is measured (34.4 across, from the assembled rear receiver view in references/ak47/ref_06.jpg). Handguard 42, magazine 28, grip 31, and a buttstock tapering from 27 at the wrist to 40 at the butt, are all inferred from the receiver and should be treated as the least trustworthy dimensions in the model.\n"
        "Known simplifications, all repeatedly flagged in review and all consequences of building from hexahedra and swept sections: the magazine ribs are swept strips standing 0.3 proud that fade out at both ends rather than true pressings; the gas block, front sight base and handguard band are chamfered prisms rather than forgings flowing into rounded barrel collars; and the receiver flanks are flat plates with milled pockets rather than a body that changes section along its length.\n"
        "The texturing is diffuse only: there is no normal or specular map, so grain, pitting and machining marks change colour but never catch the light, and every surface stays as smooth as its mesh. Sweep and tube end caps project side grain rather than end grain, which is wrong on the muzzle face and on the butt under the buttplate but nowhere else visible.\n"
        "The magazine ribs were queried in four review rounds running as looking additive; references/ak47/ref_04.png shows the AK-47 steel magazine carrying raised longitudinal pressings, not recessed channels, so they stay raised and were only reduced in projection.",
    .init = Init,
    .unload = Unload,
    .update = Update,
    .duration = CYCLE,
    .animYaw = 20.0f,
    // The window opens at 0.6, and nothing about the rifle is 0.6. A three-round burst at 600 rpm really is over in 0.3 s, and three flashes 100 ms apart are at the limit of what an eye can count, so the window opened on a motion nobody could follow and the honesty of that was worth less than being able to see it. duration stays the real 0.5 s and rendered frames are untouched; press ] four times for real time.
    .previewSpeed = 0.6f,
    .parts = PARTS,
    .partCount = (int)(sizeof(PARTS) / sizeof(PARTS[0])),
    .target = { 0.0f, 1.30f, 0.0f },
    .orbitRadius = 10.5f,
    .orbitHeight = 3.4f,
};
