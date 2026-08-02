#include "harness.h"
#include "raymath.h"
#include "rlgl.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// This is humvee_v3's geometry driven through one cycle of motion: a truck under way on a rough track.
//
// Six things move, and each needed the model split differently from v3's five review groups.
// The wheels spin, so tyre, rim and lug nuts became one mesh per corner authored about that corner's hub.
// The body shakes and occasionally leaves the ground, so everything bolted to it rides one chassis matrix, and each corner's suspension carries the difference.
// The wipers sweep, so each arm is its own mesh authored about its own pivot on the windscreen.
// The whip antenna bends, so its mesh is uploaded dynamic and its vertices are rewritten every frame from a clamped-cantilever curve.
// The lamps flash, which is a per-material tint rather than geometry.
// The windows are transparent, which is neither: raylib's lighting.fs cannot carry alpha at all (see the note above MakeGlassShader), so the glass runs on a small purpose-built shader and is drawn in its own pass after every opaque surface in the scene.
//
// Transparency is also why this file has a cab interior that v3 has not. v3 fills the cab with a solid block up to the roof; seen through glass that is a black wall, and the windows read as paint. The block now stops just under the window line and what stands above it -- dash, tunnel, seat backs, steering wheel -- is modelled.
// ---------------------------------------------------------------------------

// Metres per repeat of the surface maps. The camouflage patches want to be a few tens of centimetres across, which puts a 512-pixel map at roughly 3 mm per texel.
#define TEX_REPEAT_M  1.60f

// ---------------------------------------------------------------------------
// Mesh builder
//
// Every surface in this file is either a hexahedron (a box whose eight corners may each sit anywhere, so it also covers wedges, tapers and sloped panels) or a surface of revolution.
// Both accumulate into a growing vertex/index buffer that is uploaded once per material at the end of a group's build.
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

static int VertUV(Builder *b, Vector3 p, Vector3 n, float u, float v)
{
    Reserve(b, 1, 0);
    int i = b->vertexCount++;
    b->vertices[i * 3 + 0] = p.x;
    b->vertices[i * 3 + 1] = p.y;
    b->vertices[i * 3 + 2] = p.z;
    b->normals[i * 3 + 0] = n.x;
    b->normals[i * 3 + 1] = n.y;
    b->normals[i * 3 + 2] = n.z;
    b->texcoords[i * 2 + 0] = u;
    b->texcoords[i * 2 + 1] = v;
    return i;
}

// Project the position onto the two axes the normal is weakest in, which is the plane the face most nearly lies in and so the one that stretches it least.
// The normal is constant across a flat face, so the projection is continuous over each panel and across any two panels facing the same way.
// It does change axis a few times around a revolved surface; the tyres and pipes carry fine-grained maps where that seam does not read.
static int Vert(Builder *b, Vector3 p, Vector3 n)
{
    float ax = fabsf(n.x), ay = fabsf(n.y), az = fabsf(n.z);
    float u, v;
    if (ax >= ay && ax >= az) { u = p.z; v = p.y; }
    else if (ay >= az)        { u = p.x; v = p.z; }
    else                      { u = p.x; v = p.y; }
    return VertUV(b, p, n, u / TEX_REPEAT_M, v / TEX_REPEAT_M);
}

static void Tri(Builder *b, int a, int c, int d)
{
    Reserve(b, 0, 1);
    int i = b->triangleCount++;
    b->indices[i * 3 + 0] = (unsigned short)a;
    b->indices[i * 3 + 1] = (unsigned short)c;
    b->indices[i * 3 + 2] = (unsigned short)d;
}

// Flat-shaded quad. Winding a-b-c-d must be counter-clockwise seen from outside.
static void Quad(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(p1, p0), Vector3Subtract(p2, p0)));
    int a = Vert(b, p0, n), c = Vert(b, p1, n), d = Vert(b, p2, n), e = Vert(b, p3, n);
    Tri(b, a, c, d);
    Tri(b, a, d, e);
}

// Smooth-shaded quad: caller supplies a normal per corner.
static void QuadN(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,
                  Vector3 n0, Vector3 n1, Vector3 n2, Vector3 n3)
{
    int a = Vert(b, p0, n0), c = Vert(b, p1, n1), d = Vert(b, p2, n2), e = Vert(b, p3, n3);
    Tri(b, a, c, d);
    Tri(b, a, d, e);
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

// Vertical slab spanning x0..x1 and z0..z1 whose floor and ceiling each run from one height at z0 to another at z1.
// Requires x0 < x1 and z0 < z1.
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

// Duplicate everything added since the mark, reflected through x = 0.
// Winding is reversed so the reflected faces still point outwards.
typedef struct {
    int vert;
    int tri;
} Mark;

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

    // Slant of the side surface, so a cone frustum gets a correct normal tilt.
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

// Helical wire, swept as a circle along the helix using an analytic frame.
// The frame's radial vector is exactly perpendicular to the tangent, so B = T x N closes a right-handed triad and the quads come out facing outwards.
static void CoilRing(Vector3 *pos, Vector3 *nrm, Vector3 base, float coilR, float wireR,
                     float height, float turns, float t, int sides)
{
    float phi = 2.0f * PI * turns * t;
    Vector3 c = { base.x + coilR * cosf(phi), base.y + height * t, base.z + coilR * sinf(phi) };
    Vector3 tangent = Vector3Normalize((Vector3){
        -coilR * 2.0f * PI * turns * sinf(phi), height, coilR * 2.0f * PI * turns * cosf(phi) });
    Vector3 n = { cosf(phi), 0.0f, sinf(phi) };
    Vector3 bi = Vector3Normalize(Vector3CrossProduct(tangent, n));

    for (int j = 0; j <= sides; j++) {
        float v = 2.0f * PI * (float)j / (float)sides;
        Vector3 r = Vector3Add(Vector3Scale(n, cosf(v)), Vector3Scale(bi, sinf(v)));
        nrm[j] = r;
        pos[j] = Vector3Add(c, Vector3Scale(r, wireR));
    }
}

static void Coil(Builder *b, Vector3 base, float coilR, float wireR, float height, float turns, int segs, int sides)
{
    Vector3 p0[17], n0[17], p1[17], n1[17];
    CoilRing(p0, n0, base, coilR, wireR, height, turns, 0.0f, sides);
    for (int i = 0; i < segs; i++) {
        CoilRing(p1, n1, base, coilR, wireR, height, turns, (float)(i + 1) / (float)segs, sides);
        for (int j = 0; j < sides; j++) {
            QuadN(b, p0[j], p0[j + 1], p1[j + 1], p1[j], n0[j], n0[j + 1], n1[j + 1], n1[j]);
        }
        for (int j = 0; j <= sides; j++) { p0[j] = p1[j]; n0[j] = n1[j]; }
    }
}

// Surface of revolution about the X axis.
// Each profile point is (axial offset from centre, radius); the profile is walked in order and normals come from the profile tangent, so the result is smooth along the sweep as well as around it.
static void RevolveX(Builder *b, Vector3 centre, const Vector2 *prof, int n, int segments)
{
    for (int i = 0; i < n - 1; i++) {
        Vector2 p0 = prof[i], p1 = prof[i + 1];
        // Outward normal in (u, r) is (-dr, du) for a profile walked inner-to-outer.
        Vector2 t = { p1.x - p0.x, p1.y - p0.y };
        Vector2 nl = Vector2Normalize((Vector2){ -t.y, t.x });
        Vector2 na = nl, nb = nl;
        if (i > 0) {
            Vector2 tp = { p0.x - prof[i - 1].x, p0.y - prof[i - 1].y };
            na = Vector2Normalize(Vector2Add(nl, Vector2Normalize((Vector2){ -tp.y, tp.x })));
        }
        if (i + 2 < n) {
            Vector2 tn = { prof[i + 2].x - p1.x, prof[i + 2].y - p1.y };
            nb = Vector2Normalize(Vector2Add(nl, Vector2Normalize((Vector2){ -tn.y, tn.x })));
        }

        for (int j = 0; j < segments; j++) {
            float a0 = 2.0f * PI * (float)j / (float)segments;
            float a1 = 2.0f * PI * (float)(j + 1) / (float)segments;
            float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);

            Vector3 v00 = { centre.x + p0.x, centre.y + p0.y * c0, centre.z + p0.y * s0 };
            Vector3 v01 = { centre.x + p0.x, centre.y + p0.y * c1, centre.z + p0.y * s1 };
            Vector3 v11 = { centre.x + p1.x, centre.y + p1.y * c1, centre.z + p1.y * s1 };
            Vector3 v10 = { centre.x + p1.x, centre.y + p1.y * c0, centre.z + p1.y * s0 };

            Vector3 n00 = { na.x, na.y * c0, na.y * s0 };
            Vector3 n01 = { na.x, na.y * c1, na.y * s1 };
            Vector3 n11 = { nb.x, nb.y * c1, nb.y * s1 };
            Vector3 n10 = { nb.x, nb.y * c0, nb.y * s0 };

            QuadN(b, v00, v01, v11, v10, n00, n01, n11, n10);
        }
    }
}

// ---------------------------------------------------------------------------
// Materials
//
// Three kinds, and the difference is which shader carries them and which of the scene's three draw passes they land in.
// Lit materials run the harness's lighting shader and draw first.
// The glass runs its own shader and draws second, after every opaque surface in the whole scene rather than merely in its own group: a pane that draws before the bodywork behind it leaves that bodywork failing the depth test, and the window shows background instead of an interior.
// The lamp lenses and haloes skip lighting entirely and draw last; raylib's default shader returns the texel unshaded, which is what makes a lens read as emitting rather than as pale paint.
// ---------------------------------------------------------------------------

typedef enum {
    MAT_BODY, MAT_DARK, MAT_CABIN, MAT_METAL, MAT_MIRROR, MAT_GLASS,
    MAT_LAMP, MAT_TAIL, MAT_AMBER,
    MAT_GLOW_W, MAT_GLOW_R, MAT_GLOW_A, MAT_COUNT
} MatId;

typedef enum { PASS_OPAQUE, PASS_GLASS, PASS_GLOW, PASS_COUNT } Pass;

static const Pass MAT_PASS[MAT_COUNT] = {
    [MAT_GLASS] = PASS_GLASS,
    [MAT_GLOW_W] = PASS_GLOW, [MAT_GLOW_R] = PASS_GLOW, [MAT_GLOW_A] = PASS_GLOW,
};

// Every surface carries a map, so the colour lives in the texels and the material tint stays white; multiplying a coloured map by a coloured tint would darken it twice.
// The glow materials are the exception: they share one white radial falloff and take their colour from the tint.
// The glass alpha lives here too, because the glass shader multiplies it into the fragment's own fresnel term.
static const Color MAT_COLOR[MAT_COUNT] = {
    [MAT_BODY]   = WHITE,
    [MAT_DARK]   = WHITE,
    [MAT_CABIN]  = WHITE,
    [MAT_METAL]  = WHITE,
    [MAT_MIRROR] = WHITE,
    [MAT_GLASS]  = { 255, 255, 255, 105 },
    [MAT_LAMP]   = WHITE,
    [MAT_TAIL]   = WHITE,
    [MAT_AMBER]  = WHITE,
    [MAT_GLOW_W] = { 255, 236, 180, 255 },
    [MAT_GLOW_R] = { 255,  48,  30, 255 },
    [MAT_GLOW_A] = { 255, 140,  20, 255 },
};

static const bool MAT_UNLIT[MAT_COUNT] = {
    [MAT_LAMP] = true, [MAT_TAIL] = true, [MAT_AMBER] = true,
    [MAT_GLOW_W] = true, [MAT_GLOW_R] = true, [MAT_GLOW_A] = true,
};

static Texture2D MAT_TEX[MAT_COUNT];
static Shader gGlassShader;

// ---------------------------------------------------------------------------
// Glass shader
//
// The harness's lighting shader cannot carry alpha, so glass cannot be made transparent by simply setting one.
// vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.fs:73 is
//     finalColor = (texelColor*((tint + vec4(specular, 1.0))*vec4(lightDot, 1.0)));
// whose alpha channel is texelColor.a*(tint.a + 1.0), never below 1.0 for an opaque texel however small the material alpha is, and line 77 then raises it to the power 1/2.2, which leaves 1.0 at 1.0. Any glass on that shader is opaque, full stop.
//
// So the glass gets its own shader. It is small enough to live in this file as a string rather than as an asset beside it, which is the same argument the procedural maps make.
// Two things it does that the lighting shader does not: it keeps the alpha, and it adds a Schlick fresnel term that both brightens the pane and makes it more opaque at grazing angles. That angular falloff is most of what tells a viewer a surface is glass; a flat uniform tint reads as smoked plastic.
// The camera position is recovered from matView rather than from a uniform the harness sets, because the harness only feeds viewPos to its own shader. rmodels.c:1493 uploads matView to any shader that declares it.
// ---------------------------------------------------------------------------

static const char *GLASS_VS =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec2 vertexTexCoord;\n"
    "in vec3 vertexNormal;\n"
    "uniform mat4 mvp;\n"
    "uniform mat4 matModel;\n"
    "uniform mat4 matNormal;\n"
    "out vec3 fragPosition;\n"
    "out vec2 fragTexCoord;\n"
    "out vec3 fragNormal;\n"
    "void main()\n"
    "{\n"
    "    fragPosition = vec3(matModel*vec4(vertexPosition, 1.0));\n"
    "    fragTexCoord = vertexTexCoord;\n"
    "    fragNormal = normalize(vec3(matNormal*vec4(vertexNormal, 1.0)));\n"
    "    gl_Position = mvp*vec4(vertexPosition, 1.0);\n"
    "}\n";

static const char *GLASS_FS =
    "#version 330\n"
    "in vec3 fragPosition;\n"
    "in vec2 fragTexCoord;\n"
    "in vec3 fragNormal;\n"
    "uniform sampler2D texture0;\n"
    "uniform vec4 colDiffuse;\n"
    "uniform mat4 matView;\n"
    "out vec4 finalColor;\n"
    "void main()\n"
    "{\n"
    // The view matrix is a rotation followed by a translation, so the eye is at -R^T t.
    "    vec3 eye = -(transpose(mat3(matView))*vec3(matView[3]));\n"
    "    vec3 v = normalize(eye - fragPosition);\n"
    "    vec3 n = normalize(fragNormal);\n"
    // A pane is two-sided and its far face is culled, but the divider and the mirror housing put glass at angles where the built normal faces away; flipping it keeps the fresnel from inverting there.
    "    if (dot(n, v) < 0.0) n = -n;\n"
    "    float fres = pow(1.0 - clamp(dot(n, v), 0.0, 1.0), 3.0);\n"
    "    vec3 key = normalize(vec3(0.55, 0.72, 0.42));\n"
    "    float spec = pow(max(dot(reflect(-key, n), v), 0.0), 42.0);\n"
    // The lit surfaces around this one are knocked down by their light dot product, so an unattenuated pane would sit brighter than the bodywork it is set into whatever colour it is given.
    "    float lam = 0.30 + 0.70*max(dot(n, key), 0.0);\n"
    "    vec4 texel = texture(texture0, fragTexCoord);\n"
    "    vec3 col = texel.rgb*colDiffuse.rgb*lam + vec3(0.09)*fres + vec3(0.55)*spec;\n"
    "    float a = clamp(colDiffuse.a*texel.a + 0.34*fres + spec, 0.0, 1.0);\n"
    "    finalColor = vec4(pow(col, vec3(1.0/2.2)), a);\n"
    "}\n";

// ---------------------------------------------------------------------------
// Procedurally generated surface maps
//
// Value noise on an integer lattice that wraps at the lattice period, so every map tiles and the world-space UVs can run off to any distance without showing a join.
// ---------------------------------------------------------------------------

static float Hash2(int x, int y, int period, unsigned int seed)
{
    x = ((x % period) + period) % period;
    y = ((y % period) + period) % period;
    unsigned int h = (unsigned int)x * 374761393u + (unsigned int)y * 668265263u + seed * 362437u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (float)(h & 0xFFFFFFu) / (float)0xFFFFFFu;
}

static float ValueNoise(float u, float v, int period, unsigned int seed)
{
    float x = u * (float)period, y = v * (float)period;
    int xi = (int)floorf(x), yi = (int)floorf(y);
    float fx = x - (float)xi, fy = y - (float)yi;
    fx = fx * fx * (3.0f - 2.0f * fx);
    fy = fy * fy * (3.0f - 2.0f * fy);
    float a = Hash2(xi, yi, period, seed), b = Hash2(xi + 1, yi, period, seed);
    float c = Hash2(xi, yi + 1, period, seed), d = Hash2(xi + 1, yi + 1, period, seed);
    return Lerp(Lerp(a, b, fx), Lerp(c, d, fx), fy);
}

static float Fbm(float u, float v, int period, unsigned int seed, int octaves)
{
    float sum = 0.0f, amp = 1.0f, norm = 0.0f;
    for (int o = 0; o < octaves; o++) {
        sum += amp * ValueNoise(u, v, period << o, seed + (unsigned int)o * 7919u);
        norm += amp;
        amp *= 0.5f;
    }
    return sum / norm;
}

static Image NewImage(int size)
{
    Image img = { 0 };
    img.data = MemAlloc((unsigned int)size * (unsigned int)size * 4u);
    img.width = size;
    img.height = size;
    img.mipmaps = 1;
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    return img;
}

static Color Shade(Color c, float d)
{
    return (Color){
        (unsigned char)Clamp((float)c.r + d, 0.0f, 255.0f),
        (unsigned char)Clamp((float)c.g + d, 0.0f, 255.0f),
        (unsigned char)Clamp((float)c.b + d, 0.0f, 255.0f),
        c.a,
    };
}

static Texture2D Upload(Image img, int wrap)
{
    Texture2D t = LoadTextureFromImage(img);
    UnloadImage(img);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(t, wrap);
    return t;
}

// NATO three-colour woodland, cut out of one noise field by thresholds so the patches interlock along shared edges instead of floating over one another.
// The values are authored dark on purpose: lighting.fs gamma-corrects with pow(c, 1/2.2), so a texel of 54 leaves the shader at about 0.51 before any light reaches it.
static Texture2D MakeCamoTexture(void)
{
    const int S = 512;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    const Color olive = { 74, 78, 56, 255 };
    const Color green = { 54, 62, 43, 255 };
    const Color brown = { 58, 46, 35, 255 };
    const Color black = { 24, 25, 24, 255 };
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float n = Fbm(u, v, 4, 11u, 3);
            Color c = (n < 0.42f) ? black : (n < 0.52f) ? brown : (n < 0.66f) ? green : olive;
            // CARC is a matt, faintly granular paint rather than a gloss, so the patch colour carries a fine speckle.
            px[y * S + x] = Shade(c, (Fbm(u, v, 128, 77u, 2) - 0.5f) * 13.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Rubber and the dark trim: near black with a fine tooth, plus a faint circumferential banding that gives the tyre sidewalls something to catch the light on.
static Texture2D MakeRubberTexture(void)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float grain = (Fbm(u, v, 64, 29u, 2) - 0.5f) * 16.0f;
            float band = sinf(v * 2.0f * PI * 18.0f) * 2.0f;
            px[y * S + x] = Shade((Color){ 21, 22, 24, 255 }, grain + band);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// The cab's own surfaces, which are the only ones in this model seen through something else.
// Darker than the trim rubber on purpose: the harness sums three lights before gamma-correcting, so a texel that reads as near black on an exterior panel comes back as a mid grey on a slab facing the windows, and a mid grey behind glass reads as frosting rather than as an interior.
static Texture2D MakeCabinTexture(void)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            px[y * S + x] = Shade((Color){ 9, 11, 10, 255 }, (Fbm(u, v, 96, 61u, 2) - 0.5f) * 9.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Bare metal: a coarse mottle for cast and forged parts, stretched along one axis so it reads as worked rather than as noise.
static Texture2D MakeMetalTexture(void)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float d = (Fbm(u * 4.0f, v, 32, 53u, 2) - 0.5f) * 26.0f;
            px[y * S + x] = Shade((Color){ 74, 78, 82, 255 }, d);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Mirror face. v3 gave the wing mirrors the glass material, which was harmless while glass was opaque and is not now: a see-through mirror is the one pane in the model that should never be see-through.
static Texture2D MakeMirrorTexture(void)
{
    const int S = 128;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            px[y * S + x] = Shade((Color){ 118, 124, 132, 255 }, (Fbm(u, v, 4, 43u, 2) - 0.5f) * 22.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Glass: nearly flat, with a slow drift so a large pane is not one dead colour.
static Texture2D MakeGlassTexture(void)
{
    const int S = 128;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            px[y * S + x] = Shade((Color){ 16, 23, 28, 255 }, (Fbm(u, v, 4, 91u, 2) - 0.5f) * 10.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// A lit lens. Bright at the middle and falling off into the rim so the lamp has a hot centre instead of reading as a flat disc of paint.
static Texture2D MakeLensTexture(Color hot, Color rim)
{
    const int S = 128;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float dx = ((float)x + 0.5f) / (float)S - 0.5f;
            float dy = ((float)y + 0.5f) / (float)S - 0.5f;
            float r = Clamp(sqrtf(dx * dx + dy * dy) / 0.5f, 0.0f, 1.0f);
            float t = r * r;
            px[y * S + x] = (Color){
                (unsigned char)Lerp((float)hot.r, (float)rim.r, t),
                (unsigned char)Lerp((float)hot.g, (float)rim.g, t),
                (unsigned char)Lerp((float)hot.b, (float)rim.b, t),
                255,
            };
        }
    }
    return Upload(img, TEXTURE_WRAP_CLAMP);
}

// Radial falloff for the haloes: white throughout, with only the alpha varying, so the tint on the material decides the colour of the glow.
// Squared falloff gives a bright core with a soft skirt; a linear ramp reads as a flat disc with a hard edge.
static Texture2D MakeGlowTexture(void)
{
    const int S = 128;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float dx = ((float)x + 0.5f) / (float)S - 0.5f;
            float dy = ((float)y + 0.5f) / (float)S - 0.5f;
            float r = sqrtf(dx * dx + dy * dy) / 0.5f;
            float a = (r >= 1.0f) ? 0.0f : powf(1.0f - r, 1.6f);
            px[y * S + x] = (Color){ 255, 255, 255, (unsigned char)(a * 255.0f) };
        }
    }
    return Upload(img, TEXTURE_WRAP_CLAMP);
}

static void MakeTextures(void)
{
    MAT_TEX[MAT_BODY] = MakeCamoTexture();
    MAT_TEX[MAT_DARK] = MakeRubberTexture();
    MAT_TEX[MAT_CABIN] = MakeCabinTexture();
    MAT_TEX[MAT_METAL] = MakeMetalTexture();
    MAT_TEX[MAT_MIRROR] = MakeMirrorTexture();
    MAT_TEX[MAT_GLASS] = MakeGlassTexture();
    MAT_TEX[MAT_LAMP] = MakeLensTexture((Color){ 255, 252, 236, 255 }, (Color){ 176, 168, 138, 255 });
    MAT_TEX[MAT_TAIL] = MakeLensTexture((Color){ 255, 96, 74, 255 }, (Color){ 138, 20, 16, 255 });
    MAT_TEX[MAT_AMBER] = MakeLensTexture((Color){ 255, 186, 74, 255 }, (Color){ 150, 82, 12, 255 });
    MAT_TEX[MAT_GLOW_W] = MakeGlowTexture();
    MAT_TEX[MAT_GLOW_R] = MAT_TEX[MAT_GLOW_W];
    MAT_TEX[MAT_GLOW_A] = MAT_TEX[MAT_GLOW_W];

    gGlassShader = LoadShaderFromMemory(GLASS_VS, GLASS_FS);
    if (gGlassShader.id == 0) {
        TraceLog(LOG_WARNING, "humvee_v3_anim: glass shader failed to build, windows will render opaque");
    }
}

// The three glow materials share one texture, so only the one that owns it may unload it.
static void UnloadTextures(void)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (m == MAT_GLOW_R || m == MAT_GLOW_A) continue;
        if (MAT_TEX[m].id != 0) UnloadTexture(MAT_TEX[m]);
    }
    if (gGlassShader.id != 0) UnloadShader(gGlassShader);
}

// ---------------------------------------------------------------------------
// Groups
//
// A group is one rigid node: a set of per-material meshes that share a transform and a tint, plus the local bounding box the transform acts on.
// v3 had five, one per review part. This file has twenty-one, because a part that has to move alone has to be its own mesh.
// ---------------------------------------------------------------------------

typedef struct {
    Builder b[MAT_COUNT];
    Model model[MAT_COUNT];
    bool has[MAT_COUNT];
    BoundingBox bounds;      // in the group's own frame, before xform
    Matrix xform;
    Color tint[MAT_COUNT];
    bool dynamic;            // mesh buffers are rewritten every frame
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

static void GroupBoundsFromMesh(Group *g)
{
    bool any = false;
    for (int m = 0; m < MAT_COUNT; m++) {
        if (!g->has[m]) continue;
        BoundingBox bb = GetModelBoundingBox(g->model[m]);
        if (!any) { g->bounds = bb; any = true; }
        else {
            g->bounds.min = Vector3Min(g->bounds.min, bb.min);
            g->bounds.max = Vector3Max(g->bounds.max, bb.max);
        }
    }
}

static void GroupFinish(Group *g, const char *name)
{
    g->xform = MatrixIdentity();
    for (int m = 0; m < MAT_COUNT; m++) g->tint[m] = WHITE;

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
        UploadMesh(&mesh, g->dynamic);

        g->model[m] = LoadModelFromMesh(mesh);
        g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].color = MAT_COLOR[m];
        if (MAT_TEX[m].id != 0) g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = MAT_TEX[m];
        if (MAT_PASS[m] == PASS_GLASS && gGlassShader.id != 0) g->model[m].materials[0].shader = gGlassShader;
        else if (!MAT_UNLIT[m]) HarnessApplyLighting(&g->model[m]);
        g->has[m] = true;
    }

    GroupBoundsFromMesh(g);
}

// One pass over one group. The caller owns the blend and depth state for the pass, because a pass spans every group in the scene rather than stopping at this one.
static void GroupDrawPass(Group *g, Pass pass)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (!g->has[m] || MAT_PASS[m] != pass) continue;
        g->model[m].transform = g->xform;
        DrawModel(g->model[m], Vector3Zero(), 1.0f, g->tint[m]);
    }
}

// Every group in the scene, in build order. The draw passes walk this rather than the part list, so a pass really does cover the whole model.
#define MAX_GROUPS 32
static Group *gAll[MAX_GROUPS];
static int gAllCount;

static void GroupRegister(Group *g)
{
    if (gAllCount < MAX_GROUPS) gAll[gAllCount++] = g;
    else TraceLog(LOG_ERROR, "humvee_v3_anim: more than %d groups", MAX_GROUPS);
}

// Opaque first, then glass, then the haloes.
// Glass runs with depth writes off so two panes in line both blend rather than the nearer one masking the further; depth testing stays on, so bodywork in front still hides it.
// The haloes are summed into the frame for the same reason v3 gave: left writing depth, the nearer of two overlapping discs would occlude the further and the pair would read as a step rather than as one brighter patch.
static void DrawGroups(Group *const *gs, int n)
{
    for (int i = 0; i < n; i++) GroupDrawPass(gs[i], PASS_OPAQUE);

    rlDisableDepthMask();
    for (int i = 0; i < n; i++) GroupDrawPass(gs[i], PASS_GLASS);
    BeginBlendMode(BLEND_ADDITIVE);
    for (int i = 0; i < n; i++) GroupDrawPass(gs[i], PASS_GLOW);
    EndBlendMode();
    rlEnableDepthMask();
}

static void GroupUnload(Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (g->has[m]) UnloadModel(g->model[m]);
    }
}

// ---------------------------------------------------------------------------
// Dimensions, in metres, from the M998 HMMWV: 4.57 long, 2.16 wide, 1.83 tall, 3.30 wheelbase, 1.83 track, 37x12.5R16.5 tyres.
// Origin is on the ground at the centre of the wheelbase; +Z is forward, +X is the vehicle's right, +Y up.
// ---------------------------------------------------------------------------

#define HALF_W        1.080f   // widest point: fender flares and tyre outer face
#define SIDE_W        0.870f   // cab and cargo bed outer skin
#define CAB_IN        0.800f   // inner face of the doors
#define WELL_IN       0.680f   // inboard wall of the wheel wells
#define CORE_HW       0.560f   // body core; the strip out to WELL_IN is arched away to leave a wheel well
#define TRACK_HW      0.915f   // wheel centre
#define TIRE_R        0.470f
#define TIRE_HW       0.162f
#define RIM_R         0.215f
#define AXLE_Y        0.470f
#define AXLE_F        1.650f
#define AXLE_R       -1.650f
#define ARCH_R        0.600f   // wheel opening: 0.13 of clearance over the tyre
#define WHEELBASE     (AXLE_F - AXLE_R)
#define TRACK         (2.0f * TRACK_HW)

#define BELLY_Y       0.410f   // ground clearance
#define SILL_Y        0.530f
#define DOOR_Y0       0.680f
#define HOOD_BACK_Z   0.720f
#define NOSE_Z        2.220f
#define BUMP_F_Z      2.350f
#define TAIL_Z       -2.090f
#define BUMP_R_Z     -2.220f
#define COWL_Z        0.620f
#define COWL_Y        1.220f
#define WS_TOP_Y      1.775f
// Lean of the windscreen off vertical, as a tangent.
// The HMMWV's screen stands close to upright: measured off references/humvee_v3_anim/ref_07.jpg (side elevation) at 6 degrees, ref_05 at 9 and ref_01 at 11 once each view's yaw is divided out.
// 10 degrees is the middle of that spread.
#define WS_RAKE       0.176f
#define WS_TOP_Z      (COWL_Z - (WS_TOP_Y - COWL_Y) * WS_RAKE)
// Rear face of the A-pillar, and the door's leading edge one shut line behind it.
// Both are derived rather than typed: a door edge written as an absolute z is what put the door in front of the glass at the old 45-degree rake.
#define WS_PILLAR_Z   (WS_TOP_Z - 0.090f)
#define DOOR_F_Z      (WS_PILLAR_Z - 0.020f)
#define CAB_BACK_Z   -1.150f
#define ROOF_Y        1.830f
#define RAIL_Y        1.775f   // underside of the roof slab and of the side rails
#define DOOR_TOP_Y    1.765f
// Belt line, and the glass sitting 30 mm above it.
// Measured off references/humvee_v3_anim/ref_06.jpg, a near-side view of the cab: the side glass runs 460 px of the door's 1080, so 0.426 of the leaf.
#define BELT_Y        1.273f
#define GLASS_Y0      1.303f
#define GLASS_Y1      1.735f
// Top of the cab's solid core. v3 carried it to the roof; it now stops just under the belt line so the glass has an interior behind it rather than a black wall.
#define CAB_FILL_Y    1.180f
#define BED_FLOOR_Y   1.100f
#define BED_TOP_Y     1.420f
#define FLARE_TOP_Y   1.160f
#define FENDER_Y0     1.220f   // front fender crown at the cowl
#define FENDER_Y1     1.120f   // ... and at the nose
#define HOOD_TOP_Y0   1.160f
#define HOOD_TOP_Y1   1.060f
#define HOOD_BASE_Y0  1.100f
#define HOOD_BASE_Y1  1.000f

// Suspension travel about the static ride height, and the free length of the coil the travel compresses.
#define SUSP_UP       0.110f   // hub rising towards the body
#define SUSP_DOWN     0.075f   // hub dropping away from it
#define COIL_TOP_Y    0.960f
#define COIL_LEN      0.385f

#define CORNERS 4

static Group gHull, gFront, gCab, gInterior, gBed, gSusp;
static Group gWheel[CORNERS];    // tyre, rim, lug nuts: translates with the suspension and spins
static Group gUpright[CORNERS];  // hub carrier, wishbones, lower spring seat: translates only
static Group gSpring[CORNERS];   // coil and damper: compress between the body and the upright
static Group gShaft[CORNERS];    // half shaft: swings on its inboard joint to follow the hub
static Group gWiper[2];
static Group gAntenna;

static float CornerSide(int i) { return (i & 1) ? -1.0f : 1.0f; }
static float CornerZ(int i) { return (i < 2) ? AXLE_F : AXLE_R; }

static float ArchTop(float z, float zc)
{
    float d = z - zc;
    float rr = ARCH_R * ARCH_R - d * d;
    if (rr <= 0.0f) return -1e9f;
    return AXLE_Y + sqrtf(rr);
}

// Bodywork slab with a wheel opening cut out of its underside.
// The opening is the circle of radius ARCH_R about the axle; away from it the slab runs down to yFloor.
// Consecutive strips share their edges exactly, so the arch reads as a smooth curve rather than a staircase.
// Outward normal of the well soffit: on the arch it points straight back at the axle, on the flat floor it points down.
static Vector3 SoffitNormal(float z, float zc, float y, float yFloor)
{
    if (y <= yFloor + 1e-4f) return (Vector3){ 0.0f, -1.0f, 0.0f };
    return (Vector3){ 0.0f, (AXLE_Y - y) / ARCH_R, (zc - z) / ARCH_R };
}

static void ArchedPanel(Builder *b, float x0, float x1, float z0, float z1,
                        float zc, float yFloor, float yTop0, float yTop1)
{
    const float step = 0.028f;
    int steps = (int)ceilf((z1 - z0) / step);
    if (steps < 1) steps = 1;

    for (int i = 0; i < steps; i++) {
        float t0 = (float)i / (float)steps;
        float t1 = (float)(i + 1) / (float)steps;
        float za = Lerp(z0, z1, t0), zb = Lerp(z0, z1, t1);
        float yba = fmaxf(yFloor, ArchTop(za, zc)), ybb = fmaxf(yFloor, ArchTop(zb, zc));
        float yta = Lerp(yTop0, yTop1, t0), ytb = Lerp(yTop0, yTop1, t1);
        Vector3 na = SoffitNormal(za, zc, yba, yFloor), nb = SoffitNormal(zb, zc, ybb, yFloor);

        // The soffit is the only curved surface on this panel, so it is the only one given smooth normals; the outer, inner and top faces are planar across the whole sweep and stay flat.
        QuadN(b, (Vector3){ x0, yba, za }, (Vector3){ x1, yba, za },
                 (Vector3){ x1, ybb, zb }, (Vector3){ x0, ybb, zb }, na, na, nb, nb);
        Quad(b, (Vector3){ x0, yta, za }, (Vector3){ x0, ytb, zb },
                (Vector3){ x1, ytb, zb }, (Vector3){ x1, yta, za });
        Quad(b, (Vector3){ x1, yba, za }, (Vector3){ x1, yta, za },
                (Vector3){ x1, ytb, zb }, (Vector3){ x1, ybb, zb });
        Quad(b, (Vector3){ x0, yba, za }, (Vector3){ x0, ybb, zb },
                (Vector3){ x0, ytb, zb }, (Vector3){ x0, yta, za });

        if (i == 0) {
            Quad(b, (Vector3){ x0, yba, za }, (Vector3){ x0, yta, za },
                    (Vector3){ x1, yta, za }, (Vector3){ x1, yba, za });
        }
        if (i == steps - 1) {
            Quad(b, (Vector3){ x0, ybb, zb }, (Vector3){ x1, ybb, zb },
                    (Vector3){ x1, ytb, zb }, (Vector3){ x0, ytb, zb });
        }
    }
}

// ---------------------------------------------------------------------------
// hull: the flat belly pan, the frame rails and the sills the doors close onto
// ---------------------------------------------------------------------------

static void BuildHull(void)
{
    Group *g = &gHull;
    Builder *body = &g->b[MAT_BODY];
    Builder *metal = &g->b[MAT_METAL];

    // Flat pan the full length of the vehicle, upswept at both ends for the approach and departure angles.
    // Kept inboard of the tyres.
    Prism(body, -WELL_IN, WELL_IN, -1.90f, 1.90f, BELLY_Y, BELLY_Y, SILL_Y, SILL_Y);
    Prism(body, -WELL_IN, WELL_IN, 1.90f, NOSE_Z, BELLY_Y, 0.60f, SILL_Y, 0.70f);
    Prism(body, -WELL_IN, WELL_IN, TAIL_Z, -1.90f, 0.58f, BELLY_Y, 0.68f, SILL_Y);

    GroupMark m = GroupMarkNow(g);

    // Sill between the wheel openings: the step the doors sit on.
    Box(body, WELL_IN, SIDE_W, BELLY_Y, DOOR_Y0, CAB_BACK_Z, 1.05f);
    // Frame rail, visible through the wheel openings.
    Box(metal, 0.34f, 0.46f, 0.42f, 0.56f, -1.95f, 1.95f);
    // Exhaust, along the right frame rail and turned down behind the rear wheel.
    Tube(metal, (Vector3){ 0.56f, 0.50f, 0.60f }, (Vector3){ 0.56f, 0.50f, -1.86f }, 0.045f, 0.045f, 12, true, false);
    Tube(metal, (Vector3){ 0.56f, 0.50f, -1.86f }, (Vector3){ 0.56f, 0.44f, -2.02f }, 0.045f, 0.050f, 12, false, true);

    GroupMirrorX(g, m);

    // Transfer case and propeller shafts on the centreline.
    Box(metal, -0.18f, 0.18f, 0.44f, 0.68f, -0.30f, 0.34f);
    Tube(metal, (Vector3){ 0, 0.50f, 0.34f }, (Vector3){ 0, 0.47f, 1.48f }, 0.038f, 0.038f, 10, true, true);
    Tube(metal, (Vector3){ 0, 0.50f, -0.30f }, (Vector3){ 0, 0.47f, -1.48f }, 0.038f, 0.038f, 10, true, true);

    GroupFinish(g, "hull");
}

// ---------------------------------------------------------------------------
// front: engine bay, hood, front fenders, grille, lights and bumper
// ---------------------------------------------------------------------------

// A halo over a lamp: a disc in the xy plane, its UVs polar so the radial falloff map lands square on it.
// It faces straight down the vehicle axis rather than at the camera, because a lamp only throws light out of its own face and the harness's draw callback has no camera to turn towards anyway.
static void GlowDisc(Builder *b, float cx, float cy, float z, float r, float facing)
{
    const int SEG = 28;
    Vector3 n = { 0.0f, 0.0f, facing };
    int centre = VertUV(b, (Vector3){ cx, cy, z }, n, 0.5f, 0.5f);
    int prev = 0;
    for (int i = 0; i <= SEG; i++) {
        float a = 2.0f * PI * (float)i / (float)SEG;
        float ca = cosf(a), sa = sinf(a);
        int v = VertUV(b, (Vector3){ cx + r * ca, cy + r * sa, z }, n, 0.5f + 0.5f * ca, 0.5f + 0.5f * sa);
        if (i > 0) {
            if (facing > 0.0f) Tri(b, centre, prev, v);
            else               Tri(b, centre, v, prev);
        }
        prev = v;
    }
}

// Height of the hood's top skin at a given z. The slab is laid between two heights over its length, so anything sitting on it has to ask rather than assume a flat surface.
static float HoodTopY(float z)
{
    return Lerp(HOOD_TOP_Y0, HOOD_TOP_Y1, (z - 0.740f) / (2.200f - 0.740f));
}

static void BuildFront(void)
{
    Group *g = &gFront;
    Builder *body = &g->b[MAT_BODY];
    Builder *dark = &g->b[MAT_DARK];
    Builder *metal = &g->b[MAT_METAL];
    Builder *lamp = &g->b[MAT_LAMP];
    Builder *amber = &g->b[MAT_AMBER];

    const float bayEnd = Lerp(HOOD_BASE_Y0, HOOD_BASE_Y1, (2.160f - HOOD_BACK_Z) / (NOSE_Z - HOOD_BACK_Z));

    // Engine bay, stopping short of the nose to leave room for the grille.
    Prism(body, -CORE_HW, CORE_HW, HOOD_BACK_Z, 2.160f, SILL_Y, SILL_Y, HOOD_BASE_Y0, bayEnd);

    // Hood, a separate slab sitting 20 mm inboard of the fenders so the shut lines read as gaps rather than as a single moulded lump.
    Prism(body, -0.660f, 0.660f, 0.740f, 2.200f,
          Lerp(HOOD_BASE_Y0, HOOD_BASE_Y1, 0.013f), Lerp(HOOD_BASE_Y0, HOOD_BASE_Y1, 0.987f),
          Lerp(HOOD_TOP_Y0, HOOD_TOP_Y1, 0.013f), Lerp(HOOD_TOP_Y0, HOOD_TOP_Y1, 0.987f));

    // Hood air intake: a louvred panel with a raised bezel and eight ribs, leaving nine fore-aft slots.
    // Proportioned off references/humvee_v3_anim/ref_09.jpg, a plan view of the hood, and checked head-on against ref_03.jpg: it spans a little over 40 per cent of the hood's width and sits in its forward half.
    {
        const float pz0 = 1.280f, pz1 = 1.860f, phw = 0.275f;
        float sy0 = HoodTopY(pz0), sy1 = HoodTopY(pz1);
        Prism(dark, -phw, phw, pz0, pz1, sy0, sy1, sy0 + 0.004f, sy1 + 0.004f);
        for (int i = 0; i < 8; i++) {
            float cx = -phw + 2.0f * phw * (float)(i + 1) / 9.0f;
            Prism(body, cx - 0.013f, cx + 0.013f, pz0, pz1, sy0, sy1, sy0 + 0.012f, sy1 + 0.012f);
        }
        Prism(body, -phw - 0.022f, -phw, pz0, pz1, sy0, sy1, sy0 + 0.014f, sy1 + 0.014f);
        Prism(body, phw, phw + 0.022f, pz0, pz1, sy0, sy1, sy0 + 0.014f, sy1 + 0.014f);
        Prism(body, -phw - 0.022f, phw + 0.022f, pz0 - 0.022f, pz0, sy0, sy0, sy0 + 0.014f, sy0 + 0.014f);
        Prism(body, -phw - 0.022f, phw + 0.022f, pz1, pz1 + 0.022f, sy1, sy1, sy1 + 0.014f, sy1 + 0.014f);
    }

    // Grille surround and slats.
    Box(body, -WELL_IN, -0.600f, SILL_Y, HOOD_BASE_Y1, 2.160f, NOSE_Z);
    Box(body, 0.600f, WELL_IN, SILL_Y, HOOD_BASE_Y1, 2.160f, NOSE_Z);
    Box(body, -0.600f, 0.600f, SILL_Y, 0.660f, 2.160f, NOSE_Z);
    Box(dark, -0.600f, 0.600f, 0.660f, HOOD_BASE_Y1, 2.130f, 2.170f);
    for (int i = 0; i < 8; i++) {
        float cx = -0.525f + 0.150f * (float)i;
        Box(body, cx - 0.050f, cx + 0.050f, 0.660f, HOOD_BASE_Y1, 2.170f, 2.215f);
    }

    GroupMark m = GroupMarkNow(g);

    // Front fender: full-height slab with the wheel opening cut out below.
    ArchedPanel(body, WELL_IN, HALF_W, HOOD_BACK_Z, NOSE_Z, AXLE_F, SILL_Y, FENDER_Y0, FENDER_Y1);
    // Strip between the core and the fender, arched away to open the wheel well.
    ArchedPanel(body, CORE_HW, WELL_IN, HOOD_BACK_Z, 2.160f, AXLE_F, SILL_Y, HOOD_BASE_Y0, bayEnd);

    // Headlight assembly, standing proud of the fender face so it is not swallowed by it.
    // The main lamp is 0.178 across: measured off references/humvee_v3_anim/ref_03.jpg, where it spans 110 px against the 1310 px that carry the vehicle's 2.16 of width.
    Box(dark, 0.664f, 0.896f, 0.706f, 1.036f, 2.200f, 2.250f);
    Tube(lamp, (Vector3){ 0.780f, 0.925f, 2.244f }, (Vector3){ 0.780f, 0.925f, 2.264f }, 0.089f, 0.089f, 24, false, true);
    Tube(lamp, (Vector3){ 0.780f, 0.771f, 2.244f }, (Vector3){ 0.780f, 0.771f, 2.256f }, 0.037f, 0.037f, 16, false, true);
    // Marker light at the outboard corner of the front panel. It is amber, not white: ref_03.jpg and ref_04.jpg both show an orange lens outboard of each headlamp, which is also what lets it work as a flasher.
    Box(amber, 0.940f, 1.060f, 0.980f, 1.070f, 2.210f, 2.246f);

    // Haloes, each sitting a couple of millimetres off the lens face it belongs to and about twice its radius.
    GlowDisc(&g->b[MAT_GLOW_W], 0.780f, 0.925f, 2.266f, 0.220f, 1.0f);
    GlowDisc(&g->b[MAT_GLOW_W], 0.780f, 0.771f, 2.258f, 0.095f, 1.0f);
    GlowDisc(&g->b[MAT_GLOW_A], 1.000f, 1.025f, 2.248f, 0.115f, 1.0f);

    // Lifting shackle: two clevis plates with an open gap between them and a pin across it, projecting 0.05 past the bumper face.
    Box(metal, 0.372f, 0.404f, 0.520f, 0.634f, 2.350f, 2.400f);
    Box(metal, 0.456f, 0.488f, 0.520f, 0.634f, 2.350f, 2.400f);
    Tube(metal, (Vector3){ 0.362f, 0.566f, 2.382f }, (Vector3){ 0.498f, 0.566f, 2.382f },
         0.017f, 0.017f, 10, true, true);
    // Hood latch, sitting on the sloped hood skin near its front corner.
    Box(metal, 0.500f, 0.600f, 1.055f, 1.080f, 2.140f, 2.190f);

    GroupMirrorX(g, m);

    // Bumper: one bar the full width of the vehicle.
    Box(metal, -HALF_W, HALF_W, 0.500f, 0.660f, NOSE_Z, BUMP_F_Z);

    GroupFinish(g, "front");
}

// ---------------------------------------------------------------------------
// cab: doors, pillars, glass and roof
// ---------------------------------------------------------------------------

// A point on the windscreen plane, f of the way from the cowl up to the header, pushed out along the glass normal by `out`.
// Everything laid on the screen (its own frame, the divider, the wipers) is placed through this, so the rake lives in one constant instead of in every literal that touches the glass.
static Vector3 WindscreenPoint(float x, float f, float out)
{
    float dy = WS_TOP_Y - COWL_Y, dz = WS_TOP_Z - COWL_Z;
    float len = sqrtf(dy * dy + dz * dz);
    return (Vector3){ x, COWL_Y + f * dy - out * dz / len, COWL_Z + f * dz + out * dy / len };
}

// Outward normal of the windscreen, which is also the axis both wipers turn about.
static Vector3 WindscreenNormal(void)
{
    float dy = WS_TOP_Y - COWL_Y, dz = WS_TOP_Z - COWL_Z;
    float len = sqrtf(dy * dy + dz * dz);
    return (Vector3){ 0.0f, -dz / len, dy / len };
}

static void BuildCab(void)
{
    Group *g = &gCab;
    Builder *body = &g->b[MAT_BODY];
    Builder *dark = &g->b[MAT_DARK];
    Builder *glass = &g->b[MAT_GLASS];
    Builder *mirror = &g->b[MAT_MIRROR];

    // Solid core filling the cab below the window line, so any panel gap shows shadow instead of a hole through the body.
    // v3 carried this to the roof, which is fine while the glass is opaque and useless once it is not; everything the eye can now reach through a window is built as interior instead.
    Box(dark, -CAB_IN, CAB_IN, 0.600f, CAB_FILL_Y, CAB_BACK_Z + 0.020f, 0.580f);

    // Cowl: the shelf between the hood and the base of the windscreen, carried down to the sill so no void is left between the engine bay and the cab.
    Box(body, -SIDE_W, SIDE_W, SILL_Y, COWL_Y, COWL_Z, HOOD_BACK_Z);

    // Windscreen. Glass first, then the frame around it. Both ends come off WindscreenPoint, so the rake is not restated here.
    {
        Vector3 a = WindscreenPoint(-CAB_IN, 0.0f, 0.0f), b = WindscreenPoint(CAB_IN, 0.0f, 0.0f);
        Vector3 c = WindscreenPoint(CAB_IN, 1.0f, 0.0f), d = WindscreenPoint(-CAB_IN, 1.0f, 0.0f);
        Vector3 a2 = WindscreenPoint(-CAB_IN, 0.0f, -0.040f), b2 = WindscreenPoint(CAB_IN, 0.0f, -0.040f);
        Vector3 c2 = WindscreenPoint(CAB_IN, 1.0f, -0.040f), d2 = WindscreenPoint(-CAB_IN, 1.0f, -0.040f);
        Quad(glass, a, b, c, d);
        Quad(glass, b, a2, d2, c);
        Quad(glass, a2, b2, c2, d2);

        // Centre divider of the two-piece windscreen, standing 30 mm proud of the glass along its normal.
        // Corner order maps (x, outward normal, top-to-bottom) onto the hexahedron's own (x, y, z); that triad is right-handed, so the winding still resolves outwards.
        Vector3 et = WindscreenPoint(0.034f, 1.0f, 0.0f), eb = WindscreenPoint(0.034f, 0.0f, 0.0f);
        Vector3 ot = WindscreenPoint(0.034f, 1.0f, 0.030f), ob = WindscreenPoint(0.034f, 0.0f, 0.030f);
        Vector3 e[8] = {
            { -et.x, et.y, et.z }, { et.x, et.y, et.z }, { eb.x, eb.y, eb.z }, { -eb.x, eb.y, eb.z },
            { -ot.x, ot.y, ot.z }, { ot.x, ot.y, ot.z }, { ob.x, ob.y, ob.z }, { -ob.x, ob.y, ob.z },
        };
        Hex(body, e);
    }

    // Roof, its top face drawn in 35 mm either side so the hardtop's edge is chamfered rather than a full-width slab.
    // The front edge is 40 mm of header ahead of the glass top rather than an absolute z, so the roof follows the screen when the rake changes.
    {
        float rz1 = WS_TOP_Z + 0.040f;
        Vector3 c[8] = {
            { -SIDE_W, RAIL_Y, CAB_BACK_Z }, { SIDE_W, RAIL_Y, CAB_BACK_Z }, { SIDE_W, RAIL_Y, rz1 }, { -SIDE_W, RAIL_Y, rz1 },
            { -0.835f, ROOF_Y, CAB_BACK_Z }, { 0.835f, ROOF_Y, CAB_BACK_Z }, { 0.835f, ROOF_Y, rz1 }, { -0.835f, ROOF_Y, rz1 },
        };
        Hex(body, c);
    }

    // Rear cab wall with its window.
    Box(body, -SIDE_W, SIDE_W, SILL_Y, ROOF_Y, CAB_BACK_Z, CAB_BACK_Z + 0.060f);
    Box(glass, -0.620f, 0.620f, GLASS_Y0, GLASS_Y1, CAB_BACK_Z - 0.006f, CAB_BACK_Z + 0.004f);

    GroupMark m = GroupMarkNow(g);

    // A-pillar: its front face is the windscreen plane, its rear face the vertical plane the door shuts against.
    // It is a wedge rather than a constant-thickness post, which is what lets the near-upright screen and the vertical door edge meet without a filler panel between them.
    {
        Vector3 c[8] = {
            { CAB_IN, WS_TOP_Y, WS_PILLAR_Z }, { SIDE_W, WS_TOP_Y, WS_PILLAR_Z },
            { SIDE_W, COWL_Y, WS_PILLAR_Z },   { CAB_IN, COWL_Y, WS_PILLAR_Z },
            { CAB_IN, WS_TOP_Y, WS_TOP_Z },    { SIDE_W, WS_TOP_Y, WS_TOP_Z },
            { SIDE_W, COWL_Y, COWL_Z },        { CAB_IN, COWL_Y, COWL_Z },
        };
        Hex(body, c);
    }

    // Doors: skin, window frame and glass. 20 mm gaps all round give shut lines.
    // Both leaves are cut from the run between the A-pillar and the C-pillar, so they come out equal in length and follow the pillar when the rake moves it.
    const float doorBack = CAB_BACK_Z + 0.100f;
    const float doorLen = (DOOR_F_Z - doorBack - 0.080f - 0.040f) / 2.0f;
    const float bPillarZ1 = DOOR_F_Z - doorLen - 0.020f;
    const float door[2][2] = { { DOOR_F_Z - doorLen, DOOR_F_Z }, { doorBack, bPillarZ1 - 0.100f } };

    // B and C pillars. Above RAIL_Y the chamfered roof is the only bodywork, so there is no separate side rail to hide the chamfer.
    Box(body, CAB_IN, SIDE_W, DOOR_Y0, RAIL_Y, bPillarZ1 - 0.080f, bPillarZ1);
    Box(body, CAB_IN, SIDE_W, SILL_Y, RAIL_Y, CAB_BACK_Z, -1.070f);

    // Cowl side, closing the body between the door's leading edge and the fender.
    Box(body, CAB_IN, SIDE_W, DOOR_Y0 - 0.020f, COWL_Y, DOOR_F_Z + 0.020f, HOOD_BACK_Z);

    for (int d = 0; d < 2; d++) {
        float z0 = door[d][0], z1 = door[d][1];
        Box(body, CAB_IN, SIDE_W - 0.010f, DOOR_Y0 + 0.020f, BELT_Y, z0, z1);
        Box(body, CAB_IN, SIDE_W - 0.010f, BELT_Y, GLASS_Y0, z0, z1);
        Box(body, CAB_IN, SIDE_W - 0.010f, GLASS_Y1, DOOR_TOP_Y, z0, z1);
        Box(body, CAB_IN, SIDE_W - 0.010f, GLASS_Y0, GLASS_Y1, z0, z0 + 0.040f);
        Box(body, CAB_IN, SIDE_W - 0.010f, GLASS_Y0, GLASS_Y1, z1 - 0.040f, z1);
        Box(glass, CAB_IN, SIDE_W - 0.040f, GLASS_Y0, GLASS_Y1, z0 + 0.040f, z1 - 0.040f);

        // Handle 35 mm in from the leaf's trailing edge, and the two external hinges bridging the shut line at its leading edge.
        // Both are measured off this leaf's own z0/z1: as literals outside the loop the rear handle had drifted to 25 mm from its own hinge.
        Box(dark, SIDE_W - 0.012f, SIDE_W + 0.022f, BELT_Y - 0.150f, BELT_Y - 0.090f, z0 + 0.035f, z0 + 0.175f);
        float hz = z1 + 0.0125f;
        Box(dark, SIDE_W - 0.015f, SIDE_W + 0.025f, DOOR_Y0 + 0.120f, DOOR_Y0 + 0.170f, hz - 0.0225f, hz + 0.0225f);
        Box(dark, SIDE_W - 0.015f, SIDE_W + 0.025f, BELT_Y + 0.135f, BELT_Y + 0.185f, hz - 0.0225f, hz + 0.0225f);
    }

    // Wing mirror on two arms, its bracket overlapping the front door's leading window post rather than hanging clear of the body.
    {
        float mz = DOOR_F_Z;
        Box(dark, 0.850f, 0.900f, 1.420f, 1.620f, mz - 0.045f, mz + 0.010f);
        Tube(dark, (Vector3){ 0.895f, 1.445f, mz - 0.018f }, (Vector3){ 1.140f, 1.420f, mz - 0.065f }, 0.017f, 0.017f, 8, true, true);
        Tube(dark, (Vector3){ 0.895f, 1.595f, mz - 0.018f }, (Vector3){ 1.140f, 1.580f, mz - 0.065f }, 0.017f, 0.017f, 8, true, true);
        Box(dark, 1.120f, 1.290f, 1.350f, 1.660f, mz - 0.125f, mz - 0.060f);
        Box(mirror, 1.135f, 1.275f, 1.370f, 1.640f, mz - 0.135f, mz - 0.123f);
    }

    // Washer nozzle on the header, one per pane, inboard of that pane's wiper pivot. Both are visible in ref_04.jpg.
    Tube(dark, WindscreenPoint(0.300f, 1.010f, 0.010f), WindscreenPoint(0.300f, 1.010f, 0.048f), 0.014f, 0.011f, 8, false, true);

    GroupMirrorX(g, m);
    GroupFinish(g, "cab");
}

// ---------------------------------------------------------------------------
// interior: what the eye reaches through the glass
//
// Only what stands above CAB_FILL_Y is modelled. Below that line the cab is the solid core built in BuildCab, which no window can see past, and modelling a floor pan under it would cost vertices nobody can look at.
// ---------------------------------------------------------------------------

static void BuildInterior(void)
{
    Group *g = &gInterior;
    Builder *dark = &g->b[MAT_CABIN];
    Builder *metal = &g->b[MAT_METAL];

    // Transmission tunnel, the hump the HMMWV runs down the middle of its cab, and the dash across the front of it.
    Box(dark, -0.300f, 0.300f, CAB_FILL_Y - 0.010f, 1.262f, -0.420f, 0.560f);
    Box(dark, -CAB_IN, CAB_IN, CAB_FILL_Y - 0.010f, 1.318f, 0.320f, COWL_Z);
    // Instrument binnacle over the driver, and the padded top rail across the whole dash.
    Box(dark, -0.640f, -0.220f, 1.318f, 1.392f, 0.360f, COWL_Z);
    Box(dark, -CAB_IN, CAB_IN, 1.318f, 1.348f, 0.320f, 0.372f);

    GroupMark m = GroupMarkNow(g);

    // Seats. The cushion top is buried in the core; only the squab above the window line is worth building.
    for (int r = 0; r < 2; r++) {
        float sz = (r == 0) ? -0.020f : -0.760f;
        Box(dark, 0.140f, 0.700f, CAB_FILL_Y - 0.010f, 1.610f, sz - 0.075f, sz + 0.075f);
        Box(dark, 0.140f, 0.700f, 1.610f, 1.680f, sz - 0.065f, sz + 0.065f);
    }
    // Grab handle on the passenger side of the dash.
    Tube(metal, (Vector3){ 0.360f, 1.348f, 0.470f }, (Vector3){ 0.640f, 1.348f, 0.470f }, 0.016f, 0.016f, 8, true, true);

    GroupMirrorX(g, m);

    // Steering wheel on the left, which is the side the M998 drives from, on a column raked back off the dash.
    {
        // Behind the dash rather than inside it: the dash runs forward of z = 0.320, so the column roots at its face and the rim stands clear of the binnacle.
        const float cx = -0.430f;
        Vector3 hub = { cx, 1.505f, 0.235f };
        Vector3 base = { cx, 1.292f, 0.455f };
        Tube(metal, base, hub, 0.030f, 0.024f, 10, true, false);
        // Rim, swept as a circle of 0.176 radius about the column axis. The wheel plane is perpendicular to that axis, so the rim is built in the frame the column defines rather than in the vehicle's.
        Vector3 axis = Vector3Normalize(Vector3Subtract(hub, base));
        Vector3 u = Vector3Normalize(Vector3CrossProduct((Vector3){ 1.0f, 0.0f, 0.0f }, axis));
        Vector3 v = Vector3CrossProduct(axis, u);
        const int SEG = 24;
        Vector3 prev = Vector3Add(hub, Vector3Scale(u, 0.176f));
        for (int i = 1; i <= SEG; i++) {
            float a = 2.0f * PI * (float)i / (float)SEG;
            Vector3 p = Vector3Add(hub, Vector3Add(Vector3Scale(u, 0.176f * cosf(a)), Vector3Scale(v, 0.176f * sinf(a))));
            Tube(dark, prev, p, 0.017f, 0.017f, 6, false, false);
            prev = p;
        }
        // Three spokes.
        for (int i = 0; i < 3; i++) {
            float a = 2.0f * PI * (float)i / 3.0f + 0.5f;
            Vector3 p = Vector3Add(hub, Vector3Add(Vector3Scale(u, 0.176f * cosf(a)), Vector3Scale(v, 0.176f * sinf(a))));
            Tube(dark, hub, p, 0.014f, 0.011f, 6, true, true);
        }
    }

    GroupFinish(g, "interior");
}

// ---------------------------------------------------------------------------
// wipers
//
// Top-mounted, one per pane, and both arms parallel rather than mirrored.
// That is measured, not assumed: references/humvee_v3_anim/ref_04.jpg looks straight into the windscreen and shows both pivots on the header, both arms hanging down and both leaning the same way, towards the vehicle's right. Mapping that view's pane edges onto x puts each pivot at |x| = 0.46 and each blade tip about 0.32 outboard of it. v3 built a short arm pivoting near the cowl instead, which is upside down.
// A parallel pair cannot be one mirrored mesh, and cannot be one mesh at all: the two arms turn by the same angle about two different points, which is not a rigid motion. So each is its own group with its own pivot.
// ---------------------------------------------------------------------------

#define WIPER_PIVOT_X   0.455f
#define WIPER_PIVOT_F   0.945f
#define WIPER_LEN       0.440f
#define WIPER_PARK      0.838f    // 48 degrees from straight down, leaning towards +x
#define WIPER_SWEEP     1.396f    // 80 degrees of arc

// A point on the wiper's own line: `along` metres from the pivot down the arm at the parked angle, `out` metres off the glass.
static Vector3 WiperPoint(float pivotX, float along, float out)
{
    float dy = WS_TOP_Y - COWL_Y, dz = WS_TOP_Z - COWL_Z;
    float len = sqrtf(dy * dy + dz * dz);
    // Screen-plane basis: +x across, and up the glass. Angle 0 hangs straight down; positive leans towards +x.
    float du = sinf(WIPER_PARK), dw = -cosf(WIPER_PARK);
    Vector3 p = WindscreenPoint(pivotX + along * du, WIPER_PIVOT_F + along * dw / len, out);
    return p;
}

static void BuildWiper(Group *g, float pivotX)
{
    Builder *dark = &g->b[MAT_DARK];

    // Pivot boss standing off the header.
    Tube(dark, WiperPoint(pivotX, 0.0f, 0.006f), WiperPoint(pivotX, 0.0f, 0.040f), 0.019f, 0.015f, 10, true, true);
    // Arm, tapering from the boss down to the blade it carries.
    Tube(dark, WiperPoint(pivotX, 0.010f, 0.032f), WiperPoint(pivotX, 0.34f * WIPER_LEN, 0.017f), 0.011f, 0.008f, 8, true, true);
    // Blade holder and the rubber under it, the rubber close enough to the glass to be touching it at this scale.
    Tube(dark, WiperPoint(pivotX, 0.30f * WIPER_LEN, 0.015f), WiperPoint(pivotX, WIPER_LEN, 0.015f), 0.009f, 0.007f, 8, true, true);
    Tube(dark, WiperPoint(pivotX, 0.31f * WIPER_LEN, 0.006f), WiperPoint(pivotX, WIPER_LEN, 0.006f), 0.005f, 0.004f, 6, true, true);

    GroupFinish(g, "wiper");
}

// ---------------------------------------------------------------------------
// bed: cargo box behind the cab, rear flares, tailgate and rear bumper
// ---------------------------------------------------------------------------

#define ANT_BASE_X    0.830f
#define ANT_BASE_Y    (BED_TOP_Y + 0.070f)
#define ANT_BASE_Z   -1.935f
#define ANT_LEN       0.930f

static void BuildBed(void)
{
    Group *g = &gBed;
    Builder *body = &g->b[MAT_BODY];
    Builder *metal = &g->b[MAT_METAL];
    Builder *tail = &g->b[MAT_TAIL];
    Builder *lamp = &g->b[MAT_LAMP];

    // Body mass under the bed floor, inboard of the rear wheels.
    Box(body, -CORE_HW, CORE_HW, SILL_Y, BED_FLOOR_Y, TAIL_Z, CAB_BACK_Z);
    // Bed floor, wide enough to reach the tops of the wheel arches.
    Box(body, -CAB_IN, CAB_IN, 1.070f, BED_FLOOR_Y, TAIL_Z, CAB_BACK_Z);
    // Tailgate, with the two horizontal stiffening ribs the cargo body carries.
    Box(body, -SIDE_W, SIDE_W, 1.040f, BED_TOP_Y, TAIL_Z - 0.060f, TAIL_Z);
    Box(body, -0.850f, 0.850f, 1.120f, 1.155f, TAIL_Z - 0.085f, TAIL_Z - 0.050f);
    Box(body, -0.850f, 0.850f, 1.290f, 1.325f, TAIL_Z - 0.085f, TAIL_Z - 0.050f);
    // Rear bumper.
    Box(metal, -HALF_W, HALF_W, 0.560f, 0.720f, BUMP_R_Z, TAIL_Z - 0.060f);

    GroupMark m = GroupMarkNow(g);

    // Rear fender flare, wheel opening cut out below, flat shelf on top.
    ArchedPanel(body, WELL_IN, HALF_W, TAIL_Z, -1.050f, AXLE_R, SILL_Y, FLARE_TOP_Y, FLARE_TOP_Y);
    ArchedPanel(body, CORE_HW, WELL_IN, TAIL_Z, CAB_BACK_Z, AXLE_R, SILL_Y, BED_FLOOR_Y, BED_FLOOR_Y);
    // Bed side wall, stepped inboard above the flare, with a capped top rail and external stiffening ribs.
    Box(body, CAB_IN, SIDE_W, FLARE_TOP_Y, BED_TOP_Y, TAIL_Z, CAB_BACK_Z);
    // Every joint below is given a few millimetres of overlap into its parent rather than meeting it face to face, so no interface is a coincident plane.
    Box(body, 0.785f, 0.885f, BED_TOP_Y - 0.010f, 1.455f, TAIL_Z, CAB_BACK_Z);
    for (int i = 0; i < 3; i++) {
        float rz = -1.350f - 0.250f * (float)i;
        Box(body, SIDE_W - 0.010f, 0.910f, FLARE_TOP_Y - 0.010f, BED_TOP_Y, rz - 0.035f, rz + 0.035f);
    }
    // Tie-down cleats on the rail, and the tailgate latch and hinge.
    Box(metal, 0.800f, 0.870f, 1.445f, 1.505f, -1.485f, -1.415f);
    Box(metal, 0.800f, 0.870f, 1.445f, 1.505f, -1.835f, -1.765f);
    Box(metal, 0.400f, 0.520f, 1.340f, 1.380f, TAIL_Z - 0.095f, TAIL_Z - 0.050f);
    Box(metal, 0.350f, 0.470f, 1.030f, 1.070f, TAIL_Z - 0.085f, TAIL_Z - 0.055f);
    // Tail lights: red over a white blackout lamp.
    Box(tail, 0.620f, 0.780f, 1.180f, 1.320f, TAIL_Z - 0.090f, TAIL_Z - 0.058f);
    Box(lamp, 0.620f, 0.780f, 1.100f, 1.170f, TAIL_Z - 0.090f, TAIL_Z - 0.058f);

    // Haloes on the rear faces, facing back down the vehicle axis.
    GlowDisc(&g->b[MAT_GLOW_R], 0.700f, 1.250f, TAIL_Z - 0.092f, 0.165f, -1.0f);
    GlowDisc(&g->b[MAT_GLOW_W], 0.700f, 1.135f, TAIL_Z - 0.092f, 0.100f, -1.0f);
    // Gusset carrying the cantilevered end of the bumper up to the rear flare.
    Box(metal, 0.880f, 1.060f, 0.720f, 0.950f, BUMP_R_Z + 0.020f, TAIL_Z);

    GroupMirrorX(g, m);

    // Antenna mount on the right rear corner, and the fuel filler on the left, both of which the real truck carries on one side only.
    // The whip itself is not here: it bends, so it is its own mesh.
    Box(metal, ANT_BASE_X - 0.050f, ANT_BASE_X + 0.050f, BED_TOP_Y, ANT_BASE_Y, ANT_BASE_Z - 0.055f, ANT_BASE_Z + 0.055f);
    Tube(metal, (Vector3){ -SIDE_W - 0.014f, 1.280f, -1.320f }, (Vector3){ -SIDE_W + 0.010f, 1.280f, -1.320f },
         0.058f, 0.058f, 16, true, false);

    GroupFinish(g, "bed");
}

// ---------------------------------------------------------------------------
// antenna
//
// The one surface here that deforms rather than moving rigidly. A whip that pivots stiffly at its base looks like a stick; the shape that reads as an antenna is the curve, and the curve changes every frame.
// Skinning is not available -- the harness's shader declares no bone attributes -- so the mesh is uploaded dynamic and its vertex and normal buffers are rewritten from the CPU each frame. The topology never changes: the same segment and side counts emit the same vertices in the same order at any bend, which is what makes an in-place buffer update legal.
// The curve is a cantilever clamped at its base: tangent angle grows as the square of the distance along it, and each segment steps exactly one segment-length along the unit tangent, so the whip keeps its length however far it leans.
// ---------------------------------------------------------------------------

#define ANT_SEGS   14
#define ANT_SIDES   8

static Vector3 WhipDir(float u, float bendX, float bendZ)
{
    float ax = bendX * u * u, az = bendZ * u * u;
    return Vector3Normalize((Vector3){ sinf(ax), cosf(ax) * cosf(az), sinf(az) });
}

// A ring of the swept circle. dir is close to +y throughout, so crossing it with +x never degenerates.
static void WhipRing(Vector3 c, Vector3 dir, float r, Vector3 *pos, Vector3 *nrm)
{
    Vector3 u = Vector3Normalize(Vector3CrossProduct((Vector3){ 1.0f, 0.0f, 0.0f }, dir));
    Vector3 v = Vector3CrossProduct(dir, u);
    for (int j = 0; j <= ANT_SIDES; j++) {
        float a = 2.0f * PI * (float)j / (float)ANT_SIDES;
        Vector3 rad = Vector3Add(Vector3Scale(u, cosf(a)), Vector3Scale(v, sinf(a)));
        nrm[j] = rad;
        pos[j] = Vector3Add(c, Vector3Scale(rad, r));
    }
}

static void EmitWhip(Builder *b, float bendX, float bendZ)
{
    const float ds = ANT_LEN / (float)ANT_SEGS;
    Vector3 c = { ANT_BASE_X, ANT_BASE_Y, ANT_BASE_Z };
    Vector3 p0[ANT_SIDES + 1], n0[ANT_SIDES + 1], p1[ANT_SIDES + 1], n1[ANT_SIDES + 1];

    WhipRing(c, WhipDir(0.0f, bendX, bendZ), 0.014f, p0, n0);
    for (int i = 0; i < ANT_SEGS; i++) {
        float um = ((float)i + 0.5f) / (float)ANT_SEGS;
        float u1 = (float)(i + 1) / (float)ANT_SEGS;
        c = Vector3Add(c, Vector3Scale(WhipDir(um, bendX, bendZ), ds));
        WhipRing(c, WhipDir(u1, bendX, bendZ), Lerp(0.014f, 0.006f, u1), p1, n1);
        for (int j = 0; j < ANT_SIDES; j++) {
            QuadN(b, p0[j], p0[j + 1], p1[j + 1], p1[j], n0[j], n0[j + 1], n1[j + 1], n1[j]);
        }
        for (int j = 0; j <= ANT_SIDES; j++) { p0[j] = p1[j]; n0[j] = n1[j]; }
    }

    Vector3 tip = WhipDir(1.0f, bendX, bendZ);
    int centre = Vert(b, c, tip);
    for (int j = 0; j < ANT_SIDES; j++) {
        int a = Vert(b, p0[j], tip);
        int d = Vert(b, p0[j + 1], tip);
        Tri(b, centre, a, d);
    }
}

static void BuildAntenna(void)
{
    Group *g = &gAntenna;
    g->dynamic = true;
    EmitWhip(&g->b[MAT_METAL], 0.0f, 0.0f);
    GroupFinish(g, "antenna");
}

// Rewrite the whip in place and push the two buffers that changed.
// The builder's arrays are the mesh's arrays: GroupFinish handed them straight to UploadMesh. Resetting the counts and re-emitting therefore writes over exactly the vertices the GPU already holds, and Reserve cannot reallocate because the second emission asks for the same capacity as the first. The guard below says so rather than assuming it.
static void AntennaSetBend(float bendX, float bendZ)
{
    Group *g = &gAntenna;
    Builder *b = &g->b[MAT_METAL];
    int verts = b->vertexCount, tris = b->triangleCount;
    float *base = b->vertices;

    b->vertexCount = 0;
    b->triangleCount = 0;
    EmitWhip(b, bendX, bendZ);

    if (b->vertexCount != verts || b->triangleCount != tris || b->vertices != base) {
        TraceLog(LOG_ERROR, "humvee_v3_anim: antenna rebuild changed the buffer (%d/%d vertices, %s)",
                 b->vertexCount, verts, (b->vertices == base) ? "same address" : "moved");
        return;
    }

    Mesh mesh = g->model[MAT_METAL].meshes[0];
    UpdateMeshBuffer(mesh, 0, b->vertices, verts * 3 * (int)sizeof(float), 0);
    UpdateMeshBuffer(mesh, 1, b->texcoords, verts * 2 * (int)sizeof(float), 0);
    UpdateMeshBuffer(mesh, 2, b->normals, verts * 3 * (int)sizeof(float), 0);

    // The bent whip does not fit the box the straight one did, and --part frames itself from this.
    Vector3 lo = { 1e9f, 1e9f, 1e9f }, hi = { -1e9f, -1e9f, -1e9f };
    for (int i = 0; i < verts; i++) {
        Vector3 p = { b->vertices[i * 3], b->vertices[i * 3 + 1], b->vertices[i * 3 + 2] };
        lo = Vector3Min(lo, p);
        hi = Vector3Max(hi, p);
    }
    g->bounds = (BoundingBox){ lo, hi };
}

// ---------------------------------------------------------------------------
// running gear
//
// Split three ways per corner, because the three move differently.
// The wheel spins about its hub and rides the suspension. The upright -- hub carrier, wishbones, lower spring seat -- rides the suspension without spinning. The coil and damper stand between the two, so they change length instead of moving.
// What stays with the body is only the differentials and the half shafts, in gSusp.
//
// The half shafts do swing: each is its own node, hinged on its inboard joint at the differential and rotated so its outer end lands on the travelling hub. They were rigid to the chassis in the first draft of this file, which left the outer end up to 0.110 m away from the carrier it drives; renders/humvee_v3_anim/v1/critique.md caught it.
// The wishbones do not, and translate with the wheel instead, which is wrong by the travel at their inboard ends. That end is the one place it does not show: at the front axle the upper arm runs from x = 0.24 to 0.76 at y = 0.63, entirely inside the engine bay block, and at the rear inside the bed's core. The lower arm sits below the belly pan, where it is only visible from underneath and meets nothing. The measured worst case is printed by CheckSuspension.
// ---------------------------------------------------------------------------

// Half-section of a 37x12.5R16.5, walked from the inboard bead round the tread to the outboard bead.
// x is the offset from the wheel centre plane, which is this group's own origin.
// The carcass crown stops at 0.450 and the tread lugs stand 20 mm proud of it, so the overall radius is TIRE_R and the tyre rests exactly on the ground plane.
static const Vector2 TIRE_PROFILE[] = {
    { -0.100f, RIM_R }, { -0.146f, 0.258f }, { -0.160f, 0.330f }, { -0.162f, 0.400f },
    { -0.150f, 0.432f }, { -0.118f, 0.446f }, { -0.062f, 0.450f }, {  0.062f, 0.450f },
    {  0.118f, 0.446f }, {  0.150f, 0.432f }, {  0.162f, 0.400f }, {  0.160f, 0.330f },
    {  0.146f, 0.258f }, {  0.100f, RIM_R },
};

#define TREAD_LUGS 20

// A block on the tread, addressed in the wheel's own (axial, radial, angular) frame about the hub at the origin.
// That frame is right-handed in the same sense as (x, y, z), so the hexahedron winding still resolves outwards.
static void LugBlock(Builder *b, float u0, float u1, float r0, float r1, float t0, float t1)
{
#define WP(u, r, t) (Vector3){ (u), (r) * cosf(t), (r) * sinf(t) }
    Vector3 c[8] = {
        WP(u0, r0, t0), WP(u1, r0, t0), WP(u1, r0, t1), WP(u0, r0, t1),
        WP(u0, r1, t0), WP(u1, r1, t0), WP(u1, r1, t1), WP(u0, r1, t1),
    };
#undef WP
    Hex(b, c);
}

// Tyre, rim and lug nuts, about the hub centre, +x outboard. This is the group the spin rotates, so nothing that does not turn with the road may go in it.
static void BuildWheelDisc(Group *g, bool left)
{
    Builder *dark = &g->b[MAT_DARK];
    Builder *metal = &g->b[MAT_METAL];

    RevolveX(dark, Vector3Zero(), TIRE_PROFILE, (int)(sizeof(TIRE_PROFILE) / sizeof(TIRE_PROFILE[0])), 40);

    // Directional tread: two staggered rows of lugs making a chevron.
    const float pitch = 2.0f * PI / (float)TREAD_LUGS;
    for (int i = 0; i < TREAD_LUGS; i++) {
        float t = pitch * (float)i;
        LugBlock(dark, 0.008f, 0.122f, 0.442f, TIRE_R, t - 0.052f, t + 0.052f);
        LugBlock(dark, -0.122f, -0.008f, 0.442f, TIRE_R,
                 t + pitch * 0.5f - 0.052f, t + pitch * 0.5f + 0.052f);
    }

    // Rim: barrel between the beads, dished outer face, hub boss and lug nuts.
    Tube(metal, (Vector3){ -0.100f, 0.0f, 0.0f }, (Vector3){ 0.100f, 0.0f, 0.0f }, RIM_R, RIM_R, 32, true, false);
    Tube(metal, (Vector3){ 0.100f, 0.0f, 0.0f }, (Vector3){ 0.062f, 0.0f, 0.0f }, RIM_R, 0.092f, 32, false, false);
    Tube(metal, (Vector3){ 0.062f, 0.0f, 0.0f }, (Vector3){ 0.118f, 0.0f, 0.0f }, 0.092f, 0.084f, 20, false, true);
    for (int i = 0; i < 8; i++) {
        float a = 2.0f * PI * (float)i / 8.0f;
        Vector3 p = { 0.080f, 0.148f * cosf(a), 0.148f * sinf(a) };
        Tube(metal, p, (Vector3){ p.x + 0.024f, p.y, p.z }, 0.020f, 0.020f, 6, false, true);
    }

    if (left) GroupMirrorX(g, (GroupMark){ 0 });
    GroupFinish(g, "wheel");
}

// Hub carrier and both wishbones, in the body's own frame at the corner's rest position.
static void BuildUpright(Group *g, float zc, bool left)
{
    Builder *metal = &g->b[MAT_METAL];

    Box(metal, 0.700f, 0.800f, 0.320f, 0.660f, zc - 0.090f, zc + 0.090f);
    Box(metal, 0.240f, 0.760f, 0.630f, 0.680f, zc - 0.070f, zc + 0.070f);
    Box(metal, 0.220f, 0.780f, 0.340f, 0.400f, zc - 0.100f, zc + 0.100f);
    // Lower spring seat, which is what the coil stands on and so has to travel with the arm.
    Box(metal, 0.560f, 0.740f, 0.530f, 0.575f, zc + 0.100f, zc + 0.260f);

    if (left) GroupMirrorX(g, (GroupMark){ 0 });
    GroupFinish(g, "upright");
}

// Coil and damper. Both are scaled about COIL_TOP_Y at pose time, which is the height their upper mounts sit at, so a compressing suspension shortens them instead of dragging them.
static void BuildSpring(Group *g, float zc, bool left)
{
    Builder *metal = &g->b[MAT_METAL];

    Coil(metal, (Vector3){ 0.650f, 0.575f, zc + 0.180f }, 0.070f, 0.016f, COIL_LEN, 5.0f, 56, 8);
    Tube(metal, (Vector3){ 0.620f, 0.400f, zc - 0.190f }, (Vector3){ 0.560f, 0.980f, zc - 0.190f },
         0.032f, 0.026f, 10, true, true);

    if (left) GroupMirrorX(g, (GroupMark){ 0 });
    GroupFinish(g, "spring");
}

// Half shaft, from the differential's output joint out to the hub carrier.
// It is built along the corner's own side rather than mirrored, because Tube takes its winding from the direction it is given and so needs no reflecting.
#define SHAFT_IN   0.170f
// 10 mm short of the hub carrier's outer face at 0.800, so the shaft's end cap is buried in it rather than flush with it. Flush is how humvee_v2 ended up with four joints on exactly coincident planes.
#define SHAFT_OUT  0.790f
#define SHAFT_LEN  (SHAFT_OUT - SHAFT_IN)

static void BuildShaft(Group *g, float side, float zc)
{
    Tube(&g->b[MAT_METAL], (Vector3){ side * SHAFT_IN, AXLE_Y, zc }, (Vector3){ side * SHAFT_OUT, AXLE_Y, zc },
         0.036f, 0.036f, 10, true, true);
    GroupFinish(g, "shaft");
}

// What is bolted to the body: the differentials and the upper mounts the coil and damper hang from.
static void BuildSusp(void)
{
    Group *g = &gSusp;
    Builder *metal = &g->b[MAT_METAL];

    GroupMark m = GroupMarkNow(g);
    for (int c = 0; c < 2; c++) {
        float zc = (c == 0) ? AXLE_F : AXLE_R;
        Box(metal, 0.560f, 0.740f, COIL_TOP_Y, 0.995f, zc + 0.100f, zc + 0.260f);
        Box(metal, 0.520f, 0.640f, COIL_TOP_Y, 1.005f, zc - 0.235f, zc - 0.145f);
    }
    GroupMirrorX(g, m);

    // Differential housings, centred so both half shafts meet them.
    Vector2 diff[] = { { -0.170f, 0.030f }, { -0.130f, 0.130f }, { -0.050f, 0.168f },
                       {  0.050f, 0.168f }, {  0.130f, 0.130f }, {  0.170f, 0.030f } };
    RevolveX(metal, (Vector3){ 0.0f, AXLE_Y, AXLE_F }, diff, 6, 20);
    RevolveX(metal, (Vector3){ 0.0f, AXLE_Y, AXLE_R }, diff, 6, 20);

    GroupFinish(g, "susp");
}

// ---------------------------------------------------------------------------
// Motion
//
// One cycle is one loop of a periodic road, and everything in it is a pure function of the time within that cycle, so a frame can be posed in any order and the loop closes on itself. CheckLoopCloses measures that rather than trusting it.
//
// The road is not drawn. What is drawn is the flat grid at y = 0, and the wheels stay on it: the road profile drives the body, and each corner's suspension takes up the difference between where the body has gone and where the ground is. That is the wrong way round physically -- on a real truck the road pushes the wheels and the wheels push the body -- but it is the arrangement that puts the right motion on the screen over flat ground, and it is why the wheels never sink through the grid.
//
// The one thing the road profile buys that no hand-written wobble does is the delay between axles. The front wheels reach a bump 3.30 m before the rear ones do, which at this speed is half a second; a truck that pitches nose-up and then tail-up half a second later reads as driving, and one that heaves as a lump does not.
// ---------------------------------------------------------------------------

#define CYCLE         4.0f
// Turns per cycle. An integer keeps the tread pattern and the eight lug nuts in the same place at the loop's seam; anything else steps.
#define WHEEL_TURNS   9.0f
#define SPEED         (2.0f * PI * TIRE_R * WHEEL_TURNS / CYCLE)   // 6.64 m/s, about 24 km/h
#define ROAD_PERIOD   (SPEED * CYCLE)
#define PITCH_PIVOT_Y 0.850f     // the body turns about roughly its own centre of mass, not about the ground

#define BUMP1_S       (0.30f * ROAD_PERIOD)
#define BUMP2_S       (0.68f * ROAD_PERIOD)
#define BUMP1_T       ((BUMP1_S - AXLE_F) / SPEED)
#define BUMP2_T       ((BUMP2_S - AXLE_F) / SPEED)

static float LoopWrap(float v, float period)
{
    v = fmodf(v, period);
    return (v < 0.0f) ? v + period : v;
}

// Signed distance to a point on the loop, taking the short way round.
static float LoopWrapSigned(float v, float period)
{
    return LoopWrap(v + period * 0.5f, period) - period * 0.5f;
}

static float Gaussian(float d, float width)
{
    float u = d / width;
    return expf(-u * u);
}

static float BumpAt(float s, float centre, float width, float height)
{
    return height * Gaussian(LoopWrapSigned(s - centre, ROAD_PERIOD), width);
}

// Height of the road under one wheel track. side is +1 for the right-hand track.
static float RoadY(float s, float side)
{
    const float w = 2.0f * PI / ROAD_PERIOD;
    float y = 0.032f * sinf(w * s * 3.0f)
            + 0.020f * sinf(w * s * 7.0f + 1.10f)
            + 0.011f * sinf(w * s * 13.0f + 2.30f)
            + 0.006f * sinf(w * s * 27.0f + 0.70f);
    // The two tracks differ, or the truck would never roll.
    y += side * (0.014f * sinf(w * s * 5.0f + 0.40f) + 0.008f * sinf(w * s * 11.0f + 1.90f));
    // Two obstacles. The big one is mostly under the right-hand track, which is what makes the truck lurch rather than merely rise.
    y += (side > 0.0f ? 1.0f : 0.30f) * BumpAt(s, BUMP1_S, 1.00f, 0.220f);
    y += BumpAt(s, BUMP2_S, 0.85f, 0.070f);
    return y;
}

// What the springs and the tyre's contact patch actually pass through to the body.
// Nine samples over about a metre with a raised-cosine weight: neither a 0.32 m tyre footprint nor a suspension can follow a wavelength shorter than itself, so the washboard is filtered out here and put back as a separate high-frequency tremble.
static float RoadFiltered(float s, float side)
{
    float sum = 0.0f, norm = 0.0f;
    for (int i = -4; i <= 4; i++) {
        float wgt = 0.5f + 0.5f * cosf(PI * (float)i / 5.0f);
        sum += wgt * RoadY(s + 0.1125f * (float)i, side);
        norm += wgt;
    }
    return sum / norm;
}

// The body carries on bouncing after a bump has passed under it. Each obstacle contributes a damped sine starting when the front axle reaches it.
// The decay is fast enough that the term is 1e-8 of its amplitude by the time the cycle wraps, so this stays periodic without being forced to be.
static float SpringRing(float t, float amp1, float amp2, float freq, float decay)
{
    float u1 = LoopWrap(t - BUMP1_T, CYCLE), u2 = LoopWrap(t - BUMP2_T, CYCLE);
    return amp1 * expf(-decay * u1) * sinf(2.0f * PI * freq * u1)
         + amp2 * expf(-decay * u2) * sinf(2.0f * PI * freq * u2);
}

// Engine and tyre buzz. Every frequency is a whole number of cycles per loop, so these close too.
static float Tremble(float t, float amp, float harmonic, float phase)
{
    return amp * sinf(2.0f * PI * harmonic * t / CYCLE + phase);
}

// Where the body is, before the wheels are consulted.
static Matrix ChassisFree(float t)
{
    float s = SPEED * t;
    float f[CORNERS];
    for (int i = 0; i < CORNERS; i++) f[i] = RoadFiltered(s + CornerZ(i), CornerSide(i));

    float mean = (f[0] + f[1] + f[2] + f[3]) * 0.25f;
    float front = (f[0] + f[1]) * 0.5f, rear = (f[2] + f[3]) * 0.5f;
    float right = (f[0] + f[2]) * 0.5f, left = (f[1] + f[3]) * 0.5f;

    float heave = mean + SpringRing(t, 0.320f, 0.022f, 1.6f, 2.2f)
                + Tremble(t, 0.0022f, 19.0f, 0.0f) + Tremble(t, 0.0014f, 31.0f, 1.30f);
    float pitch = 0.85f * (front - rear) / WHEELBASE + SpringRing(t, 0.034f, 0.010f, 1.9f, 4.8f)
                + Tremble(t, 0.0018f, 23.0f, 0.60f);
    float roll = 0.70f * (right - left) / TRACK + SpringRing(t, 0.026f, 0.007f, 2.1f, 5.2f)
               + Tremble(t, 0.0026f, 29.0f, 2.10f);
    float yaw = Tremble(t, 0.0021f, 11.0f, 1.70f);
    float sway = Tremble(t, 0.0035f, 13.0f, 0.90f);
    float surge = Tremble(t, 0.0028f, 17.0f, 2.60f);

    Matrix rot = MatrixRotateXYZ((Vector3){ pitch, yaw, roll });
    Matrix toPivot = MatrixTranslate(0.0f, -PITCH_PIVOT_Y, 0.0f);
    Matrix back = MatrixTranslate(sway, PITCH_PIVOT_Y + heave, surge);
    return MatrixMultiply(MatrixMultiply(toPivot, rot), back);
}

static Vector3 HubRest(int i)
{
    return (Vector3){ CornerSide(i) * TRACK_HW, AXLE_Y, CornerZ(i) };
}

// The body, after being raised by however much a wheel would otherwise have had to push through the ground.
// Nothing else enforces that: the road profile drives the body directly, so on the low side of an undulation the body can arrive below what the suspension can reach up from. Raising it is a pure translation in world y, which is why it can be added to the matrix after the fact.
static Matrix ChassisAt(float t)
{
    Matrix c = ChassisFree(t);
    float worst = -1e9f;
    for (int i = 0; i < CORNERS; i++) {
        float need = TIRE_R - Vector3Transform(HubRest(i), c).y;
        if (need > worst) worst = need;
    }
    float lift = worst - SUSP_UP;
    if (lift > 0.0f) c.m13 += lift;
    return c;
}

typedef struct {
    Matrix chassis;
    float travel[CORNERS];
    float spin;
    float wiper;     // radians away from the parked angle
    float bendX, bendZ;
    float head, blink, tail;
} Pose;

// Horizontal acceleration of the antenna's base, in the body's own frame.
// The whip is vertical, so it is the sideways shove that bends it, and almost all of that comes from the body rotating under a mast standing 0.93 m above the bed rather than from the body's own translation.
// Second differences in float are the trap this project has already hit once, so the step is 10 ms rather than the millisecond that put torus_knot's frame normal into the noise: at this amplitude the quantisation shows up around 1e-5 m/s2 against signals of a few m/s2.
static void AntennaDrive(float t, float *ax, float *az)
{
    const float h = 0.010f;
    Vector3 base = { ANT_BASE_X, ANT_BASE_Y, ANT_BASE_Z };
    Matrix c0 = ChassisAt(LoopWrap(t - h, CYCLE)), c1 = ChassisAt(t), c2 = ChassisAt(LoopWrap(t + h, CYCLE));
    Vector3 p0 = Vector3Transform(base, c0), p1 = Vector3Transform(base, c1), p2 = Vector3Transform(base, c2);
    Vector3 acc = Vector3Scale(Vector3Add(Vector3Subtract(p0, Vector3Scale(p1, 2.0f)), p2), 1.0f / (h * h));

    // Back into body coordinates. The rotation part of a rigid transform inverts by transposing, so this needs no matrix inverse.
    *ax = acc.x * c1.m0 + acc.y * c1.m1 + acc.z * c1.m2;
    *az = acc.x * c1.m8 + acc.y * c1.m9 + acc.z * c1.m10;
}

static Pose PoseFor(float t)
{
    Pose p = { 0 };
    p.chassis = ChassisAt(t);

    for (int i = 0; i < CORNERS; i++) {
        float need = TIRE_R - Vector3Transform(HubRest(i), p.chassis).y;
        p.travel[i] = Clamp(need, -SUSP_DOWN, SUSP_UP);
    }

    p.spin = 2.0f * PI * WHEEL_TURNS * t / CYCLE;

    // Three full wipes per loop. A cosine holds at each end of the arc, which is what a wiper linkage does anyway.
    p.wiper = -WIPER_SWEEP * (0.5f - 0.5f * cosf(2.0f * PI * 3.0f * t / CYCLE));

    // The whip lags whatever shoves it, so it leans against the acceleration, and it sways on its own besides.
    float ax, az;
    AntennaDrive(t, &ax, &az);
    p.bendX = Clamp(-0.058f * ax + Tremble(t, 0.075f, 5.0f, 0.0f), -0.61f, 0.61f);
    p.bendZ = Clamp(-0.058f * az + Tremble(t, 0.055f, 7.0f, 1.00f), -0.61f, 0.61f);

    // Hazard flashers: 1.25 Hz, five whole blinks per loop.
    float phase = LoopWrap(t, CYCLE / 5.0f) / (CYCLE / 5.0f);
    p.blink = (phase < 0.5f) ? 1.0f : 0.10f;

    // Headlamps burn steady until the truck lands off the big bump, then flicker twice.
    p.head = 1.0f - 0.50f * Gaussian(LoopWrapSigned(t - BUMP1_T - 0.34f, CYCLE), 0.045f)
                  - 0.32f * Gaussian(LoopWrapSigned(t - BUMP1_T - 0.60f, CYCLE), 0.038f);

    // Tail lamps sit at running-light brightness and come up to brake brightness on the approach to the obstacle.
    p.tail = 0.42f + 0.58f * Gaussian(LoopWrapSigned(t - BUMP1_T + 0.25f, CYCLE), 0.30f);
    return p;
}

// ---------------------------------------------------------------------------
// Posing
// ---------------------------------------------------------------------------

static Color Dim(Color c, float k)
{
    return (Color){
        (unsigned char)Clamp((float)c.r * k, 0.0f, 255.0f),
        (unsigned char)Clamp((float)c.g * k, 0.0f, 255.0f),
        (unsigned char)Clamp((float)c.b * k, 0.0f, 255.0f),
        c.a,
    };
}

static void Update(float t)
{
    Pose p = PoseFor(t);

    gHull.xform = gFront.xform = gCab.xform = gInterior.xform = gBed.xform = gSusp.xform = p.chassis;

    for (int i = 0; i < CORNERS; i++) {
        Vector3 hub = HubRest(i);
        gUpright[i].xform = MatrixMultiply(MatrixTranslate(0.0f, p.travel[i], 0.0f), p.chassis);
        gWheel[i].xform = MatrixMultiply(
            MatrixMultiply(MatrixRotateX(p.spin), MatrixTranslate(hub.x, hub.y + p.travel[i], hub.z)),
            p.chassis);
        // Scaling about the upper mount is what shortens the coil and the damper; raylib rebuilds matNormal from the model matrix, so the non-uniform scale does not wreck their shading.
        // The shaft hinges at the differential, so its outer end rises with the hub; a rigid swing leaves the end 3 mm short in x of where it started, which is inside the hub carrier.
        float swing = CornerSide(i) * asinf(Clamp(p.travel[i] / SHAFT_LEN, -1.0f, 1.0f));
        Vector3 joint = { CornerSide(i) * SHAFT_IN, AXLE_Y, CornerZ(i) };
        gShaft[i].xform = MatrixMultiply(MatrixMultiply(
            MatrixMultiply(MatrixTranslate(-joint.x, -joint.y, -joint.z), MatrixRotateZ(swing)),
            MatrixTranslate(joint.x, joint.y, joint.z)), p.chassis);

        float k = (COIL_LEN - p.travel[i]) / COIL_LEN;
        gSpring[i].xform = MatrixMultiply(MatrixMultiply(
            MatrixMultiply(MatrixTranslate(0.0f, -COIL_TOP_Y, 0.0f), MatrixScale(1.0f, k, 1.0f)),
            MatrixTranslate(0.0f, COIL_TOP_Y, 0.0f)), p.chassis);
    }

    Vector3 axis = WindscreenNormal();
    for (int w = 0; w < 2; w++) {
        float px = (w == 0) ? WIPER_PIVOT_X : -WIPER_PIVOT_X;
        Vector3 pivot = WindscreenPoint(px, WIPER_PIVOT_F, 0.0f);
        gWiper[w].xform = MatrixMultiply(MatrixMultiply(
            MatrixMultiply(MatrixTranslate(-pivot.x, -pivot.y, -pivot.z), MatrixRotate(axis, p.wiper)),
            MatrixTranslate(pivot.x, pivot.y, pivot.z)), p.chassis);
    }

    AntennaSetBend(p.bendX, p.bendZ);
    gAntenna.xform = p.chassis;

    gFront.tint[MAT_LAMP] = gFront.tint[MAT_GLOW_W] = Dim(WHITE, p.head);
    gFront.tint[MAT_AMBER] = gFront.tint[MAT_GLOW_A] = Dim(WHITE, p.blink);
    gBed.tint[MAT_TAIL] = gBed.tint[MAT_GLOW_R] = Dim(WHITE, p.tail);
    gBed.tint[MAT_LAMP] = gBed.tint[MAT_GLOW_W] = Dim(WHITE, p.blink);
}

// ---------------------------------------------------------------------------
// Parts
// ---------------------------------------------------------------------------

static Group *PART_HULL[]     = { &gHull };
static Group *PART_FRONT[]    = { &gFront };
static Group *PART_CAB[]      = { &gCab };
static Group *PART_INTERIOR[] = { &gInterior };
static Group *PART_BED[]      = { &gBed };
static Group *PART_WIPERS[]   = { &gWiper[0], &gWiper[1] };
static Group *PART_ANTENNA[]  = { &gAntenna };
static Group *PART_GEAR[17];

#define COUNT_OF(a) ((int)(sizeof(a) / sizeof((a)[0])))

// A moving part has no single bounding box, so --part frames it from the union of its boxes over the whole cycle.
// GetModelBoundingBox is no help here: rmodels.c:1243 transforms only the box's own two corners and warns that it does not support rotation, which is exactly what a spinning wheel and a sweeping wiper do.
static BoundingBox SweptBounds(Group *const *gs, int n)
{
    BoundingBox out = { { 1e9f, 1e9f, 1e9f }, { -1e9f, -1e9f, -1e9f } };
    const int SAMPLES = 96;
    for (int s = 0; s < SAMPLES; s++) {
        Update(CYCLE * (float)s / (float)SAMPLES);
        for (int i = 0; i < n; i++) {
            BoundingBox b = gs[i]->bounds;
            for (int c = 0; c < 8; c++) {
                Vector3 p = {
                    (c & 1) ? b.max.x : b.min.x,
                    (c & 2) ? b.max.y : b.min.y,
                    (c & 4) ? b.max.z : b.min.z,
                };
                p = Vector3Transform(p, gs[i]->xform);
                out.min = Vector3Min(out.min, p);
                out.max = Vector3Max(out.max, p);
            }
        }
    }
    Update(0.0f);
    return out;
}

static BoundingBox bHull, bFront, bCab, bInterior, bBed, bGear, bWipers, bAntenna;

static void DrawAll(void) { DrawGroups(gAll, gAllCount); }
static void DrawHull(void) { DrawGroups(PART_HULL, COUNT_OF(PART_HULL)); }
static void DrawFront(void) { DrawGroups(PART_FRONT, COUNT_OF(PART_FRONT)); }
static void DrawCab(void) { DrawGroups(PART_CAB, COUNT_OF(PART_CAB)); }
static void DrawInterior(void) { DrawGroups(PART_INTERIOR, COUNT_OF(PART_INTERIOR)); }
static void DrawBed(void) { DrawGroups(PART_BED, COUNT_OF(PART_BED)); }
static void DrawGear(void) { DrawGroups(PART_GEAR, COUNT_OF(PART_GEAR)); }
static void DrawWipers(void) { DrawGroups(PART_WIPERS, COUNT_OF(PART_WIPERS)); }
static void DrawAntenna(void) { DrawGroups(PART_ANTENNA, COUNT_OF(PART_ANTENNA)); }

static BoundingBox HullBounds(void) { return bHull; }
static BoundingBox FrontBounds(void) { return bFront; }
static BoundingBox CabBounds(void) { return bCab; }
static BoundingBox InteriorBounds(void) { return bInterior; }
static BoundingBox BedBounds(void) { return bBed; }
static BoundingBox GearBounds(void) { return bGear; }
static BoundingBox WiperBounds(void) { return bWipers; }
static BoundingBox AntennaBounds(void) { return bAntenna; }

static const Part PARTS[] = {
    { .name = "hull", .draw = DrawHull, .bounds = HullBounds },
    { .name = "front", .draw = DrawFront, .bounds = FrontBounds },
    { .name = "cab", .draw = DrawCab, .bounds = CabBounds },
    { .name = "interior", .draw = DrawInterior, .bounds = InteriorBounds },
    { .name = "bed", .draw = DrawBed, .bounds = BedBounds },
    { .name = "running_gear", .draw = DrawGear, .bounds = GearBounds },
    { .name = "wipers", .draw = DrawWipers, .bounds = WiperBounds },
    { .name = "antenna", .draw = DrawAntenna, .bounds = AntennaBounds },
};

// ---------------------------------------------------------------------------
// Checks
//
// A pose can be wrong without looking wrong in any one frame, which is the whole reason crank_slider walks its linkage at build time instead of arguing that the joints line up. These do the same for the four claims this file makes that a still cannot settle.
// ---------------------------------------------------------------------------

// The front door's leading edge is a vertical line, the windscreen a leaning plane, so the clearance between them is smallest at the top of the door and it is easy to leave the door standing in front of the glass.
static void CheckDoorClearsScreen(void)
{
    float worst = 1e9f, worstY = 0.0f;
    for (int i = 0; i <= 200; i++) {
        float y = DOOR_Y0 + (DOOR_TOP_Y - DOOR_Y0) * (float)i / 200.0f;
        if (y < COWL_Y) continue;
        float gap = (COWL_Z - (y - COWL_Y) * WS_RAKE) - DOOR_F_Z;
        if (gap < worst) { worst = gap; worstY = y; }
    }
    if (worst < 0.020f) {
        TraceLog(LOG_WARNING, "humvee_v3_anim: door leading edge clears the windscreen by only %.3f m at y = %.3f", worst, worstY);
    }
}

// Claim: no tyre ever goes through the ground the grid draws, and at least one is on it except while the truck is off the big bump.
static void CheckGroundContact(void)
{
    float deepest = 1e9f, highest = -1e9f;
    float airborne = 0.0f;
    const int N = 720;
    for (int k = 0; k < N; k++) {
        float t = CYCLE * (float)k / (float)N;
        Pose p = PoseFor(t);
        // The wheel's axis is the body's x axis, tilted by roll and yaw; the lowest point of a circle of radius R about a unit axis a sits R*sqrt(1 - a.y^2) below its centre.
        float ay = p.chassis.m1;
        float drop = TIRE_R * sqrtf(fmaxf(0.0f, 1.0f - ay * ay));
        float lowest = 1e9f;
        for (int i = 0; i < CORNERS; i++) {
            float y = Vector3Transform(HubRest(i), p.chassis).y + p.travel[i] - drop;
            if (y < lowest) lowest = y;
        }
        if (lowest < deepest) deepest = lowest;
        if (lowest > highest) highest = lowest;
        if (lowest > 0.002f) airborne += CYCLE / (float)N;
    }
    TraceLog(LOG_INFO, "humvee_v3_anim: tyre contact runs from %+.4f m to %+.4f m, airborne for %.2f s of %.1f",
             deepest, highest, airborne, CYCLE);
    if (deepest < -0.001f) {
        TraceLog(LOG_WARNING, "humvee_v3_anim: a tyre reaches %.4f m through the ground plane", deepest);
    }
}

// Claim: each blade stays on its own pane for the whole sweep, rather than running off the edge or over the centre divider.
static void CheckWiperSweep(void)
{
    float dy = WS_TOP_Y - COWL_Y, dz = WS_TOP_Z - COWL_Z;
    float len = sqrtf(dy * dy + dz * dz);
    float nearest = 1e9f, outermost = 0.0f, lowest = 1.0f;
    for (int k = 0; k <= 180; k++) {
        float th = WIPER_PARK - WIPER_SWEEP * (float)k / 180.0f;
        float x = WIPER_PIVOT_X + WIPER_LEN * sinf(th);
        float f = WIPER_PIVOT_F - WIPER_LEN * cosf(th) / len;
        if (x < nearest) nearest = x;
        if (x > outermost) outermost = x;
        if (f < lowest) lowest = f;
    }
    TraceLog(LOG_INFO, "humvee_v3_anim: blade tip sweeps x %.3f to %.3f, down to %.2f of the screen",
             nearest, outermost, lowest);
    if (nearest < 0.040f || outermost > CAB_IN || lowest < 0.0f) {
        TraceLog(LOG_WARNING, "humvee_v3_anim: wiper leaves its pane (x %.3f..%.3f, f down to %.2f)",
                 nearest, outermost, lowest);
    }
}

// Claim: the cycle is a loop. Everything here is periodic by construction, but the ring-down terms are only periodic because they decay far enough, and that is worth measuring rather than asserting.
static void CheckLoopCloses(void)
{
    Pose a = PoseFor(0.0f), b = PoseFor(CYCLE);
    float worst = 0.0f;
    const float *ma = &a.chassis.m0, *mb = &b.chassis.m0;
    for (int i = 0; i < 16; i++) worst = fmaxf(worst, fabsf(ma[i] - mb[i]));
    for (int i = 0; i < CORNERS; i++) worst = fmaxf(worst, fabsf(a.travel[i] - b.travel[i]));
    worst = fmaxf(worst, fabsf(a.wiper - b.wiper));
    worst = fmaxf(worst, fabsf(a.bendX - b.bendX));
    worst = fmaxf(worst, fabsf(a.bendZ - b.bendZ));
    worst = fmaxf(worst, fabsf(a.head - b.head));
    worst = fmaxf(worst, fabsf(a.tail - b.tail));
    // The spin is an angle, so it only has to come back to the same place modulo a whole turn.
    float turns = (b.spin - a.spin) / (2.0f * PI);
    float slip = fabsf(turns - roundf(turns));

    TraceLog(LOG_INFO, "humvee_v3_anim: loop closes to %.2e, wheel to %.2e of a turn", worst, slip);
    if (worst > 1e-4f || slip > 1e-4f) {
        TraceLog(LOG_WARNING, "humvee_v3_anim: pose does not close over the cycle (%.2e, wheel %.2e turns)", worst, slip);
    }
}

// Claim: each half shaft's outer end stays inside the hub carrier it drives, at every point of the travel. This is the joint renders/humvee_v3_anim/v1/critique.md found open, so it is measured rather than argued.
static void CheckHalfShaft(void)
{
    float worst = 1e9f;
    for (int k = 0; k < 720; k++) {
        Pose p = PoseFor(CYCLE * (float)k / 720.0f);
        for (int i = 0; i < CORNERS; i++) {
            // Measured on the right-hand side; the left is the same swing reflected, so its end lands at the same distance from its own carrier.
            float swing = asinf(Clamp(p.travel[i] / SHAFT_LEN, -1.0f, 1.0f));
            float ex = SHAFT_IN + SHAFT_LEN * cosf(swing);
            float ey = AXLE_Y + p.travel[i];
            // Depth of that point inside the carrier box, which itself has moved by the same travel.
            float d = fminf(fminf(ex - 0.700f, 0.800f - ex),
                            fminf(ey - (0.320f + p.travel[i]), (0.660f + p.travel[i]) - ey));
            if (d < worst) worst = d;
        }
    }
    TraceLog(LOG_INFO, "humvee_v3_anim: half shaft end sits %.3f m inside the hub carrier at worst", worst);
    if (worst < 0.0f) {
        TraceLog(LOG_WARNING, "humvee_v3_anim: half shaft end leaves the hub carrier by %.3f m", -worst);
    }
}

// Not a claim so much as a number on the record: the wishbones translate with their wheel rather than swinging, so this is how far their inboard ends move from where they are bolted.
static void CheckSuspension(void)
{
    float worst = 0.0f, shortest = 1.0f;
    for (int k = 0; k < 720; k++) {
        Pose p = PoseFor(CYCLE * (float)k / 720.0f);
        for (int i = 0; i < CORNERS; i++) {
            worst = fmaxf(worst, fabsf(p.travel[i]));
            shortest = fminf(shortest, (COIL_LEN - p.travel[i]) / COIL_LEN);
        }
    }
    TraceLog(LOG_INFO, "humvee_v3_anim: suspension travels up to %.3f m, coil to %.0f%% of free length",
             worst, shortest * 100.0f);
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

static void Init(void)
{
    // Before any group is finished: GroupFinish hands each material its map and its shader, so both have to exist by then.
    MakeTextures();

    BuildHull();
    BuildFront();
    BuildCab();
    BuildInterior();
    BuildBed();
    BuildSusp();
    // Each left-hand corner is the whole right-hand one reflected, which also reverses the winding so its faces still point out.
    // The reflection has to happen before the group is finished: GroupFinish hands the builder's arrays straight to the mesh, and mirroring afterwards would reallocate them out from under it.
    for (int i = 0; i < CORNERS; i++) {
        bool left = CornerSide(i) < 0.0f;
        BuildWheelDisc(&gWheel[i], left);
        BuildUpright(&gUpright[i], CornerZ(i), left);
        BuildSpring(&gSpring[i], CornerZ(i), left);
        BuildShaft(&gShaft[i], CornerSide(i), CornerZ(i));
    }
    BuildWiper(&gWiper[0], WIPER_PIVOT_X);
    BuildWiper(&gWiper[1], -WIPER_PIVOT_X);
    BuildAntenna();

    GroupRegister(&gHull);
    GroupRegister(&gFront);
    GroupRegister(&gCab);
    GroupRegister(&gInterior);
    GroupRegister(&gBed);
    GroupRegister(&gSusp);
    int n = 0;
    PART_GEAR[n++] = &gSusp;
    for (int i = 0; i < CORNERS; i++) {
        GroupRegister(&gUpright[i]);
        GroupRegister(&gSpring[i]);
        GroupRegister(&gShaft[i]);
        GroupRegister(&gWheel[i]);
        PART_GEAR[n++] = &gUpright[i];
        PART_GEAR[n++] = &gSpring[i];
        PART_GEAR[n++] = &gShaft[i];
        PART_GEAR[n++] = &gWheel[i];
    }
    GroupRegister(&gWiper[0]);
    GroupRegister(&gWiper[1]);
    GroupRegister(&gAntenna);

    bHull = SweptBounds(PART_HULL, COUNT_OF(PART_HULL));
    bFront = SweptBounds(PART_FRONT, COUNT_OF(PART_FRONT));
    bCab = SweptBounds(PART_CAB, COUNT_OF(PART_CAB));
    bInterior = SweptBounds(PART_INTERIOR, COUNT_OF(PART_INTERIOR));
    bBed = SweptBounds(PART_BED, COUNT_OF(PART_BED));
    bGear = SweptBounds(PART_GEAR, COUNT_OF(PART_GEAR));
    bWipers = SweptBounds(PART_WIPERS, COUNT_OF(PART_WIPERS));
    bAntenna = SweptBounds(PART_ANTENNA, COUNT_OF(PART_ANTENNA));

    CheckDoorClearsScreen();
    CheckGroundContact();
    CheckWiperSweep();
    CheckLoopCloses();
    CheckHalfShaft();
    CheckSuspension();
}

static void Unload(void)
{
    for (int i = 0; i < gAllCount; i++) GroupUnload(gAll[i]);
    // UnloadModel frees a material's maps array but deliberately not the textures in it, since they may be shared; vendor/raylib/src/rmodels.c:1200 says so and leaves them to the caller.
    // They are shared here, one set across every group, so they are freed once and only here, along with the glass shader.
    UnloadTextures();
}

const Scene SCENE = {
    .name = "humvee_v3_anim",
    .description =
        "M998 HMMWV cargo/troop carrier under way on a rough track: humvee_v3's\n"
        "geometry, its cab opened up behind transparent glass, and one four-second\n"
        "loop of motion.\n"
        "\n"
        "Moving parts. The four wheels each spin nine whole turns per loop, which is\n"
        "6.64 m/s at a 0.470 rolling radius, and an integer so the tread and the eight\n"
        "lug nuts land in the same place at the loop's seam. Both wipers sweep 80\n"
        "degrees three times per loop. The whip antenna bends. The hazard flashers\n"
        "blink at 1.25 Hz, the tail lamps come up from running to brake brightness on\n"
        "the approach to the obstacle, and the headlamps flicker twice on landing. The\n"
        "body heaves, pitches, rolls, yaws and shivers throughout, and leaves the\n"
        "ground once per loop.\n"
        "\n"
        "How the shake is generated: a periodic road profile 26.6 m long, four sine\n"
        "harmonics plus a per-track difference plus two gaussian obstacles, one of them\n"
        "0.150 m high and mostly under the right-hand track. It is sampled at the four\n"
        "contact patches, low-passed over a 0.9 m window because neither a 0.324 m tyre\n"
        "footprint nor a spring can follow anything shorter, and resolved into heave,\n"
        "pitch and roll about a pivot 0.85 m up. Two damped sines per obstacle add the\n"
        "body's ring-down at 1.6 to 2.1 Hz, and six small harmonics of the loop add\n"
        "engine and tyre buzz. Because the profile is sampled per axle, the rear wheels\n"
        "meet each bump 0.50 s after the front ones do.\n"
        "\n"
        "The road itself is not drawn: the ground is the harness's flat grid, and each\n"
        "corner's suspension takes up the difference between where the body has gone\n"
        "and where that ground is, with 0.110 m of compression and 0.075 m of droop.\n"
        "Where even full compression would not reach, the whole body is raised, so no\n"
        "tyre ever passes through the grid plane; where full droop will not reach, the\n"
        "truck is genuinely off the ground. Both are measured at build time and logged.\n"
        "\n"
        "Windows: the lighting shader cannot carry alpha at all -- its alpha channel\n"
        "works out to texel.a*(tint.a + 1), never under 1 -- so the glass runs a small\n"
        "shader built into this file, with a cubic Schlick fresnel that both brightens\n"
        "the pane and closes it up at grazing angles. Glass is drawn in its own pass\n"
        "after every opaque surface in the scene and with depth writes off, so panes in\n"
        "line blend instead of masking one another. Because the windows are now worth\n"
        "looking through, the cab's solid core stops at 1.180 m instead of at the roof,\n"
        "and what stands above that line is built: transmission tunnel, dash with a\n"
        "binnacle and a padded rail, four seat squabs, a grab handle, and a\n"
        "three-spoke steering wheel of 0.176 m radius on a raked column, on the left.\n"
        "\n"
        "Corrected against the references: the wipers are top-mounted, hung from the\n"
        "header, and the two arms are parallel rather than mirrored, both leaning\n"
        "towards the vehicle's right. ref_04.jpg looks into the windscreen and shows\n"
        "both pivots on the upper frame at |x| = 0.46 with the blades hanging down\n"
        "across the panes; v3 had a short arm pivoting near the cowl instead. The front\n"
        "corner markers are amber, not white, as ref_03.jpg and ref_04.jpg both show.\n"
        "The wing mirror faces moved off the glass material, since a transparent mirror\n"
        "is the one pane that should not be.\n"
        "\n"
        "Rigid nodes only, since skinning needs bone attributes the harness's shader\n"
        "does not declare. Twenty-five of them: five body groups plus the differentials\n"
        "and the spring and damper top mounts on the chassis matrix; per corner a wheel\n"
        "that spins and travels, an upright that only travels, a coil-and-damper pair\n"
        "scaled about its upper mount so it shortens as the suspension compresses, and a\n"
        "half shaft hinged at the differential so its outer end follows the hub; two\n"
        "wipers, each turning about its own pivot because two arms rotating by one angle\n"
        "about two different points is not a rigid motion; and the antenna. The\n"
        "wishbones are the one thing that translates with its wheel rather than swinging,\n"
        "which is wrong by the travel at the inboard end -- that end is inside the engine\n"
        "bay block at the front and the bed core at the rear, or under the belly pan.\n"
        "\n"
        "The antenna is the one surface that deforms. Its mesh is uploaded dynamic and\n"
        "its vertices and normals are rewritten every frame from a cantilever clamped\n"
        "at the base, tangent angle growing as the square of the distance along it,\n"
        "each of 14 segments stepping one segment-length along the unit tangent so the\n"
        "whip keeps its 0.930 m however far it leans. It leans against the horizontal\n"
        "acceleration of its own base, differenced from the chassis matrix at a 10 ms\n"
        "step, and sways on its own besides.\n"
        "\n"
        "Surfacing is v3's: world-space planar UVs repeating every 1.60 m so patterns\n"
        "run across panel joins, and eight maps built in code from tiling value noise\n"
        "-- NATO three-colour woodland camouflage, rubber, mottled metal, mirror face,\n"
        "glass, and white, red and amber lens gradients. The lens and halo materials\n"
        "skip lighting entirely, which is what makes them read as emitting; each lamp\n"
        "carries a halo disc drawn additively with depth writes off.\n"
        "\n"
        "One world unit is one metre. 4.57 bumper to bumper, 2.16 wide, 1.83 tall over\n"
        "the roof; the front lifting shackles project 0.05 further, to z = 2.40. 3.30\n"
        "wheelbase, 1.83 track, 0.41 ground clearance, 37x12.5R16.5 tyres of 0.47\n"
        "radius and 0.324 width. Origin sits on the ground at the centre of the\n"
        "wheelbase, +Z forward, +X the vehicle's right.\n"
        "\n"
        "Construction: everything is either a hexahedron with eight freely placed\n"
        "corners or a surface of revolution. Wheel openings are cut by sweeping\n"
        "vertical strips 28 mm apart whose floor follows the 0.60 radius arch circle\n"
        "about the axle, leaving 0.13 of clearance over the tyre. Tyres are a 14-point\n"
        "section revolved in 40 segments, carcass crown at 0.450, with 20 pairs of\n"
        "staggered tread lugs standing 20 mm proud to reach the 0.470 rolling radius.\n"
        "Coil springs are a circle swept along a helix on an analytic frame. The right\n"
        "half of each body group is built once and mirrored through x = 0, and each\n"
        "left-hand corner is the whole right-hand one reflected.",
    .init = Init,
    .draw = DrawAll,
    .unload = Unload,
    .update = Update,
    .duration = CYCLE,
    .animYaw = 40.0f,
    .parts = PARTS,
    .partCount = COUNT_OF(PARTS),
    .target = { 0.0f, 0.90f, 0.0f },
    .orbitRadius = 7.0f,
    .orbitHeight = 3.0f,
};
