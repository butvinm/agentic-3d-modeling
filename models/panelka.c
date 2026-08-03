#include "harness.h"
#include "raymath.h"
#include "rlgl.h"
// Declarations only: harness.c is the translation unit that defines RLIGHTS_IMPLEMENTATION, and this file only adds a light to the shader it already set up.
#include "rlights.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// A three-section 1-464 khrushchyovka standing in its own courtyard.
//
// The building is a set of precast panels rather than a shell with holes cut in it, and that is the one decision the rest of the file follows from. A panel block's identity is the grid of joints between its panels, which comes out of real panels for free and out of a lofted facade only by drawing it on; and the demolition this model exists to show needs those same panels as the rigid bodies it throws, so the static building and the moving one share a single decomposition instead of carrying two.
//
// So there are no facade meshes. There is one mesh per panel *type* -- a 2.6 m bay with a window, a 3.2 m bay with a balcony door, an end-wall panel -- and a list of transforms that says where each of the 700-odd instances of those types sits. Everything the harness needs (bounds, part isolation, unloading) then works on the list rather than on the building.
//
// Dimensions come from references/panelka/, and the two that matter are documented rather than measured: the series' outer panels are 2.6 and 3.2 m wide and its floor slabs span 5.76 m.
// Scaling ref_01.png (a plan of one section) by that span gives 66.9 px/m, and reading the section back off that scale returns seven bays of 2.69, 3.24, 2.64, 2.57, 2.54, 3.23 and 2.66 m over an 11.52 m depth: the nominal panel widths to within 3.5 per cent. Two independent documented numbers agreeing is what makes the scale worth building on.
// ---------------------------------------------------------------------------

// Metres per repeat of the surface maps. Concrete wants a coarse aggregate mottle at roughly half a metre, which puts a 512-pixel map at about 2 mm per texel.
#define TEX_REPEAT_M  1.00f

// ---------------------------------------------------------------------------
// Mesh builder
//
// Every surface here is either a hexahedron -- a box whose eight corners may each sit anywhere,
// so it also covers wedges, tapers and sloped panels -- or a surface of revolution. Both accumulate into a growing vertex/index buffer uploaded once per material when a group closes.
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

// Project the position onto the two axes the normal is weakest in, which is the plane the face most nearly lies in and so the one that stretches it least. The normal is constant across a flat face, so the projection is continuous over each panel and across any two panels facing the same way.
//
// The projection is in the *group's own* frame, not the world's, because almost every surface here belongs to an instanced mesh that is drawn in several hundred places: a world-space projection would need a world position the mesh does not have. The cost is that every instance of a type carries an identical texture placement, which is what the per-instance tint below exists to break up.
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
    if (x1 - x0 < 1e-5f || y1 - y0 < 1e-5f || z1 - z0 < 1e-5f) return;
    Vector3 c[8] = {
        { x0, y0, z0 }, { x1, y0, z0 }, { x1, y0, z1 }, { x0, y0, z1 },
        { x0, y1, z0 }, { x1, y1, z0 }, { x1, y1, z1 }, { x0, y1, z1 },
    };
    Hex(b, c);
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

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

typedef enum {
    MAT_PANEL, MAT_CONCRETE, MAT_PLINTH, MAT_ROOF,
    MAT_GLASS, MAT_FRAME, MAT_METAL, MAT_TIMBER,
    MAT_GRASS, MAT_ASPHALT,
    MAT_BARK, MAT_BIRCH, MAT_LEAF, MAT_PAINT, MAT_RUBBER,
    MAT_COUNT
} MatId;

typedef enum { PASS_OPAQUE, PASS_COUNT } Pass;

static const Pass MAT_PASS[MAT_COUNT] = { 0 };

// Every surface carries a map, so the colour lives in the texels and the material tint stays white; multiplying a coloured map by a coloured tint would darken it twice.
static const Color MAT_COLOR[MAT_COUNT] = {
    [MAT_PANEL]    = WHITE,
    [MAT_CONCRETE] = WHITE,
    [MAT_PLINTH]   = WHITE,
    [MAT_ROOF]     = WHITE,
    [MAT_GLASS]    = WHITE,
    [MAT_FRAME]    = WHITE,
    [MAT_METAL]    = WHITE,
    [MAT_TIMBER]   = WHITE,
    [MAT_GRASS]    = WHITE,
    [MAT_ASPHALT]  = WHITE,
    [MAT_BARK]     = WHITE,
    [MAT_BIRCH]    = WHITE,
    [MAT_LEAF]     = WHITE,
    // The one map authored white on purpose: a car's colour is the per-instance tint, so four
    // cars are four tints on one mesh rather than four meshes.
    [MAT_PAINT]    = WHITE,
    [MAT_RUBBER]   = WHITE,
};

static const bool MAT_UNLIT[MAT_COUNT] = { 0 };

static Texture2D MAT_TEX[MAT_COUNT];

// ---------------------------------------------------------------------------
// Procedurally generated surface maps
//
// Value noise on an integer lattice that wraps at the lattice period, so every map tiles and the UVs can run off to any distance without showing a join.
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

// Painted precast concrete: a fine paint speckle over a coarser blotching that reads as the weathering and patch repairs every one of these buildings carries by its fifth decade.
// Authored dark, because lighting.fs gamma-corrects with pow(c, 1/2.2) and a texel of 110 leaves the shader at about 0.72 before any light reaches it.
static Texture2D MakeRenderTexture(Color base, unsigned int seed, float blotch, float speckle)
{
    const int S = 512;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float d = (Fbm(u, v, 4, seed, 4) - 0.5f) * blotch
                    + (Fbm(u, v, 128, seed + 13u, 2) - 0.5f) * speckle;
            px[y * S + x] = Shade(base, d);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Structural concrete seen where the skin has come off: greyer than the painted face, with the aggregate showing as a coarser mottle and a faint horizontal banding from the casting bed.
static Texture2D MakeConcreteTexture(void)
{
    const int S = 512;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float grain = (Fbm(u, v, 96, 41u, 3) - 0.5f) * 26.0f;
            float band = sinf(v * 2.0f * PI * 6.0f) * 3.0f;
            px[y * S + x] = Shade((Color){ 96, 95, 90, 255 }, grain + band);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Window glazing seen from outside on an overcast day: nearly black, with a slow vertical gradient standing in for the sky it reflects and a faint mottle for the glass itself.
// The gradient runs with the map's v, which is the panel's own height, so every pane carries the same falloff rather than one shared with the world.
static Texture2D MakeGlassTexture(void)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float sky = (1.0f - v) * 26.0f;
            float mottle = (Fbm(u, v, 24, 91u, 2) - 0.5f) * 10.0f;
            px[y * S + x] = Shade((Color){ 24, 29, 36, 255 }, sky + mottle);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Built-up bitumen roofing: near black, with the roll joints running one way.
static Texture2D MakeRoofTexture(void)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float grain = (Fbm(u, v, 64, 17u, 2) - 0.5f) * 14.0f;
            float roll = (fmodf(v * 4.0f, 1.0f) < 0.04f) ? -6.0f : 0.0f;
            px[y * S + x] = Shade((Color){ 62, 60, 58, 255 }, grain + roll);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Painted steelwork: railings, doors, posts. A mottle plus scattered rust blooms.
static Texture2D MakeMetalTexture(void)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float rust = Fbm(u, v, 8, 63u, 3);
            Color base = (rust > 0.68f) ? (Color){ 84, 52, 34, 255 } : (Color){ 62, 66, 68, 255 };
            px[y * S + x] = Shade(base, (Fbm(u, v, 64, 29u, 2) - 0.5f) * 18.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

static Texture2D MakeTimberTexture(void)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float grain = sinf(u * 2.0f * PI * 3.0f + Fbm(u, v, 8, 5u, 2) * 9.0f) * 7.0f;
            px[y * S + x] = Shade((Color){ 68, 54, 38, 255 }, grain + (Fbm(u, v, 96, 7u, 2) - 0.5f) * 12.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

static Texture2D MakeGrassTexture(void)
{
    const int S = 512;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float patch = Fbm(u, v, 6, 23u, 3);
            Color base = (patch > 0.56f) ? (Color){ 54, 62, 34, 255 } : (Color){ 62, 60, 42, 255 };
            px[y * S + x] = Shade(base, (Fbm(u, v, 160, 71u, 2) - 0.5f) * 22.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

static Texture2D MakeAsphaltTexture(void)
{
    const int S = 512;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float grit = (Fbm(u, v, 200, 37u, 2) - 0.5f) * 20.0f;
            float patch = (Fbm(u, v, 5, 97u, 3) - 0.5f) * 12.0f;
            px[y * S + x] = Shade((Color){ 50, 50, 52, 255 }, grit + patch);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Bark. A birch is the one tree whose surface is a graphic rather than a texture: pale, with
// black lenticel dashes across it. Everything else gets vertical furrows.
static Texture2D MakeBarkTexture(Color base, bool birch)
{
    const int S = 256;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float d;
            if (birch) {
                float dash = Fbm(u * 3.0f, v * 26.0f, 16, 211u, 2);
                d = (dash < 0.31f) ? -96.0f : (Fbm(u, v, 48, 19u, 2) - 0.5f) * 14.0f;
            } else {
                float furrow = sinf(u * 2.0f * PI * 9.0f + Fbm(u, v, 8, 31u, 3) * 7.0f);
                d = furrow * 13.0f + (Fbm(u, v, 96, 43u, 2) - 0.5f) * 16.0f;
            }
            px[y * S + x] = Shade(base, d);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

// Foliage. A crown here is a handful of lumpy blobs rather than leaves, so the map has to carry
// the leaf-scale break-up the geometry does not: a fine mottle over a coarser one, so a crown
// reads as a mass of leaves at a distance rather than as a green boulder.
static Texture2D MakeLeafTexture(void)
{
    const int S = 512;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float mass = Fbm(u, v, 6, 67u, 3);
            Color base = (mass > 0.54f) ? (Color){ 58, 74, 36, 255 } : (Color){ 42, 56, 28, 255 };
            px[y * S + x] = Shade(base, (Fbm(u, v, 180, 101u, 2) - 0.5f) * 34.0f);
        }
    }
    return Upload(img, TEXTURE_WRAP_REPEAT);
}

static void MakeTextures(void)
{
    MAT_TEX[MAT_PANEL]    = MakeRenderTexture((Color){ 118, 112, 100, 255 }, 3u, 20.0f, 12.0f);
    MAT_TEX[MAT_CONCRETE] = MakeConcreteTexture();
    MAT_TEX[MAT_PLINTH]   = MakeRenderTexture((Color){ 66, 64, 60, 255 }, 51u, 16.0f, 14.0f);
    MAT_TEX[MAT_ROOF]     = MakeRoofTexture();
    MAT_TEX[MAT_GLASS]    = MakeGlassTexture();
    MAT_TEX[MAT_FRAME]    = MakeRenderTexture((Color){ 146, 145, 138, 255 }, 83u, 8.0f, 10.0f);
    MAT_TEX[MAT_METAL]    = MakeMetalTexture();
    MAT_TEX[MAT_TIMBER]   = MakeTimberTexture();
    MAT_TEX[MAT_GRASS]    = MakeGrassTexture();
    MAT_TEX[MAT_ASPHALT]  = MakeAsphaltTexture();
    MAT_TEX[MAT_BARK]     = MakeBarkTexture((Color){ 62, 54, 44, 255 }, false);
    MAT_TEX[MAT_BIRCH]    = MakeBarkTexture((Color){ 176, 172, 160, 255 }, true);
    MAT_TEX[MAT_LEAF]     = MakeLeafTexture();
    MAT_TEX[MAT_PAINT]    = MakeRenderTexture((Color){ 196, 194, 190, 255 }, 137u, 5.0f, 6.0f);
    MAT_TEX[MAT_RUBBER]   = MakeRenderTexture((Color){ 30, 30, 32, 255 }, 149u, 6.0f, 10.0f);
}

static void UnloadTextures(void)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (MAT_TEX[m].id != 0) UnloadTexture(MAT_TEX[m]);
    }
}

// ---------------------------------------------------------------------------
// Groups
//
// A group is one set of per-material meshes sharing a transform and a tint, plus the local bounding box that transform acts on. Two kinds exist here and the difference matters.
//
// A *placed* group is a thing that exists once: the ground, the plinth, a particular tree. It draws at its own xform.
//
// An *instanced* group is a thing that exists in hundreds of places and differs between them only by where it is: a 2.6 m facade panel, a floor slab. It carries one mesh and a list of transforms, and a matching list of tints, because every instance sharing one texture placement is the price of instancing and a per-instance shade is what buys it back.
// ---------------------------------------------------------------------------

typedef struct {
    Builder b[MAT_COUNT];
    Model model[MAT_COUNT];
    bool has[MAT_COUNT];
    BoundingBox bounds;      // in the group's own frame, before xform
    Matrix xform;
    Color tint[MAT_COUNT];
    const Matrix *inst;      // if set, the meshes are drawn once per instance instead of at xform
    const Color *instTint;   // optional, one per instance; multiplies the material tint
    int instCount;
} Group;

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
    if (!any) g->bounds = (BoundingBox){ 0 };
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
        UploadMesh(&mesh, false);

        g->model[m] = LoadModelFromMesh(mesh);
        g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].color = MAT_COLOR[m];
        if (MAT_TEX[m].id != 0) g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = MAT_TEX[m];
        if (!MAT_UNLIT[m]) HarnessApplyLighting(&g->model[m]);
        g->has[m] = true;
    }

    GroupBoundsFromMesh(g);
}

static Color TintMul(Color a, Color b)
{
    return (Color){
        (unsigned char)(((int)a.r * (int)b.r) / 255),
        (unsigned char)(((int)a.g * (int)b.g) / 255),
        (unsigned char)(((int)a.b * (int)b.b) / 255),
        (unsigned char)(((int)a.a * (int)b.a) / 255),
    };
}

// One pass over one group. The caller owns the blend and depth state, because a pass spans every group in the scene rather than stopping at this one.
static void GroupDrawPass(Group *g, Pass pass)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (!g->has[m] || MAT_PASS[m] != pass) continue;

        if (g->instCount > 0) {
            for (int i = 0; i < g->instCount; i++) {
                Color t = g->instTint ? TintMul(g->tint[m], g->instTint[i]) : g->tint[m];
                if (t.a == 0) continue;
                g->model[m].transform = g->inst[i];
                DrawModel(g->model[m], Vector3Zero(), 1.0f, t);
            }
            continue;
        }

        g->model[m].transform = g->xform;
        DrawModel(g->model[m], Vector3Zero(), 1.0f, g->tint[m]);
    }
}

// Every group in the scene, in build order. The draw passes walk this rather than the part list, so a pass really does cover the whole model.
#define MAX_GROUPS 64
static Group *gAll[MAX_GROUPS];
static int gAllCount;

static void GroupRegister(Group *g)
{
    if (gAllCount < MAX_GROUPS) gAll[gAllCount++] = g;
    else TraceLog(LOG_ERROR, "panelka: more than %d groups", MAX_GROUPS);
}

static void DrawGroups(Group *const *gs, int n)
{
    for (int i = 0; i < n; i++) GroupDrawPass(gs[i], PASS_OPAQUE);
}

static void GroupUnload(Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (g->has[m]) UnloadModel(g->model[m]);
    }
}

// ---------------------------------------------------------------------------
// Dimensions, in metres, from the 1-464 series.
//
// Origin sits on the ground at the centre of the block's footprint; +X runs along the block,
// +Z out through the main facade, +Y up.
//
// The two numbers everything else hangs off are documented rather than measured: outer panels 2.6 and 3.2 m wide, and a 5.76 m floor-slab span. The span is centre-to-centre from an outer wall to the spine, so the depth over the outer faces is 2 x 5.76 + one wall thickness, and ref_01.png measures 11.52 m between those two wall centre-lines, which is the same statement.
//
// Storey height is 2.5 m of clear ceiling (specified) plus a 0.10 m slab (specified) plus 0.10 m of floor build-up, so 2.70.
// ---------------------------------------------------------------------------

#define SECTIONS      3
#define FLOORS        5
#define BAYS          7

// Bay widths across one section, stairwell in the middle. Sum is SECTION_LEN, which CheckBaysTile proves rather than assumes.
static const float BAY_W[BAYS] = { 2.6f, 3.2f, 2.6f, 2.6f, 2.6f, 3.2f, 2.6f };
#define BAY_STAIR     3       // index of the stairwell bay
#define BAY_BALC_A    1       // the two 3.2 m bays, which are the ones that carry balconies
#define BAY_BALC_B    5

#define SECTION_LEN   19.4f
#define BLOCK_LEN     (SECTIONS * SECTION_LEN)

#define STOREY        2.700f
#define SLAB_T        0.220f   // structural slab plus screed
#define SLAB_BEAR     0.100f   // how far a slab's end is built into the facade panel it lands on
#define PLINTH_Y      0.750f   // ground-floor level above grade
#define WALL_T        0.300f   // outer panel: the series specifies 0.21 to 0.35 by climate zone
#define XWALL_T       0.140f   // internal cross wall, the load-bearing direction
#define SPINE_T       0.160f   // central longitudinal wall
#define HALF_D        5.760f   // slab span: outer wall centre-line to the spine centre-line
#define FACE_Z        (HALF_D + WALL_T * 0.5f)   // 5.91, outer face of the facade
#define INNER_Z       (HALF_D - WALL_T * 0.5f)   // 5.61, inner face

#define JOINT         0.030f   // half the groove between two panels: each face plate insets by this
// How far every piece is grown past its own edge so that it knits into its neighbour instead of
// meeting it on an exactly coincident plane. Two coplanar back-to-back faces are the one thing
// this file draws hundreds of, and left touching they speckle along every joint: the rasterizer
// picks between them per pixel and the facade comes out ruled with dashed lines. It is also the
// honest construction, since a real panel is butted and caulked rather than laid edge to edge.
#define KNIT          0.006f
#define FACE_T        0.040f   // how far the face plate stands proud of the panel core
#define CORE_T        (WALL_T - FACE_T)

#define ROOF_Y        (PLINTH_Y + FLOORS * STOREY)      // 14.25, top of the fifth-floor ceiling
#define PARAPET_H     0.450f
#define PARAPET_T     0.200f

// Openings. Sills are measured up from the floor the panel stands on.
#define WIN_SILL      0.850f
#define WIN_H         1.450f
#define WIN_W26       1.300f
#define WIN_W32       1.500f
#define BDOOR_W       0.750f
#define BDOOR_SILL    0.100f
#define BDOOR_H       2.100f
#define STAIR_W       0.950f
#define STAIR_SILL    1.000f
#define STAIR_H       1.300f
#define ENTRY_W       1.400f
#define ENTRY_H       2.150f

#define BALC_D        1.000f   // how far a balcony slab projects past the facade
#define BALC_W        3.000f
#define BALC_T        0.140f
#define BALC_RAIL_H   1.050f

// Where a section's centre sits along X.
static float SectionX(int s) { return ((float)s - (float)(SECTIONS - 1) * 0.5f) * SECTION_LEN; }

// Centre of bay b of section s, along X.
static float BayX(int s, int b)
{
    float x = SectionX(s) - SECTION_LEN * 0.5f;
    for (int i = 0; i < b; i++) x += BAY_W[i];
    return x + BAY_W[b] * 0.5f;
}

static float FloorY(int f) { return PLINTH_Y + (float)f * STOREY; }

// ---------------------------------------------------------------------------
// Panels
//
// A panel is authored in its own frame: x runs -w/2 to +w/2 across it, y runs 0 to h up from the floor it stands on, and z runs 0 at the inner face to WALL_T at the outer one. Placing it is then a yaw and a translation of that origin, which is what lets one mesh stand in 105 places and what keeps a panel's openings derived from the panel rather than from the world.
// ---------------------------------------------------------------------------

typedef struct { float x0, x1, y0, y1; } Rect;

// One layer of a wall panel: a slab of thickness z0..z1 spanning the panel inset by `inset` on all four edges, with rectangular openings cut out of it.
//
// Decomposed into horizontal bands at every opening edge and, within a band, into the x-segments the openings leave. Doing it that way rather than case by case means no opening count needs its own code, and every reveal face comes out as the side of a closed box rather than as a hole that has to be stitched.
//
// Openings must be listed left to right; CheckPanelOpenings proves they are.
static void WallLayer(Builder *b, float w, float h, float z0, float z1, float inset,
                      const Rect *op, int nop)
{
    float x0 = -w * 0.5f + inset, x1 = w * 0.5f - inset;
    float y0 = inset, y1 = h - inset;

    float ys[10];
    int ny = 0;
    ys[ny++] = y0;
    ys[ny++] = y1;
    for (int i = 0; i < nop; i++) { ys[ny++] = op[i].y0; ys[ny++] = op[i].y1; }
    for (int i = 1; i < ny; i++) {
        float v = ys[i];
        int j = i - 1;
        while (j >= 0 && ys[j] > v) { ys[j + 1] = ys[j]; j--; }
        ys[j + 1] = v;
    }

    for (int k = 0; k + 1 < ny; k++) {
        float ya = ys[k], yb = ys[k + 1];
        if (yb - ya < 1e-4f) continue;
        if (ya < y0 - 1e-4f || yb > y1 + 1e-4f) continue;
        float ym = 0.5f * (ya + yb);

        float cur = x0;
        for (int i = 0; i < nop; i++) {
            if (op[i].y0 > ym || op[i].y1 < ym) continue;
            float a = Clamp(op[i].x0, x0, x1), c = Clamp(op[i].x1, x0, x1);
            if (a > cur) Box(b, cur, a, ya, yb, z0, z1);
            if (c > cur) cur = c;
        }
        if (cur < x1) Box(b, cur, x1, ya, yb, z0, z1);
    }
}

// A whole facade panel: the structural core across its full width, plus a face plate standing FACE_T proud and inset JOINT all round. The inset is the joint: two neighbouring panels leave a 2*JOINT groove between their plates, which is the grid that makes a panel block read as one rather than as a rendered wall with windows in it.
static void FacadePanel(Group *g, float w, float h, const Rect *op, int nop)
{
    WallLayer(&g->b[MAT_PANEL], w, h, -KNIT, CORE_T, -KNIT, op, nop);
    WallLayer(&g->b[MAT_PANEL], w, h, CORE_T, WALL_T, JOINT, op, nop);
}

// The joinery that fills an opening: a frame around it, a centre mullion, a transom near the top, and a pane behind all three. Authored in the same frame as its panel, so it rides the same placement matrix, and recessed so the reveal reads as a reveal.
#define GLZ_Z0        (CORE_T - 0.060f)
#define GLZ_Z1        (CORE_T - 0.010f)
#define GLZ_BAR       0.055f

static void Glazing(Group *g, Rect r, bool transom)
{
    Builder *f = &g->b[MAT_FRAME];
    float ix0 = r.x0 + GLZ_BAR, ix1 = r.x1 - GLZ_BAR;
    float iy0 = r.y0 + GLZ_BAR, iy1 = r.y1 - GLZ_BAR;

    Box(f, r.x0, r.x1, r.y0, iy0, GLZ_Z0, GLZ_Z1);
    Box(f, r.x0, r.x1, iy1, r.y1, GLZ_Z0, GLZ_Z1);
    Box(f, r.x0, ix0, iy0, iy1, GLZ_Z0, GLZ_Z1);
    Box(f, ix1, r.x1, iy0, iy1, GLZ_Z0, GLZ_Z1);

    float mx = 0.5f * (r.x0 + r.x1);
    Box(f, mx - GLZ_BAR * 0.5f, mx + GLZ_BAR * 0.5f, iy0, iy1, GLZ_Z0, GLZ_Z1);
    if (transom) {
        float ty = iy1 - (iy1 - iy0) * 0.28f;
        Box(f, ix0, ix1, ty - GLZ_BAR * 0.5f, ty + GLZ_BAR * 0.5f, GLZ_Z0, GLZ_Z1);
    }

    // The pane sits behind the frame rather than flush with it, so the frame casts across it.
    Box(&g->b[MAT_GLASS], ix0, ix1, iy0, iy1, GLZ_Z0 - 0.030f, GLZ_Z0 - 0.010f);
}

// A window sill: the one moulding on a khrushchyovka facade that is not flat, and the only thing that stops a window reading as a sticker.
static void Sill(Group *g, Rect r)
{
    Box(&g->b[MAT_PANEL], r.x0 - 0.060f, r.x1 + 0.060f, r.y0 - 0.070f, r.y0, WALL_T - 0.020f, WALL_T + 0.060f);
}

// ---------------------------------------------------------------------------
// Fragments
//
// Every piece of the building is an instance of one of these types. A fragment record says which type and where its rest pose is; Update turns the whole list into transforms, so the intact building and the collapsing one differ only in what Update writes.
// ---------------------------------------------------------------------------

typedef enum {
    FR_P26, FR_P32, FR_P32B, FR_PSTAIR, FR_PDOOR, FR_PEND,
    FR_G26, FR_G32, FR_G32B, FR_GSTAIR, FR_GDOOR,
    FR_SLAB26, FR_SLAB32, FR_ROOF26, FR_ROOF32,
    FR_XWALL, FR_SPINE26, FR_SPINE32,
    FR_BALC, FR_PAR26, FR_PAR32, FR_PAREND, FR_CANOPY, FR_VENT,
    FR_BIRCH, FR_MAPLE, FR_CARBODY, FR_CARTRIM, FR_BENCH, FR_RUGFRAME, FR_LAMP, FR_BIN,
    FR_COUNT
} FragType;

typedef struct {
    FragType type;
    Vector3 pos;     // world position of the fragment's local origin, at rest
    float yaw;       // rotation about Y that takes the local frame to the world one, in degrees
    float scale;     // uniform, so one tree mesh can stand in eight sizes
    Color tint;      // per-instance, which is how four cars share one body mesh
    float shade;     // per-instance brightness, so 105 copies of one mesh are not 105 identical panels
} Fragment;

#define MAX_FRAG 1400
static Fragment gFrag[MAX_FRAG];
static int gFragCount;

static Matrix gFragMat[MAX_FRAG];
static Color gFragTint[MAX_FRAG];
static int gTypeStart[FR_COUNT];
static int gTypeCount[FR_COUNT];
static Group gType[FR_COUNT];

static Fragment *Emit(FragType t, float x, float y, float z, float yaw)
{
    if (gFragCount >= MAX_FRAG) {
        TraceLog(LOG_ERROR, "panelka: more than %d fragments", MAX_FRAG);
        return NULL;
    }
    Fragment *f = &gFrag[gFragCount++];
    f->type = t;
    f->pos = (Vector3){ x, y, z };
    f->yaw = yaw;
    f->scale = 1.0f;
    f->tint = WHITE;
    // Hashed off the rest position, so a panel keeps its shade wherever it ends up and two neighbours never draw the same.
    f->shade = Hash2((int)floorf(x * 4.0f), (int)floorf(y * 4.0f + z * 13.0f), 4096, 5u);
    return f;
}

// Same, for anything whose instances differ by more than where they are.
static Fragment *EmitAt(FragType t, float x, float y, float z, float yaw, float scale, Color tint)
{
    Fragment *f = Emit(t, x, y, z, yaw);
    if (f) { f->scale = scale; f->tint = tint; }
    return f;
}

static Matrix FragRest(const Fragment *f)
{
    return MatrixMultiply(MatrixMultiply(MatrixScale(f->scale, f->scale, f->scale),
                                         MatrixRotateY(f->yaw * DEG2RAD)),
                          MatrixTranslate(f->pos.x, f->pos.y, f->pos.z));
}

// ---------------------------------------------------------------------------
// Building the fragment meshes
// ---------------------------------------------------------------------------

static Rect WinRect(float winW)
{
    return (Rect){ -winW * 0.5f, winW * 0.5f, WIN_SILL, WIN_SILL + WIN_H };
}

static void BuildFacadeTypes(void)
{
    // A plain bay: one window, centred.
    Rect w26 = WinRect(WIN_W26);
    FacadePanel(&gType[FR_P26], 2.6f, STOREY, &w26, 1);
    Sill(&gType[FR_P26], w26);
    Glazing(&gType[FR_G26], w26, true);

    Rect w32 = WinRect(WIN_W32);
    FacadePanel(&gType[FR_P32], 3.2f, STOREY, &w32, 1);
    Sill(&gType[FR_P32], w32);
    Glazing(&gType[FR_G32], w32, true);

    // A balcony bay: the door on the left, a window beside it. Listed left to right, which WallLayer requires and CheckPanelOpenings proves.
    Rect bal[2] = {
        { -1.300f, -1.300f + BDOOR_W, BDOOR_SILL, BDOOR_SILL + BDOOR_H },
        { 0.100f, 0.100f + 1.200f, WIN_SILL, WIN_SILL + WIN_H },
    };
    FacadePanel(&gType[FR_P32B], 3.2f, STOREY, bal, 2);
    Sill(&gType[FR_P32B], bal[1]);
    Glazing(&gType[FR_G32B], bal[0], false);
    Glazing(&gType[FR_G32B], bal[1], true);

    // The stairwell: one tall narrow light per storey, its sill well above an apartment's because the landing it serves is half a flight up. The real thing staggers a window per half-flight; this is one per storey, which is as far as a one-storey panel can go.
    Rect st = { -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL, STAIR_SILL + STAIR_H };
    FacadePanel(&gType[FR_PSTAIR], 2.6f, STOREY, &st, 1);
    Glazing(&gType[FR_GSTAIR], st, false);

    // The entrance: a doorway to the floor, with a fanlight over it.
    Rect en = { -ENTRY_W * 0.5f, ENTRY_W * 0.5f, 0.0f, ENTRY_H };
    FacadePanel(&gType[FR_PDOOR], 2.6f, STOREY, &en, 1);
    {
        Builder *m = &gType[FR_GDOOR].b[MAT_METAL];
        // Two leaves with a rebate between them, set back in the reveal, and a fanlight above.
        float leafTop = ENTRY_H - 0.500f;
        Box(m, en.x0 + 0.040f, -0.020f, 0.020f, leafTop, GLZ_Z0, GLZ_Z1);
        Box(m, 0.020f, en.x1 - 0.040f, 0.020f, leafTop, GLZ_Z0, GLZ_Z1);
        Box(m, en.x0, en.x1, leafTop, leafTop + 0.070f, GLZ_Z0, GLZ_Z1);
        Box(&gType[FR_GDOOR].b[MAT_GLASS], en.x0 + 0.060f, en.x1 - 0.060f,
            leafTop + 0.070f, en.y1 - 0.040f, GLZ_Z0 - 0.030f, GLZ_Z0 - 0.010f);
        // Handles, on the meeting stiles.
        Box(m, -0.130f, -0.060f, 1.000f, 1.120f, GLZ_Z1, GLZ_Z1 + 0.060f);
        Box(m, 0.060f, 0.130f, 1.000f, 1.120f, GLZ_Z1, GLZ_Z1 + 0.060f);

        // A bulkhead lamp over the door and the enamel number plate beside it. Both hang off ENTRY_H rather than off a copy of it, so they follow the doorway if it ever changes.
        float lampY = ENTRY_H + 0.180f;
        Tube(m, (Vector3){ 0.0f, lampY, WALL_T }, (Vector3){ 0.0f, lampY, WALL_T + 0.150f }, 0.075f, 0.115f, 12, false, false);
        Box(&gType[FR_GDOOR].b[MAT_GLASS], -0.105f, 0.105f, lampY - 0.105f, lampY + 0.105f, WALL_T + 0.148f, WALL_T + 0.160f);
        Box(&gType[FR_GDOOR].b[MAT_FRAME], 0.520f, 0.880f, ENTRY_H - 0.240f, ENTRY_H + 0.020f, WALL_T, WALL_T + 0.018f);
    }

    // The end wall: two blank panels per floor, each spanning half the depth. Blank is the point -- ref_02, ref_05 and ref_07 all show a gable with no openings at all, which is why ref_04 exists to show one painted instead.
    FacadePanel(&gType[FR_PEND], FACE_Z, STOREY, NULL, 0);
}

// A floor slab, authored about the centre of its bay at the underside of the structural slab.
// It spans from the inner face of the facade to the centre of the spine wall, which is the 5.76 m the series casts its slabs to.
static void BuildSlab(Group *g, float w, MatId top)
{
    float z0 = -(INNER_Z + SLAB_BEAR), z1 = KNIT;
    Box(&g->b[MAT_CONCRETE], -w * 0.5f - KNIT, w * 0.5f + KNIT, 0.0f, SLAB_T - 0.030f, z0, z1);
    Box(&g->b[top], -w * 0.5f - KNIT, w * 0.5f + KNIT, SLAB_T - 0.030f, SLAB_T, z0, z1);
}

static void BuildStructureTypes(void)
{
    BuildSlab(&gType[FR_SLAB26], 2.6f, MAT_CONCRETE);
    BuildSlab(&gType[FR_SLAB32], 3.2f, MAT_CONCRETE);
    BuildSlab(&gType[FR_ROOF26], 2.6f, MAT_ROOF);
    BuildSlab(&gType[FR_ROOF32], 3.2f, MAT_ROOF);

    // The cross wall: the load-bearing direction in this series, spanning the whole depth,
    // authored about its own centre line at the floor it stands on.
    Box(&gType[FR_XWALL].b[MAT_CONCRETE], -XWALL_T * 0.5f, XWALL_T * 0.5f,
        -KNIT, STOREY - SLAB_T + KNIT, -INNER_Z - KNIT, INNER_Z + KNIT);

    // The spine: one segment per bay, so it breaks with the bay rather than across it.
    Box(&gType[FR_SPINE26].b[MAT_CONCRETE], -1.3f - KNIT, 1.3f + KNIT, -KNIT, STOREY - SLAB_T + KNIT, -SPINE_T * 0.5f, SPINE_T * 0.5f);
    Box(&gType[FR_SPINE32].b[MAT_CONCRETE], -1.6f - KNIT, 1.6f + KNIT, -KNIT, STOREY - SLAB_T + KNIT, -SPINE_T * 0.5f, SPINE_T * 0.5f);

    // Parapet segments, authored about the centre of the bay they cap, at roof level.
    Box(&gType[FR_PAR26].b[MAT_CONCRETE], -1.3f - KNIT, 1.3f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);
    Box(&gType[FR_PAR32].b[MAT_CONCRETE], -1.6f - KNIT, 1.6f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);
    Box(&gType[FR_PAREND].b[MAT_CONCRETE], -FACE_Z * 0.5f - KNIT, FACE_Z * 0.5f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);

    // A balcony: a slab cantilevered past the facade, a kerb round its edge, and a railing of uprights between two rails. Authored about the facade's outer face at floor level, so it hangs off the panel rather than off a copy of the panel's coordinates.
    {
        Group *g = &gType[FR_BALC];
        Builder *c = &g->b[MAT_CONCRETE];
        float hw = BALC_W * 0.5f;
        // The slab's root runs 0.20 back into the facade, which is where it is cast in.
        Box(c, -hw, hw, -BALC_T, 0.0f, -0.200f, BALC_D);
        Box(c, -hw, -hw + 0.080f, 0.0f, 0.120f, 0.0f, BALC_D);
        Box(c, hw - 0.080f, hw, 0.0f, 0.120f, 0.0f, BALC_D);
        Box(c, -hw, hw, 0.0f, 0.120f, BALC_D - 0.080f, BALC_D);

        Builder *m = &g->b[MAT_METAL];
        float rt = BALC_RAIL_H;
        Box(m, -hw, hw, rt - 0.050f, rt, 0.0f, 0.050f);
        Box(m, -hw, hw, rt - 0.050f, rt, BALC_D - 0.050f, BALC_D);
        Box(m, -hw, -hw + 0.050f, rt - 0.050f, rt, 0.0f, BALC_D);
        Box(m, hw - 0.050f, hw, rt - 0.050f, rt, 0.0f, BALC_D);
        for (int i = 0; i <= 12; i++) {
            float x = -hw + BALC_W * (float)i / 12.0f;
            Box(m, x - 0.014f, x + 0.014f, 0.120f, rt - 0.050f, BALC_D - 0.040f, BALC_D - 0.012f);
        }
        for (int i = 0; i <= 4; i++) {
            float z = BALC_D * (float)i / 4.0f;
            Box(m, -hw + 0.012f, -hw + 0.040f, 0.120f, rt - 0.050f, z - 0.014f, z + 0.014f);
            Box(m, hw - 0.040f, hw - 0.012f, 0.120f, rt - 0.050f, z - 0.014f, z + 0.014f);
        }
        // Corrugated infill sheeting below the rail, which is what these are actually filled with once a resident has been at them.
        Box(&g->b[MAT_METAL], -hw + 0.050f, hw - 0.050f, 0.120f, rt - 0.070f,
            BALC_D - 0.032f, BALC_D - 0.020f);
    }

    // The entrance canopy: a slab on two brackets over the door.
    {
        Builder *c = &gType[FR_CANOPY].b[MAT_CONCRETE];
        Box(c, -1.200f, 1.200f, 0.0f, 0.140f, 0.0f, 1.300f);
        Box(c, -1.200f, -1.060f, -0.500f, 0.0f, 0.0f, 0.260f);
        Box(c, 1.060f, 1.200f, -0.500f, 0.0f, 0.0f, 0.260f);
    }

    // A roof ventilation stack, brick with a concrete cap.
    {
        Builder *c = &gType[FR_VENT].b[MAT_CONCRETE];
        Box(c, -0.450f, 0.450f, 0.0f, 1.150f, -0.300f, 0.300f);
        Box(c, -0.540f, 0.540f, 1.150f, 1.250f, -0.390f, 0.390f);
    }
}


// ---------------------------------------------------------------------------
// The yard
//
// Everything out here is an instanced type too, for the same reason the building is: a tree that
// the blast is going to lash, a car it is going to rock and bury, and a bench it is going to
// knock over all need to be things the pose function can move, and none of them is worth a mesh
// of its own eight times over.
// ---------------------------------------------------------------------------

// A lumpy ellipsoid. Every crown here is a handful of these rather than leaves.
//
// The normal is the underlying ellipsoid's, not the lumpy surface's. That is deliberate: a true
// normal makes each lump shade as its own object and the crown breaks into a bag of boulders,
// where the smooth one keeps it reading as one soft mass and leaves the leaf-scale break-up to
// the map, which is where it can actually be resolved.
#define BLOB_SEGS 18
static void Blob(Builder *b, Vector3 c, float rx, float ry, float rz, float lumpy,
                 unsigned int seed, int rings)
{
    Vector3 p0[BLOB_SEGS + 1], n0[BLOB_SEGS + 1], p1[BLOB_SEGS + 1], n1[BLOB_SEGS + 1];

    for (int i = 0; i <= rings; i++) {
        float phi = PI * (float)i / (float)rings;
        for (int j = 0; j <= BLOB_SEGS; j++) {
            float th = 2.0f * PI * (float)j / (float)BLOB_SEGS;
            Vector3 d = { sinf(phi) * cosf(th), cosf(phi), sinf(phi) * sinf(th) };
            float k = 1.0f + lumpy * (Fbm((float)j / (float)BLOB_SEGS, phi / PI, 4, seed, 2) - 0.5f) * 2.0f;
            p1[j] = (Vector3){ c.x + d.x * rx * k, c.y + d.y * ry * k, c.z + d.z * rz * k };
            n1[j] = Vector3Normalize((Vector3){ d.x / rx, d.y / ry, d.z / rz });
        }
        if (i > 0) {
            for (int j = 0; j < BLOB_SEGS; j++) {
                // The top and bottom rings collapse to a point, so those bands are fans rather than quads.
                if (i == 1) { int a = Vert(b, p0[j], n0[j]), c1 = Vert(b, p1[j], n1[j]), d1 = Vert(b, p1[j + 1], n1[j + 1]); Tri(b, a, c1, d1); }
                else if (i == rings) { int a = Vert(b, p0[j], n0[j]), c1 = Vert(b, p0[j + 1], n0[j + 1]), d1 = Vert(b, p1[j], n1[j]); Tri(b, a, d1, c1); }
                else QuadN(b, p0[j], p1[j], p1[j + 1], p0[j + 1], n0[j], n1[j], n1[j + 1], n0[j + 1]);
            }
        }
        for (int j = 0; j <= BLOB_SEGS; j++) { p0[j] = p1[j]; n0[j] = n1[j]; }
    }
}

// A tree, authored at its nominal height about the point where its trunk meets the ground, so a
// fragment's scale is the only thing that has to vary between one instance and the next.
static void BuildTree(Group *g, MatId bark, float height, float trunkR, float crownR,
                      float crownSquash, int blobs, unsigned int seed)
{
    Builder *t = &g->b[bark];
    float clear = height * 0.46f;    // height of the lowest branch
    Tube(t, (Vector3){ 0, 0, 0 }, (Vector3){ 0, clear, 0 }, trunkR, trunkR * 0.62f, 10, false, false);
    Tube(t, (Vector3){ 0, clear, 0 }, (Vector3){ 0, height * 0.86f, 0 }, trunkR * 0.62f, trunkR * 0.18f, 8, false, true);

    // Limbs, spiralled up the trunk rather than dropped at one height, so the crown has something under it from every side.
    // Their reach is a fraction of crownR rather than most of it: a limb that ends outside the foliage reads as a white stick through a bush, which is what the first build's birches did.
    const int LIMBS = 5;
    for (int i = 0; i < LIMBS; i++) {
        float f = (float)i / (float)(LIMBS - 1);
        float y = clear + (height * 0.80f - clear) * f;
        float th = 2.0f * PI * (0.37f * (float)i) + (float)seed * 0.11f;
        float reach = crownR * (0.52f - 0.22f * f);
        float r = trunkR * (0.40f - 0.16f * f);
        Tube(t, (Vector3){ 0, y, 0 },
             (Vector3){ cosf(th) * reach, y + reach * 0.85f, sinf(th) * reach }, r, r * 0.35f, 7, false, true);
    }

    // The crown is a cloud of small blobs rather than a handful of big ones. The first build used
    // seven of up to 0.8 crownR and every tree came out a cabbage: at that size a blob's own
    // silhouette is the tree's silhouette, so the outline is four or five arcs and nothing about
    // it reads as foliage. Twenty-two at a third of the radius put the outline where the map's
    // mottle can carry it.
    Builder *l = &g->b[MAT_LEAF];
    float cy = height * 0.76f;
    float ry = crownR * crownSquash;
    for (int i = 0; i < blobs; i++) {
        // Golden-angle spiral over the crown's own ellipsoid, so the blobs cover it evenly
        // instead of clumping the way a hashed scatter of this few would.
        float f = ((float)i + 0.5f) / (float)blobs;
        float th = 2.0f * PI * 0.618034f * (float)i;
        float cosp = 1.0f - 2.0f * f;
        float sinp = sqrtf(fmaxf(0.0f, 1.0f - cosp * cosp));
        float shell = 0.42f + 0.46f * Hash2(i, (int)seed, 128, 7u);
        float rad = crownR * (0.24f + 0.13f * Hash2(i, (int)seed + 5, 128, 13u));
        Vector3 c = {
            sinp * cosf(th) * crownR * shell,
            cy + cosp * ry * shell * 0.92f,
            sinp * sinf(th) * crownR * shell,
        };
        Blob(l, c, rad, rad * (0.80f + 0.30f * crownSquash), rad, 0.22f, seed + (unsigned int)i * 17u, 6);
    }
    // A denser core, so the crown is not a hollow shell wherever two blobs fail to overlap.
    Blob(l, (Vector3){ 0, cy, 0 }, crownR * 0.56f, ry * 0.60f, crownR * 0.56f, 0.18f, seed + 91u, 8);
}

// ---------------------------------------------------------------------------
// A VAZ-2101, from the specification and references/panelka/ref_08.jpg.
//
// 4.073 long, 1.611 wide, 1.382 tall, 2.424 wheelbase, 1.349 front track, 0.170 clearance. The
// elevation sets what the numbers cannot: the beltline runs at 0.83 of the 1.382, and the bonnet
// crown sits above it rather than below, which is what gives the car its flat-topped nose.
//
// Origin on the ground under the centre of the wheelbase, +Z forward, so an instance is a yaw.
// ---------------------------------------------------------------------------

#define CAR_L      4.073f
#define CAR_HW     0.806f    // half of 1.611
#define CAR_H      1.382f
#define CAR_WB     2.424f
#define CAR_TRACK  1.349f
#define CAR_CLR    0.170f
#define CAR_WR     0.305f    // wheel rolling radius
#define CAR_WW     0.175f
#define CAR_BELT   0.830f
#define CAR_BONNET 0.900f
#define CAR_NOSE   2.100f
#define CAR_TAIL   (CAR_NOSE - CAR_L)

static void BuildCar(Group *body, Group *trim)
{
    Builder *p = &body->b[MAT_PAINT];

    // Lower body. The outer 0.22 m of each side is swept as vertical strips whose floor follows
    // the wheel arch, and the band between them keeps a flat floor, so the arch is a recess in
    // the flank rather than a tunnel straight through the car.
    //
    // This is the difference between a car and a box: the track is 1.349 inside a 1.611 body, so
    // with no arch cut the wheels sit entirely within the body's own width and half their
    // diameter above its underside. The first build had exactly that and the car read as a crate
    // with something grey behind it.
    const float SILL_Y = CAR_CLR + 0.100f;
    const float ARCH_R = CAR_WR + 0.095f;
    const float SKIN = 0.220f;
    for (int band = 0; band < 3; band++) {
        float bx0, bx1;
        bool arched = (band != 1);
        if (band == 0) { bx0 = -CAR_HW; bx1 = -CAR_HW + SKIN; }
        else if (band == 1) { bx0 = -CAR_HW + SKIN; bx1 = CAR_HW - SKIN; }
        else { bx0 = CAR_HW - SKIN; bx1 = CAR_HW; }

        const int STRIPS = 96;
        for (int i = 0; i < STRIPS; i++) {
            float z0 = CAR_TAIL + (CAR_L * (float)i) / (float)STRIPS;
            float z1 = CAR_TAIL + (CAR_L * (float)(i + 1)) / (float)STRIPS;
            float floorY = SILL_Y;
            if (arched) {
                float zc = 0.5f * (z0 + z1);
                for (int a = 0; a < 2; a++) {
                    float az = (a == 0) ? -CAR_WB * 0.5f : CAR_WB * 0.5f;
                    float d = fabsf(zc - az);
                    if (d < ARCH_R) floorY = fmaxf(floorY, CAR_WR + sqrtf(ARCH_R * ARCH_R - d * d));
                }
            }
            // The flanks tuck in below the beltline, which is what the elevation shows and what
            // stops the section reading as a slab.
            float tuck = (floorY > SILL_Y + 1e-4f) ? 0.0f : 0.030f;
            Box(p, bx0 + tuck, bx1 - tuck, floorY, CAR_BELT, z0, z1);
        }
    }

    // Bonnet and boot, each a shallow wedge off the beltline. The bonnet crown stands above the
    // beltline, which is the one thing the elevation settles that the dimension table does not.
    Prism(p, -CAR_HW + 0.020f, CAR_HW - 0.020f, 0.560f, CAR_NOSE - 0.030f,
          CAR_BELT, CAR_BELT, CAR_BONNET, CAR_BONNET - 0.055f);
    Prism(p, -CAR_HW + 0.020f, CAR_HW - 0.020f, CAR_TAIL + 0.030f, -0.620f,
          CAR_BELT, CAR_BELT, CAR_BONNET - 0.055f, CAR_BONNET - 0.030f);

    // Greenhouse: one tapered hexahedron of glass with the roof panel and the pillars laid over
    // it. At any distance this scene is ever seen from, a pillar is three pixels and a pane is
    // twenty, so the glass is the shape and the pillars are the detail.
    const float GB0 = -1.100f, GB1 = 0.760f;     // beltline ends of the cabin
    const float GT0 = -0.880f, GT1 = 0.190f;     // roof ends: the windscreen rakes far harder than the backlight
    const float GBW = CAR_HW - 0.030f, GTW = CAR_HW - 0.130f;
    const float ROOF_Y0 = CAR_H - 0.055f;
    {
        Vector3 c[8] = {
            { -GBW, CAR_BELT, GB0 }, { GBW, CAR_BELT, GB0 }, { GBW, CAR_BELT, GB1 }, { -GBW, CAR_BELT, GB1 },
            { -GTW, CAR_H, GT0 }, { GTW, CAR_H, GT0 }, { GTW, CAR_H, GT1 }, { -GTW, CAR_H, GT1 },
        };
        Hex(&trim->b[MAT_GLASS], c);
    }
    {
        Vector3 c[8] = {
            { -GTW - 0.012f, ROOF_Y0, GT0 - 0.012f }, { GTW + 0.012f, ROOF_Y0, GT0 - 0.012f },
            { GTW + 0.012f, ROOF_Y0, GT1 + 0.012f },  { -GTW - 0.012f, ROOF_Y0, GT1 + 0.012f },
            { -GTW - 0.012f, CAR_H + 0.012f, GT0 - 0.012f }, { GTW + 0.012f, CAR_H + 0.012f, GT0 - 0.012f },
            { GTW + 0.012f, CAR_H + 0.012f, GT1 + 0.012f },  { -GTW - 0.012f, CAR_H + 0.012f, GT1 + 0.012f },
        };
        Hex(p, c);
    }
    // A, B and C pillars, each a wedge running from its beltline foot to its roof head.
    const float PZ[3][2] = { { GB1, GT1 }, { -0.230f, -0.190f }, { GB0, GT0 } };
    for (int i = 0; i < 3; i++) {
        for (int side = 0; side < 2; side++) {
            float sx = (side == 0) ? 1.0f : -1.0f;
            float w = (i == 1) ? 0.055f : 0.045f;
            Vector3 c[8] = {
                { sx * (GBW - w), CAR_BELT, PZ[i][0] - w }, { sx * GBW, CAR_BELT, PZ[i][0] - w },
                { sx * GBW, CAR_BELT, PZ[i][0] + w },       { sx * (GBW - w), CAR_BELT, PZ[i][0] + w },
                { sx * (GTW - w), ROOF_Y0, PZ[i][1] - w },  { sx * GTW, ROOF_Y0, PZ[i][1] - w },
                { sx * GTW, ROOF_Y0, PZ[i][1] + w },        { sx * (GTW - w), ROOF_Y0, PZ[i][1] + w },
            };
            Hex(p, c);
        }
    }

    Builder *m = &trim->b[MAT_METAL];
    // Bumpers, overriders and the grille.
    Box(m, -CAR_HW + 0.010f, CAR_HW - 0.010f, 0.400f, 0.500f, CAR_NOSE - 0.020f, CAR_NOSE + 0.075f);
    Box(m, -CAR_HW + 0.010f, CAR_HW - 0.010f, 0.400f, 0.500f, CAR_TAIL - 0.075f, CAR_TAIL + 0.020f);
    Box(m, -0.620f, 0.620f, 0.560f, 0.760f, CAR_NOSE - 0.040f, CAR_NOSE + 0.010f);
    // Headlamps and tail lamps.
    Box(&trim->b[MAT_GLASS], -0.740f, -0.410f, 0.580f, 0.760f, CAR_NOSE - 0.030f, CAR_NOSE + 0.015f);
    Box(&trim->b[MAT_GLASS], 0.410f, 0.740f, 0.580f, 0.760f, CAR_NOSE - 0.030f, CAR_NOSE + 0.015f);
    Box(&trim->b[MAT_GLASS], -0.730f, -0.360f, 0.560f, 0.760f, CAR_TAIL - 0.015f, CAR_TAIL + 0.030f);
    Box(&trim->b[MAT_GLASS], 0.360f, 0.730f, 0.560f, 0.760f, CAR_TAIL - 0.015f, CAR_TAIL + 0.030f);
    // Wing mirrors, which are most of what breaks a saloon's silhouette from in front.
    Box(m, -CAR_HW - 0.090f, -CAR_HW + 0.010f, CAR_BELT + 0.020f, CAR_BELT + 0.110f, 0.560f, 0.660f);
    Box(m, CAR_HW - 0.010f, CAR_HW + 0.090f, CAR_BELT + 0.020f, CAR_BELT + 0.110f, 0.560f, 0.660f);

    Builder *r = &trim->b[MAT_RUBBER];
    for (int i = 0; i < 4; i++) {
        float x = ((i & 1) ? 1.0f : -1.0f) * CAR_TRACK * 0.5f;
        float z = (i & 2) ? CAR_WB * 0.5f : -CAR_WB * 0.5f;
        Tube(r, (Vector3){ x - CAR_WW * 0.5f, CAR_WR, z }, (Vector3){ x + CAR_WW * 0.5f, CAR_WR, z },
             CAR_WR, CAR_WR, 16, false, false);
        // Hub discs, buried 5 mm inside the tyre's outer face rather than flush with it.
        float hx = x + ((i & 1) ? CAR_WW * 0.5f - 0.005f : -CAR_WW * 0.5f + 0.005f);
        Tube(m, (Vector3){ hx, CAR_WR, z }, (Vector3){ hx + ((i & 1) ? -0.012f : 0.012f), CAR_WR, z },
             CAR_WR * 0.60f, CAR_WR * 0.60f, 14, true, true);
    }
}

// The yard's furniture, all of it authored about the point where it meets the ground.
static void BuildYardTypes(void)
{
    // Bench: two cast cheeks and five timber slats, which is the pattern every one of these
    // courtyards has and the only seat that survives thirty winters.
    {
        Group *g = &gType[FR_BENCH];
        Builder *c = &g->b[MAT_CONCRETE];
        for (int side = 0; side < 2; side++) {
            float x = (side == 0) ? -0.860f : 0.760f;
            Box(c, x, x + 0.100f, 0.0f, 0.420f, -0.240f, 0.240f);
            Box(c, x, x + 0.100f, 0.420f, 0.860f, -0.240f, -0.140f);
        }
        Builder *t = &g->b[MAT_TIMBER];
        for (int i = 0; i < 3; i++) {
            float z = -0.200f + 0.170f * (float)i;
            Box(t, -0.900f, 0.900f, 0.420f, 0.465f, z, z + 0.120f);
        }
        for (int i = 0; i < 2; i++) {
            float y = 0.590f + 0.180f * (float)i;
            Box(t, -0.900f, 0.900f, y, y + 0.130f, -0.235f, -0.190f);
        }
    }

    // Rug-beating frame. Two uprights and a top rail, in tube, and the one piece of yard
    // equipment that says which country this is; ref_07 has three of them in the snow.
    {
        Builder *m = &gType[FR_RUGFRAME].b[MAT_METAL];
        for (int side = 0; side < 2; side++) {
            float x = (side == 0) ? -1.350f : 1.350f;
            Tube(m, (Vector3){ x, 0.0f, 0.0f }, (Vector3){ x, 1.900f, 0.0f }, 0.048f, 0.044f, 8, false, false);
            Tube(m, (Vector3){ x, 1.100f, -0.520f }, (Vector3){ x, 1.900f, 0.0f }, 0.032f, 0.032f, 6, true, false);
        }
        Tube(m, (Vector3){ -1.420f, 1.880f, 0.0f }, (Vector3){ 1.420f, 1.880f, 0.0f }, 0.044f, 0.044f, 8, true, true);
    }

    // Lamp post: a column, a curved bracket faked as two straight runs, and a lantern.
    {
        Group *g = &gType[FR_LAMP];
        Builder *m = &g->b[MAT_METAL];
        Box(&g->b[MAT_CONCRETE], -0.180f, 0.180f, 0.0f, 0.260f, -0.180f, 0.180f);
        Tube(m, (Vector3){ 0, 0.180f, 0 }, (Vector3){ 0, 7.400f, 0 }, 0.115f, 0.070f, 10, false, false);
        Tube(m, (Vector3){ 0, 7.400f, 0 }, (Vector3){ 0, 7.900f, 0.420f }, 0.065f, 0.058f, 8, false, false);
        Tube(m, (Vector3){ 0, 7.900f, 0.420f }, (Vector3){ 0, 7.980f, 1.180f }, 0.058f, 0.052f, 8, false, false);
        Box(m, -0.230f, 0.230f, 7.900f, 8.010f, 0.960f, 1.560f);
        Box(&g->b[MAT_GLASS], -0.205f, 0.205f, 7.820f, 7.905f, 0.985f, 1.535f);
    }

    // Bin: a drum on a hooped stand.
    {
        Group *g = &gType[FR_BIN];
        Builder *m = &g->b[MAT_METAL];
        Tube(m, (Vector3){ 0, 0.260f, 0 }, (Vector3){ 0, 0.960f, 0 }, 0.250f, 0.285f, 12, true, false);
        for (int i = 0; i < 3; i++) {
            float th = 2.0f * PI * (float)i / 3.0f;
            Tube(m, (Vector3){ cosf(th) * 0.250f, 0.0f, sinf(th) * 0.250f },
                 (Vector3){ cosf(th) * 0.190f, 0.320f, sinf(th) * 0.190f }, 0.030f, 0.030f, 6, false, false);
        }
    }

    // A birch is tall, narrow and airy; the broadleaf beside it is shorter and much wider. ref_07 has both, and the birches in it stand well clear of a five-storey block's parapet.
    BuildTree(&gType[FR_BIRCH], MAT_BIRCH, 16.5f, 0.200f, 2.55f, 1.45f, 22, 5u);
    BuildTree(&gType[FR_MAPLE], MAT_BARK, 11.5f, 0.290f, 3.60f, 0.86f, 24, 23u);
    BuildCar(&gType[FR_CARBODY], &gType[FR_CARTRIM]);
}

// Where the yard's things stand. Hand-placed rather than scattered by a hash: a courtyard is
// laid out, not sprinkled, and the trees that matter are the ones close enough to the facade for
// the collapse to reach.
static const Color CAR_PAINT[] = {
    { 214, 212, 198, 255 },   // the beige every second Zhiguli was
    { 132, 152, 168, 255 },
    { 172, 88, 62, 255 },
    { 208, 200, 168, 255 },
    { 96, 118, 96, 255 },
};

static void PlaceYard(void)
{
    // Trees. x, z, which species, and the scale that makes each one its own tree.
    static const float TREES[][4] = {
        { -34.0f, 9.6f, 0, 0.92f }, { -12.5f, 9.4f, 0, 1.06f }, { 9.0f, 9.8f, 0, 0.86f }, { 31.0f, 9.5f, 0, 1.00f },
        { -44.0f, 26.5f, 1, 1.05f }, { -25.0f, 27.5f, 0, 1.14f }, { -3.0f, 26.8f, 1, 0.94f },
        { 17.0f, 27.6f, 0, 1.08f }, { 38.0f, 26.4f, 1, 1.00f },
        { -46.0f, -24.0f, 1, 0.98f }, { -27.5f, -25.5f, 0, 1.10f }, { -6.0f, -24.5f, 1, 1.16f },
        { 15.0f, -25.8f, 0, 0.90f }, { 36.0f, -24.2f, 1, 1.04f },
        { -40.0f, -9.5f, 0, 0.82f }, { 42.0f, -10.5f, 1, 0.88f },
    };
    for (int i = 0; i < (int)(sizeof(TREES) / sizeof(TREES[0])); i++) {
        FragType t = (TREES[i][2] < 0.5f) ? FR_BIRCH : FR_MAPLE;
        EmitAt(t, TREES[i][0], 0.0f, TREES[i][1], 360.0f * Hash2(i, 3, 64, 29u), TREES[i][3], WHITE);
    }

    // Cars, nose-in on the parking bay off the drive, and one on the service road behind.
    static const float CARS[][3] = {
        { -23.0f, 13.4f, 178.0f }, { -9.6f, 13.4f, 183.0f }, { 4.8f, 13.4f, 176.0f }, { 21.5f, 13.4f, 181.0f },
        { -34.0f, -17.4f, 92.0f },
    };
    for (int i = 0; i < (int)(sizeof(CARS) / sizeof(CARS[0])); i++) {
        Color c = CAR_PAINT[i % (int)(sizeof(CAR_PAINT) / sizeof(CAR_PAINT[0]))];
        EmitAt(FR_CARBODY, CARS[i][0], 0.0f, CARS[i][1], CARS[i][2], 1.0f, c);
        EmitAt(FR_CARTRIM, CARS[i][0], 0.0f, CARS[i][1], CARS[i][2], 1.0f, WHITE);
    }

    // Two benches flanking each entrance, facing the path.
    for (int s = 0; s < SECTIONS; s++) {
        float x = BayX(s, BAY_STAIR);
        Emit(FR_BENCH, x - 4.20f, 0.0f, -8.60f, 12.0f);
        Emit(FR_BENCH, x + 4.20f, 0.0f, -8.60f, -12.0f);
    }

    Emit(FR_RUGFRAME, -21.0f, 0.0f, -12.8f, 8.0f);
    Emit(FR_RUGFRAME, 12.5f, 0.0f, -13.4f, -6.0f);
    Emit(FR_BIN, BayX(0, BAY_STAIR) + 2.6f, 0.0f, -9.9f, 0.0f);
    Emit(FR_BIN, BayX(2, BAY_STAIR) - 2.6f, 0.0f, -9.9f, 0.0f);

    for (int i = 0; i < 4; i++) Emit(FR_LAMP, -36.0f + 24.0f * (float)i, 0.0f, 18.4f, 180.0f);
}

// ---------------------------------------------------------------------------
// Placing the fragments
// ---------------------------------------------------------------------------

// Which panel type a bay carries on a given facade and floor.
// front is +Z. The stairwell touches the front, so that is where its windows go and where the entrance cannot be; the entrance is on the back, which is the courtyard side.
static FragType PanelAt(int bay, int floor, bool front, FragType *glazing)
{
    if (bay == BAY_STAIR) {
        if (front) { *glazing = FR_GSTAIR; return FR_PSTAIR; }
        if (floor == 0) { *glazing = FR_GDOOR; return FR_PDOOR; }
        *glazing = FR_GSTAIR;
        return FR_PSTAIR;
    }
    if (bay == BAY_BALC_A || bay == BAY_BALC_B) { *glazing = FR_G32B; return FR_P32B; }
    if (BAY_W[bay] > 2.9f) { *glazing = FR_G32; return FR_P32; }
    *glazing = FR_G26;
    return FR_P26;
}

static void PlaceBuilding(void)
{
    for (int s = 0; s < SECTIONS; s++) {
        for (int f = 0; f < FLOORS; f++) {
            float y = FloorY(f);

            for (int b = 0; b < BAYS; b++) {
                float x = BayX(s, b);
                FragType glz;

                FragType p = PanelAt(b, f, true, &glz);
                Emit(p, x, y, INNER_Z, 0.0f);
                Emit(glz, x, y, INNER_Z, 0.0f);

                p = PanelAt(b, f, false, &glz);
                Emit(p, x, y, -INNER_Z, 180.0f);
                Emit(glz, x, y, -INNER_Z, 180.0f);

                // Floor slabs, one each side of the spine.
                FragType slab = (BAY_W[b] > 2.9f) ? FR_SLAB32 : FR_SLAB26;
                Emit(slab, x, y - SLAB_T, 0.0f, 0.0f);
                Emit(slab, x, y - SLAB_T, 0.0f, 180.0f);

                Emit((BAY_W[b] > 2.9f) ? FR_SPINE32 : FR_SPINE26, x, y, 0.0f, 0.0f);

                // Balconies, on the two wide bays of every section and every floor, both facades. ref_07 shows them running to the ground floor rather than starting at the first, which is what this follows.
                if (b == BAY_BALC_A || b == BAY_BALC_B) {
                    Emit(FR_BALC, x, y, FACE_Z, 0.0f);
                    Emit(FR_BALC, x, y, -FACE_Z, 180.0f);
                }
            }

            // Cross walls on every bay boundary of the section, plus the one that closes it.
            for (int b = 0; b <= BAYS; b++) {
                float x = SectionX(s) - SECTION_LEN * 0.5f;
                for (int i = 0; i < b; i++) x += BAY_W[i];
                // The boundary between two sections is one wall, not two.
                if (b == BAYS && s != SECTIONS - 1) continue;
                Emit(FR_XWALL, x, y, 0.0f, 0.0f);
            }

            // End walls, two panels deep, on the two ends of the block only.
            for (int e = 0; e < 2; e++) {
                if (!((s == 0 && e == 0) || (s == SECTIONS - 1 && e == 1))) continue;
                float sx = (e == 0) ? -BLOCK_LEN * 0.5f : BLOCK_LEN * 0.5f;
                // MatrixRotateY sends a panel's local +Z, which is the direction it faces, to world +X at yaw 90.
                // The left-hand gable has to face -X, so it takes -90 and the right-hand one +90, and each origin sits on its own inner face the way every other facade panel's does.
                float yaw = (e == 0) ? -90.0f : 90.0f;
                float ex = sx + ((e == 0) ? WALL_T : -WALL_T);
                Emit(FR_PEND, ex, y, FACE_Z * 0.5f, yaw);
                Emit(FR_PEND, ex, y, -FACE_Z * 0.5f, yaw);
            }

            // The entrance canopy hangs over the back door, one storey up.
            if (f == 0) {
                Emit(FR_CANOPY, BayX(s, BAY_STAIR), PLINTH_Y + ENTRY_H + 0.350f, -FACE_Z, 180.0f);
            }
        }

        // Roof: slabs over every bay, a parapet round the perimeter, and the vent stacks that serve the kitchen and bathroom risers either side of the stairwell.
        for (int b = 0; b < BAYS; b++) {
            float x = BayX(s, b);
            FragType roof = (BAY_W[b] > 2.9f) ? FR_ROOF32 : FR_ROOF26;
            Emit(roof, x, ROOF_Y - SLAB_T, 0.0f, 0.0f);
            Emit(roof, x, ROOF_Y - SLAB_T, 0.0f, 180.0f);
            Emit((BAY_W[b] > 2.9f) ? FR_PAR32 : FR_PAR26, x, ROOF_Y, FACE_Z, 0.0f);
            Emit((BAY_W[b] > 2.9f) ? FR_PAR32 : FR_PAR26, x, ROOF_Y, -FACE_Z, 180.0f);
        }
        Emit(FR_VENT, BayX(s, BAY_STAIR) - 1.9f, ROOF_Y, 2.4f, 0.0f);
        Emit(FR_VENT, BayX(s, BAY_STAIR) + 1.9f, ROOF_Y, -2.4f, 0.0f);
    }

    // Parapet across the two ends.
    for (int e = 0; e < 2; e++) {
        float sx = (e == 0) ? -BLOCK_LEN * 0.5f : BLOCK_LEN * 0.5f;
        float yaw = (e == 0) ? -90.0f : 90.0f;
        Emit(FR_PAREND, sx, ROOF_Y, FACE_Z * 0.5f, yaw);
        Emit(FR_PAREND, sx, ROOF_Y, -FACE_Z * 0.5f, yaw);
    }
}

// Counting-sort the fragments into type-contiguous order, then point each type's group at its own slice. Placement above walks the building in the order the building is built; instancing needs one contiguous run per mesh, and doing it here means the placement code never has to know that.
static void SortFragments(void)
{
    for (int t = 0; t < FR_COUNT; t++) gTypeCount[t] = 0;
    for (int i = 0; i < gFragCount; i++) gTypeCount[gFrag[i].type]++;

    int at = 0;
    for (int t = 0; t < FR_COUNT; t++) { gTypeStart[t] = at; at += gTypeCount[t]; }

    static Fragment sorted[MAX_FRAG];
    int cursor[FR_COUNT];
    for (int t = 0; t < FR_COUNT; t++) cursor[t] = gTypeStart[t];
    for (int i = 0; i < gFragCount; i++) sorted[cursor[gFrag[i].type]++] = gFrag[i];
    for (int i = 0; i < gFragCount; i++) gFrag[i] = sorted[i];

    for (int t = 0; t < FR_COUNT; t++) {
        gType[t].inst = &gFragMat[gTypeStart[t]];
        gType[t].instTint = &gFragTint[gTypeStart[t]];
        gType[t].instCount = gTypeCount[t];
    }
}

// ---------------------------------------------------------------------------
// The things that are not fragments
//
// The plinth, the steps and the ground exist once each and do not move, so they are placed groups rather than instanced ones. The plinth in particular stays put through a demolition:
// the charges cut above it, and what is left of it ends up under the pile.
// ---------------------------------------------------------------------------

static Group gPlinth, gGround;

static void BuildPlinth(void)
{
    Group *g = &gPlinth;
    Builder *p = &g->b[MAT_PLINTH];
    float hx = BLOCK_LEN * 0.5f, hz = FACE_Z;
    float t = 0.100f;   // how far the plinth stands proud of the facade above it

    // A closed band round the footprint rather than a solid block: nothing ever sees inside it,
    // and a solid one would be 58 x 12 x 0.75 of geometry for four visible faces.
    Box(p, -hx - t, hx + t, 0.0f, PLINTH_Y, hz, hz + t);
    Box(p, -hx - t, hx + t, 0.0f, PLINTH_Y, -hz - t, -hz);
    Box(p, -hx - t, -hx, 0.0f, PLINTH_Y, -hz, hz);
    Box(p, hx, hx + t, 0.0f, PLINTH_Y, -hz, hz);

    // Basement lights, one per bay along both long sides, set into the band.
    for (int s = 0; s < SECTIONS; s++) {
        for (int b = 0; b < BAYS; b++) {
            if (b == BAY_STAIR) continue;
            float x = BayX(s, b);
            for (int side = 0; side < 2; side++) {
                float z = (side == 0) ? hz + t : -hz - t;
                float zi = (side == 0) ? hz + t - 0.040f : -hz - t + 0.040f;
                Box(&g->b[MAT_GLASS], x - 0.300f, x + 0.300f, 0.300f, 0.640f,
                    fminf(z, zi), fmaxf(z, zi));
            }
        }
    }

    // Entrance steps, one flight per section, up to the ground floor.
    for (int s = 0; s < SECTIONS; s++) {
        float x = BayX(s, BAY_STAIR);
        const int RISERS = 5;
        float rise = PLINTH_Y / (float)RISERS, tread = 0.320f;
        for (int i = 0; i < RISERS; i++) {
            float z0 = -hz - t - tread * (float)(RISERS - i);
            Box(&g->b[MAT_CONCRETE], x - 1.100f, x + 1.100f, 0.0f, rise * (float)(i + 1),
                z0, z0 + tread + 0.001f);
        }
        // Landing in front of the door.
        Box(&g->b[MAT_CONCRETE], x - 1.100f, x + 1.100f, 0.0f, PLINTH_Y, -hz - t, -hz + 0.200f);
        // Cheek walls either side of the flight.
        for (int side = 0; side < 2; side++) {
            float cx = (side == 0) ? x - 1.240f : x + 1.100f;
            Box(&g->b[MAT_CONCRETE], cx, cx + 0.140f, 0.0f, PLINTH_Y + 0.120f,
                -hz - t - tread * (float)RISERS, -hz);
        }
    }
}

// The site: grass over the whole plot, an asphalt drive and parking bay along the front, and the paths that connect the three entrances to it.
#define SITE_HX  240.0f
#define SITE_HZ  200.0f

static void BuildGround(void)
{
    Group *g = &gGround;

    // Grass sits fractionally below zero, so the asphalt laid on top of it wins the depth test everywhere it covers rather than fighting it.
    Box(&g->b[MAT_GRASS], -SITE_HX, SITE_HX, -0.120f, -0.010f, -SITE_HZ, SITE_HZ);

    // The drive along the main facade, and the parking bay off it.
    Box(&g->b[MAT_ASPHALT], -SITE_HX, SITE_HX, -0.010f, 0.030f, 16.0f, 22.5f);
    Box(&g->b[MAT_ASPHALT], -26.0f, 14.0f, -0.010f, 0.030f, 11.5f, 16.0f);

    // The service road behind, and a landing outside each entrance.
    Box(&g->b[MAT_ASPHALT], -SITE_HX, SITE_HX, -0.010f, 0.030f, -20.0f, -15.0f);
    for (int s = 0; s < SECTIONS; s++) {
        float x = BayX(s, BAY_STAIR);
        Box(&g->b[MAT_ASPHALT], x - 2.6f, x + 2.6f, -0.010f, 0.030f, -15.0f, -FACE_Z - 2.4f);
    }

    // Kerbs along the drive.
    for (int i = 0; i < 2; i++) {
        float z = (i == 0) ? 16.0f : 22.5f;
        Box(&g->b[MAT_CONCRETE], -SITE_HX, SITE_HX, -0.010f, 0.130f, z - 0.120f, z + 0.120f);
    }
}

// ---------------------------------------------------------------------------
// Pose
// ---------------------------------------------------------------------------

#define CYCLE 8.0f

static void Update(float t)
{
    (void)t;
    for (int i = 0; i < gFragCount; i++) {
        gFragMat[i] = FragRest(&gFrag[i]);
        // +-8 either side of white. Enough to separate two neighbouring panels, not enough to read as a repainted one.
        int k = (int)(230.0f + 25.0f * gFrag[i].shade);
        gFragTint[i] = TintMul((Color){ (unsigned char)k, (unsigned char)k, (unsigned char)k, 255 },
                               gFrag[i].tint);
    }
}

// ---------------------------------------------------------------------------
// Parts
// ---------------------------------------------------------------------------

#define COUNT_OF(a) ((int)(sizeof(a) / sizeof((a)[0])))

static Group *PART_SHELL[]  = { &gType[FR_P26], &gType[FR_P32], &gType[FR_P32B],
                                &gType[FR_PSTAIR], &gType[FR_PDOOR], &gType[FR_PEND] };
static Group *PART_GLAZ[]   = { &gType[FR_G26], &gType[FR_G32], &gType[FR_G32B],
                                &gType[FR_GSTAIR], &gType[FR_GDOOR] };
static Group *PART_STRUCT[] = { &gType[FR_SLAB26], &gType[FR_SLAB32],
                                &gType[FR_XWALL], &gType[FR_SPINE26], &gType[FR_SPINE32] };
static Group *PART_ROOF[]   = { &gType[FR_ROOF26], &gType[FR_ROOF32], &gType[FR_PAR26],
                                &gType[FR_PAR32], &gType[FR_PAREND], &gType[FR_VENT] };
static Group *PART_BALC[]   = { &gType[FR_BALC] };
static Group *PART_ENTRY[]  = { &gType[FR_PDOOR], &gType[FR_GDOOR], &gType[FR_CANOPY] };
static Group *PART_PLINTH[] = { &gPlinth };
static Group *PART_TREES[]  = { &gType[FR_BIRCH], &gType[FR_MAPLE] };
static Group *PART_CARS[]   = { &gType[FR_CARBODY], &gType[FR_CARTRIM] };
static Group *PART_YARD[]   = { &gType[FR_BENCH], &gType[FR_RUGFRAME], &gType[FR_LAMP], &gType[FR_BIN] };
static Group *PART_GROUND[] = { &gGround };

static BoundingBox bShell, bGlaz, bStruct, bRoof, bBalc, bEntry, bPlinth, bGround, bTrees, bCars, bYard;

static void DrawAll(void) { DrawGroups(gAll, gAllCount); }
static void DrawShell(void) { DrawGroups(PART_SHELL, COUNT_OF(PART_SHELL)); }
static void DrawGlaz(void) { DrawGroups(PART_GLAZ, COUNT_OF(PART_GLAZ)); }
static void DrawStruct(void) { DrawGroups(PART_STRUCT, COUNT_OF(PART_STRUCT)); }
static void DrawRoof(void) { DrawGroups(PART_ROOF, COUNT_OF(PART_ROOF)); }
static void DrawBalc(void) { DrawGroups(PART_BALC, COUNT_OF(PART_BALC)); }
static void DrawEntry(void) { DrawGroups(PART_ENTRY, COUNT_OF(PART_ENTRY)); }
static void DrawPlinth(void) { DrawGroups(PART_PLINTH, COUNT_OF(PART_PLINTH)); }
static void DrawGround(void) { DrawGroups(PART_GROUND, COUNT_OF(PART_GROUND)); }
static void DrawTrees(void) { DrawGroups(PART_TREES, COUNT_OF(PART_TREES)); }
static void DrawCars(void) { DrawGroups(PART_CARS, COUNT_OF(PART_CARS)); }
static void DrawYard(void) { DrawGroups(PART_YARD, COUNT_OF(PART_YARD)); }

static BoundingBox ShellBounds(void) { return bShell; }
static BoundingBox GlazBounds(void) { return bGlaz; }
static BoundingBox StructBounds(void) { return bStruct; }
static BoundingBox RoofBounds(void) { return bRoof; }
static BoundingBox BalcBounds(void) { return bBalc; }
static BoundingBox EntryBounds(void) { return bEntry; }
static BoundingBox PlinthBounds(void) { return bPlinth; }
static BoundingBox GroundBounds(void) { return bGround; }
static BoundingBox TreeBounds(void) { return bTrees; }
static BoundingBox CarBounds(void) { return bCars; }
static BoundingBox YardBounds(void) { return bYard; }

static const Part PARTS[] = {
    { .name = "shell", .draw = DrawShell, .bounds = ShellBounds },
    { .name = "glazing", .draw = DrawGlaz, .bounds = GlazBounds },
    { .name = "structure", .draw = DrawStruct, .bounds = StructBounds },
    { .name = "roof", .draw = DrawRoof, .bounds = RoofBounds },
    { .name = "balconies", .draw = DrawBalc, .bounds = BalcBounds },
    { .name = "entrance", .draw = DrawEntry, .bounds = EntryBounds },
    { .name = "plinth", .draw = DrawPlinth, .bounds = PlinthBounds },
    { .name = "ground", .draw = DrawGround, .bounds = GroundBounds },
    { .name = "trees", .draw = DrawTrees, .bounds = TreeBounds },
    { .name = "cars", .draw = DrawCars, .bounds = CarBounds },
    { .name = "yard", .draw = DrawYard, .bounds = YardBounds },
};

// A group that exists in several hundred places has no single bounding box either. Sample the pose over the cycle and union the eight transformed corners of the local box over every instance, which is the same argument SweepBounds makes for a part that moves: GetModelBoundingBox transforms only the box's own two corners and warns that it does not support rotation (vendor/raylib/src/rmodels.c:1243), which is exactly what a yawed panel is.
static BoundingBox InstBounds(Group *const *gs, int n)
{
    BoundingBox out = { { 1e9f, 1e9f, 1e9f }, { -1e9f, -1e9f, -1e9f } };
    const int SAMPLES = 24;
    for (int s = 0; s < SAMPLES; s++) {
        Update(CYCLE * (float)s / (float)SAMPLES);
        for (int i = 0; i < n; i++) {
            BoundingBox b = gs[i]->bounds;
            int reps = (gs[i]->instCount > 0) ? gs[i]->instCount : 1;
            for (int k = 0; k < reps; k++) {
                Matrix m = (gs[i]->instCount > 0) ? gs[i]->inst[k] : gs[i]->xform;
                for (int c = 0; c < 8; c++) {
                    Vector3 p = {
                        (c & 1) ? b.max.x : b.min.x,
                        (c & 2) ? b.max.y : b.min.y,
                        (c & 4) ? b.max.z : b.min.z,
                    };
                    p = Vector3Transform(p, m);
                    out.min = Vector3Min(out.min, p);
                    out.max = Vector3Max(out.max, p);
                }
            }
        }
    }
    Update(0.0f);
    return out;
}

// ---------------------------------------------------------------------------
// Checks
//
// A building assembled out of 800 separately placed pieces can be wrong in ways no single view shows: one bay wider than its panel, a slab that stops short of the wall it bears on, an opening that runs off the edge of the panel it is cut into. These compute the answer.
// ---------------------------------------------------------------------------

static void CheckBaysTile(void)
{
    float sum = 0.0f;
    for (int b = 0; b < BAYS; b++) sum += BAY_W[b];
    if (fabsf(sum - SECTION_LEN) > 1e-3f) {
        TraceLog(LOG_ERROR, "panelka: bays sum to %.3f m but SECTION_LEN is %.3f", sum, SECTION_LEN);
    }
    TraceLog(LOG_INFO, "panelka: %d sections of %.2f m, block %.2f x %.2f x %.2f m",
             SECTIONS, SECTION_LEN, BLOCK_LEN, 2.0f * FACE_Z, ROOF_Y + PARAPET_H);
}

// Every opening must sit inside the panel it is cut into, clear of the joint groove, and the openings of one panel must be listed left to right, which is what WallLayer's single sweep across a band assumes.
static void CheckOpenings(const char *name, float w, float h, const Rect *op, int nop)
{
    float worst = 1e9f;
    for (int i = 0; i < nop; i++) {
        worst = fminf(worst, op[i].x0 - (-w * 0.5f + JOINT));
        worst = fminf(worst, (w * 0.5f - JOINT) - op[i].x1);
        if (op[i].y0 > 1e-4f) worst = fminf(worst, op[i].y0 - JOINT);   // a doorway reaches the floor: there is no panel under it to leave a margin in
        worst = fminf(worst, (h - JOINT) - op[i].y1);
        if (i > 0 && op[i].x0 < op[i - 1].x1) {
            TraceLog(LOG_ERROR, "panelka: %s openings %d and %d are out of order or overlap", name, i - 1, i);
        }
    }
    if (nop > 0 && worst < 0.060f) {
        TraceLog(LOG_WARNING, "panelka: %s leaves only %.3f m of panel beside an opening", name, worst);
    }
}

static void CheckPanels(void)
{
    Rect w26 = WinRect(WIN_W26);
    CheckOpenings("2.6 m window panel", 2.6f, STOREY, &w26, 1);
    Rect w32 = WinRect(WIN_W32);
    CheckOpenings("3.2 m window panel", 3.2f, STOREY, &w32, 1);
    Rect bal[2] = {
        { -1.300f, -1.300f + BDOOR_W, BDOOR_SILL, BDOOR_SILL + BDOOR_H },
        { 0.100f, 0.100f + 1.200f, WIN_SILL, WIN_SILL + WIN_H },
    };
    CheckOpenings("balcony panel", 3.2f, STOREY, bal, 2);
    Rect st = { -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL, STAIR_SILL + STAIR_H };
    CheckOpenings("stairwell panel", 2.6f, STOREY, &st, 1);
    Rect en = { -ENTRY_W * 0.5f, ENTRY_W * 0.5f, 0.0f, ENTRY_H };
    CheckOpenings("entrance panel", 2.6f, STOREY, &en, 1);
}

// A window head must clear the ceiling it is cut under, and a balcony door must clear the slab it opens onto. Both are one subtraction, and both are the kind of thing that stops being true the moment someone changes STOREY.
static void CheckHeadroom(void)
{
    float ceiling = STOREY - SLAB_T;
    float win = ceiling - (WIN_SILL + WIN_H);
    float door = ceiling - (BDOOR_SILL + BDOOR_H);
    float stair = ceiling - (STAIR_SILL + STAIR_H);
    float entry = ceiling - ENTRY_H;
    TraceLog(LOG_INFO, "panelka: headroom over window %.3f, balcony door %.3f, stair light %.3f, entrance %.3f m",
             win, door, stair, entry);
    if (win < 0.0f || door < 0.0f || stair < 0.0f || entry < 0.0f) {
        TraceLog(LOG_ERROR, "panelka: an opening runs through the slab above it");
    }
}

// The balcony slab is cast into the facade rather than butted against it. Measure how much of its root is actually inside the panel, because a cantilever that only touches reads as a shelf stuck on with glue from every angle that shows the joint.
static void CheckBalconyRoot(void)
{
    // The balcony is placed at FACE_Z with its slab running from -0.200 back, so it reaches to FACE_Z - 0.200; the panel's inner face is at INNER_Z = FACE_Z - WALL_T.
    float embed = 0.200f;
    float through = embed - WALL_T;
    TraceLog(LOG_INFO, "panelka: balcony slab is cast %.3f m into a %.3f m panel", embed, WALL_T);
    if (through > 0.0f) {
        TraceLog(LOG_WARNING, "panelka: balcony slab pokes %.3f m out of the inner face", through);
    }
    if (embed < 0.100f) {
        TraceLog(LOG_WARNING, "panelka: balcony slab only reaches %.3f m into the facade", embed);
    }
}

// Every floor slab must bear on the facade at one end and reach the spine at the other, and the cross walls must stand between two slabs rather than through one.
static void CheckStructureMeets(void)
{
    float slabSpan = INNER_Z + SLAB_BEAR;   // spine centre to the far end of the slab, inside the panel
    float bearing = slabSpan - INNER_Z;
    TraceLog(LOG_INFO, "panelka: slab spans %.3f m from the spine and bears %.3f m into a %.3f m panel; the series casts to %.2f",
             slabSpan, bearing, WALL_T, HALF_D);
    if (bearing < 0.060f) {
        TraceLog(LOG_WARNING, "panelka: floor slab bears only %.3f m onto the facade", bearing);
    }
    if (bearing > WALL_T) {
        TraceLog(LOG_WARNING, "panelka: floor slab runs %.3f m out through the facade", bearing - WALL_T);
    }
    float wallH = STOREY - SLAB_T;
    if (wallH <= 0.0f) TraceLog(LOG_ERROR, "panelka: slab is thicker than the storey it sits in");
}

// Nothing may be left inside the ground. The plinth, the steps and the grass are the only things allowed to touch y = 0.
static void CheckNothingBuried(void)
{
    float lowest = 1e9f;
    int worst = -1;
    for (int i = 0; i < gFragCount; i++) {
        BoundingBox b = gType[gFrag[i].type].bounds;
        Matrix m = FragRest(&gFrag[i]);
        for (int c = 0; c < 8; c++) {
            Vector3 p = {
                (c & 1) ? b.max.x : b.min.x,
                (c & 2) ? b.max.y : b.min.y,
                (c & 4) ? b.max.z : b.min.z,
            };
            p = Vector3Transform(p, m);
            if (p.y < lowest) { lowest = p.y; worst = i; }
        }
    }
    TraceLog(LOG_INFO, "panelka: %d fragments, lowest sits at y = %.3f m (type %d)",
             gFragCount, lowest, worst >= 0 ? (int)gFrag[worst].type : -1);
    // A few millimetres under is a leg seated in the ground rather than balanced on it, which is
    // the same rule as every other joint here. Anything properly buried is a placement error.
    if (lowest < -0.050f) {
        TraceLog(LOG_WARNING, "panelka: a fragment reaches %.3f m below grade", -lowest);
    }
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

static const char *TYPE_NAME[FR_COUNT] = {
    "p26", "p32", "p32b", "pstair", "pdoor", "pend",
    "g26", "g32", "g32b", "gstair", "gdoor",
    "slab26", "slab32", "roof26", "roof32",
    "xwall", "spine26", "spine32",
    "balc", "par26", "par32", "parend", "canopy", "vent",
    "birch", "maple", "carbody", "cartrim", "bench", "rugframe", "lamp", "bin",
};

// The one scene-wide thing this model sets that is normally the harness's business, and it is
// here because the harness's three lights are point lights standing at (8,10,8), (-9,5,-7) and
// (0,-9,5). Those surround a 4.5 m vehicle. They sit *inside* a 58 m building, so every outward
// facing surface on it points away from all three, and lighting.fs falls back on its ambient
// term -- which line 75 divides by ten. That is measurable rather than arguable: a 118 texel
// with no light on it comes out of the gamma correction at 0.16, which is what turned the gable
// wall into a black slab.
// A directional light is the only kind whose geometry does not depend on the size of the scene,
// so this fills the shader's one free slot with a sun rather than moving the harness's lights,
// which would change every other model in the repo. The three point lights stay as they are and
// act as the fill.
static void AddSun(void)
{
    Shader sh = HarnessLightingShader();
    if (sh.id == 0) return;
    Light sun = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ 46.0f, 58.0f, 38.0f }, Vector3Zero(),
                            (Color){ 176, 168, 148, 255 }, sh);
    if (!sun.enabled) TraceLog(LOG_WARNING, "panelka: no free light slot, the far gable will render on ambient alone");
}

static void Init(void)
{
    AddSun();
    // Before any group is finished: GroupFinish hands each material its map, so both have to exist by then.
    MakeTextures();

    BuildFacadeTypes();
    BuildStructureTypes();
    BuildYardTypes();
    for (int t = 0; t < FR_COUNT; t++) {
        GroupFinish(&gType[t], TYPE_NAME[t]);
        GroupRegister(&gType[t]);
    }

    BuildPlinth();
    GroupFinish(&gPlinth, "plinth");
    GroupRegister(&gPlinth);

    BuildGround();
    GroupFinish(&gGround, "ground");
    GroupRegister(&gGround);

    PlaceBuilding();
    PlaceYard();
    SortFragments();
    Update(0.0f);

    bShell = InstBounds(PART_SHELL, COUNT_OF(PART_SHELL));
    bGlaz = InstBounds(PART_GLAZ, COUNT_OF(PART_GLAZ));
    bStruct = InstBounds(PART_STRUCT, COUNT_OF(PART_STRUCT));
    bRoof = InstBounds(PART_ROOF, COUNT_OF(PART_ROOF));
    bBalc = InstBounds(PART_BALC, COUNT_OF(PART_BALC));
    bEntry = InstBounds(PART_ENTRY, COUNT_OF(PART_ENTRY));
    bPlinth = InstBounds(PART_PLINTH, COUNT_OF(PART_PLINTH));
    bGround = InstBounds(PART_GROUND, COUNT_OF(PART_GROUND));
    bTrees = InstBounds(PART_TREES, COUNT_OF(PART_TREES));
    bCars = InstBounds(PART_CARS, COUNT_OF(PART_CARS));
    bYard = InstBounds(PART_YARD, COUNT_OF(PART_YARD));

    CheckBaysTile();
    CheckPanels();
    CheckHeadroom();
    CheckBalconyRoot();
    CheckStructureMeets();
    CheckNothingBuried();
}

static void Unload(void)
{
    for (int i = 0; i < gAllCount; i++) GroupUnload(gAll[i]);
    // UnloadModel frees a material's maps array but deliberately not the textures in it, since they may be shared; vendor/raylib/src/rmodels.c:1200 says so and leaves them to the caller. They are shared here, one set across every group, so they are freed once and only here.
    UnloadTextures();
}

const Scene SCENE = {
    .name = "panelka",
    .description =
        "A three-section five-storey 1-464 panel block -- a khrushchyovka -- standing in its own courtyard.\n"
        "Static so far: the demolition it exists for is not built yet.\n"
        "\n"
        "One world unit is one metre. 58.20 m along the block, 11.82 m over the facades, 14.70 m to the top of the parapet.\n"
        "Origin sits on the ground at the centre of the footprint, +X along the block, +Z out through the main facade.\n"
        "\n"
        "Dimensions, and where they come from.\n"
        "The series specifies outer panels 2.6 and 3.2 m wide, floor slabs spanning 5.76 m, a 2.5 m clear ceiling, a 0.10 m slab and 0.21 to 0.35 m outer walls.\n"
        "references/panelka/ref_01.png is a plan of one section; scaled by that 5.76 m span it comes out at 66.9 px/m, and reading the section back off that scale gives seven bays of 2.69, 3.24, 2.64, 2.57, 2.54, 3.23 and 2.66 m over an 11.52 m depth between the outer wall centre-lines.\n"
        "Those are the specified 2.6 and 3.2 m to within 3.5 per cent, so the bays here are 2.6, 3.2, 2.6, 2.6, 2.6, 3.2, 2.6 = 19.40 m per section, the depth is 2 x 5.76 plus one 0.30 m wall, and the storey is 2.5 clear + 0.10 slab + 0.10 of floor build-up = 2.70 m.\n"
        "\n"
        "The building is precast panels, not a shell with holes in it.\n"
        "There is one mesh per panel type and a list of 1022 transforms saying where the instances go, which is what makes the joint grid real rather than drawn on, and which is the decomposition the demolition will throw.\n"
        "24 types: six facade panels, five sets of joinery, floor and roof slabs in both bay widths, cross walls, spine wall segments, balconies, three parapet lengths, the entrance canopy and a vent stack.\n"
        "\n"
        "Each panel is a structural core 0.26 m thick across its full width plus a face plate standing 0.04 m proud and inset 0.03 m all round, so two neighbours leave a 0.06 m groove between them.\n"
        "Every piece is then grown 6 mm past its own edge so it knits into its neighbour instead of meeting it on an exactly coincident plane: two coplanar back-to-back faces are what this file draws hundreds of, and left touching they speckle, which ruled the first build's facade with dashed lines down every joint.\n"        "Floor slabs bear 0.10 m into the panel they land on for the same reason, which puts their span at 5.71 m from the spine against the 5.76 the series casts to.\n"
        "Openings are cut by decomposing the panel into horizontal bands at every opening edge and, within a band, into the x-segments the openings leave: any number of openings then needs no special case, and every reveal is the side of a closed box.\n"
        "\n"
        "Composition.\n"
        "The stairwell is the middle 2.6 m bay of each section and touches the front facade, so its tall narrow lights are on the front and the entrance is on the back, which is the courtyard side.\n"
        "The two 3.2 m bays of each section carry balconies on both facades and on all five floors, which is what ref_07 shows.\n"
        "Apartment windows are 1.30 m wide in a 2.6 m bay and 1.50 in a 3.2 m one, 1.45 m tall on a 0.85 m sill, with a centre mullion and a transom; the stair light is 0.95 by 1.30 on a 1.00 m sill, so it reads as a different window rather than the same one moved up.\n"
        "The end walls are blank: ref_02, ref_05 and ref_07 all show a gable with no openings, which is why ref_04 exists to show one painted instead.\n"
        "\n"
        "Behind the skin the structure is real, because the demolition has to break it: floor slabs spanning bay by bay from the facade to the central spine, a spine wall segment per bay, and a cross wall on every bay boundary, which is the load-bearing direction in this series.\n"
        "Windows are opaque, not transparent: the alternative is sixty modelled interiors, and a dark pane with a bright frame is what a window reads as from outside on an overcast day anyway.\n"
        "\n"
        "The yard.\n"
        "Sixteen trees, five cars, six benches, two rug-beating frames, two bins and four lamp posts, all of them instanced types like the building, because a tree the blast is going to lash and a car it is going to bury both have to be things the pose function can move.\n"
        "A tree crown is twenty-two lumpy ellipsoids on a golden-angle spiral over its own crown ellipsoid, each about a quarter of the crown radius, with the normal taken from the underlying ellipsoid rather than the lumpy surface: a true normal shades every lump as its own object and the crown breaks into a bag of boulders. An earlier build used seven blobs of up to 0.8 of the radius and every tree came out a cabbage, because at that size one blob's silhouette is the whole tree's.\n"
        "The car is a VAZ-2101 from the specification and ref_08.jpg: 4.073 by 1.611 by 1.382, 2.424 wheelbase, 1.349 front track, 0.170 clearance, with the beltline read off the elevation at 0.83 and the bonnet crown standing above it rather than below, which is what gives the car its flat-topped nose. Its wheel arches are swept as vertical strips whose floor follows the arch circle, in the outer 0.22 m of each flank only, so the arch is a recess rather than a tunnel through the car. Without that cut the 1.349 track sits entirely inside the 1.611 body and the car reads as a crate.\n"
        "Four cars share one body mesh and differ by the per-instance tint, which is why the body and the trim are separate types: tinting one instance tints all of its materials, and a green car should not have green glass or green tyres.\n"
        "\n"
        "The plinth, the entrance steps and the ground exist once each and are placed groups rather than instanced ones.\n"
        "One thing here is normally the harness's business: a fourth light. The harness lights with three point lights standing at (8,10,8), (-9,5,-7) and (0,-9,5), which surround a 4.5 m vehicle and sit inside a 58 m building, so every outward-facing surface on this one points away from all three and falls back on lighting.fs's ambient term, which line 75 divides by ten -- a 118 texel then leaves the gamma correction at 0.16, which is what turned the gable into a black slab.\n"        "A directional light is the only kind whose geometry does not depend on how big the scene is, so this model fills the shader's one free slot with a sun and leaves the three point lights as the fill, rather than moving them and changing every other model in the repo.\n"
        "Surfacing is eight maps built in code from tiling value noise, projected planar in each mesh's own frame rather than the world's, because an instanced mesh has no world position; every instance would then carry an identical texture placement, so each also carries a hashed per-instance shade of +-8 to break that up.",
    .init = Init,
    .draw = DrawAll,
    .unload = Unload,
    .update = Update,
    .duration = CYCLE,
    .animYaw = 35.0f,
    .parts = PARTS,
    .partCount = COUNT_OF(PARTS),
    .target = { 0.0f, 6.0f, 0.0f },
    .orbitRadius = 62.0f,
    .orbitHeight = 26.0f,
    .hideGrid = true,
};
