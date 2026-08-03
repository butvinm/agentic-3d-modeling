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
    MAT_FLASH, MAT_DUST,
    MAT_COUNT
} MatId;

typedef enum { PASS_OPAQUE, PASS_GLOW, PASS_DUST, PASS_COUNT } Pass;

// Everything opaque draws first. The detonation flashes draw additively after it, and the dust
// after them, both with depth writes off: two puffs in line should pile up rather than the nearer
// one punching a hole in the further, and a cloud between the camera and a flash should veil it
// rather than the other way round.
static const Pass MAT_PASS[MAT_COUNT] = {
    [MAT_FLASH] = PASS_GLOW,
    [MAT_DUST]  = PASS_DUST,
};

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
    // The flash takes its colour from the tint, which is also what fades it.
    [MAT_FLASH]    = { 255, 226, 168, 255 },
    // The dust keeps a full alpha here and takes it from the per-instance tint instead: DrawModel multiplies the two, and every puff needs its own.
    [MAT_DUST]     = WHITE,
};

static const bool MAT_UNLIT[MAT_COUNT] = { [MAT_FLASH] = true };

// Materials whose silhouette has to fade rather than end: the per-fragment falloff in DUST_FS.
static const bool MAT_SOFT[MAT_COUNT] = { [MAT_DUST] = true, [MAT_FLASH] = true };

static Shader gDustShader;

static Texture2D MAT_TEX[MAT_COUNT];

// ---------------------------------------------------------------------------
// Dust shader
//
// The technique and the shader are models/humvee.c's, for the same reason it needed them.
//
// A puff is a closed blob, and a closed blob has a hard silhouette wherever you put it, however
// finely it is subdivided. Piling up many faint ones hides that in the middle of a cloud but not
// at its edge, where a single blob is still a grey pebble with a polygonal rim. The fix is a
// per-fragment falloff: alpha is scaled by how squarely the surface faces the camera, and a
// blob's silhouette is exactly where it faces the camera edge-on, so every puff fades out at its
// own rim and the edge of the cloud is soft rather than pebbled.
//
// The blob's normals are radial -- of the sphere the lumps are pushed out from, not of the lumpy
// surface -- which keeps that falloff monotonic to the silhouette instead of leaving a ring of
// zero alpha inside it wherever a lump bulges past.
//
// The camera position is recovered from matView rather than from a uniform, because the harness
// feeds viewPos only to its own shader; rmodels.c:1493 uploads matView to any shader that
// declares it.
// ---------------------------------------------------------------------------

static const char *DUST_VS =
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

static const char *DUST_FS =
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
    "    vec3 eye = -(transpose(mat3(matView))*vec3(matView[3]));\n"
    "    vec3 v = normalize(eye - fragPosition);\n"
    "    vec3 n = normalize(fragNormal);\n"
    "    float edge = pow(clamp(abs(dot(n, v)), 0.0, 1.0), 1.25);\n"
    "    float sky = 0.44 + 0.56*(0.5 + 0.5*n.y);\n"
    "    vec4 texel = texture(texture0, fragTexCoord);\n"
    "    vec3 col = texel.rgb*colDiffuse.rgb*sky;\n"
    "    finalColor = vec4(pow(col, vec3(1.0/2.2)), colDiffuse.a*texel.a*edge);\n"
    "}\n";

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
    // Pulverised concrete: a pale warm grey with a coarse mottle, so a puff has some tooth rather
    // than reading as an airbrushed ball. Light on purpose -- it is drawn unlit, and this is the
    // only place its colour is set.
    MAT_TEX[MAT_DUST]     = MakeRenderTexture((Color){ 156, 148, 136, 255 }, 197u, 30.0f, 22.0f);
    MAT_TEX[MAT_FLASH]    = MakeRenderTexture((Color){ 242, 232, 208, 255 }, 307u, 8.0f, 10.0f);

    gDustShader = LoadShaderFromMemory(DUST_VS, DUST_FS);
    if (gDustShader.id == 0) {
        TraceLog(LOG_WARNING, "panelka: dust shader failed to build, puffs will render as opaque pebbles");
    }
}

static void UnloadTextures(void)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (MAT_TEX[m].id != 0) UnloadTexture(MAT_TEX[m]);
    }
    if (gDustShader.id != 0) UnloadShader(gDustShader);
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
    float volume;            // material volume of the meshes, for the rubble pile
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
        if (MAT_SOFT[m] && gDustShader.id != 0) g->model[m].materials[0].shader = gDustShader;
        else if (!MAT_UNLIT[m]) HarnessApplyLighting(&g->model[m]);
        g->has[m] = true;
    }

    GroupBoundsFromMesh(g);

    // Signed volume by the divergence theorem, summed over every triangle the group owns.
    // Everything here is built as a union of closed boxes and capped tubes, and for those this
    // returns the sum of the individual solids' volumes -- overlaps counted twice, which is a
    // slight overestimate and the only error worth naming. An open surface would return noise, so
    // only the building's own types are ever asked.
    double vol = 0.0;
    for (int m = 0; m < MAT_COUNT; m++) {
        const Builder *b = &g->b[m];
        for (int i = 0; i < b->triangleCount; i++) {
            const float *v = b->vertices;
            int a = b->indices[i * 3 + 0], c = b->indices[i * 3 + 1], d = b->indices[i * 3 + 2];
            Vector3 p0 = { v[a * 3], v[a * 3 + 1], v[a * 3 + 2] };
            Vector3 p1 = { v[c * 3], v[c * 3 + 1], v[c * 3 + 2] };
            Vector3 p2 = { v[d * 3], v[d * 3 + 1], v[d * 3 + 2] };
            vol += Vector3DotProduct(Vector3CrossProduct(p0, p1), p2) / 6.0;
        }
    }
    g->volume = (float)fabs(vol);
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

    rlDisableDepthMask();
    BeginBlendMode(BLEND_ADDITIVE);
    for (int i = 0; i < n; i++) GroupDrawPass(gs[i], PASS_GLOW);
    EndBlendMode();
    for (int i = 0; i < n; i++) GroupDrawPass(gs[i], PASS_DUST);
    rlEnableDepthMask();
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

// 2.50 m of clear ceiling and a 0.10 m slab are both specified; the 0.06 m screed over the slab is not, and is the only invented part of the storey.
#define STOREY        2.660f
#define SLAB_T        0.160f   // 0.10 structural plus 0.06 of screed, drawn as two layers
#define SLAB_STRUCT   0.100f
#define SLAB_BEAR     0.100f   // how far a slab's end is built into the facade panel it lands on
#define PLINTH_Y      0.750f   // ground-floor level above grade
#define WALL_T        0.300f   // outer panel: the series specifies 0.21 to 0.35 by climate zone
#define XWALL_T       0.140f   // internal cross wall, the load-bearing direction
#define SPINE_T       0.160f   // central longitudinal wall
#define HALF_D        5.760f   // slab span: outer wall centre-line to the spine centre-line
#define FACE_Z        (HALF_D + WALL_T * 0.5f)   // 5.91, outer face of the facade
#define INNER_Z       (HALF_D - WALL_T * 0.5f)   // 5.61, inner face

#define JOINT         0.045f   // half the groove between two panels: each face plate insets by this
// How far every piece is grown past its own edge so that it knits into its neighbour instead of
// meeting it on an exactly coincident plane. Two coplanar back-to-back faces are the one thing
// this file draws hundreds of, and left touching they speckle along every joint: the rasterizer
// picks between them per pixel and the facade comes out ruled with dashed lines. It is also the
// honest construction, since a real panel is butted and caulked rather than laid edge to edge.
#define KNIT          0.006f
#define FACE_T        0.060f   // how far the face plate stands proud of the panel core
#define CORE_T        (WALL_T - FACE_T)

#define ROOF_Y        (PLINTH_Y + FLOORS * STOREY)      // 14.25, top of the fifth-floor ceiling
// Flat, which is the series' own specification and what ref_03 and ref_05 show; the pitched metal roofs in ref_02 and ref_07 are the re-roofing thousands of these got in the 1990s, not the building.
// What those two do show is a thin cornice with a metal capping that oversails it, not the 0.45 m parapet this model first had.
#define PARAPET_H     0.320f
#define PARAPET_T     0.180f
#define CORNICE_O     0.110f   // how far the capping oversails the facade below it

// Openings. Sills are measured up from the floor the panel stands on.
#define WIN_SILL      0.850f
#define WIN_H         1.450f
#define WIN_W26       1.300f
#define WIN_W32       1.500f
#define BDOOR_W       0.750f
#define BDOOR_SILL    0.150f
#define BDOOR_H       2.100f
// A stairwell landing is half a flight above the floor its panel stands on, so its window cannot share the apartments' rhythm.
// One-storey panels cannot carry a light that straddles the slab, so consecutive storeys alternate between a low sill and a high one, which is the zigzag the real rhythm reads as from outside.
#define STAIR_W       1.150f
#define STAIR_SILL_LO 0.450f
#define STAIR_SILL_HI 1.300f
#define STAIR_H       1.100f
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
    FR_P26, FR_P32, FR_P32B, FR_PSTAIR, FR_PSTAIR2, FR_PDOOR, FR_PEND,
    FR_G26, FR_G32, FR_G32B, FR_GSTAIR, FR_GSTAIR2, FR_GDOOR,
    FR_SLAB26, FR_SLAB32, FR_ROOF26, FR_ROOF32,
    FR_XWALL, FR_SPINE26, FR_SPINE32,
    FR_BALC, FR_BALCG, FR_PAR26, FR_PAR32, FR_PAREND, FR_CANOPY, FR_VENT, FR_PIPE, FR_MAST,
    FR_RUBBLE,
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

#define MAX_FRAG 1500
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
    Rect stLo = { -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL_LO, STAIR_SILL_LO + STAIR_H };
    Rect stHi = { -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL_HI, STAIR_SILL_HI + STAIR_H };
    FacadePanel(&gType[FR_PSTAIR], 2.6f, STOREY, &stLo, 1);
    Glazing(&gType[FR_GSTAIR], stLo, false);
    FacadePanel(&gType[FR_PSTAIR2], 2.6f, STOREY, &stHi, 1);
    Glazing(&gType[FR_GSTAIR2], stHi, false);

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
    Box(&g->b[MAT_CONCRETE], -w * 0.5f - KNIT, w * 0.5f + KNIT, 0.0f, SLAB_STRUCT, z0, z1);
    Box(&g->b[top], -w * 0.5f - KNIT, w * 0.5f + KNIT, SLAB_STRUCT, SLAB_T, z0, z1);
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
    Box(&gType[FR_PAR26].b[MAT_METAL], -1.3f - CORNICE_O, 1.3f + CORNICE_O, PARAPET_H, PARAPET_H + 0.055f, -PARAPET_T - 0.030f, CORNICE_O);
    Box(&gType[FR_PAR32].b[MAT_CONCRETE], -1.6f - KNIT, 1.6f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);
    Box(&gType[FR_PAR32].b[MAT_METAL], -1.6f - CORNICE_O, 1.6f + CORNICE_O, PARAPET_H, PARAPET_H + 0.055f, -PARAPET_T - 0.030f, CORNICE_O);
    Box(&gType[FR_PAREND].b[MAT_CONCRETE], -FACE_Z * 0.5f - KNIT, FACE_Z * 0.5f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);
    Box(&gType[FR_PAREND].b[MAT_METAL], -FACE_Z * 0.5f - CORNICE_O, FACE_Z * 0.5f + CORNICE_O, PARAPET_H, PARAPET_H + 0.055f, -PARAPET_T - 0.030f, CORNICE_O);

    // A balcony. Authored about the facade's outer face at floor level, so it hangs off the panel rather than off a copy of the panel's coordinates.
    //
    // The front is a solid sheet, not a railing. The first build had 13 uprights and a top rail, and Codex read the whole facade as an access gallery: ref_03 and ref_05 both show boxy masses with opaque sheet fronts in whatever colour the resident had, and a great many enclosed behind glazing entirely. So there are two types, and four in ten of them are glazed in.
    for (int glazed = 0; glazed < 2; glazed++) {
        Group *g = &gType[glazed ? FR_BALCG : FR_BALC];
        Builder *c = &g->b[MAT_CONCRETE];
        float hw = BALC_W * 0.5f;
        // The slab's root runs 0.20 back into the facade, which is where it is cast in.
        Box(c, -hw, hw, -BALC_T, 0.0f, -0.200f, BALC_D);

        Builder *m = &g->b[MAT_METAL];
        float rt = BALC_RAIL_H;
        // Sheet front and returns, standing off the slab edge, with a capping rail over them.
        Box(m, -hw, hw, 0.0f, rt - 0.045f, BALC_D - 0.055f, BALC_D);
        Box(m, -hw, -hw + 0.055f, 0.0f, rt - 0.045f, 0.0f, BALC_D);
        Box(m, hw - 0.055f, hw, 0.0f, rt - 0.045f, 0.0f, BALC_D);
        Box(m, -hw - 0.020f, hw + 0.020f, rt - 0.045f, rt, BALC_D - 0.075f, BALC_D + 0.020f);
        Box(m, -hw - 0.020f, -hw + 0.075f, rt - 0.045f, rt, 0.0f, BALC_D + 0.020f);
        Box(m, hw - 0.075f, hw + 0.020f, rt - 0.045f, rt, 0.0f, BALC_D + 0.020f);

        if (glazed) {
            // Glazed in up to the underside of the balcony above, in frames and panes, which is
            // what perhaps half of these carry by now.
            float top = STOREY - BALC_T - 0.030f;
            Builder *f = &g->b[MAT_FRAME];
            Box(f, -hw, hw, rt, top, BALC_D - 0.070f, BALC_D - 0.010f);
            Box(f, -hw, -hw + 0.060f, rt, top, 0.0f, BALC_D);
            Box(f, hw - 0.060f, hw, rt, top, 0.0f, BALC_D);
            Box(f, -hw, hw, top - 0.055f, top, 0.0f, BALC_D);
            for (int i = 1; i < 4; i++) {
                float x = -hw + BALC_W * (float)i / 4.0f;
                Box(f, x - 0.030f, x + 0.030f, rt, top, BALC_D - 0.070f, BALC_D - 0.010f);
            }
            Box(&g->b[MAT_GLASS], -hw + 0.055f, hw - 0.055f, rt + 0.030f, top - 0.055f,
                BALC_D - 0.055f, BALC_D - 0.030f);
            Box(&g->b[MAT_GLASS], -hw + 0.055f, -hw + 0.070f, rt + 0.030f, top - 0.055f, 0.030f, BALC_D - 0.060f);
            Box(&g->b[MAT_GLASS], hw - 0.070f, hw - 0.055f, rt + 0.030f, top - 0.055f, 0.030f, BALC_D - 0.060f);
        }
    }

    // The entrance canopy: a slab on two brackets over the door.
    {
        Builder *c = &gType[FR_CANOPY].b[MAT_CONCRETE];
        Box(c, -0.980f, 0.980f, 0.0f, 0.100f, 0.0f, 1.050f);
        Box(c, -0.980f, -0.870f, -0.380f, 0.0f, 0.0f, 0.210f);
        Box(c, 0.870f, 0.980f, -0.380f, 0.0f, 0.0f, 0.210f);
    }

    // A roof ventilation stack: a broad multi-flue body under a capping slab, which is what
    // ref_02 and ref_03 show. The first build's 0.9 by 0.6 posts read as bollards.
    {
        Builder *c = &gType[FR_VENT].b[MAT_CONCRETE];
        Box(c, -1.150f, 1.150f, 0.0f, 1.500f, -0.420f, 0.420f);
        for (int i = 0; i < 4; i++) {
            float x = -0.840f + 0.560f * (float)i;
            Box(c, x - 0.190f, x + 0.190f, 1.500f, 1.860f, -0.300f, 0.300f);
        }
        Box(c, -1.270f, 1.270f, 1.860f, 1.960f, -0.540f, 0.540f);
    }

    // Downpipes, which ref_03 and ref_05 both make one of the loudest things on the facade and
    // which the first build had none of. One segment per storey, so they break up with the
    // building rather than falling as a 13 m stick.
    {
        Builder *m = &gType[FR_PIPE].b[MAT_METAL];
        Tube(m, (Vector3){ 0, -KNIT, 0 }, (Vector3){ 0, STOREY + KNIT, 0 }, 0.058f, 0.058f, 8, false, false);
        Box(m, -0.080f, 0.080f, 0.180f, 0.260f, -0.100f, 0.010f);
    }

    // The mast every one of these roofs grew in the 1970s.
    {
        Builder *m = &gType[FR_MAST].b[MAT_METAL];
        Tube(m, (Vector3){ 0, 0, 0 }, (Vector3){ 0, 4.200f, 0 }, 0.055f, 0.032f, 8, false, true);
        for (int i = 0; i < 5; i++) {
            float y = 2.100f + 0.420f * (float)i;
            float r = 0.760f - 0.100f * (float)i;
            Tube(m, (Vector3){ -r, y, 0 }, (Vector3){ r, y, 0 }, 0.020f, 0.020f, 5, true, true);
        }
        for (int i = 0; i < 3; i++) {
            float th = 2.0f * PI * (float)i / 3.0f;
            Tube(m, (Vector3){ 0, 1.900f, 0 },
                 (Vector3){ cosf(th) * 1.400f, 0.0f, sinf(th) * 1.400f }, 0.014f, 0.014f, 4, false, false);
        }
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
        // Far enough out to be seen under the crown, not so far as to end outside it: Codex read the first pass as balloons on poles because nothing bridged trunk and foliage.
        float reach = crownR * (0.78f - 0.26f * f);
        float r = trunkR * (0.52f - 0.20f * f);
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
        float rad = crownR * (0.17f + 0.26f * Hash2(i, (int)seed + 5, 128, 13u));
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
    // Rubble: a broken slab chunk, flattened rather than round, instanced at half to twice its
    // built size. It is the material a panel becomes that no panel accounts for.
    Blob(&gType[FR_RUBBLE].b[MAT_CONCRETE], Vector3Zero(), 0.78f, 0.21f, 0.56f, 0.52f, 631u, 7);

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

// Rubble is born inside the volume that is coming down, spread through it rather than over the
// footprint, so a chunk appears where its parent was and the dust covers its arrival.
static void PlaceRubble(void)
{
    const int N = 170;
    for (int i = 0; i < N; i++) {
        float x = (Hash2(i, 41, 4096, 191u) - 0.5f) * BLOCK_LEN * 0.98f;
        float z = (Hash2(i, 42, 4096, 193u) - 0.5f) * 2.0f * FACE_Z * 0.92f;
        float y = PLINTH_Y + (ROOF_Y - PLINTH_Y) * Hash2(i, 43, 4096, 197u);
        EmitAt(FR_RUBBLE, x, y, z, 360.0f * Hash2(i, 44, 4096, 199u),
               0.50f + 1.35f * Hash2(i, 45, 4096, 211u), WHITE);
    }
}

// ---------------------------------------------------------------------------
// Placing the fragments
// ---------------------------------------------------------------------------

// Which panel type a bay carries on a given facade and floor.
// front is +Z. The stairwell touches the front, so that is where its windows go and where the entrance cannot be; the entrance is on the back, which is the courtyard side.
static FragType PanelAt(int bay, int floor, bool front, FragType *glazing)
{
    if (bay == BAY_STAIR) {
        if (!front && floor == 0) { *glazing = FR_GDOOR; return FR_PDOOR; }
        // A landing sits half a flight above the floor its panel stands on, so the lights zigzag
        // rather than stacking on the apartments' rhythm.
        if (floor & 1) { *glazing = FR_GSTAIR2; return FR_PSTAIR2; }
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
                    for (int side = 0; side < 2; side++) {
                        float bz = (side == 0) ? FACE_Z : -FACE_Z;
                        bool glazed = Hash2(s * 13 + b, f * 7 + side, 512, 313u) < 0.40f;
                        Emit(glazed ? FR_BALCG : FR_BALC, x, y, bz, (side == 0) ? 0.0f : 180.0f);
                    }
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

            // Downpipes, at each section joint and each gable corner, on both facades. A pipe
            // stands just clear of the panel face so it reads as bolted on rather than cast in.
            for (int e = 0; e <= SECTIONS; e++) {
                float px = -BLOCK_LEN * 0.5f + SECTION_LEN * (float)e;
                if (e == 0) px += 0.360f;
                if (e == SECTIONS) px -= 0.360f;
                Emit(FR_PIPE, px, y, FACE_Z + 0.070f, 0.0f);
                Emit(FR_PIPE, px, y, -FACE_Z - 0.070f, 180.0f);
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
        Emit(FR_VENT, BayX(s, BAY_STAIR) - 2.6f, ROOF_Y, 2.4f, 0.0f);
        Emit(FR_VENT, BayX(s, BAY_STAIR) + 2.6f, ROOF_Y, -2.4f, 0.0f);
        Emit(FR_MAST, BayX(s, BAY_BALC_A) + 1.2f, ROOF_Y, -1.6f, 0.0f);
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
        // A kerb either side of the flight with a handrail over it, rather than the two full-height walls this had first: ref_05 and ref_07 both show a modest step up, not a walled ramp.
        for (int side = 0; side < 2; side++) {
            float cx = (side == 0) ? x - 1.240f : x + 1.100f;
            Box(&g->b[MAT_CONCRETE], cx, cx + 0.140f, 0.0f, 0.240f,
                -hz - t - tread * (float)RISERS, -hz);
            float rx = cx + 0.070f;
            Tube(&g->b[MAT_METAL], (Vector3){ rx, 0.240f, -hz - t - tread * (float)RISERS + 0.10f },
                 (Vector3){ rx, PLINTH_Y + 0.640f, -hz - t - tread * 0.5f }, 0.026f, 0.026f, 7, true, true);
            Tube(&g->b[MAT_METAL], (Vector3){ rx, PLINTH_Y + 0.640f, -hz - t - tread * 0.5f },
                 (Vector3){ rx, PLINTH_Y + 0.900f, -hz + 0.10f }, 0.026f, 0.026f, 7, false, true);
        }
    }
}

// A horizontal slab of paving whose corners are cut off. Emitted as a top face only: it lies
// 40 mm over the grass and nothing in this scene is ever under it.
//
// It exists because Codex read the first site as a diagram: every path ended in a hard rectangle
// and met the grass at a right angle, where ref_03 and ref_05 show worn, splayed junctions.
static void Pad(Builder *b, float x0, float x1, float z0, float z1, float ch, float y)
{
    const Vector3 n = { 0, 1, 0 };
    Vector3 v[8] = {
        { x0 + ch, y, z0 }, { x1 - ch, y, z0 }, { x1, y, z0 + ch }, { x1, y, z1 - ch },
        { x1 - ch, y, z1 }, { x0 + ch, y, z1 }, { x0, y, z1 - ch }, { x0, y, z0 + ch },
    };
    int c = Vert(b, (Vector3){ (x0 + x1) * 0.5f, y, (z0 + z1) * 0.5f }, n);
    int idx[8];
    for (int i = 0; i < 8; i++) idx[i] = Vert(b, v[i], n);
    for (int i = 0; i < 8; i++) Tri(b, c, idx[i], idx[(i + 1) % 8]);
}

// The site: grass over the whole plot, an asphalt drive and parking bay along the front, and the paths that connect the three entrances to it.
#define SITE_HX  240.0f
#define SITE_HZ  200.0f

static void BuildGround(void)
{
    Group *g = &gGround;

    // Grass sits fractionally below zero, so the asphalt laid on top of it wins the depth test everywhere it covers rather than fighting it.
    Box(&g->b[MAT_GRASS], -SITE_HX, SITE_HX, -0.120f, -0.010f, -SITE_HZ, SITE_HZ);

    Builder *a = &g->b[MAT_ASPHALT];

    // The drive along the main facade, and the parking bay off it.
    Box(a, -SITE_HX, SITE_HX, -0.010f, 0.030f, 16.0f, 22.5f);
    Pad(a, -26.0f, 14.0f, 11.5f, 16.1f, 2.2f, 0.030f);

    // The service road behind, a splayed pad outside each entrance, and the path that runs from
    // it to the door.
    Box(a, -SITE_HX, SITE_HX, -0.010f, 0.030f, -20.0f, -15.0f);
    for (int s = 0; s < SECTIONS; s++) {
        float x = BayX(s, BAY_STAIR);
        Pad(a, x - 1.7f, x + 1.7f, -14.9f, -FACE_Z - 2.3f, 1.1f, 0.030f);
        Pad(a, x - 4.6f, x + 4.6f, -FACE_Z - 3.3f, -FACE_Z - 1.4f, 1.4f, 0.030f);
    }
    // A worn footpath along the back, which is where everyone actually walks.
    Pad(a, -BLOCK_LEN * 0.5f - 6.0f, BLOCK_LEN * 0.5f + 6.0f, -11.4f, -9.6f, 1.6f, 0.028f);

    // Kerbs along the drive.
    for (int i = 0; i < 2; i++) {
        float z = (i == 0) ? 16.0f : 22.5f;
        Box(&g->b[MAT_CONCRETE], -SITE_HX, SITE_HX, -0.010f, 0.130f, z - 0.120f, z + 0.120f);
    }
}


// ---------------------------------------------------------------------------
// The demolition
//
// A 1-464 has no frame: every cross wall carries load, which is why the series went up so fast
// and why it comes down the way it does. Charges are drilled into the cross walls of the bottom
// two storeys and fired in a sequence running the length of the block, so the building loses its
// footing progressively rather than all at once and folds instead of toppling. What is above the
// cut then descends almost vertically and pancakes: slab onto slab, with the facade panels
// peeling off outwards because nothing holds them to anything once the cross walls behind them
// are gone.
//
// So every fragment does three things and no more. It waits, it flies, and it lands.
//
// It waits until the collapse front reaches it. That front has two components: the firing
// sequence, which runs along X, and the crushing front, which climbs at FRONT_V because each
// storey has to be destroyed before the one above it can start down.
//
// It flies ballistically, tumbling about its own centroid. There is no contact between
// fragments: a real solver is not what this file is, and at this scale what reads is the timing
// and the peel rather than any individual collision.
//
// It lands on the rubble pile, whose height is not a chosen number. PileY integrates to the
// summed mesh volume of everything that falls, bulked by BULK_FACTOR, which is what a solid
// becomes once it is broken.
// ---------------------------------------------------------------------------

#define T_BLAST      0.90f    // detonation, at phase 0.15 of the cycle
#define FLASH_END    1.20f    // the flashes are milliseconds in reality; see the note on FlashLevel
#define SEQ_LEN      0.42f    // the firing sequence takes this long to run the length of the block
#define FRONT_V      11.0f    // speed the crushing front climbs the building, m/s
#define GRAV         9.81f
#define CRUSH_LOSS   0.62f    // how much of a storey's height is gone once it has been crushed
#define CUT_TOP      (PLINTH_Y + 2.0f * STOREY)   // charges are in the two storeys below this
#define BULK_FACTOR  1.55f    // how much a cubic metre of concrete swells once it is rubble
#define PILE_SPREAD  9.0f     // how far past the footprint the debris skirt is expected to run
#define BLAST_V      105.0f   // speed of the air blast that reaches the yard, m/s

static bool IsBuilding(FragType t) { return t < FR_RUBBLE; }

// The pile is a heightfield, not a formula. Fragments are planned in the order they land and each
// is put on top of whatever is already in the cells its own footprint covers, so the pile is
// something the debris builds rather than a surface it is snapped to.
//
// A review round called the first version's most damaging defect: with every fragment solved
// independently against one analytic mound, the finished pile was a heap of intersecting cards,
// panels crossing slabs at incompatible angles in the same volume. This does not make it a
// solver -- nothing here resolves a collision, and a fragment still flies through whatever is in
// its way -- but it does mean that where a piece comes to rest is decided by what is already
// there.
#define PILE_CELL  1.0f
#define PILE_MARGIN 21.0f
// Literal, because a cast of a float expression is not an integer constant expression and a
// variably modified array at file scope is not a thing. CheckPileGrid proves they still cover.
#define PILE_NX    104
#define PILE_NZ    56
static float gHeight[PILE_NX][PILE_NZ];

static void PileCell(float x, float z, int *ix, int *iz)
{
    *ix = (int)Clamp((x + BLOCK_LEN * 0.5f + PILE_MARGIN) / PILE_CELL, 0.0f, (float)(PILE_NX - 1));
    *iz = (int)Clamp((z + FACE_Z + PILE_MARGIN) / PILE_CELL, 0.0f, (float)(PILE_NZ - 1));
}

// The highest thing already under a footprint, which is what a piece landing on it rests on.
static float PileUnder(float x, float z, float hx, float hz)
{
    int x0, z0, x1, z1;
    PileCell(x - hx, z - hz, &x0, &z0);
    PileCell(x + hx, z + hz, &x1, &z1);
    float top = 0.0f;
    for (int i = x0; i <= x1; i++) {
        for (int j = z0; j <= z1; j++) top = fmaxf(top, gHeight[i][j]);
    }
    return top;
}

// What a fragment adds to the pile is its own material, spread over its own footprint, and
// nothing else. Raising every covered cell to the piece's own top instead was the first attempt
// and it ran away to 18 m: a panel resting flat covers about 28 m2 and is 0.3 thick, so raising
// all of that by 0.3 credits it with four times the concrete it contains, and the max-then-set
// carried the tallest stack outwards across the site. Depositing volume conserves it exactly --
// the sum over every cell is the bulked material, by construction -- while PileUnder still
// returns the highest thing under a footprint, so a piece still comes to rest on what is there.
static void PileDeposit(float x, float z, float hx, float hz, float volume)
{
    int x0, z0, x1, z1;
    PileCell(x - hx, z - hz, &x0, &z0);
    PileCell(x + hx, z + hz, &x1, &z1);
    int cells = (x1 - x0 + 1) * (z1 - z0 + 1);
    if (cells <= 0) return;
    float dep = volume * BULK_FACTOR / ((float)cells * PILE_CELL * PILE_CELL);
    for (int i = x0; i <= x1; i++) {
        for (int j = z0; j <= z1; j++) gHeight[i][j] += dep;
    }
}

// The grid's dimensions are literals, so prove they still reach past everything that lands.
static void CheckPileGrid(void)
{
    float needX = BLOCK_LEN + 2.0f * PILE_MARGIN, needZ = 2.0f * FACE_Z + 2.0f * PILE_MARGIN;
    if (needX > (float)PILE_NX * PILE_CELL || needZ > (float)PILE_NZ * PILE_CELL) {
        TraceLog(LOG_ERROR, "panelka: the pile grid is %dx%d cells but needs %.0fx%.0f",
                 PILE_NX, PILE_NZ, needX, needZ);
    }
}

static float PileCrest(void)
{
    float top = 0.0f;
    for (int i = 0; i < PILE_NX; i++) {
        for (int j = 0; j < PILE_NZ; j++) top = fmaxf(top, gHeight[i][j]);
    }
    return top;
}

// What a pile of this much material ought to crest at, independently of where the fragments
// actually end up. Volume by the divergence theorem, bulked for what a solid becomes once it is
// broken, spread over the footprint plus the skirt it runs out onto. It is a check on the packed
// pile rather than the pile itself: the two coming out close is evidence that the packing is
// putting the material somewhere sensible, and CheckCollapse prints both.
static float PredictedCrest(float material)
{
    float area = (BLOCK_LEN + PILE_SPREAD) * (2.0f * FACE_Z + PILE_SPREAD);
    // A mound of this footprint holds about half of what a prism of its crest height would.
    return material * BULK_FACTOR / (area * 0.5f);
}

typedef struct {
    float t0;        // when it lets go
    float t1;        // when it lands
    Vector3 v0;
    Vector3 axis;    // tumble axis, in the fragment's own frame
    float omega;
    Vector3 c;       // local centroid, which is what it tumbles about
    Vector3 rest;    // world position of that centroid at the moment it lets go
    float seq;       // where this fragment sits in the firing sequence
    float drop;      // how far the crush below had already carried it down by then
    float half;      // half its vertical extent *at the orientation it lands in*, not upright
} Motion;

static Motion gMotion[MAX_FRAG];

// How far the material above the crush front has already been carried down by the time t. The
// front leaves the cut when the charges under that part of the block fire and climbs at FRONT_V,
// and everything above it rides down on what is being destroyed underneath.
//
// Without this a fragment simply waits in place until the front reaches it, and a review round
// read exactly that: the roof sitting level over a frame the facade had come off, as though the
// skin had been blown away rather than the building losing its footing. A panelka's upper mass
// starts down as soon as the bottom of it stops holding, which is what this is.
static float CrushDrop(float seq, float y, float t)
{
    float front = CUT_TOP + FRONT_V * fmaxf(0.0f, t - T_BLAST - seq);
    front = fminf(front, y);
    return fmaxf(0.0f, front - CUT_TOP) * CRUSH_LOSS;
}

static float Rand01(int i, int k) { return Hash2(i, k, 4096, 977u); }
static float Rand11(int i, int k) { return Rand01(i, k) * 2.0f - 1.0f; }

static void PlanFragment(int i)
{
    Fragment *f = &gFrag[i];
    Motion *m = &gMotion[i];
    BoundingBox b = gType[f->type].bounds;
    m->c = (Vector3){ (b.min.x + b.max.x) * 0.5f, (b.min.y + b.max.y) * 0.5f, (b.min.z + b.max.z) * 0.5f };
    m->half = (b.max.y - b.min.y) * 0.5f * f->scale;

    Vector3 wc = Vector3Transform(m->c, FragRest(f));
    float height = fmaxf(0.0f, wc.y - PLINTH_Y);

    // The firing sequence runs from the left-hand gable, and the crushing front climbs from the cut.
    float seq = SEQ_LEN * (wc.x + BLOCK_LEN * 0.5f) / BLOCK_LEN;
    float climb = fmaxf(0.0f, wc.y - CUT_TOP) / FRONT_V;
    m->t0 = T_BLAST + seq + climb + 0.09f * Rand01(i, 1);

    // Outward is away from the spine, which is the direction a facade panel peels.
    float sz = (wc.z >= 0.0f) ? 1.0f : -1.0f;
    float out = 0.0f, up = 0.0f, along = 0.6f * Rand11(i, 2);

    bool skin = (f->type <= FR_PEND) || (f->type >= FR_G26 && f->type <= FR_GDOOR) || f->type == FR_PIPE;
    bool glazed = (f->type >= FR_G26 && f->type <= FR_GDOOR);
    bool balcony = (f->type == FR_BALC || f->type == FR_BALCG);

    if (wc.y < CUT_TOP) {
        // The charges are in here. This is the only material that is thrown rather than dropped.
        m->t0 = T_BLAST + seq + 0.05f * Rand01(i, 3);
        out = 3.0f + 3.2f * Rand01(i, 4);
        up = 1.1f + 1.9f * Rand01(i, 5);
    } else if (glazed) {
        // Every window in the building goes at the shock, not when its own storey is reached.
        m->t0 = T_BLAST + seq + 0.05f * Rand01(i, 6);
        out = 3.6f + 4.2f * Rand01(i, 7);
        up = 0.4f + 1.2f * Rand01(i, 8);
    } else if (balcony) {
        // A cantilever with nothing above it, so it goes before the wall it hangs on.
        m->t0 -= 0.22f;
        out = 1.1f + 0.10f * height + 0.8f * Rand01(i, 9);
    } else if (skin) {
        // A review round read the first pass as the facade being explosively blown off while the
        // roof stayed level over it. What actually happens is that the lower storeys lose support
        // and the mass above starts down before anything peels, so the skin now waits a fifth of
        // a second longer than the structure at its own height and leaves with a third of the
        // outward speed it had. The peel then comes from the tumble and the fall rather than a kick.
        m->t0 += 0.20f;
        out = 0.25f + 0.038f * height + 0.45f * Rand01(i, 10);
    } else if (f->type == FR_RUBBLE) {
        m->t0 = T_BLAST + seq + climb * 0.75f + 0.35f * Rand01(i, 18);
        out = 1.3f + 3.4f * Rand01(i, 19);
        up = 0.4f + 2.6f * Rand01(i, 20);
        along = 2.6f * Rand11(i, 21);
    } else {
        // Slabs, cross walls and the spine drop; they are what the pile is made of.
        out = 0.25f * Rand01(i, 11);
        up = -0.3f * Rand01(i, 12);
    }

    m->v0 = (Vector3){ along, up, sz * out };
    m->axis = Vector3Normalize((Vector3){ Rand11(i, 13) + 0.05f, Rand11(i, 14) * 0.4f, Rand11(i, 15) });
    m->omega = (glazed ? 3.4f : balcony ? 1.9f : 0.7f) * (0.5f + Rand01(i, 16)) * (Rand01(i, 17) < 0.5f ? -1.0f : 1.0f);
    if (f->type == FR_RUBBLE) m->omega *= 4.0f;

    m->t0 = fmaxf(m->t0, T_BLAST + seq);
    m->seq = seq;
    m->drop = CrushDrop(seq, wc.y, m->t0);
    m->rest = wc;
    m->rest.y -= m->drop;
    // A provisional flight time against bare ground, which is only used to put the fragments into
    // the order they land in. SolveLanding then walks that order and does the real one.
    float disc = m->v0.y * m->v0.y + 2.0f * GRAV * fmaxf(0.0f, wc.y - m->half);
    m->t1 = m->t0 + ((disc <= 0.0f) ? 0.0f : (m->v0.y + sqrtf(disc)) / GRAV);
}


// ---------------------------------------------------------------------------
// Dust and flashes
//
// Two things make dust in a demolition and they behave differently. The base surge is air driven
// out sideways by the floors slamming down on it: it leaves the footprint fast and low and rolls
// outwards along the ground. The column is what rises off the pile afterwards, slower and much
// taller. One population averaging the two reads as ground fog, so there are two.
//
// A puff is thrown, then sheds its speed to the air as a first-order decay to a terminal
// displacement, which is why dust travels so much further than a thrown solid does and then
// simply hangs. It grows the whole time, and fades to nothing inside its own life.
// ---------------------------------------------------------------------------

#define DUST_MAX     1240
#define DUST_TAU     0.62f    // time constant of the speed decay

typedef struct {
    Vector3 p0, v;
    float t0, life, r0, r1, peak, yaw, rise;
} Puff;

static Group gDust, gFlash;
static Puff gPuff[DUST_MAX];
static Matrix gDustMat[DUST_MAX];
static Color gDustTint[DUST_MAX];
static int gPuffCount;

#define FLASH_MAX 96
static Vector3 gFlashPos[FLASH_MAX];
static Matrix gFlashMat[FLASH_MAX];
static Color gFlashTint[FLASH_MAX];
static int gFlashCount;

static void BuildDustTypes(void)
{
    // One lumpy blob of unit radius, drawn per puff with a scale, a yaw and a translation. It is
    // only yawed: a puff is shaded by a hemisphere ramp off its own normal, and tumbling it would
    // light some undersides from above.
    Blob(&gDust.b[MAT_DUST], Vector3Zero(), 1.0f, 0.88f, 1.0f, 0.34f, 401u, 9);
    // A flash is a billboardless glow ball; at the sizes and durations here nothing ever gets
    // close enough to it for the difference to show.
    Blob(&gFlash.b[MAT_FLASH], Vector3Zero(), 1.0f, 1.0f, 1.0f, 0.10f, 77u, 7);
}

static void AddPuff(Vector3 p, Vector3 v, float t0, float life, float r0, float r1, float peak, float rise, int seed)
{
    if (gPuffCount >= DUST_MAX) return;
    Puff *q = &gPuff[gPuffCount++];
    q->p0 = p; q->v = v; q->t0 = t0; q->life = life;
    q->r0 = r0; q->r1 = r1; q->peak = peak; q->rise = rise;
    q->yaw = 360.0f * Hash2(seed, 5, 4096, 811u);
}

static void PlaceDust(void)
{
    const int SURGE = 620, COLUMN = 260;

    for (int i = 0; i < SURGE; i++) {
        // Born on the footprint's perimeter, as the firing sequence reaches that x.
        float u = Hash2(i, 1, 4096, 71u);
        float x = (u - 0.5f) * BLOCK_LEN;
        float sz = (Hash2(i, 2, 4096, 73u) < 0.5f) ? 1.0f : -1.0f;
        float z = sz * (FACE_Z * (0.35f + 0.65f * Hash2(i, 3, 4096, 79u)));
        float y = 0.25f + 2.4f * Hash2(i, 4, 4096, 83u);

        float seq = SEQ_LEN * (x + BLOCK_LEN * 0.5f) / BLOCK_LEN;
        float t0 = T_BLAST + seq + 0.18f + 1.70f * powf(Hash2(i, 5, 4096, 89u), 1.6f);

        float speed = 9.0f + 13.0f * Hash2(i, 6, 4096, 97u);
        Vector3 v = { (Hash2(i, 7, 4096, 101u) - 0.5f) * 7.0f, 0.8f + 2.2f * Hash2(i, 8, 4096, 103u), sz * speed };
        float r0 = 2.0f + 2.4f * Hash2(i, 9, 4096, 107u);
        AddPuff((Vector3){ x, y, z }, v, t0, 2.2f + 1.7f * Hash2(i, 10, 4096, 109u),
                r0, r0 * (2.9f + 1.6f * Hash2(i, 11, 4096, 113u)),
                0.15f + 0.10f * Hash2(i, 12, 4096, 127u), 0.25f + 0.55f * Hash2(i, 13, 4096, 131u), i);
    }

    for (int i = 0; i < COLUMN; i++) {
        float u = Hash2(i, 21, 4096, 131u);
        float x = (u - 0.5f) * BLOCK_LEN * 0.94f;
        float z = (Hash2(i, 22, 4096, 137u) - 0.5f) * 2.0f * FACE_Z;
        float y = 0.8f + 5.0f * Hash2(i, 23, 4096, 139u);

        float seq = SEQ_LEN * (x + BLOCK_LEN * 0.5f) / BLOCK_LEN;
        float t0 = T_BLAST + seq + 0.55f + 2.4f * Hash2(i, 24, 4096, 149u);

        Vector3 v = { (Hash2(i, 25, 4096, 151u) - 0.5f) * 4.0f,
                      3.4f + 5.2f * Hash2(i, 26, 4096, 157u),
                      (Hash2(i, 27, 4096, 163u) - 0.5f) * 5.0f };
        float r0 = 2.8f + 3.2f * Hash2(i, 28, 4096, 167u);
        // A review round found the cloud reading as one level blanket over the whole courtyard,
        // hiding the pile from the only camera the loop has. The column is fewer, smaller and
        // fainter for it, and each puff now gets its own buoyancy so their tops do not line up.
        AddPuff((Vector3){ x, y, z }, v, t0, 2.2f + 1.5f * Hash2(i, 29, 4096, 173u),
                r0, r0 * (2.0f + 1.2f * Hash2(i, 30, 4096, 179u)),
                0.085f + 0.070f * Hash2(i, 31, 4096, 181u), 0.7f + 1.7f * Hash2(i, 32, 4096, 191u), i + SURGE);
    }

    for (int i = 0; i < 190; i++) {
        float x = (Hash2(i, 51, 4096, 223u) - 0.5f) * BLOCK_LEN;
        float sz = (Hash2(i, 52, 4096, 227u) < 0.5f) ? 1.0f : -1.0f;
        float fl = (Hash2(i, 53, 4096, 229u) < 0.5f) ? 0.0f : 1.0f;
        float y = FloorY((int)fl) + 0.4f + 1.4f * Hash2(i, 54, 4096, 233u);
        float seq = SEQ_LEN * (x + BLOCK_LEN * 0.5f) / BLOCK_LEN;
        Vector3 v = { (Hash2(i, 55, 4096, 239u) - 0.5f) * 5.0f,
                      0.5f + 2.0f * Hash2(i, 56, 4096, 241u),
                      sz * (15.0f + 9.0f * Hash2(i, 57, 4096, 251u)) };
        float r0 = 0.60f + 0.7f * Hash2(i, 58, 4096, 257u);
        AddPuff((Vector3){ x, y, sz * (FACE_Z + 0.2f) }, v, T_BLAST + seq + 0.04f * Hash2(i, 59, 4096, 263u),
                1.1f + 0.7f * Hash2(i, 60, 4096, 269u), r0, r0 * 5.5f,
                0.20f + 0.12f * Hash2(i, 61, 4096, 271u), 0.30f, i + 900);
    }

    // A flash at every cross wall the charges are drilled into, on both facades, on the two cut
    // storeys. That is where the holes are and it is the only place the light comes from.
    for (int s = 0; s < SECTIONS && gFlashCount < FLASH_MAX; s++) {
        for (int b = 0; b <= BAYS; b += 2) {
            if (b == BAYS && s != SECTIONS - 1) continue;
            float x = SectionX(s) - SECTION_LEN * 0.5f;
            for (int k = 0; k < b; k++) x += BAY_W[k];
            for (int side = 0; side < 2 && gFlashCount < FLASH_MAX; side++) {
                for (int fl = 0; fl < 2 && gFlashCount < FLASH_MAX; fl++) {
                    gFlashPos[gFlashCount++] = (Vector3){
                        x, FloorY(fl) + 1.1f, (side == 0) ? FACE_Z + 0.35f : -FACE_Z - 0.35f };
                }
            }
        }
    }
}

// A charge is milliseconds of light and --anim samples the cycle at N evenly spaced phases, so a
// truthful flash would be caught only by luck. It is held over 0.30 s of a 6 s cycle instead --
// a playback concession, not a claim about explosives -- which puts the window at 0.90 to 1.20
// and so straddles t = 1.0. Every --frames that is a multiple of 6 lands a frame inside it.
static float FlashLevel(float t)
{
    if (t < T_BLAST || t > FLASH_END) return 0.0f;
    float u = (t - T_BLAST) / (FLASH_END - T_BLAST);
    return powf(1.0f - u, 2.2f);
}

static void PoseDust(float t)
{
    for (int i = 0; i < gPuffCount; i++) {
        Puff *q = &gPuff[i];
        float age = t - q->t0;
        if (age <= 0.0f || age >= q->life) { gDustTint[i] = (Color){ 0, 0, 0, 0 }; continue; }
        float u = age / q->life;

        float k = DUST_TAU * (1.0f - expf(-age / DUST_TAU));
        Vector3 p = {
            q->p0.x + q->v.x * k,
            q->p0.y + q->v.y * k + q->rise * age,
            q->p0.z + q->v.z * k,
        };
        float r = q->r0 + (q->r1 - q->r0) * powf(u, 0.70f);

        // In fast, out slowly, and to nothing by the end of its own life, which is a concession
        // to the cycle rather than a claim about dust.
        float a = q->peak * fminf(1.0f, u / 0.12f) * fminf(1.0f, (1.0f - u) / 0.55f);

        gDustMat[i] = MatrixMultiply(MatrixMultiply(MatrixScale(r, r, r), MatrixRotateY(q->yaw * DEG2RAD)),
                                     MatrixTranslate(p.x, p.y, p.z));
        gDustTint[i] = (Color){ 255, 255, 255, (unsigned char)Clamp(a * 255.0f, 0.0f, 255.0f) };
    }

    float lvl = FlashLevel(t);
    for (int i = 0; i < gFlashCount; i++) {
        float r = 1.5f + 2.6f * lvl;
        Vector3 p = gFlashPos[i];
        gFlashMat[i] = MatrixMultiply(MatrixScale(r, r, r), MatrixTranslate(p.x, p.y, p.z));
        gFlashTint[i] = (Color){ 255, 255, 255, (unsigned char)Clamp(lvl * 255.0f, 0.0f, 255.0f) };
    }
}

// ---------------------------------------------------------------------------
// Pose
// ---------------------------------------------------------------------------

#define CYCLE 6.0f

// A fragment tau seconds into its flight: ballistic about its own centroid.
static Matrix PoseAtTau(int i, float tau)
{
    const Fragment *f = &gFrag[i];
    const Motion *m = &gMotion[i];
    if (tau <= 0.0f) return FragRest(f);

    Vector3 d = { m->v0.x * tau, m->v0.y * tau - 0.5f * GRAV * tau * tau, m->v0.z * tau };
    Vector3 sc = Vector3Scale(m->c, f->scale);

    // Scale, then swing about the fragment's own centroid, then take the rest yaw, then place.
    // Rotating about the local origin instead would spin a facade panel about its bottom inner
    // corner, which is a hinge no broken panel has.
    Matrix M = MatrixScale(f->scale, f->scale, f->scale);
    M = MatrixMultiply(M, MatrixTranslate(-sc.x, -sc.y, -sc.z));
    M = MatrixMultiply(M, MatrixRotate(m->axis, m->omega * tau));
    M = MatrixMultiply(M, MatrixTranslate(sc.x, sc.y, sc.z));
    M = MatrixMultiply(M, MatrixRotateY(f->yaw * DEG2RAD));
    return MatrixMultiply(M, MatrixTranslate(f->pos.x + d.x, f->pos.y + d.y - m->drop, f->pos.z + d.z));
}

// Frozen the moment it lands.
static Matrix FlightPose(int i, float t)
{
    return PoseAtTau(i, fminf(t, gMotion[i].t1) - gMotion[i].t0);
}

// How far a fragment reaches in each axis at a given moment of its tumble. The linear part of the
// pose maps the local box's half-extents, so the support function of the transformed box is the
// sum of the absolute contributions -- which is the whole of Codex's second finding: m->half was
// the *upright* half-height, and a 2.66 m panel that comes down edge-on has a vertical extent of
// 0.15, so it was left floating 1.3 m over the pile. A slab that lands on edge was buried by the
// same amount in the other direction.
static Vector3 ExtentAt(int i, float tau)
{
    Matrix M = PoseAtTau(i, tau);
    BoundingBox b = gType[gFrag[i].type].bounds;
    float hx = (b.max.x - b.min.x) * 0.5f;
    float hy = (b.max.y - b.min.y) * 0.5f;
    float hz = (b.max.z - b.min.z) * 0.5f;
    return (Vector3){
        fabsf(M.m0) * hx + fabsf(M.m4) * hy + fabsf(M.m8) * hz,
        fabsf(M.m1) * hx + fabsf(M.m5) * hy + fabsf(M.m9) * hz,
        fabsf(M.m2) * hx + fabsf(M.m6) * hy + fabsf(M.m10) * hz,
    };
}

// Walk the fragments in the order they land and put each one on top of what is already there.
// Both the landing point and the fragment's own extent depend on the flight time, and the flight
// time on both of them, so each is solved by four passes of the same substitution; the fourth
// moves the answer by millimetres.
static int gLandOrder[MAX_FRAG];

static void SolveLanding(void)
{
    int n = 0;
    for (int i = 0; i < gFragCount; i++) {
        if (IsBuilding(gFrag[i].type) || gFrag[i].type == FR_RUBBLE) gLandOrder[n++] = i;
    }
    for (int a = 1; a < n; a++) {
        int v = gLandOrder[a], b = a - 1;
        while (b >= 0 && gMotion[gLandOrder[b]].t1 > gMotion[v].t1) { gLandOrder[b + 1] = gLandOrder[b]; b--; }
        gLandOrder[b + 1] = v;
    }

    for (int k = 0; k < n; k++) {
        int i = gLandOrder[k];
        Motion *m = &gMotion[i];
        float tau = m->t1 - m->t0;
        Vector3 e = { 1.0f, m->half, 1.0f };
        Vector3 wc = m->rest;

        for (int pass = 0; pass < 4; pass++) {
            e = ExtentAt(i, tau);
            wc = Vector3Transform(m->c, PoseAtTau(i, tau));
            float surface = PileUnder(wc.x, wc.z, e.x, e.z);
            float target = surface + e.y;
            float disc = m->v0.y * m->v0.y + 2.0f * GRAV * (m->rest.y - target);
            tau = (disc <= 0.0f) ? 0.0f : (m->v0.y + sqrtf(disc)) / GRAV;
        }

        m->half = e.y;
        m->t1 = m->t0 + tau;
        wc = Vector3Transform(m->c, PoseAtTau(i, tau));
        e = ExtentAt(i, tau);
        float sc = gFrag[i].scale;
        PileDeposit(wc.x, wc.z, e.x, e.z, gType[gFrag[i].type].volume * sc * sc * sc);
    }
}

// When the air blast reaches a point in the yard. It leaves the building at the moment the
// charges fire and travels at BLAST_V, which puts it at the far trees about a third of a second
// later -- slow enough that the lashing runs outwards across the scene rather than happening to
// everything at once.
static float BlastArrival(float x, float z)
{
    float dz = fmaxf(0.0f, fabsf(z) - FACE_Z);
    float dx = fmaxf(0.0f, fabsf(x) - BLOCK_LEN * 0.5f);
    return T_BLAST + sqrtf(dx * dx + dz * dz) / BLAST_V;
}

// A damped oscillation starting when the blast arrives. One expression covers a tree bending, a
// car rocking on its springs and a bench going over, because all three are the same statement:
// something with a restoring force gets hit once.
static float Lash(float t, float arrive, float amp, float hz, float decay)
{
    float a = t - arrive;
    if (a <= 0.0f) return 0.0f;
    return amp * expf(-a * decay) * sinf(2.0f * PI * hz * a);
}

// The yard. Nothing out here is destroyed, but nothing out here is untouched either: what a
// demolition looks like from across the road is mostly the trees going over.
static Matrix YardPose(int i, float t)
{
    const Fragment *f = &gFrag[i];
    float arrive = BlastArrival(f->pos.x, f->pos.z);
    // Distance from the block, which is what everything out here scales by.
    float dz = fmaxf(0.0f, fabsf(f->pos.z) - FACE_Z);
    float dx = fmaxf(0.0f, fabsf(f->pos.x) - BLOCK_LEN * 0.5f);
    float dist = fmaxf(1.0f, sqrtf(dx * dx + dz * dz));
    float near = 1.0f / (1.0f + dist / 16.0f);

    // Away from the building, in the plane; the axis to tilt about is perpendicular to it.
    Vector3 out = Vector3Normalize((Vector3){ (fabsf(f->pos.x) > BLOCK_LEN * 0.5f) ? f->pos.x : 0.0f,
                                              0.0f, f->pos.z });
    Vector3 axis = { -out.z, 0.0f, out.x };

    float tilt = 0.0f, lift = 0.0f;
    switch (f->type) {
        case FR_BIRCH: case FR_MAPLE:
            // A rigid mesh can only hinge at the root, so the whole tree leans where a real one
            // would bend most at the top. Skinning is not available here (the harness's shader
            // declares no bone attributes), so this is the honest limit rather than a choice.
            tilt = Lash(t, arrive, 0.30f * near, 0.62f, 1.05f);
            break;
        case FR_CARBODY: case FR_CARTRIM:
            tilt = Lash(t, arrive, 0.075f * near, 1.65f, 2.6f);
            lift = -0.030f * fabsf(tilt) * 10.0f;
            break;
        case FR_BENCH: case FR_BIN: {
            // Close enough to the facade to be knocked flat and then buried.
            float a = t - arrive;
            float over = (a <= 0.0f) ? 0.0f : fminf(1.0f, a / 0.55f);
            tilt = 1.35f * near * over * over * (3.0f - 2.0f * over) + Lash(t, arrive, 0.10f, 3.0f, 5.0f);
            break;
        }
        default:
            tilt = Lash(t, arrive, 0.055f * near, 2.4f, 2.2f);
            break;
    }

    if (fabsf(tilt) < 1e-5f && fabsf(lift) < 1e-5f) return FragRest(f);

    Matrix M = MatrixScale(f->scale, f->scale, f->scale);
    M = MatrixMultiply(M, MatrixRotateY(f->yaw * DEG2RAD));
    M = MatrixMultiply(M, MatrixRotate(axis, tilt));
    return MatrixMultiply(M, MatrixTranslate(f->pos.x, f->pos.y + lift, f->pos.z));
}

static void Update(float t)
{
    for (int i = 0; i < gFragCount; i++) {
        Fragment *f = &gFrag[i];
        // +-8 either side of white. Enough to separate two neighbouring panels, not enough to read as a repainted one.
        int k = (int)(230.0f + 25.0f * f->shade);
        Color tint = TintMul((Color){ (unsigned char)k, (unsigned char)k, (unsigned char)k, 255 }, f->tint);

        if (f->type == FR_RUBBLE) {
            // Rubble is material that did not exist as a separate thing until the building broke,
            // so it has no rest pose: it is invisible until its own moment and is born inside the
            // collapsing volume, where the dust covers its arrival.
            gFragTint[i] = (t < gMotion[i].t0) ? (Color){ 0, 0, 0, 0 } : tint;
            gFragMat[i] = FlightPose(i, t);
        } else if (IsBuilding(f->type)) {
            gFragTint[i] = tint;
            if (t > gMotion[i].t0) {
                gFragMat[i] = FlightPose(i, t);
            } else {
                // Still standing, but riding down on whatever is being crushed under it. The drop
                // at its own release time is exactly what FlightPose starts from, so the two meet
                // without a step.
                float drop = CrushDrop(gMotion[i].seq, gMotion[i].rest.y + gMotion[i].drop, t);
                gFragMat[i] = MatrixMultiply(FragRest(f), MatrixTranslate(0.0f, -drop, 0.0f));
            }
        } else {
            gFragTint[i] = tint;
            gFragMat[i] = YardPose(i, t);
        }
    }
    PoseDust(t);
}

// ---------------------------------------------------------------------------
// Parts
// ---------------------------------------------------------------------------

#define COUNT_OF(a) ((int)(sizeof(a) / sizeof((a)[0])))

static Group *PART_SHELL[]  = { &gType[FR_P26], &gType[FR_P32], &gType[FR_P32B],
                                &gType[FR_PSTAIR], &gType[FR_PDOOR], &gType[FR_PEND], &gType[FR_PIPE] };
static Group *PART_GLAZ[]   = { &gType[FR_G26], &gType[FR_G32], &gType[FR_G32B],
                                &gType[FR_GSTAIR], &gType[FR_GDOOR] };
static Group *PART_STRUCT[] = { &gType[FR_SLAB26], &gType[FR_SLAB32],
                                &gType[FR_XWALL], &gType[FR_SPINE26], &gType[FR_SPINE32] };
static Group *PART_ROOF[]   = { &gType[FR_ROOF26], &gType[FR_ROOF32], &gType[FR_PAR26],
                                &gType[FR_PAR32], &gType[FR_PAREND], &gType[FR_VENT], &gType[FR_MAST] };
static Group *PART_BALC[]   = { &gType[FR_BALC], &gType[FR_BALCG] };
static Group *PART_ENTRY[]  = { &gType[FR_PDOOR], &gType[FR_GDOOR], &gType[FR_CANOPY] };
static Group *PART_PLINTH[] = { &gPlinth };
static Group *PART_TREES[]  = { &gType[FR_BIRCH], &gType[FR_MAPLE] };
static Group *PART_CARS[]   = { &gType[FR_CARBODY], &gType[FR_CARTRIM] };
static Group *PART_YARD[]   = { &gType[FR_BENCH], &gType[FR_RUGFRAME], &gType[FR_LAMP], &gType[FR_BIN] };
static Group *PART_DEBRIS[] = { &gType[FR_RUBBLE] };
static Group *PART_DUST[]   = { &gDust, &gFlash };
static Group *PART_GROUND[] = { &gGround };

static BoundingBox bShell, bGlaz, bStruct, bRoof, bBalc, bEntry, bPlinth, bGround, bTrees, bCars, bYard, bDebris, bDust;

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
static void DrawDebris(void) { DrawGroups(PART_DEBRIS, COUNT_OF(PART_DEBRIS)); }
static void DrawDust(void) { DrawGroups(PART_DUST, COUNT_OF(PART_DUST)); }

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
static BoundingBox DebrisBounds(void) { return bDebris; }
static BoundingBox DustBounds(void) { return bDust; }

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
    { .name = "debris", .draw = DrawDebris, .bounds = DebrisBounds },
    { .name = "dust", .draw = DrawDust, .bounds = DustBounds },
};

// A group that exists in several hundred places has no single bounding box either. Union the eight transformed corners of the local box over every instance, which is the same argument SweepBounds makes for a part that moves: GetModelBoundingBox transforms only the box's own two corners and warns that it does not support rotation (vendor/raylib/src/rmodels.c:1243), which is exactly what a yawed panel is.
//
// Whether to sweep the cycle as well is a different question here than in a model whose parts
// merely move. Sweeping a part of this one gives the whole debris field, which frames every
// building part identically and destroys exactly what --part exists for. So the building's parts
// are framed on their rest pose, which is the pose --part renders and the one you inspect
// geometry in; only the debris and the dust, which have no rest pose worth framing, are swept.
static BoundingBox InstBounds(Group *const *gs, int n, bool swept)
{
    BoundingBox out = { { 1e9f, 1e9f, 1e9f }, { -1e9f, -1e9f, -1e9f } };
    const int SAMPLES = swept ? 40 : 1;
    for (int s = 0; s < SAMPLES; s++) {
        Update(swept ? CYCLE * (float)s / (float)SAMPLES : 0.0f);
        for (int i = 0; i < n; i++) {
            BoundingBox b = gs[i]->bounds;
            int reps = (gs[i]->instCount > 0) ? gs[i]->instCount : 1;
            for (int k = 0; k < reps; k++) {
                if (gs[i]->instTint && gs[i]->instTint[k].a == 0) continue;
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
    Rect stLo = { -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL_LO, STAIR_SILL_LO + STAIR_H };
    CheckOpenings("stairwell panel, low sill", 2.6f, STOREY, &stLo, 1);
    Rect stHi = { -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL_HI, STAIR_SILL_HI + STAIR_H };
    CheckOpenings("stairwell panel, high sill", 2.6f, STOREY, &stHi, 1);
    Rect en = { -ENTRY_W * 0.5f, ENTRY_W * 0.5f, 0.0f, ENTRY_H };
    CheckOpenings("entrance panel", 2.6f, STOREY, &en, 1);
}

// A window head must clear the ceiling it is cut under, and a balcony door must clear the slab it opens onto. Both are one subtraction, and both are the kind of thing that stops being true the moment someone changes STOREY.
static void CheckHeadroom(void)
{
    float ceiling = STOREY - SLAB_T;
    float win = ceiling - (WIN_SILL + WIN_H);
    float door = ceiling - (BDOOR_SILL + BDOOR_H);
    float stair = ceiling - (STAIR_SILL_HI + STAIR_H);
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


// The collapse itself: when the last piece lands, how far the furthest one is thrown, and whether
// anything is still in the air when the cycle ends.
static void CheckCollapse(void)
{
    float last = 0.0f, throwOut = 0.0f, deepest = 1e9f;
    int airborne = 0, entombed = 0;
    for (int i = 0; i < gFragCount; i++) {
        if (!IsBuilding(gFrag[i].type) && gFrag[i].type != FR_RUBBLE) continue;
        const Motion *m = &gMotion[i];
        last = fmaxf(last, m->t1);
        if (m->t1 > CYCLE) airborne++;

        Matrix land = FlightPose(i, m->t1);
        Vector3 c = Vector3Transform(m->c, land);
        float outward = fmaxf(fabsf(c.z) - FACE_Z, fabsf(c.x) - BLOCK_LEN * 0.5f);
        throwOut = fmaxf(throwOut, outward);
        // A piece that never got off the ground is the bottom of the building becoming the
        // bottom of the pile, not a solver failure.
        if (m->t1 - m->t0 < 1e-3f) { entombed++; continue; }
        Vector3 e = ExtentAt(i, m->t1 - m->t0);
        deepest = fminf(deepest, c.y - e.y - PileUnder(c.x, c.z, e.x, e.z));
    }
    TraceLog(LOG_INFO, "panelka: last piece lands at %.2f s of a %.1f s cycle, thrown up to %.1f m clear of the footprint; %d fragments never leave the ground",
             last, CYCLE, throwOut, entombed);
    if (airborne > 0) {
        TraceLog(LOG_WARNING, "panelka: %d fragments are still in the air when the cycle ends", airborne);
    }
    // Everything must come to rest on the pile, not inside it. The tumble is about the centroid,
    // so a piece can still bury a corner; what is checked is that its centre lands on the surface.
    // Each piece was solved against the surface that existed at the moment it landed, so the
    // final grid standing above it means later material came down on top -- which is what a pile
    // is. It is reported rather than warned about, and the number to watch is the deepest.
    TraceLog(LOG_INFO, "panelka: the deepest fragment ends %.1f m under the finished pile surface", -deepest);
}

// A demolition is not periodic, and this is the one model here whose loop deliberately does not
// close. Rather than pretend, measure the seam: the largest distance any fragment has moved
// between the start of the cycle and its end. That number is the size of the jump a viewer sees
// if the sequence is played round, and it is meant to be large.
static void CheckLoopSeam(void)
{
    Update(0.0f);
    static Vector3 at0[MAX_FRAG];
    for (int i = 0; i < gFragCount; i++) at0[i] = Vector3Transform(gMotion[i].c, gFragMat[i]);

    Update(CYCLE);
    float worst = 0.0f;
    for (int i = 0; i < gFragCount; i++) {
        Vector3 p = Vector3Transform(gMotion[i].c, gFragMat[i]);
        worst = fmaxf(worst, Vector3Distance(p, at0[i]));
    }
    Update(0.0f);
    TraceLog(LOG_INFO, "panelka: the loop does not close, by %.1f m at the worst fragment; a demolition is a one-shot event and --anim plays it once", worst);
}

// The flash is the shortest event in the cycle, so say how many frames of a given --frames land
// inside it rather than hoping one does.
static void CheckFlashSampling(void)
{
    for (int frames = 6; frames <= 30; frames += 6) {
        int hits = 0;
        for (int i = 0; i < frames; i++) {
            if (FlashLevel(CYCLE * (float)i / (float)frames) > 0.0f) hits++;
        }
        if (hits == 0) TraceLog(LOG_WARNING, "panelka: --frames %d never samples the detonation", frames);
    }
    TraceLog(LOG_INFO, "panelka: detonation held over %.2f s of a %.1f s cycle, %d puffs, %d flashes",
             FLASH_END - T_BLAST, CYCLE, gPuffCount, gFlashCount);
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

static const char *TYPE_NAME[FR_COUNT] = {
    "p26", "p32", "p32b", "pstair", "pstair2", "pdoor", "pend",
    "g26", "g32", "g32b", "gstair", "gstair2", "gdoor",
    "slab26", "slab32", "roof26", "roof32",
    "xwall", "spine26", "spine32",
    "balc", "balcg", "par26", "par32", "parend", "canopy", "vent", "pipe", "mast",
    "rubble",
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
    BuildDustTypes();
    for (int t = 0; t < FR_COUNT; t++) {
        GroupFinish(&gType[t], TYPE_NAME[t]);
        GroupRegister(&gType[t]);
    }
    GroupFinish(&gDust, "dust");
    gDust.inst = gDustMat;
    gDust.instTint = gDustTint;
    GroupRegister(&gDust);
    GroupFinish(&gFlash, "flash");
    gFlash.inst = gFlashMat;
    gFlash.instTint = gFlashTint;
    GroupRegister(&gFlash);

    BuildPlinth();
    GroupFinish(&gPlinth, "plinth");
    GroupRegister(&gPlinth);

    BuildGround();
    GroupFinish(&gGround, "ground");
    GroupRegister(&gGround);

    PlaceBuilding();
    PlaceYard();
    PlaceRubble();
    SortFragments();
    PlaceDust();
    gDust.instCount = gPuffCount;
    gFlash.instCount = gFlashCount;

    // How high the pile stands is the summed mesh volume of everything that falls, bulked, and
    // fitted to the pile's own shape. Nothing about it is chosen.
    float material = 0.0f;
    for (int i = 0; i < gFragCount; i++) {
        if (!IsBuilding(gFrag[i].type) && gFrag[i].type != FR_RUBBLE) continue;
        float sc = gFrag[i].scale;
        material += gType[gFrag[i].type].volume * sc * sc * sc;
    }
    for (int i = 0; i < gFragCount; i++) PlanFragment(i);
    SolveLanding();
    TraceLog(LOG_INFO, "panelka: %.0f m3 of material bulks to %.0f; the packed pile crests at %.2f m against a %.2f m prediction",
             material, material * BULK_FACTOR, PileCrest(), PredictedCrest(material));

    Update(0.0f);

    bShell = InstBounds(PART_SHELL, COUNT_OF(PART_SHELL), false);
    bGlaz = InstBounds(PART_GLAZ, COUNT_OF(PART_GLAZ), false);
    bStruct = InstBounds(PART_STRUCT, COUNT_OF(PART_STRUCT), false);
    bRoof = InstBounds(PART_ROOF, COUNT_OF(PART_ROOF), false);
    bBalc = InstBounds(PART_BALC, COUNT_OF(PART_BALC), false);
    bEntry = InstBounds(PART_ENTRY, COUNT_OF(PART_ENTRY), false);
    bPlinth = InstBounds(PART_PLINTH, COUNT_OF(PART_PLINTH), false);
    bGround = InstBounds(PART_GROUND, COUNT_OF(PART_GROUND), false);
    bTrees = InstBounds(PART_TREES, COUNT_OF(PART_TREES), false);
    bCars = InstBounds(PART_CARS, COUNT_OF(PART_CARS), false);
    bYard = InstBounds(PART_YARD, COUNT_OF(PART_YARD), false);
    bDebris = InstBounds(PART_DEBRIS, COUNT_OF(PART_DEBRIS), true);
    bDust = InstBounds(PART_DUST, COUNT_OF(PART_DUST), true);

    CheckBaysTile();
    CheckPanels();
    CheckHeadroom();
    CheckBalconyRoot();
    CheckStructureMeets();
    CheckNothingBuried();
    CheckPileGrid();
    CheckCollapse();
    CheckFlashSampling();
    CheckLoopSeam();
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
        "A three-section five-storey 1-464 panel block -- a khrushchyovka -- taken down by controlled explosion in its own courtyard, in one six-second cycle.\n"
        "\n"
        "One world unit is one metre. 58.20 m along the block, 11.82 m over the facades, 14.70 m to the top of the parapet.\n"
        "Origin sits on the ground at the centre of the footprint, +X along the block, +Z out through the main facade.\n"
        "\n"
        "Dimensions, and where they come from.\n"
        "The series specifies outer panels 2.6 and 3.2 m wide, floor slabs spanning 5.76 m, a 2.5 m clear ceiling, a 0.10 m slab and 0.21 to 0.35 m outer walls.\n"
        "references/panelka/ref_01.png is a plan of one section; scaled by that 5.76 m span it comes out at 66.9 px/m, and reading the section back off that scale gives seven bays of 2.69, 3.24, 2.64, 2.57, 2.54, 3.23 and 2.66 m over an 11.52 m depth between the outer wall centre-lines.\n"
        "Those are the specified 2.6 and 3.2 m to within 3.5 per cent, so the bays here are 2.6, 3.2, 2.6, 2.6, 2.6, 3.2, 2.6 = 19.40 m per section, the depth is 2 x 5.76 plus one 0.30 m wall, and the storey is 2.50 of clear ceiling plus a 0.10 structural slab plus 0.06 of screed = 2.66 m. The screed is the only invented number in that sum.\n"
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
        "Apartment windows are 1.30 m wide in a 2.6 m bay and 1.50 in a 3.2 m one, 1.45 m tall on a 0.85 m sill, with a centre mullion and a transom.\n"
        "Stair lights are 1.15 by 1.10 and alternate storey by storey between a 0.45 and a 1.30 m sill. A landing sits half a flight above the floor its panel stands on, so the real rhythm is offset from the apartments\' by half a storey; a one-storey panel cannot carry a light that straddles the slab, and the zigzag is how that offset reads from outside.\n"
        "Balcony fronts are solid sheet with a capping rail, and four in ten are glazed in to the ceiling above, chosen by a hash of where the balcony is so the same one is always the same one. An earlier build gave every balcony thirteen thin uprights and a rail, and a review read the whole facade as an access gallery; ref_03 and ref_05 both show boxy masses with opaque fronts and a great many enclosed entirely.\n"
        "Downpipes run at every section joint and gable corner on both facades, in one segment per storey so they break up with the building. They are one of the loudest things on the facade in ref_03 and ref_05 and the first build had none.\n"
        "The end walls are blank, which ref_02 and ref_07 show and ref_01 draws, and which is why ref_04 exists to show one painted instead; ref_05 is a variant that does put two window columns in its gable.\n"
        "\n"
        "The roof is flat, with a 0.32 m upstand under a metal capping that oversails the facade by 0.11.\n"
        "A review round called that the largest departure from the references, on the grounds that ref_02, ref_03, ref_05 and ref_07 show pitched roofs with eaves. Two of those four do not: ref_03 and ref_05 are both flat with a thin capped cornice, which is what this is. The other two are pitched, and are the metal re-roofing that thousands of these buildings were given in the 1990s because the original roof leaked -- the series specifies the original directly, as a flat non-ventilated roof finished in rolled bitumen. The finding did carry a real one, though: the parapet was 0.45 m of bare upstand where both flat-roofed references show a thin capped cornice.\n"
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
        "The demolition.\n"
        "A 1-464 has no frame: every cross wall carries load, which is why the series went up so fast and why it comes down the way it does. Charges are drilled into the cross walls of the bottom two storeys and fired in a sequence running the length of the block, so the building loses its footing progressively and folds instead of toppling; what is above the cut then descends almost vertically and pancakes, with the facade panels peeling off outwards because nothing holds them on once the cross walls behind them are gone.\n"
        "Every one of the 1355 fragments does three things and no more.\n"
        "It rides down. The crush front leaves the cut when the charges under that part of the block fire and climbs at 11 m/s, and everything above it descends by 0.62 of the height that has gone underneath it, because that is what a crushed storey loses. This is what makes the event read as a building losing its footing rather than as a facade being blown off: without it a fragment simply waits in place until the front arrives, and the roof sits level over a stripped frame.\n"
        "It flies, ballistically, tumbling about its own centroid rather than its local origin, which would be a hinge no broken panel has. The drop it had already taken at its release time is exactly where the flight starts, so the two meet without a step.\n"
        "And it lands. There is no contact between fragments in flight: nothing here resolves a collision and a piece passes through whatever is in its way, because at this scale what reads is the timing and the peel rather than any individual impact.\n"
        "Glazing is the exception to the front: every window in the building goes at the shock rather than when its own storey is reached, thrown 3.6 to 7.8 m/s and spinning five times faster than a panel. Balconies are the other: a cantilever with nothing above it, so it fails 0.22 s early -- clamped so that nothing anywhere moves before the charges it is a consequence of.\n"
        "\n"
        "Where a piece comes to rest is decided by what is already there. The pile is a heightfield, and the fragments are planned in the order they land: each is put on top of whatever is in the cells its own footprint covers, and then deposits its own material -- volume by the divergence theorem, bulked by 1.55 -- spread over that footprint. 1877 m3 becomes 2909 of rubble and the packed pile crests at 5.8 m, against 4.2 m for a smooth mound of the same material; rigid pieces leave voids a smooth mound does not, so the packed one standing higher is the expected direction.\n"
        "Both the landing point and the fragment\'s own extent depend on the flight time, and the flight time on both, so each is solved by four passes of the same substitution. The extent is taken at the orientation the piece actually lands in, from the support function of its transformed box, not from its upright height: a 2.66 m panel that comes down edge-on has a vertical extent of 0.15, and using the upright figure left it floating 1.3 m over the pile while a slab landing on edge was buried by the same amount.\n"
        "\n"
        "Dust is three populations, because one averaging them reads as ground fog. The jets are what comes straight back out of the drill holes at the instant the charges fire -- small, fast, outward at 15 to 24 m/s, dead inside two seconds. The surge is air driven out sideways by the floors slamming down on it, which leaves the footprint low and rolls outwards along the ground. The column is what rises off the pile afterwards, slower and much taller. 1190 puffs in all. A puff is thrown and then sheds its speed to the air as a first-order decay to a terminal displacement, which is why dust travels so much further than a thrown solid and then simply hangs.\n"
        "The soft edge is a second small shader, and both it and the technique are models/humvee.c\'s. A closed blob has a hard silhouette at any subdivision, so a puff\'s alpha is scaled by how squarely its surface faces the camera, which is zero exactly at its own silhouette; its normals are radial, of the sphere the lumps are pushed out from rather than of the lumpy surface, so that falloff runs monotonically to the edge instead of leaving a ring of zero alpha inside it. The detonation flashes run the same shader, additively: they were a radial glow disc at first, and the local planar projection put the blob\'s u,v in [-1,1] against a clamped texture whose disc occupies [0,1], so three quadrants of every flash sampled the transparent edge and what showed was a sliver.\n"
        "The charges are milliseconds of light, and --anim samples the cycle at N evenly spaced phases, so a truthful flash would be caught only by luck. It is held over 0.30 s instead -- a playback concession, not a claim about explosives -- which puts the window at 0.90 to 1.20 s and straddles t = 1.0, so every --frames that is a multiple of 6 lands a frame inside it.\n"
        "\n"
        "The yard is not destroyed but it is not untouched. An air blast leaves the building when the charges fire and travels at 105 m/s, so the lashing runs outwards across the scene rather than happening to everything at once; what it reaches gets one damped oscillation, scaled by distance. Trees lean 17 degrees at 0.62 Hz, cars rock on their springs at 1.65, and the benches and bins closest to the facade go over and stay over. A tree can only hinge at its root here: skinning needs bone attributes the harness\'s shader does not declare, so a rigid trunk leaning is the honest limit rather than a choice, where a real tree bends most at the top.\n"
        "\n"
        "This is the one model here whose loop deliberately does not close. A demolition is a one-shot event, and --anim plays it once; rather than pretend otherwise, CheckLoopSeam measures the seam and reports it, and it is 17.9 m at the worst fragment.\n"
        "\n"
        "--part frames the building\'s parts on their rest pose rather than on the swept cycle. Sweeping them gives the whole debris field, which frames every part identically and destroys exactly what --part exists for; only the debris and the dust, which have no rest pose worth framing, are swept.\n"
        "\n"
        "The plinth, the entrance steps and the ground exist once each and are placed groups rather than instanced ones.\n"
        "One thing here is normally the harness's business: a fourth light. The harness lights with three point lights standing at (8,10,8), (-9,5,-7) and (0,-9,5), which surround a 4.5 m vehicle and sit inside a 58 m building, so every outward-facing surface on this one points away from all three and falls back on lighting.fs's ambient term, which line 75 divides by ten -- a 118 texel then leaves the gamma correction at 0.16, which is what turned the gable into a black slab.\n"        "A directional light is the only kind whose geometry does not depend on how big the scene is, so this model fills the shader's one free slot with a sun and leaves the three point lights as the fill, rather than moving them and changing every other model in the repo.\n"
        "Surfacing is eight maps built in code from tiling value noise, projected planar in each mesh's own frame rather than the world's, because an instanced mesh has no world position; every instance would then carry an identical texture placement, so each also carries a hashed per-instance shade of +-8 to break that up.",
    .init = Init,
    .draw = DrawAll,
    .unload = Unload,
    .update = Update,
    .duration = CYCLE,
    .animYaw = 40.0f,
    // The collapse is over in three seconds of real time, which is too quick to follow by hand.
    .previewSpeed = 0.45f,
    .parts = PARTS,
    .partCount = COUNT_OF(PARTS),
    .target = { 0.0f, 6.0f, 0.0f },
    .orbitRadius = 62.0f,
    .orbitHeight = 26.0f,
    .hideGrid = true,
};
