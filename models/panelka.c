#include "harness.h"
#include "raymath.h"
#include "rlgl.h"
// Declarations only: harness.c is the translation unit that defines RLIGHTS_IMPLEMENTATION, and this file only adds a light to the shader it already set up.
#include "rlights.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

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
    unsigned char *colors;
    unsigned short *indices;
    int vertexCount;
    int triangleCount;
    int vertexCap;
    int triangleCap;
} Builder;

// Weathering is baked per vertex rather than into a map, because a map here is shared by every instance of a type and repeats every metre, where a streak has to sit under the one sill it runs from.
// Both shaders read the attribute -- lighting.fs computes tint = colDiffuse*fragColor at its line 42 -- so a vertex colour is a straight multiplier on whatever the texel and the instance tint already give, and darkening is the only thing it is used for here.
//
// The hook is a function of the position in the group's own frame, set around the piece being built and cleared afterwards, so a builder that wants no weathering asks for none and pays nothing but a null check.
static float (*gShadeFn)(Vector3 p);

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
    unsigned char k = 255;
    if (gShadeFn) k = (unsigned char)Clamp(gShadeFn(p) * 255.0f, 0.0f, 255.0f);
    b->colors[i * 4 + 0] = k;
    b->colors[i * 4 + 1] = k;
    b->colors[i * 4 + 2] = k;
    b->colors[i * 4 + 3] = 255;
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
    MAT_PANEL, MAT_JOINT, MAT_CONCRETE, MAT_PLINTH, MAT_ROOF,
    MAT_GLASS, MAT_PANE, MAT_FRAME, MAT_METAL, MAT_RUST, MAT_TIMBER,
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
    [MAT_JOINT]    = WHITE,
    [MAT_PANE]     = WHITE,
    [MAT_RUST]     = WHITE,
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

// Washed exposed aggregate -- the finish the 1-464's outer panels actually carry, and the thing references/panelka/ref_05.jpg makes unmistakable at the scale it was cropped to: dense pale pebbles of roughly a centimetre standing in a grey matrix, salt-and-pepper rather than mottled, at a contrast nothing else on the building comes near.
// The first build had a smooth blotch here, and it is why every render of it read as poured grey: a mottle at half a metre is invisible past twenty, where a pebble field keeps its tooth because the eye reads its variance rather than its pattern.
//
// One texel is TEX_REPEAT_M/512 = 2 mm, so a pebble is five or six texels across, which is what the two high-frequency lattices set. Taking the larger of two independent fields rather than one is what makes the pebbles read as separate stones with matrix between them instead of as a noise wash: a single field spends half its area near its own mean, and two maxed spend most of theirs at one extreme or the other.
static Texture2D MakePebbleTexture(Color matrix, Color pebble, unsigned int seed)
{
    const int S = 512;
    Image img = NewImage(S);
    Color *px = (Color *)img.data;
    for (int y = 0; y < S; y++) {
        for (int x = 0; x < S; x++) {
            float u = (float)x / (float)S, v = (float)y / (float)S;
            float a = ValueNoise(u, v, 96, seed);
            float b = ValueNoise(u + 0.37f, v + 0.11f, 128, seed + 17u);
            float stone = fmaxf(a, b);
            // Anything past the threshold is a stone, and how far past sets how much of the light face it shows, so the field has a few bright ones rather than one flat tone.
            float k = Clamp((stone - 0.58f) / 0.34f, 0.0f, 1.0f);
            k = k * k * (3.0f - 2.0f * k);
            Color c = {
                (unsigned char)Lerp((float)matrix.r, (float)pebble.r, k),
                (unsigned char)Lerp((float)matrix.g, (float)pebble.g, k),
                (unsigned char)Lerp((float)matrix.b, (float)pebble.b, k),
                255,
            };
            // A slow soiling over the top, so one panel is not uniformly the same grey across its whole 2.6 m.
            px[y * S + x] = Shade(c, (Fbm(u, v, 3, seed + 41u, 3) - 0.5f) * 9.0f
                                   + (Fbm(u, v, 224, seed + 59u, 2) - 0.5f) * 6.0f);
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
    MAT_TEX[MAT_PANEL]    = MakePebbleTexture((Color){ 100, 97, 90, 255 }, (Color){ 152, 148, 137, 255 }, 3u);
    // The smooth cast border every panel carries round its aggregate field, and the reveal of every opening cut through it: lighter than the field's average and with none of its tooth, which is exactly what makes the joint grid read as a pale cross rather than as a shadow.
    MAT_TEX[MAT_JOINT]    = MakeRenderTexture((Color){ 128, 126, 119, 255 }, 71u, 12.0f, 7.0f);
    MAT_TEX[MAT_CONCRETE] = MakeConcreteTexture();
    MAT_TEX[MAT_PLINTH]   = MakeRenderTexture((Color){ 66, 64, 60, 255 }, 51u, 16.0f, 14.0f);
    MAT_TEX[MAT_ROOF]     = MakeRoofTexture();
    MAT_TEX[MAT_GLASS]    = MakeGlassTexture();
    // A window pane is not glass here, it is what is behind the glass: a net curtain, a dark room, a kitchen. So its map is authored near white and its whole colour is the per-instance tint, where MAT_GLASS is authored nearly black and is the same everywhere.
    // Multiplying the two was the first attempt and every window came out black, because a curtain picked at 150 against a 24 texel leaves 14.
    MAT_TEX[MAT_PANE]     = MakeRenderTexture((Color){ 228, 226, 222, 255 }, 233u, 10.0f, 8.0f);
    // Authored white, because a window frame's colour is the flat's choice: the same mesh is a white PVC replacement in one apartment and painted timber in the next, and the difference is the per-instance tint.
    MAT_TEX[MAT_FRAME]    = MakeRenderTexture((Color){ 208, 206, 200, 255 }, 83u, 8.0f, 10.0f);
    MAT_TEX[MAT_METAL]    = MakeMetalTexture();
    // Downpipes and balcony steelwork, which ref_03 and ref_05 both show as rusted through rather than painted: the mottle is the rust rather than a bloom over paint.
    MAT_TEX[MAT_RUST]     = MakeRenderTexture((Color){ 92, 58, 40, 255 }, 167u, 22.0f, 16.0f);
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
        mesh.colors = b->colors;
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
#define MAX_GROUPS 128
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

// Half the smooth cast border between two panels: the aggregate field sets back by this, so two neighbours leave twice it between them.
//
// Two numbers, not one, and both read off ref_05.jpg against a metric ruler drawn on the unscaled image at 117.7 px/m -- the storey pitch recovered by autocorrelating a window-free column of the gable, 313 px for 2.66 m. The band between two fields measures about 0.085 m vertically and 0.145 m horizontally, so 0.043 and 0.072 a side. The horizontal is genuinely the wider of the two: it is the bed the floor slab bears in and it is packed with mortar, where the vertical is a caulked butt.
//
// A first pass at this put both at 0.170, off a 45 px reading taken on a crop that had been upscaled two-fold and then divided by the unscaled image's px/m. The error is worth recording rather than just fixing, because the conclusion drawn from it -- that the first build's facade read as a flat grey sheet because its border was too thin -- was wrong twice over. The border was about right at 0.045. What was missing was that both layers carried the same map, so there was nothing to see at any width; the fix that mattered was giving the core the smooth render map and the field the aggregate one.
#define JOINT_V       0.045f
#define JOINT_H       0.072f
// The margin an opening must keep clear, which is the larger of the two, since one opening is checked against both edges.
#define JOINT         JOINT_H
// How far every piece is grown past its own edge so that it knits into its neighbour instead of
// meeting it on an exactly coincident plane. Two coplanar back-to-back faces are the one thing
// this file draws hundreds of, and left touching they speckle along every joint: the rasterizer
// picks between them per pixel and the facade comes out ruled with dashed lines. It is also the
// honest construction, since a real panel is butted and caulked rather than laid edge to edge.
#define KNIT          0.006f
// How far the aggregate field stands proud of the smooth border round it. ref_05 shows the two nearly flush -- a lip you can see the shadow of and no more -- rather than the 0.06 m rebate the first build had, which threw a 60 mm shadow all the way round every panel.
#define FACE_T        0.014f
#define CORE_T        (WALL_T - FACE_T)
// How coarse a cell the face layer's decomposition may leave. It changes no geometry: it exists so the streak in PanelGrime has vertices to be written onto, and it is only asked of the layer whose face is seen. Cost is per panel *type*, not per instance -- there are eleven types behind 210 panels.
#define PANEL_SEG     0.22f

// How high the charges reach. It lives here with the storey it is counted in rather than down with the demolition, because the fines thrown out of the cut are placed long before the demolition's own constants are declared, and a second copy of the expression is how the two would drift apart.
#define CUT_TOP       (PLINTH_Y + 2.0f * STOREY)

#define ROOF_Y        (PLINTH_Y + FLOORS * STOREY)      // 14.25, top of the fifth-floor ceiling
// Flat, which is the series' own specification and what ref_03 and ref_05 show; the pitched metal roofs in ref_02 and ref_07 are the re-roofing thousands of these got in the 1990s, not the building.
// What those two do show is a thin cornice with a metal capping that oversails it, not the 0.45 m parapet this model first had.
// A thin capped cornice, not a parapet. ref_03 and ref_05 both show the flat roof ending in a shallow upstand under a metal coping, with almost none of the upstand's outer face exposed; a review round read 0.320 m of it as a modern fascia band running the whole building.
#define PARAPET_H     0.210f
#define PARAPET_T     0.180f
#define CORNICE_O     0.085f   // how far the capping oversails the facade below it

// Openings. Sills are measured up from the floor the panel stands on.
#define WIN_SILL      0.850f
#define WIN_H         1.450f
#define WIN_W26       1.300f
#define WIN_W32       1.500f
#define BDOOR_W       0.750f
#define BDOOR_SILL    0.240f   // the threshold step up onto the balcony. It has to clear JOINT, or the door starts inside the border band round the panel and reads as running off the bottom of the aggregate field.
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

// The most openings any one panel here is cut with: a balcony bay's door and the window beside it.
#define MAX_OPENINGS 2

// One layer of a wall panel: a slab of thickness z0..z1 covering x0..x1 by y0..y1 of the panel's own frame, with rectangular openings cut out of it.
//
// It takes the layer's absolute extent rather than an inset off the panel, and that is what lets one function serve three jobs that pull in different directions: the cast core runs a hair past the panel edge so it knits into its neighbour, the aggregate field stops JOINT short of it so the border shows, and a fracture piece stops where the break is on one side and where the panel does on the other. An inset cannot say the last of those, and a single inset gave every fracture piece a border down its break, so an intact building read as though it had been built out of half-panels.
//
// Decomposed into horizontal bands at every opening edge and, within a band, into the x-segments the openings leave. Doing it that way rather than case by case means no opening count needs its own code, and every reveal face comes out as the side of a closed box rather than as a hole that has to be stitched. An opening that falls outside the layer is clipped to nothing by the same Clamp that trims one straddling its edge, so a fracture piece needs no opening list of its own.
//
// `seg`, when positive, is the coarsest cell the decomposition may leave: every band and every x-segment is then divided until no piece exceeds it. It changes no surface and exists only so gShadeFn has vertices to write a streak onto. A layer nobody sees the face of asks for 0 and stays at a dozen boxes.
//
// Openings must be listed left to right; CheckOpenings proves they are.
static void WallLayer(Builder *b, float x0, float x1, float y0, float y1, float z0, float z1,
                      const Rect *op, int nop, float seg)
{
    if (x1 - x0 < 1e-5f || y1 - y0 < 1e-5f) return;

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

        int rows = (seg > 0.0f) ? (int)ceilf((yb - ya) / seg) : 1;
        for (int r = 0; r < rows; r++) {
            float ra = ya + (yb - ya) * (float)r / (float)rows;
            float rb = ya + (yb - ya) * (float)(r + 1) / (float)rows;

            float cur = x0;
            for (int i = 0; i <= nop; i++) {
                float a = x1, c = x1;
                if (i < nop) {
                    // The band is chosen by its own midpoint, so an opening either crosses the whole band or none of it.
                    if (op[i].y0 > ym || op[i].y1 < ym) continue;
                    a = Clamp(op[i].x0, x0, x1);
                    c = Clamp(op[i].x1, x0, x1);
                }
                if (a > cur) {
                    int cols = (seg > 0.0f) ? (int)ceilf((a - cur) / seg) : 1;
                    for (int q = 0; q < cols; q++) {
                        Box(b, cur + (a - cur) * (float)q / (float)cols,
                            cur + (a - cur) * (float)(q + 1) / (float)cols, ra, rb, z0, z1);
                    }
                }
                if (c > cur) cur = c;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Grime
//
// Concrete weathers where water runs over it, and on a panel block water runs from exactly three places: off every sill, off the top edge of every panel onto the one below, and down to the horizontal joint at each panel's foot, where it stops. Nothing in the first build recorded any of that, and a facade with no run-off on it reads as new whatever its texture is -- which is the one thing a 1962 building is not.
//
// It is written per vertex rather than into a map, because a map here is shared by every instance of a panel type and repeats every metre, where a streak has to start under the one sill it runs from. That is what PANEL_SEG's subdivision is for and the only thing it is for.
// ---------------------------------------------------------------------------

static Rect gGrimeOp[MAX_OPENINGS];
static int gGrimeNop;
static float gGrimeH;

// How far a streak runs before the rain has washed the last of it out of the concrete.
#define GRIME_RUN     1.15f

static float PanelGrime(Vector3 p)
{
    float d = 0.0f;

    for (int i = 0; i < gGrimeNop; i++) {
        float below = gGrimeOp[i].y0 - p.y;
        if (below <= 0.0f || below > GRIME_RUN) continue;
        // Darkest immediately under the sill and squared away downwards, which is how a stain that is being diluted the whole way down actually falls off.
        float down = 1.0f - below / GRIME_RUN;
        down *= down;
        // Wider than the opening, because the run-off spreads as it leaves the sill's ends, and ramped over a width the subdivision can actually resolve.
        float lx = (p.x - (gGrimeOp[i].x0 - 0.12f)) / 0.26f;
        float rx = ((gGrimeOp[i].x1 + 0.12f) - p.x) / 0.26f;
        float across = Clamp(fminf(lx, rx), 0.0f, 1.0f);
        // Broken into separate runs rather than laid on as one even wash: a sill sheds where its drip is worn, not along its whole length.
        float runs = 0.62f + 0.62f * Fbm(p.x * 0.47f + 0.31f, 0.17f, 10, 809u, 1);
        d = fmaxf(d, 0.46f * down * across * fminf(1.0f, runs));
    }

    // Dirt gathers along the foot of every panel, where the run-off reaches the horizontal joint and goes no further.
    d = fmaxf(d, 0.26f * Clamp((0.42f - p.y) / 0.42f, 0.0f, 1.0f));
    // And along the head, which is what the panel above shed onto this one.
    d = fmaxf(d, 0.16f * Clamp((p.y - (gGrimeH - 0.30f)) / 0.30f, 0.0f, 1.0f));

    return 1.0f - d;
}

// A facade panel, or the slice xa..xb of one that a fracture leaves.
//
// Two layers: a cast core across the full width, and the exposed-aggregate field standing FACE_T proud of it and stopping JOINT short of every edge. That setback is the border, and it is the grid the whole facade reads by. The core carries the smooth render map rather than the aggregate one, so the border and the reveal of every opening come out pale and untoothed against the pebble field, which is what ref_05.jpg shows and what the first build -- one material for both layers -- could not show at any distance.
//
// A break is not an edge. Where the slice ends inside the panel, both layers run KNIT past it instead of setting back, so the two halves of an unbroken panel are indistinguishable from the whole one until they separate.
static void FacadePiece(Group *g, float w, float h, const Rect *op, int nop, float xa, float xb)
{
    float hw = w * 0.5f;
    bool atL = (xa <= -hw + 1e-4f), atR = (xb >= hw - 1e-4f);

    WallLayer(&g->b[MAT_JOINT], atL ? -hw - KNIT : xa - KNIT, atR ? hw + KNIT : xb + KNIT,
              -KNIT, h + KNIT, -KNIT, CORE_T, op, nop, 0.0f);

    // Only the layer whose face the weather actually reaches. The core is the reveal of every opening and the panel's back, and neither of those is rained on.
    for (int i = 0; i < nop && i < MAX_OPENINGS; i++) gGrimeOp[i] = op[i];
    gGrimeNop = (nop < MAX_OPENINGS) ? nop : MAX_OPENINGS;
    gGrimeH = h;
    gShadeFn = PanelGrime;
    WallLayer(&g->b[MAT_PANEL], atL ? -hw + JOINT_V : xa - KNIT, atR ? hw - JOINT_V : xb + KNIT,
              JOINT_H, h - JOINT_H, CORE_T, WALL_T, op, nop, PANEL_SEG);
    gShadeFn = NULL;
}

static void FacadePanel(Group *g, float w, float h, const Rect *op, int nop)
{
    FacadePiece(g, w, h, op, nop, -w * 0.5f, w * 0.5f);
}

// The joinery that fills an opening, and the pane behind it. Authored in the same frame as its panel, so it rides the same placement matrix.
//
// How far back it sits is measured from the *outer* face rather than derived from the core, so the reveal stays 0.105 m whatever the aggregate field's thickness is later set to. Deriving it from CORE_T is how the reveal quietly vanished when FACE_T came down from 0.060 to 0.022.
#define GLZ_INSET     0.105f
#define GLZ_Z1        (WALL_T - GLZ_INSET)
#define GLZ_Z0        (GLZ_Z1 - 0.050f)

// Two generations of window, which is the loudest thing about a khrushchyovka facade today and the thing the first build had none of: ref_03, ref_05, ref_06 and ref_07 all show a chequerboard of white plastic replacements against the original painted timber, flat by flat, because each flat replaced its own when it could afford to.
// The plastic one is two lights on an off-centre mullion and nothing else. The original is heavier in section, splits down the middle, and carries a transom across the full width with a small hinged vent (a fortochka) in one upper light, which is the detail that dates it.
#define GLZ_BAR_PVC   0.046f
#define GLZ_BAR_TIM   0.092f

static void Joinery(Group *g, Rect r, bool pvc)
{
    Builder *f = &g->b[MAT_FRAME];
    float bar = pvc ? GLZ_BAR_PVC : GLZ_BAR_TIM;
    float ix0 = r.x0 + bar, ix1 = r.x1 - bar;
    float iy0 = r.y0 + bar, iy1 = r.y1 - bar;
    if (ix1 - ix0 < 0.10f || iy1 - iy0 < 0.10f) return;

    Box(f, r.x0, r.x1, r.y0, iy0, GLZ_Z0, GLZ_Z1);
    Box(f, r.x0, r.x1, iy1, r.y1, GLZ_Z0, GLZ_Z1);
    Box(f, r.x0, ix0, iy0, iy1, GLZ_Z0, GLZ_Z1);
    Box(f, ix1, r.x1, iy0, iy1, GLZ_Z0, GLZ_Z1);

    if (pvc) {
        // The mullion sits off centre, which is what a two-sash replacement unit does: one wide fixed light and one narrow opener.
        float mx = r.x0 + (r.x1 - r.x0) * 0.62f;
        Box(f, mx - bar * 0.5f, mx + bar * 0.5f, iy0, iy1, GLZ_Z0, GLZ_Z1);
        return;
    }

    float mx = 0.5f * (r.x0 + r.x1);
    float ty = iy1 - (iy1 - iy0) * 0.30f;
    Box(f, mx - bar * 0.5f, mx + bar * 0.5f, iy0, iy1, GLZ_Z0, GLZ_Z1);
    Box(f, ix0, ix1, ty - bar * 0.5f, ty + bar * 0.5f, GLZ_Z0, GLZ_Z1);
    // The fortochka: a small hinged vent in the upper light nearest the mullion, standing a little proud of the sash it is set into.
    Box(f, mx + bar * 0.5f, mx + bar * 0.5f + (ix1 - mx) * 0.52f, ty + bar * 0.5f, iy1, GLZ_Z1, GLZ_Z1 + 0.016f);
}

// The pane is its own group, not part of the joinery, for the same reason the car's glass is not part of its body: a per-instance tint reaches every material a group owns, and what varies here is what shows *behind* the glass -- a curtain, a dark room, a kitchen -- while the frame stays whatever the flat painted it.
// It sits behind the joinery rather than flush, so the frame casts across it.
static void Pane(Group *g, Rect r)
{
    float bar = GLZ_BAR_PVC;
    Box(&g->b[MAT_PANE], r.x0 + bar, r.x1 - bar, r.y0 + bar, r.y1 - bar, GLZ_Z0 - 0.030f, GLZ_Z0 - 0.010f);
}

// A window sill: the one moulding on a khrushchyovka facade that is not flat, and the only thing that stops a window reading as a sticker. Cast render rather than aggregate, which is what ref_05 shows and what every sill on a panel block is.
static void Sill(Group *g, Rect r)
{
    Box(&g->b[MAT_JOINT], r.x0 - 0.060f, r.x1 + 0.060f, r.y0 - 0.075f, r.y0, WALL_T - 0.020f, WALL_T + 0.055f);
}

// ---------------------------------------------------------------------------
// Fragments
//
// Every piece of the building is an instance of one of these types. A fragment record says which type and where its rest pose is; Update turns the whole list into transforms, so the intact building and the collapsing one differ only in what Update writes.
// ---------------------------------------------------------------------------

typedef enum {
    // Facade panels. Each of the four that repeat enough to be worth it also exists as the two halves it fractures into; the stairwell and entrance panels come down whole.
    FR_P26, FR_P26L, FR_P26R,
    FR_P32, FR_P32L, FR_P32R,
    FR_P32B, FR_P32BL, FR_P32BR,
    FR_PEND, FR_PENDL, FR_PENDR,
    FR_PSTAIR, FR_PSTAIR2, FR_PDOOR,
    // Joinery, in both generations where a flat has a say in it.
    FR_J26P, FR_J26T, FR_J32P, FR_J32T, FR_J32BP, FR_J32BT, FR_JSTAIR, FR_JSTAIR2, FR_JDOOR,
    // Panes, separate from the joinery so they can carry the curtain behind them.
    FR_Q26, FR_Q32, FR_Q32B, FR_QSTAIR, FR_QSTAIR2, FR_QDOOR,
    // Fittings the residents hung on it afterwards.
    FR_BARS, FR_AC, FR_PIPE,
    // Structure. Every plate is four types, one per cell it breaks into, and they must stay in this order and adjacent: PlateType indexes from the first of each four.
    FR_SLAB26, FR_SLAB26B, FR_SLAB26C, FR_SLAB26D,
    FR_SLAB32, FR_SLAB32B, FR_SLAB32C, FR_SLAB32D,
    FR_ROOF26, FR_ROOF26B, FR_ROOF26C, FR_ROOF26D,
    FR_ROOF32, FR_ROOF32B, FR_ROOF32C, FR_ROOF32D,
    FR_XWALL, FR_XWALLB, FR_XWALLC, FR_XWALLD,
    FR_SPINE26, FR_SPINE32,
    // A balcony in three pieces, so each carries its own colour and each leaves separately.
    FR_BALC, FR_BALCSHEET, FR_BALCGLZ, FR_BALCPANE,
    FR_PAR26, FR_PAR32, FR_PAREND, FR_CANOPY, FR_VENT, FR_MAST,
    FR_BIRCH, FR_MAPLE, FR_CARBODY, FR_CARTRIM, FR_BENCH, FR_RUGFRAME, FR_LAMP, FR_BIN,
    FR_COUNT
} FragType;

// What a fragment is, for the two questions the collapse asks of every one of them: when it lets go, and how hard.
//
// A table rather than a range test on the enum. The first build asked `type <= FR_PEND` for "is this skin" and `type >= FR_G26 && type <= FR_GDOOR` for "is this glass", which is correct exactly as long as nobody inserts a type, and this revision inserts thirty. A misfiled type there is silent: the piece simply falls with the wrong timing, in a scene where several hundred pieces are falling at once.
typedef enum {
    FC_SKIN,      // facade panel: held on by nothing once the cross wall behind it is gone
    FC_GLASS,     // goes at the shock, wherever in the building it is
    FC_FITTING,   // bolted to the skin, and leaves with it
    FC_STRUCT,    // slabs, cross walls, the spine: what the pile is made of
    FC_BALCONY,   // a cantilever with nothing above it, so it fails early
    FC_YARD,      // not destroyed, but not untouched either
} FragClass;

static const FragClass FR_CLASS[FR_COUNT] = {
    [FR_P26] = FC_SKIN, [FR_P26L] = FC_SKIN, [FR_P26R] = FC_SKIN,
    [FR_P32] = FC_SKIN, [FR_P32L] = FC_SKIN, [FR_P32R] = FC_SKIN,
    [FR_P32B] = FC_SKIN, [FR_P32BL] = FC_SKIN, [FR_P32BR] = FC_SKIN,
    [FR_PEND] = FC_SKIN, [FR_PENDL] = FC_SKIN, [FR_PENDR] = FC_SKIN,
    [FR_PSTAIR] = FC_SKIN, [FR_PSTAIR2] = FC_SKIN, [FR_PDOOR] = FC_SKIN,
    [FR_J26P] = FC_GLASS, [FR_J26T] = FC_GLASS, [FR_J32P] = FC_GLASS, [FR_J32T] = FC_GLASS,
    [FR_J32BP] = FC_GLASS, [FR_J32BT] = FC_GLASS, [FR_JSTAIR] = FC_GLASS,
    [FR_JSTAIR2] = FC_GLASS, [FR_JDOOR] = FC_GLASS,
    [FR_Q26] = FC_GLASS, [FR_Q32] = FC_GLASS, [FR_Q32B] = FC_GLASS,
    [FR_QSTAIR] = FC_GLASS, [FR_QSTAIR2] = FC_GLASS, [FR_QDOOR] = FC_GLASS,
    [FR_BARS] = FC_FITTING, [FR_AC] = FC_FITTING, [FR_PIPE] = FC_FITTING,
    [FR_SLAB26] = FC_STRUCT, [FR_SLAB26B] = FC_STRUCT, [FR_SLAB26C] = FC_STRUCT, [FR_SLAB26D] = FC_STRUCT,
    [FR_SLAB32] = FC_STRUCT, [FR_SLAB32B] = FC_STRUCT, [FR_SLAB32C] = FC_STRUCT, [FR_SLAB32D] = FC_STRUCT,
    [FR_ROOF26] = FC_STRUCT, [FR_ROOF26B] = FC_STRUCT, [FR_ROOF26C] = FC_STRUCT, [FR_ROOF26D] = FC_STRUCT,
    [FR_ROOF32] = FC_STRUCT, [FR_ROOF32B] = FC_STRUCT, [FR_ROOF32C] = FC_STRUCT, [FR_ROOF32D] = FC_STRUCT,
    [FR_XWALL] = FC_STRUCT, [FR_XWALLB] = FC_STRUCT, [FR_XWALLC] = FC_STRUCT, [FR_XWALLD] = FC_STRUCT,
    [FR_SPINE26] = FC_STRUCT, [FR_SPINE32] = FC_STRUCT,
    [FR_BALC] = FC_BALCONY, [FR_BALCSHEET] = FC_BALCONY, [FR_BALCGLZ] = FC_BALCONY,
    // The pane goes at the shock like every other pane in the building, not with the balcony it is fixed to.
    [FR_BALCPANE] = FC_GLASS,
    [FR_PAR26] = FC_STRUCT, [FR_PAR32] = FC_STRUCT, [FR_PAREND] = FC_STRUCT,
    [FR_CANOPY] = FC_STRUCT, [FR_VENT] = FC_STRUCT, [FR_MAST] = FC_FITTING,
    [FR_BIRCH] = FC_YARD, [FR_MAPLE] = FC_YARD, [FR_CARBODY] = FC_YARD, [FR_CARTRIM] = FC_YARD,
    [FR_BENCH] = FC_YARD, [FR_RUGFRAME] = FC_YARD, [FR_LAMP] = FC_YARD, [FR_BIN] = FC_YARD,
};

// Designated rather than positional, so a type inserted in the middle of the enum cannot silently shift every name after it onto the wrong mesh. The names only reach a diagnostic, which is exactly why nobody would have noticed.
static const char *TYPE_NAME[FR_COUNT] = {
    [FR_P26] = "p26", [FR_P26L] = "p26.l", [FR_P26R] = "p26.r",
    [FR_P32] = "p32", [FR_P32L] = "p32.l", [FR_P32R] = "p32.r",
    [FR_P32B] = "p32b", [FR_P32BL] = "p32b.l", [FR_P32BR] = "p32b.r",
    [FR_PEND] = "pend", [FR_PENDL] = "pend.l", [FR_PENDR] = "pend.r",
    [FR_PSTAIR] = "pstair", [FR_PSTAIR2] = "pstair2", [FR_PDOOR] = "pdoor",
    [FR_J26P] = "j26.pvc", [FR_J26T] = "j26.timber",
    [FR_J32P] = "j32.pvc", [FR_J32T] = "j32.timber",
    [FR_J32BP] = "j32b.pvc", [FR_J32BT] = "j32b.timber",
    [FR_JSTAIR] = "jstair", [FR_JSTAIR2] = "jstair2", [FR_JDOOR] = "jdoor",
    [FR_Q26] = "pane26", [FR_Q32] = "pane32", [FR_Q32B] = "pane32b",
    [FR_QSTAIR] = "panestair", [FR_QSTAIR2] = "panestair2", [FR_QDOOR] = "panedoor",
    [FR_BARS] = "bars", [FR_AC] = "aircon", [FR_PIPE] = "pipe",
    [FR_SLAB26] = "slab26.a", [FR_SLAB26B] = "slab26.b", [FR_SLAB26C] = "slab26.c", [FR_SLAB26D] = "slab26.d",
    [FR_SLAB32] = "slab32.a", [FR_SLAB32B] = "slab32.b", [FR_SLAB32C] = "slab32.c", [FR_SLAB32D] = "slab32.d",
    [FR_ROOF26] = "roof26.a", [FR_ROOF26B] = "roof26.b", [FR_ROOF26C] = "roof26.c", [FR_ROOF26D] = "roof26.d",
    [FR_ROOF32] = "roof32.a", [FR_ROOF32B] = "roof32.b", [FR_ROOF32C] = "roof32.c", [FR_ROOF32D] = "roof32.d",
    [FR_XWALL] = "xwall.a", [FR_XWALLB] = "xwall.b", [FR_XWALLC] = "xwall.c", [FR_XWALLD] = "xwall.d",
    [FR_SPINE26] = "spine26", [FR_SPINE32] = "spine32",
    [FR_BALC] = "balc", [FR_BALCSHEET] = "balc.sheet",
    [FR_BALCGLZ] = "balc.enclosure", [FR_BALCPANE] = "balc.pane",
    [FR_PAR26] = "par26", [FR_PAR32] = "par32", [FR_PAREND] = "parend",
    [FR_CANOPY] = "canopy", [FR_VENT] = "vent", [FR_MAST] = "mast",
    [FR_BIRCH] = "birch", [FR_MAPLE] = "maple",
    [FR_CARBODY] = "carbody", [FR_CARTRIM] = "cartrim",
    [FR_BENCH] = "bench", [FR_RUGFRAME] = "rugframe", [FR_LAMP] = "lamp", [FR_BIN] = "bin",
};

typedef struct {
    FragType type;
    Vector3 pos;     // world position of the fragment's local origin, at rest
    float yaw;       // rotation about Y that takes the local frame to the world one, in degrees
    float scale;     // uniform, so one tree mesh can stand in eight sizes
    Color tint;      // per-instance, which is how four cars share one body mesh
    float shade;     // per-instance brightness, so 105 copies of one mesh are not 105 identical panels
} Fragment;

#define MAX_FRAG 5200
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

// The seven panels the block is cut from, and the one place their openings are written down.
//
// The first build wrote each opening rectangle out twice, once for the builder and once for the check that proves it fits, which meant the check could only ever confirm that the literals had been copied correctly. Both now read the same function, so the check tests the panel that is actually built.
typedef enum { PK_26, PK_32, PK_32B, PK_STAIR, PK_STAIR2, PK_DOOR, PK_END, PK_COUNT } PanelKind;

// The gable is four panels across its 11.82 m depth, not two. ref_05.jpg reads four columns across its end wall at the same panel-border pitch the facade carries, and the first build's two 5.91 m slabs gave the gable a grid four times coarser than the rest of the building, which is why it rendered as a blank grey wall from any distance.
#define END_W         (FACE_Z * 0.5f)

static const float PANEL_W[PK_COUNT] = { 2.6f, 3.2f, 3.2f, 2.6f, 2.6f, 2.6f, END_W };

// Where each kind cracks, in its own frame. A panel goes at the side of an opening, which is where its section is thinnest; the blank gable has no opening, so its break is simply off centre, because two congruent halves read as a cut rather than as a fracture.
static const float PANEL_BREAK[PK_COUNT] = {
    -WIN_W26 * 0.5f, -WIN_W32 * 0.5f, -0.200f, 0.0f, 0.0f, 0.0f, 0.55f,
};

// Which fragment types a kind builds: the whole panel, and the two halves, which are the whole one again for a kind that is not worth fracturing.
typedef struct { FragType whole, left, right; } PanelTypes;

static const PanelTypes PANEL_TYPE[PK_COUNT] = {
    [PK_26]     = { FR_P26, FR_P26L, FR_P26R },
    [PK_32]     = { FR_P32, FR_P32L, FR_P32R },
    [PK_32B]    = { FR_P32B, FR_P32BL, FR_P32BR },
    [PK_STAIR]  = { FR_PSTAIR, FR_PSTAIR, FR_PSTAIR },
    [PK_STAIR2] = { FR_PSTAIR2, FR_PSTAIR2, FR_PSTAIR2 },
    [PK_DOOR]   = { FR_PDOOR, FR_PDOOR, FR_PDOOR },
    [PK_END]    = { FR_PEND, FR_PENDL, FR_PENDR },
};

// Openings, left to right, which is what WallLayer's single sweep across a band assumes and what CheckOpenings proves.
static int PanelOpenings(PanelKind k, Rect *out)
{
    switch (k) {
        case PK_26: out[0] = WinRect(WIN_W26); return 1;
        case PK_32: out[0] = WinRect(WIN_W32); return 1;
        case PK_32B:
            // A balcony bay: the door on the left, a window beside it.
            out[0] = (Rect){ -1.300f, -1.300f + BDOOR_W, BDOOR_SILL, BDOOR_SILL + BDOOR_H };
            out[1] = (Rect){ 0.100f, 0.100f + 1.200f, WIN_SILL, WIN_SILL + WIN_H };
            return 2;
        // The stairwell: one tall narrow light per storey, its sill well above an apartment's because the landing it serves is half a flight up. The real thing staggers a window per half-flight; this is one per storey, which is as far as a one-storey panel can go.
        case PK_STAIR:
            out[0] = (Rect){ -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL_LO, STAIR_SILL_LO + STAIR_H };
            return 1;
        case PK_STAIR2:
            out[0] = (Rect){ -STAIR_W * 0.5f, STAIR_W * 0.5f, STAIR_SILL_HI, STAIR_SILL_HI + STAIR_H };
            return 1;
        case PK_DOOR:
            out[0] = (Rect){ -ENTRY_W * 0.5f, ENTRY_W * 0.5f, 0.0f, ENTRY_H };
            return 1;
        // The end wall: two blank panels per floor, each spanning half the depth. Blank is the point -- ref_02, ref_03 and ref_07 all show a gable with no openings at all, which is why ref_04 exists to show one painted instead; ref_05 is the variant that does put two window columns in its gable.
        default: return 0;
    }
}

static const char *PANEL_KIND_NAME[PK_COUNT] = {
    "2.6 m window panel", "3.2 m window panel", "balcony panel",
    "stairwell panel, low sill", "stairwell panel, high sill", "entrance panel", "end wall panel",
};

static void BuildFacadeTypes(void)
{
    for (int k = 0; k < PK_COUNT; k++) {
        Rect op[MAX_OPENINGS];
        int nop = PanelOpenings((PanelKind)k, op);
        float w = PANEL_W[k];
        const PanelTypes *pt = &PANEL_TYPE[k];

        FacadePanel(&gType[pt->whole], w, STOREY, op, nop);
        if (pt->left != pt->whole) {
            // A panel cracks at the side of an opening, which is where the section is thinnest and where a real one goes. PK_END has no opening, so its break is simply off centre, because two halves that are congruent read as a cut rather than as a fracture.
            FacadePiece(&gType[pt->left], w, STOREY, op, nop, -w * 0.5f, PANEL_BREAK[k]);
            FacadePiece(&gType[pt->right], w, STOREY, op, nop, PANEL_BREAK[k], w * 0.5f);
        }

        // The sill belongs to the piece the window's own side of the break is on, so it neither doubles nor vanishes when the panel fractures.
        for (int i = 0; i < nop; i++) {
            if (op[i].y0 < 0.05f) continue;   // a doorway reaches the floor and has no sill
            Sill(&gType[pt->whole], op[i]);
            if (pt->left != pt->whole) {
                Sill(&gType[(0.5f * (op[i].x0 + op[i].x1) < PANEL_BREAK[k]) ? pt->left : pt->right], op[i]);
            }
        }
    }

    // Joinery and panes, one pair per opening that a flat has any say over. The two generations differ in section, in how they divide and in what the flat paid for them: ref_03, ref_05, ref_06 and ref_07 all show the chequerboard.
    Rect w26 = WinRect(WIN_W26), w32 = WinRect(WIN_W32);
    Rect bal[MAX_OPENINGS];
    PanelOpenings(PK_32B, bal);

    Joinery(&gType[FR_J26P], w26, true);
    Joinery(&gType[FR_J26T], w26, false);
    Pane(&gType[FR_Q26], w26);

    Joinery(&gType[FR_J32P], w32, true);
    Joinery(&gType[FR_J32T], w32, false);
    Pane(&gType[FR_Q32], w32);

    for (int pvc = 0; pvc < 2; pvc++) {
        Group *g = &gType[pvc ? FR_J32BP : FR_J32BT];
        Joinery(g, bal[0], pvc != 0);
        Joinery(g, bal[1], pvc != 0);
    }
    Pane(&gType[FR_Q32B], bal[0]);
    Pane(&gType[FR_Q32B], bal[1]);

    // The stairwell is communal, so nobody replaces its windows and both are the original timber.
    Rect stLo[MAX_OPENINGS], stHi[MAX_OPENINGS];
    PanelOpenings(PK_STAIR, stLo);
    PanelOpenings(PK_STAIR2, stHi);
    Joinery(&gType[FR_JSTAIR], stLo[0], false);
    Pane(&gType[FR_QSTAIR], stLo[0]);
    Joinery(&gType[FR_JSTAIR2], stHi[0], false);
    Pane(&gType[FR_QSTAIR2], stHi[0]);

    // The entrance: two steel leaves with a rebate between them, set back in the reveal, and a fanlight above.
    {
        Rect en[MAX_OPENINGS];
        PanelOpenings(PK_DOOR, en);
        Builder *m = &gType[FR_JDOOR].b[MAT_METAL];
        float leafTop = ENTRY_H - 0.500f;
        Box(m, en[0].x0 + 0.040f, -0.020f, 0.020f, leafTop, GLZ_Z0, GLZ_Z1);
        Box(m, 0.020f, en[0].x1 - 0.040f, 0.020f, leafTop, GLZ_Z0, GLZ_Z1);
        Box(m, en[0].x0, en[0].x1, leafTop, leafTop + 0.070f, GLZ_Z0, GLZ_Z1);
        Box(&gType[FR_QDOOR].b[MAT_GLASS], en[0].x0 + 0.060f, en[0].x1 - 0.060f,
            leafTop + 0.070f, en[0].y1 - 0.040f, GLZ_Z0 - 0.030f, GLZ_Z0 - 0.010f);
        // Handles, on the meeting stiles.
        Box(m, -0.130f, -0.060f, 1.000f, 1.120f, GLZ_Z1, GLZ_Z1 + 0.060f);
        Box(m, 0.060f, 0.130f, 1.000f, 1.120f, GLZ_Z1, GLZ_Z1 + 0.060f);

        // A bulkhead lamp over the door and the enamel number plate beside it. Both hang off ENTRY_H rather than off a copy of it, so they follow the doorway if it ever changes.
        float lampY = ENTRY_H + 0.180f;
        Tube(m, (Vector3){ 0.0f, lampY, WALL_T }, (Vector3){ 0.0f, lampY, WALL_T + 0.150f }, 0.075f, 0.115f, 12, false, false);
        Box(&gType[FR_QDOOR].b[MAT_GLASS], -0.105f, 0.105f, lampY - 0.105f, lampY + 0.105f, WALL_T + 0.148f, WALL_T + 0.160f);
        Box(&gType[FR_JDOOR].b[MAT_FRAME], 0.520f, 0.880f, ENTRY_H - 0.240f, ENTRY_H + 0.020f, WALL_T, WALL_T + 0.018f);
    }

    // Ground-floor window bars, which ref_06 and ref_07 both show on the flats at street level: a welded grid standing a little off the reveal. One mesh, sized to the wider window, since the narrower one is the same grid cut shorter and nobody at this distance counts its bars.
    {
        Builder *m = &gType[FR_BARS].b[MAT_METAL];
        float x0 = -WIN_W26 * 0.5f + 0.030f, x1 = WIN_W26 * 0.5f - 0.030f;
        float y0 = WIN_SILL + 0.030f, y1 = WIN_SILL + WIN_H - 0.030f;
        float z = GLZ_Z1 + 0.030f;
        for (int i = 0; i <= 7; i++) {
            float x = Lerp(x0, x1, (float)i / 7.0f);
            Box(m, x - 0.010f, x + 0.010f, y0, y1, z, z + 0.020f);
        }
        for (int i = 0; i <= 4; i++) {
            float y = Lerp(y0, y1, (float)i / 4.0f);
            Box(m, x0, x1, y - 0.010f, y + 0.010f, z + 0.020f, z + 0.040f);
        }
    }

    // A split air conditioner, which ref_06 hangs beside a window and which nothing in the first build had: the one piece of the 21st century on a 1962 facade.
    {
        Group *g = &gType[FR_AC];
        Builder *p = &g->b[MAT_PAINT];
        Box(p, -0.400f, 0.400f, 0.0f, 0.560f, WALL_T + 0.030f, WALL_T + 0.290f);
        Builder *m = &g->b[MAT_METAL];
        Box(m, -0.360f, 0.360f, -0.070f, 0.0f, WALL_T + 0.060f, WALL_T + 0.260f);
        Box(m, -0.360f, 0.360f, 0.560f, 0.630f, WALL_T + 0.060f, WALL_T + 0.260f);
    }
}

// ---------------------------------------------------------------------------
// How a plate breaks
//
// Concrete does not break at right angles, and a slab cut into rectangles lands as a stack of tiles. So a plate is cut by two straight lines that cross at its centre, each meeting the edges off-square, which leaves four quadrilateral cells: two roughly triangular and two roughly trapezoidal, none of them rectangular, and no two of them the same shape.
//
// The cells still tile the plate exactly, because they are defined by the cuts rather than by their own extents: every interior edge is shared by the two cells either side of it, and the two cuts meet at the centre by construction. So the intact building is unchanged and only the pile is different, which is the same bargain the panel fracture makes.
//
// Four cells rather than a jagged many: a cell has to stay a hexahedron for Hex to build it, which caps it at four corners in plan. A zigzag break would need two hexahedra per piece and is not worth it at the distance any of this is seen from.
// ---------------------------------------------------------------------------

// Where each cut meets the plate's edge, as a fraction of the half-extent. Off centre and unequal, so the four cells are four different shapes; at zero this degenerates to a square quartering, which is what it exists to avoid.
#define CUT_SLANT_U   0.46f
#define CUT_SLANT_V   0.32f

// The four corners of cell (iu, iv) of a plate spanning -hu..hu by -hv..hv, counter-clockwise in the (u, v) plane, which is the winding Hex wants for its first face.
//
// The cut across u meets v = -hv at u = -su and v = +hv at u = +su; the cut across v meets u = -hu at v = -sv and u = +hu at v = +sv. Both pass through the origin, so they cross there and the four cells close.
static void PlateCell(float q[4][2], int iu, int iv, float hu, float hv)
{
    float su = hu * CUT_SLANT_U, sv = hv * CUT_SLANT_V;
    const float C[4][4][2] = {
        // iu = 0, iv = 0
        { { -1, -1 }, { -2, -1 }, { 0, 0 }, { -1, -2 } },
        // iu = 1, iv = 0
        { { -2, -1 }, { 1, -1 }, { 1, 2 }, { 0, 0 } },
        // iu = 0, iv = 1
        { { -1, -2 }, { 0, 0 }, { 2, 1 }, { -1, 1 } },
        // iu = 1, iv = 1
        { { 0, 0 }, { 1, 2 }, { 1, 1 }, { 2, 1 } },
    };
    // A code of +-1 is the plate's own edge, +-2 is where a cut meets it, 0 is the crossing.
    const float (*c)[2] = C[iv * 2 + iu];
    for (int k = 0; k < 4; k++) {
        q[k][0] = (c[k][0] == 2.0f) ? su : (c[k][0] == -2.0f) ? -su : c[k][0] * hu;
        q[k][1] = (c[k][1] == 2.0f) ? sv : (c[k][1] == -2.0f) ? -sv : c[k][1] * hv;
    }
    // Grown outward from its own centroid so it knits into its neighbours instead of meeting them on an exactly coincident plane, which is what KNIT does for every other piece here.
    float cu = 0.0f, cv = 0.0f;
    for (int k = 0; k < 4; k++) { cu += q[k][0] * 0.25f; cv += q[k][1] * 0.25f; }
    for (int k = 0; k < 4; k++) {
        float du = q[k][0] - cu, dv = q[k][1] - cv;
        float d = sqrtf(du * du + dv * dv);
        if (d > 1e-5f) { q[k][0] += du / d * KNIT; q[k][1] += dv / d * KNIT; }
    }
}

// A cell of a horizontal plate: (u, v) are (x, z) and the thickness runs in y.
static void PlateCellY(Builder *b, int iu, int iv, float hu, float hv, float y0, float y1)
{
    float q[4][2];
    PlateCell(q, iu, iv, hu, hv);
    Vector3 c[8];
    for (int k = 0; k < 4; k++) {
        c[k]     = (Vector3){ q[k][0], y0, q[k][1] };
        c[k + 4] = (Vector3){ q[k][0], y1, q[k][1] };
    }
    Hex(b, c);
}

// A cell of a vertical plate: (u, v) are (z, y) and the thickness runs in x. Hex wants its first face wound counter-clockwise seen from outside, and mapping v to y reverses that sense, so the corners are taken in the opposite order.
static void PlateCellX(Builder *b, int iu, int iv, float hu, float hv, float x0, float x1)
{
    float q[4][2];
    PlateCell(q, iu, iv, hu, hv);
    Vector3 c[8];
    for (int k = 0; k < 4; k++) {
        int j = 3 - k;
        c[k]     = (Vector3){ x0, q[j][1], q[j][0] };
        c[k + 4] = (Vector3){ x1, q[j][1], q[j][0] };
    }
    Hex(b, c);
}

// How far a plate reaches from the spine centre-line to SLAB_BEAR inside the facade panel, which is the 5.76 m the series casts its slabs to less the 0.05 the bearing eats.
#define SPAN_HALF     ((INNER_Z + SLAB_BEAR) * 0.5f)

// One of the four cells a floor slab breaks into, authored about the slab's own centre at the underside of the structural slab.
static void BuildSlab(Group *g, float w, MatId top, int iu, int iv)
{
    PlateCellY(&g->b[MAT_CONCRETE], iu, iv, w * 0.5f, SPAN_HALF, 0.0f, SLAB_STRUCT);
    PlateCellY(&g->b[top], iu, iv, w * 0.5f, SPAN_HALF, SLAB_STRUCT, SLAB_T);
}

// Every plate of the building, near side of the spine and far. A slab is one plate per bay per side, cut into four; the far side is the near side mirrored in z, which is what the yaw does.
#define FOR_PLATE(side, iu, iv) \
    for (int side = 0; side < 2; side++) for (int iu = 0; iu < 2; iu++) for (int iv = 0; iv < 2; iv++)

// The type of cell (iu, iv) of the plate whose first cell is `base`. The four cells of a plate are declared adjacent in the enum so that this is arithmetic rather than a table.
static FragType PlateType(FragType base, int iu, int iv) { return (FragType)((int)base + iv * 2 + iu); }

// The clear height of a storey, which is what a cross wall stands to.
#define WALL_H        (STOREY - SLAB_T)

static void BuildStructureTypes(void)
{
    for (int iu = 0; iu < 2; iu++) {
        for (int iv = 0; iv < 2; iv++) {
            BuildSlab(&gType[PlateType(FR_SLAB26, iu, iv)], 2.6f, MAT_CONCRETE, iu, iv);
            BuildSlab(&gType[PlateType(FR_SLAB32, iu, iv)], 3.2f, MAT_CONCRETE, iu, iv);
            BuildSlab(&gType[PlateType(FR_ROOF26, iu, iv)], 2.6f, MAT_ROOF, iu, iv);
            BuildSlab(&gType[PlateType(FR_ROOF32, iu, iv)], 3.2f, MAT_ROOF, iu, iv);
            // The cross wall: the load-bearing direction in this series, and the wall the charges are drilled into. A vertical plate, so its two in-plane axes are the building's depth and a storey's clear height, and it breaks on the same two slanted cuts a slab does.
            PlateCellX(&gType[PlateType(FR_XWALL, iu, iv)].b[MAT_CONCRETE], iu, iv,
                       SPAN_HALF, WALL_H * 0.5f, -XWALL_T * 0.5f, XWALL_T * 0.5f);
        }
    }

    // The spine: one segment per bay, so it breaks with the bay rather than across it.
    Box(&gType[FR_SPINE26].b[MAT_CONCRETE], -1.3f - KNIT, 1.3f + KNIT, -KNIT, STOREY - SLAB_T + KNIT, -SPINE_T * 0.5f, SPINE_T * 0.5f);
    Box(&gType[FR_SPINE32].b[MAT_CONCRETE], -1.6f - KNIT, 1.6f + KNIT, -KNIT, STOREY - SLAB_T + KNIT, -SPINE_T * 0.5f, SPINE_T * 0.5f);

    // Parapet segments, authored about the centre of the bay they cap, at roof level.
    Box(&gType[FR_PAR26].b[MAT_CONCRETE], -1.3f - KNIT, 1.3f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);
    Box(&gType[FR_PAR26].b[MAT_METAL], -1.3f - CORNICE_O, 1.3f + CORNICE_O, PARAPET_H, PARAPET_H + 0.036f, -PARAPET_T - 0.030f, CORNICE_O);
    Box(&gType[FR_PAR32].b[MAT_CONCRETE], -1.6f - KNIT, 1.6f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);
    Box(&gType[FR_PAR32].b[MAT_METAL], -1.6f - CORNICE_O, 1.6f + CORNICE_O, PARAPET_H, PARAPET_H + 0.036f, -PARAPET_T - 0.030f, CORNICE_O);
    Box(&gType[FR_PAREND].b[MAT_CONCRETE], -FACE_Z * 0.5f - KNIT, FACE_Z * 0.5f + KNIT, -KNIT, PARAPET_H, -PARAPET_T, 0.0f);
    Box(&gType[FR_PAREND].b[MAT_METAL], -FACE_Z * 0.5f - CORNICE_O, FACE_Z * 0.5f + CORNICE_O, PARAPET_H, PARAPET_H + 0.036f, -PARAPET_T - 0.030f, CORNICE_O);

    // A balcony, in the three pieces a balcony on one of these buildings is actually made of. Authored about the facade's outer face at floor level, so it hangs off the panel rather than off a copy of the panel's coordinates.
    //
    // Three rather than one, because the colour is the whole point and a per-instance tint reaches every material its group owns. ref_03, ref_05, ref_06 and ref_07 between them show ochre, blue, sage, rust, white and grey-blue fronts on one building, and half of them enclosed in something the resident bought: the slab is the builder's and stays grey, the front and the enclosure are the flat's and take its colours. Splitting them also means the enclosure comes off before the slab does, which is what happens.
    //
    // The front is a solid sheet, not a railing. An earlier build had 13 uprights and a top rail, and Codex read the whole facade as an access gallery.
    float hw = BALC_W * 0.5f;
    float rt = BALC_RAIL_H;
    {
        // The slab, and the steel that carries the front. The slab's root runs 0.20 back into the facade, which is where it is cast in.
        Group *g = &gType[FR_BALC];
        Box(&g->b[MAT_CONCRETE], -hw, hw, -BALC_T, 0.0f, -0.200f, BALC_D);
        Builder *r = &g->b[MAT_RUST];
        for (int i = 0; i < 5; i++) {
            float x = -hw + BALC_W * (float)i / 4.0f;
            Box(r, x - 0.026f, x + 0.026f, 0.0f, rt, BALC_D - 0.045f, BALC_D - 0.005f);
        }
        Box(r, -hw, -hw + 0.052f, 0.0f, rt, 0.0f, BALC_D);
        Box(r, hw - 0.052f, hw, 0.0f, rt, 0.0f, BALC_D);
    }
    {
        // The painted sheet the resident fixed to that steel, and the capping rail over it.
        Group *g = &gType[FR_BALCSHEET];
        // Standing on the slab rather than flush with its edge, and inset from it, so the concrete the whole balcony hangs off stays visible under and beside the painted sheet. A review round read the enclosed ones as coloured rectangles applied to the wall, and this is why: the sheet reached the slab's own outer face and there was nothing left of the cantilever to see.
        Builder *p = &g->b[MAT_PAINT];
        Box(p, -hw + 0.060f, hw - 0.060f, 0.030f, rt - 0.045f, BALC_D - 0.215f, BALC_D - 0.180f);
        Box(p, -hw + 0.060f, -hw + 0.095f, 0.030f, rt - 0.045f, 0.060f, BALC_D - 0.180f);
        Box(p, hw - 0.095f, hw - 0.060f, 0.030f, rt - 0.045f, 0.060f, BALC_D - 0.180f);
        Builder *m = &g->b[MAT_METAL];
        Box(m, -hw + 0.040f, hw - 0.040f, rt - 0.045f, rt, BALC_D - 0.240f, BALC_D - 0.155f);
        Box(m, -hw + 0.040f, -hw + 0.115f, rt - 0.045f, rt, 0.040f, BALC_D - 0.155f);
        Box(m, hw - 0.115f, hw - 0.040f, rt - 0.045f, rt, 0.040f, BALC_D - 0.155f);
    }
    {
        // The enclosure, glazed in up to the underside of the balcony above. Its frame carries the flat's colour, white on a plastic one and a brown or an ochre on the timber ones ref_05 and ref_07 are full of. The glass has to be a fourth group rather than a fourth material, for the same reason the window pane is its own group and for the same reason the car's glass is not part of its body: a per-instance tint reaches every material in the group it is applied to, so glass left in here would come out brown behind a brown frame.
        Group *g = &gType[FR_BALCGLZ];
        Group *q = &gType[FR_BALCPANE];
        float top = STOREY - BALC_T - 0.030f;
        Builder *f = &g->b[MAT_PAINT];
        // Set back 0.16 m from the slab's nose, so the concrete lip stays visible under and beside it at every floor. At 0.055 the enclosures of five storeys merged into one tall box and a review round twice read them as coloured rectangles applied to the wall rather than as things standing on a cantilever.
        float fz1 = BALC_D - 0.160f, fz0 = fz1 - 0.045f;
        Box(f, -hw + 0.015f, hw - 0.015f, rt, rt + 0.045f, fz0, fz1);
        Box(f, -hw + 0.015f, hw - 0.015f, top - 0.045f, top, fz0, fz1);
        Box(f, -hw + 0.040f, -hw + 0.085f, rt, top, 0.050f, fz1);
        Box(f, hw - 0.085f, hw - 0.040f, rt, top, 0.050f, fz1);
        Box(f, -hw + 0.040f, hw - 0.040f, top - 0.045f, top, 0.050f, fz1);
        // Six lights across three metres, which is what a resident's glazier fits, and thin enough that the glass is what shows rather than the frame.
        for (int i = 1; i < 6; i++) {
            float x = -hw + BALC_W * (float)i / 6.0f;
            Box(f, x - 0.022f, x + 0.022f, rt, top, fz0, fz1);
        }
        Box(&q->b[MAT_GLASS], -hw + 0.030f, hw - 0.030f, rt + 0.040f, top - 0.045f, fz0 + 0.012f, fz0 + 0.030f);
        Box(&q->b[MAT_GLASS], -hw + 0.070f, -hw + 0.088f, rt + 0.040f, top - 0.045f, 0.070f, fz0);
        Box(&q->b[MAT_GLASS], hw - 0.088f, hw - 0.070f, rt + 0.040f, top - 0.045f, 0.070f, fz0);
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
        // Rusted through rather than painted, which is what ref_03 and ref_05 both show and the loudest colour on either facade.
        Builder *m = &gType[FR_PIPE].b[MAT_RUST];
        Tube(m, (Vector3){ 0, -KNIT, 0 }, (Vector3){ 0, STOREY + KNIT, 0 }, 0.064f, 0.064f, 9, false, false);
        // The bracket, which is what says the pipe is bolted on rather than cast into the wall, and which has to reach back to the wall from wherever the pipe now stands.
        Box(m, -0.090f, 0.090f, 0.170f, 0.270f, -0.155f, 0.020f);
        Box(m, -0.090f, 0.090f, 1.520f, 1.620f, -0.155f, 0.020f);
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

// ---------------------------------------------------------------------------
// The flat
//
// The unit of variety on a khrushchyovka facade is not the panel and not the building: it is the flat, because everything that varies was decided by whoever lived in one. Windows were replaced a flat at a time, balconies were enclosed a flat at a time in whatever the resident could get, and the curtain in every window of one flat came out of the same shop on the same afternoon. A facade scattered panel by panel reads as noise; the same amount of variation grouped by flat reads as sixty households, which is what ref_03, ref_05, ref_06 and ref_07 all show and what the first build -- one window, one balcony, one grey, 210 times -- showed none of.
//
// A 1-464 section has four flats to a landing. Here that is the three bays either side of the stairwell, on each of the two facades, which puts one of the two 3.2 m balcony bays in every flat and gives 3 x 5 x 4 = 60 of them.
// ---------------------------------------------------------------------------

typedef enum { BAL_OPEN, BAL_TIMBER, BAL_PVC } BalconyKind;

typedef struct {
    bool pvc;            // the windows have been replaced with plastic
    Color frame;         // and if not, what the timber was last painted
    Color curtain;       // what shows behind the glass
    BalconyKind balcony;
    Color sheet;         // the painted front, which is the loudest colour on the facade
    Color enclosure;     // the frames of the glazing over it, where there is any
    bool ac;
    bool bars;           // only ever asked of the ground floor
} Flat;

// Colours are authored well below where they should read, because lighting.fs gamma-corrects with pow(c, 1/2.2): a component of 128 leaves the shader at 0.60 before any light reaches it, so an ochre picked at the value it should appear comes out as cream.
static const Color TIMBER_PAINT[] = {
    { 214, 210, 200, 255 },   // the white nearly all of them started as
    { 214, 210, 200, 255 },
    { 118, 148, 162, 255 },   // the pale blue-green ref_03 has a column of
    { 132, 148, 122, 255 },
    { 104, 76, 52, 255 },     // and the ones nobody has painted since
};

static const Color CURTAIN[] = {
    { 150, 142, 124, 255 },   // net, which is most of them
    { 150, 142, 124, 255 },
    { 96, 100, 104, 255 },
    { 46, 46, 50, 255 },      // a dark room, which is what an unlit one is
    { 46, 46, 50, 255 },
    { 132, 108, 78, 255 },
    { 104, 84, 88, 255 },
    { 108, 124, 132, 255 },
};

// ref_05 and ref_07 between them carry every one of these on a single building. Muted rather than picked at full strength: these are forty years of weather on a coat of oil paint, and the gamma correction lifts them again on the way out.
static const Color BALCONY_SHEET[] = {
    { 148, 120, 62, 255 },    // the ochre that is on more of them than anything else
    { 148, 120, 62, 255 },
    { 96, 116, 134, 255 },
    { 106, 118, 92, 255 },
    { 118, 76, 52, 255 },
    { 164, 160, 150, 255 },
    { 100, 110, 118, 255 },
};

static const Color ENCLOSURE_FRAME[] = {
    { 194, 192, 186, 255 },   // white plastic
    { 126, 94, 56, 255 },     // and the varnished timber ref_05 is full of
    { 148, 120, 62, 255 },
};

#define COUNT_OF(a) ((int)(sizeof(a) / sizeof((a)[0])))

// Which flat a bay belongs to: 0 for the three bays left of the stairwell, 1 for the three right of it. The stairwell itself belongs to nobody, which is the whole reason it is the one part of the facade with no variation on it.
static int FlatHalf(int bay) { return (bay < BAY_STAIR) ? 0 : 1; }

static Flat FlatAt(int s, int floor, int side, int half)
{
    int id = ((s * FLOORS + floor) * 2 + side) * 2 + half;
    Flat fl = { 0 };
    fl.pvc = Hash2(id, 1, 4096, 617u) < 0.55f;
    fl.frame = fl.pvc ? (Color){ 226, 224, 220, 255 }
                      : TIMBER_PAINT[(int)(Hash2(id, 2, 4096, 619u) * COUNT_OF(TIMBER_PAINT)) % COUNT_OF(TIMBER_PAINT)];
    fl.curtain = CURTAIN[(int)(Hash2(id, 3, 4096, 631u) * COUNT_OF(CURTAIN)) % COUNT_OF(CURTAIN)];

    float b = Hash2(id, 4, 4096, 641u);
    fl.balcony = (b < 0.42f) ? BAL_OPEN : (b < 0.72f) ? BAL_TIMBER : BAL_PVC;
    fl.sheet = BALCONY_SHEET[(int)(Hash2(id, 5, 4096, 643u) * COUNT_OF(BALCONY_SHEET)) % COUNT_OF(BALCONY_SHEET)];
    fl.enclosure = (fl.balcony == BAL_PVC)
        ? ENCLOSURE_FRAME[0]
        : ENCLOSURE_FRAME[1 + ((int)(Hash2(id, 6, 4096, 647u) * 2.0f) & 1)];

    fl.ac = Hash2(id, 7, 4096, 653u) < 0.16f;
    fl.bars = Hash2(id, 8, 4096, 659u) < 0.45f;
    return fl;
}

// ---------------------------------------------------------------------------
// Placing the fragments
// ---------------------------------------------------------------------------

// Which panel kind a bay carries on a given facade and floor.
// front is +Z. The stairwell touches the front, so that is where its windows go and where the entrance cannot be; the entrance is on the back, which is the courtyard side.
static PanelKind PanelAt(int bay, int floor, bool front)
{
    if (bay == BAY_STAIR) {
        if (!front && floor == 0) return PK_DOOR;
        // A landing sits half a flight above the floor its panel stands on, so the lights zigzag rather than stacking on the apartments' rhythm.
        return (floor & 1) ? PK_STAIR2 : PK_STAIR;
    }
    // The 3.2 m bays carry balconies, but not on the ground floor, where they carry a plain wide window instead.
    // An earlier build ran them to the ground on the strength of ref_07, and CheckTypesBuilt then reported the plain 3.2 m panel built and never placed anywhere in the block, which is what sent this back to the photographs: ref_03 and ref_05 both show the balcony stacks starting one storey up over a ground floor of plain wide windows, and ref_07's are the reading that was wrong.
    if ((bay == BAY_BALC_A || bay == BAY_BALC_B) && floor > 0) return PK_32B;
    return (BAY_W[bay] > 2.9f) ? PK_32 : PK_26;
}

// The joinery and the pane a kind carries, given whether the flat behind it has replaced its windows. The stairwell is communal, so nobody has.
static void GlazingFor(PanelKind k, bool pvc, FragType *joinery, FragType *pane)
{
    switch (k) {
        case PK_26:     *joinery = pvc ? FR_J26P : FR_J26T;   *pane = FR_Q26; return;
        case PK_32:     *joinery = pvc ? FR_J32P : FR_J32T;   *pane = FR_Q32; return;
        case PK_32B:    *joinery = pvc ? FR_J32BP : FR_J32BT; *pane = FR_Q32B; return;
        case PK_STAIR:  *joinery = FR_JSTAIR;  *pane = FR_QSTAIR; return;
        case PK_STAIR2: *joinery = FR_JSTAIR2; *pane = FR_QSTAIR2; return;
        case PK_DOOR:   *joinery = FR_JDOOR;   *pane = FR_QDOOR; return;
        default:        *joinery = FR_COUNT;   *pane = FR_COUNT; return;
    }
}

// Whether the panel at a given place comes down in one piece or two, and which two. Hashed off the placement so it is stable, and set at three in five, because a demolition leaves some panels whole and a facade every one of which cracks identically reads as a pattern rather than as damage.
static bool PanelFractures(PanelKind k, int s, int f, int bay, int side)
{
    const PanelTypes *pt = &PANEL_TYPE[k];
    if (pt->left == pt->whole) return false;
    // Kind-dependent, so the twelve placements of a rare kind draw from a different sequence than the two hundred of a common one. Left sharing one sequence at 0.85, every one of the twelve plain 3.2 m panels happened to fracture and CheckTypesBuilt reported the whole-panel mesh built and never placed -- harmless in itself, and exactly the alarm that check exists to raise, so the cure is to stop it being a coin flip rather than to lower the threshold until it stops landing badly.
    return Hash2(((s * FLOORS + f) * BAYS + bay) * 2 + side, 9 + (int)k * 31, 4096, 661u) < 0.82f;
}

// One panel of the facade, with everything that hangs on it: the panel itself (whole, or the two pieces it will break into), the joinery and its pane, and whatever the flat behind it has fixed to the wall.
static void PlacePanel(PanelKind k, int s, int f, int bay, int side, float x, float y, float z, float yaw)
{
    const PanelTypes *pt = &PANEL_TYPE[k];
    bool split = PanelFractures(k, s, f, bay, side);
    Emit(split ? pt->left : pt->whole, x, y, z, yaw);
    if (split) Emit(pt->right, x, y, z, yaw);

    if (k == PK_END) return;

    Flat fl = FlatAt(s, f, side, FlatHalf(bay));
    bool communal = (bay == BAY_STAIR);
    FragType joinery, pane;
    GlazingFor(k, !communal && fl.pvc, &joinery, &pane);

    EmitAt(joinery, x, y, z, yaw, 1.0f, communal ? TIMBER_PAINT[0] : fl.frame);
    // The pane's tint is what is behind the glass, not the glass: a dark room, a net curtain, a kitchen with the light on. It is the cheapest thing on this facade and the one that most makes it read as lived in.
    EmitAt(pane, x, y, z, yaw, 1.0f, communal ? CURTAIN[3] : fl.curtain);

    if (communal) return;

    // Bars, on the ground floor only, which is the only floor anyone can reach, and on the plain bays only, because that is the one window the single grid mesh is centred on and sized to.
    if (f == 0 && fl.bars && k == PK_26) Emit(FR_BARS, x, y, z, yaw);

    // An air conditioner, beside the window rather than over it, on the plain bays for the same reason.
    // Placed in the panel's own frame and rotated out of it, not by testing which yaw this facade happens to have: MatrixRotateY sends local +X to world (cos yaw, 0, -sin yaw), so the offset has to go through the same rotation the panel does.
    if (fl.ac && k == PK_26 && f > 0) {
        float ox = ((bay < BAY_STAIR) ? -1.0f : 1.0f) * (WIN_W26 * 0.5f + 0.520f);
        float c = cosf(yaw * DEG2RAD), sn = sinf(yaw * DEG2RAD);
        EmitAt(FR_AC, x + ox * c, y + WIN_SILL + 0.520f, z - ox * sn, yaw, 1.0f, WHITE);
    }
}

static void PlaceBuilding(void)
{
    for (int s = 0; s < SECTIONS; s++) {
        for (int f = 0; f < FLOORS; f++) {
            float y = FloorY(f);

            for (int b = 0; b < BAYS; b++) {
                float x = BayX(s, b);

                for (int side = 0; side < 2; side++) {
                    float pz = (side == 0) ? INNER_Z : -INNER_Z;
                    PlacePanel(PanelAt(b, f, side == 0), s, f, b, side, x, y, pz, (side == 0) ? 0.0f : 180.0f);
                }

                // Floor slabs: two pieces each side of the spine, so what lands in the pile is a plate the length of a room.
                FragType slab = (BAY_W[b] > 2.9f) ? FR_SLAB32 : FR_SLAB26;
                FOR_PLATE(side, iu, iv) Emit(PlateType(slab, iu, iv), x, y - SLAB_T,
                                            (side == 0) ? -SPAN_HALF : SPAN_HALF, (side == 0) ? 0.0f : 180.0f);

                Emit((BAY_W[b] > 2.9f) ? FR_SPINE32 : FR_SPINE26, x, y, 0.0f, 0.0f);

                // Balconies, on the two wide bays of every section, on the four floors above the ground one, both facades. Which floors is settled in PanelAt, and this asks the same question the same way rather than repeating the answer.
                // The slab is the builder's and is the same grey everywhere; the front and whatever is glazed over it belong to the flat behind them.
                if (PanelAt(b, f, true) == PK_32B) {
                    for (int side = 0; side < 2; side++) {
                        float bz = (side == 0) ? FACE_Z : -FACE_Z;
                        float yaw = (side == 0) ? 0.0f : 180.0f;
                        Flat fl = FlatAt(s, f, side, FlatHalf(b));
                        Emit(FR_BALC, x, y, bz, yaw);
                        EmitAt(FR_BALCSHEET, x, y, bz, yaw, 1.0f, fl.sheet);
                        if (fl.balcony != BAL_OPEN) {
                            EmitAt(FR_BALCGLZ, x, y, bz, yaw, 1.0f, fl.enclosure);
                            Emit(FR_BALCPANE, x, y, bz, yaw);
                        }
                    }
                }
            }

            // Cross walls on every bay boundary of the section, plus the one that closes it, in four pieces across the depth for the same reason the slabs are in two.
            for (int b = 0; b <= BAYS; b++) {
                float x = SectionX(s) - SECTION_LEN * 0.5f;
                for (int i = 0; i < b; i++) x += BAY_W[i];
                // The boundary between two sections is one wall, not two.
                if (b == BAYS && s != SECTIONS - 1) continue;

                // At the two ends of the block that boundary is the gable itself, and a wall centred on it stands XWALL_T/2 - 0 = 0.07 m *outside* the gable's own outer face, hiding all four of its panels behind a blank strip. That is what the block's ends had rendered as from the first build: a smooth grey slab banded once per storey, which is this wall's own gap at each floor slab and not a panel joint at all. Two review rounds read the gable as under-divided and neither could see why, because the gable was never what was being looked at.
                // Pulled inboard until its outer face is buried 10 mm inside the panel, which is where a cross wall meeting an end wall is, and derived from the panel it meets rather than typed.
                float end = BLOCK_LEN * 0.5f;
                if (x < -end + 1e-3f) x += WALL_T + XWALL_T * 0.5f - 0.010f;
                else if (x > end - 1e-3f) x -= WALL_T + XWALL_T * 0.5f - 0.010f;

                FOR_PLATE(side, iu, iv) Emit(PlateType(FR_XWALL, iu, iv), x, y + WALL_H * 0.5f,
                                             (side == 0) ? -SPAN_HALF : SPAN_HALF, (side == 0) ? 0.0f : 180.0f);
            }

            // End walls, four panels across the depth, on the two ends of the block only.
            for (int e = 0; e < 2; e++) {
                if (!((s == 0 && e == 0) || (s == SECTIONS - 1 && e == 1))) continue;
                float sx = (e == 0) ? -BLOCK_LEN * 0.5f : BLOCK_LEN * 0.5f;
                // MatrixRotateY sends a panel's local +Z, which is the direction it faces, to world +X at yaw 90.
                // The left-hand gable has to face -X, so it takes -90 and the right-hand one +90, and each origin sits on its own inner face the way every other facade panel's does.
                float yaw = (e == 0) ? -90.0f : 90.0f;
                float ex = sx + ((e == 0) ? WALL_T : -WALL_T);
                for (int q = 0; q < 4; q++) {
                    PlacePanel(PK_END, s, f, BAYS + e, q, ex, y, END_W * (-1.5f + (float)q), yaw);
                }
            }

            // Downpipes, at each section joint and each gable corner, on both facades. A pipe
            // stands just clear of the panel face so it reads as bolted on rather than cast in.
            for (int e = 0; e <= SECTIONS; e++) {
                float px = -BLOCK_LEN * 0.5f + SECTION_LEN * (float)e;
                if (e == 0) px += 0.360f;
                if (e == SECTIONS) px -= 0.360f;
                Emit(FR_PIPE, px, y, FACE_Z + 0.125f, 0.0f);
                Emit(FR_PIPE, px, y, -FACE_Z - 0.125f, 180.0f);
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
            FOR_PLATE(side, iu, iv) Emit(PlateType(roof, iu, iv), x, ROOF_Y - SLAB_T,
                                        (side == 0) ? -SPAN_HALF : SPAN_HALF, (side == 0) ? 0.0f : 180.0f);
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
        // Three risers and a shallow landing, not the five-tread flight this had. ref_05's entrance door stands within a step or two of grade with a thin shelf of a canopy over it, and ref_03 and ref_07 are the same; at five treads of 0.32 the flight projected 1.6 m and a review round read the entrances as later elevated annexes stuck onto the courtyard facade.
        // The ground floor really is PLINTH_Y above grade, which is why there are steps at all: what a khrushchyovka entrance does with the rest of that height is take it inside, on the half flight up to the first landing.
        const int RISERS = 3;
        float rise = PLINTH_Y / (float)RISERS, tread = 0.300f;
        for (int i = 0; i < RISERS; i++) {
            float z0 = -hz - t - tread * (float)(RISERS - i);
            Box(&g->b[MAT_CONCRETE], x - 1.100f, x + 1.100f, 0.0f, rise * (float)(i + 1),
                z0, z0 + tread + 0.001f);
        }
        // Landing in front of the door.
        Box(&g->b[MAT_CONCRETE], x - 1.100f, x + 1.100f, 0.0f, PLINTH_Y, -hz - t - 0.360f, -hz + 0.200f);
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
#define BULK_FACTOR  1.55f    // how much a cubic metre of concrete swells once it is rubble
#define PILE_SPREAD  9.0f     // how far past the footprint the debris skirt is expected to run
#define BLAST_V      105.0f   // speed of the air blast that reaches the yard, m/s

// Everything the demolition takes down, as against the yard, which it only shakes. Also everything that ends up in the pile, because since the invented rubble went those are the same set: every piece of the pile is a piece the building was built from.
static bool IsBuilding(FragType t) { return FR_CLASS[t] != FC_YARD; }

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

// What a piece landing on a footprint comes to rest on.
//
// Not simply the highest cell under it. A rigid plate does rest on the highest point it touches, and taking the maximum is what the first version of this did; with the slabs and cross walls broken into thirds it then ran the crest to 8.5 m on a 14.3 m building, where the same material spread as a smooth mound predicts 4.6 and a real demolition pile is a quarter to a third of the building's height. Each small piece perched on the last one's high corner, and the error compounded over 3500 of them.
//
// Weighted towards the mean instead, which says that a pile of broken concrete crushes and settles under what lands on it rather than holding every point of its own relief. The weight is the one number here fitted to an outcome rather than derived, and CheckCollapse prints the packed crest against the smooth-mound prediction so that the fit stays visible.
#define PILE_PERCH  0.20f

static float PileUnder(float x, float z, float hx, float hz)
{
    int x0, z0, x1, z1;
    PileCell(x - hx, z - hz, &x0, &z0);
    PileCell(x + hx, z + hz, &x1, &z1);
    float top = 0.0f, sum = 0.0f;
    int n = 0;
    for (int i = x0; i <= x1; i++) {
        for (int j = z0; j <= z1; j++) { top = fmaxf(top, gHeight[i][j]); sum += gHeight[i][j]; n++; }
    }
    if (n == 0) return 0.0f;
    return Lerp(sum / (float)n, top, PILE_PERCH);
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

    FragClass cls = FR_CLASS[f->type];
    bool glazed = (cls == FC_GLASS);
    bool balcony = (cls == FC_BALCONY);

    if (wc.y < CUT_TOP) {
        // The charges are in here. This is the only material that is thrown rather than dropped.
        m->t0 = T_BLAST + seq + 0.05f * Rand01(i, 3);
        out = 3.0f + 3.2f * Rand01(i, 4);
        up = 1.1f + 1.9f * Rand01(i, 5);
    } else if (glazed) {
        // Every window in the building goes at the shock, not when its own storey is reached. It goes *out* of its frame and falls, though, at a fraction of the speed it once had: a review round read the first second of the sequence as the skin being explosively removed from a standing building, and 210 panes leaving at 8 m/s while nothing above them had moved yet was most of that picture.
        m->t0 = T_BLAST + seq + 0.05f * Rand01(i, 6);
        out = 1.1f + 1.7f * Rand01(i, 7);
        up = 0.15f + 0.55f * Rand01(i, 8);
    } else if (balcony) {
        // A cantilever, so it fails as its own storey is reached rather than with the wall behind it -- but after, not before. Going 0.22 s early put a ring of balconies in the air around a block that was still standing at full height.
        m->t0 += 0.12f;
        out = 0.8f + 0.07f * height + 0.6f * Rand01(i, 9);
    } else if (cls == FC_SKIN || cls == FC_FITTING) {
        // A review round read the first pass as the facade being explosively blown off while the
        // roof stayed level over it. What actually happens is that the lower storeys lose support
        // and the mass above starts down before anything peels, so the skin now waits a fifth of
        // a second longer than the structure at its own height and leaves with a third of the
        // outward speed it had. The peel then comes from the tumble and the fall rather than a kick.
        // Nearly half a second behind the structure at its own height, where the first attempt at this used a fifth of one. What actually happens is that the lower storeys lose support and the mass above starts down before anything peels, so the peel has to come out of the tumble and the fall rather than out of a kick.
        m->t0 += 0.42f;
        out = 0.18f + 0.030f * height + 0.35f * Rand01(i, 10);
    } else {
        // Slabs, cross walls and the spine drop; they are what the pile is made of.
        // Each with its own small delay, so a floor does not come down as one plane. Sharing a release time to the millisecond is what kept the roof legible as a complete deck through the middle of the sequence, sitting in its original plane while the facade under it had already gone.
        m->t0 += 0.19f * Rand01(i, 22);
        out = 0.25f * Rand01(i, 11);
        up = -0.3f * Rand01(i, 12);
    }

    m->v0 = (Vector3){ along, up, sz * out };
    m->axis = Vector3Normalize((Vector3){ Rand11(i, 13) + 0.05f, Rand11(i, 14) * 0.4f, Rand11(i, 15) });
    m->omega = (glazed ? 3.4f : balcony ? 1.9f : cls == FC_STRUCT ? 1.25f : 0.7f) * (0.5f + Rand01(i, 16)) * (Rand01(i, 17) < 0.5f ? -1.0f : 1.0f);

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

#define DUST_MAX     2600
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
    const int SURGE = 360, COLUMN = 120;

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
                r0, r0 * (2.1f + 1.1f * Hash2(i, 11, 4096, 113u)),
                0.095f + 0.065f * Hash2(i, 12, 4096, 127u), 0.25f + 0.55f * Hash2(i, 13, 4096, 131u), i);
    }

    for (int i = 0; i < COLUMN; i++) {
        float u = Hash2(i, 21, 4096, 131u);
        float x = (u - 0.5f) * BLOCK_LEN * 0.94f;
        float z = (Hash2(i, 22, 4096, 137u) - 0.5f) * 2.0f * FACE_Z;
        float y = 0.8f + 5.0f * Hash2(i, 23, 4096, 139u);

        float seq = SEQ_LEN * (x + BLOCK_LEN * 0.5f) / BLOCK_LEN;
        // Later than it was, because the column is what rises off a pile that already exists, and it was arriving in time to hang evenly over a block that was still coming down.
        float t0 = T_BLAST + seq + 1.15f + 2.4f * Hash2(i, 24, 4096, 149u);

        Vector3 v = { (Hash2(i, 25, 4096, 151u) - 0.5f) * 4.0f,
                      3.4f + 5.2f * Hash2(i, 26, 4096, 157u),
                      (Hash2(i, 27, 4096, 163u) - 0.5f) * 5.0f };
        float r0 = 2.8f + 3.2f * Hash2(i, 28, 4096, 167u);
        // A review round found the cloud reading as one level blanket over the whole courtyard,
        // hiding the pile from the only camera the loop has. The column is fewer, smaller and
        // fainter for it, and each puff now gets its own buoyancy so their tops do not line up.
        AddPuff((Vector3){ x, y, z }, v, t0, 2.2f + 1.5f * Hash2(i, 29, 4096, 173u),
                r0, r0 * (2.0f + 1.2f * Hash2(i, 30, 4096, 179u)),
                0.060f + 0.055f * Hash2(i, 31, 4096, 181u), 0.7f + 1.7f * Hash2(i, 32, 4096, 191u), i + SURGE);
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
        if (IsBuilding(gFrag[i].type)) gLandOrder[n++] = i;
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

// The fourth population, and the one the first build had no answer for: the dust a piece raises where it actually hits.
//
// The other three are placed before anything is thrown, off the footprint and the firing sequence, and so they are a plausible cloud rather than this collapse's cloud. They bloom whether or not anything has landed yet, they bloom evenly along a block whose two ends land three tenths of a second apart, and they are silent about the debris skirt, which is where the largest pieces end up and where a demolition throws its most conspicuous dust.
//
// This one is placed after SolveLanding, so a puff exists because a slab hit the pile there at that moment. That makes the cloud a consequence of the collapse rather than a decoration over it, and it is the only population that reaches out over the skirt, because it goes where the debris went.
//
// One puff per landing would be 1900, so it takes a hashed share of them and scales what it takes by how big the piece was. Everything smaller than a wheelbarrow raises nothing worth drawing.
static void AddImpactDust(void)
{
    int taken = 0;
    for (int i = 0; i < gFragCount; i++) {
        if (!IsBuilding(gFrag[i].type)) continue;

        const Motion *m = &gMotion[i];
        // A piece that never left the ground is the bottom of the building becoming the bottom of the pile. It did not fall, so it raises nothing.
        if (m->t1 - m->t0 < 0.12f) continue;

        float sc = gFrag[i].scale;
        float vol = gType[gFrag[i].type].volume * sc * sc * sc;
        if (vol < 0.25f) continue;
        // Anything the size of a slab raises dust every time. Leaving that to a hash meant three quarters of the most conspicuous landings -- the plates that reach the debris skirt, which is where a demolition throws its most visible dust -- arrived in silence.
        if (vol < 1.10f && Hash2(i, 71, 4096, 743u) > 0.30f) continue;

        Vector3 c = Vector3Transform(m->c, FlightPose(i, m->t1));
        // How hard it arrived, which is what sets how much it throws up. Ballistic, so this is just the fall.
        float fall = fmaxf(0.0f, m->rest.y - c.y);
        float speed = sqrtf(fmaxf(0.0f, m->v0.y * m->v0.y + 2.0f * GRAV * fall));

        float r0 = 0.55f + 0.85f * cbrtf(vol) + 0.020f * speed;
        Vector3 v = {
            (Hash2(i, 72, 4096, 751u) - 0.5f) * 5.0f,
            0.9f + 0.10f * speed,
            (Hash2(i, 73, 4096, 757u) - 0.5f) * 5.0f,
        };
        // Later arrivals land on a pile that is already there and already smoking, so they are given a little less: by then the surge and the column are both up and a fourth full-strength cloud on top of them is what turns the courtyard into fog.
        float peak = 0.075f + 0.055f * Hash2(i, 74, 4096, 761u);
        AddPuff((Vector3){ c.x, fmaxf(0.20f, c.y - 0.4f), c.z }, v,
                m->t1, 1.5f + 1.4f * Hash2(i, 75, 4096, 769u),
                r0, r0 * (2.4f + 1.3f * Hash2(i, 76, 4096, 773u)),
                peak, 0.30f + 0.60f * Hash2(i, 77, 4096, 787u), i + 4000);
        taken++;
    }
    TraceLog(LOG_INFO, "panelka: %d landings raise their own dust, of %d puffs in all", taken, gPuffCount);
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

    // And then the debris arrives.
    //
    // Nothing here resolves a collision, and adding one is not what this is: the pile is already solved, cell by cell, before the first frame is drawn, so how deep the rubble ends up over any point of the yard is a number this function can simply look up. What it cannot do is leave a birch standing vertically through four metres of it, which is what a review round found in every frame after the fourth -- a tree, a lamp post and a car intact inside a volume full of slabs.
    //
    // So anything the pile reaches goes over, away from the building, and sinks into what buried it. The threshold is a third of a metre, which is a scatter of chunks rather than a pile; below that a thing in the yard has been hit by the blast and nothing else.
    float pile = PileUnder(f->pos.x, f->pos.z, 1.0f, 1.0f);
    if (pile > 0.35f) {
        // The debris front is slower than the air blast that precedes it, and it does not reach the far side of the courtyard at all.
        float a = t - (arrive + 0.30f + 0.030f * dist);
        if (a > 0.0f) {
            float over = fminf(1.0f, a / 1.10f);
            over = over * over * (3.0f - 2.0f * over);
            // Flat, or nearly. A lamp post lies down; a tree with a crown on it does not quite.
            float flat = (f->type == FR_BIRCH || f->type == FR_MAPLE) ? 1.28f : 1.48f;
            tilt = Lerp(tilt, (tilt < 0.0f ? -flat : flat), over);
            lift -= fminf(pile, 3.0f) * 0.55f * over;
        }
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
        // A few per cent either side of white. Enough that two neighbours are not bit-identical, and no more: at +-5 per cent a review read the facade as a patchwork of rectangular tiles, which is a worse artefact than the repetition it was put in to break up.
        int k = (int)(240.0f + 15.0f * f->shade);
        Color tint = TintMul((Color){ (unsigned char)k, (unsigned char)k, (unsigned char)k, 255 }, f->tint);

        if (IsBuilding(f->type)) {
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

static Group *PART_SHELL[]  = { &gType[FR_P26], &gType[FR_P26L], &gType[FR_P26R],
                                &gType[FR_P32], &gType[FR_P32L], &gType[FR_P32R],
                                &gType[FR_P32B], &gType[FR_P32BL], &gType[FR_P32BR],
                                &gType[FR_PEND], &gType[FR_PENDL], &gType[FR_PENDR],
                                &gType[FR_PSTAIR], &gType[FR_PSTAIR2], &gType[FR_PDOOR], &gType[FR_PIPE] };
static Group *PART_GLAZ[]   = { &gType[FR_J26P], &gType[FR_J26T], &gType[FR_J32P], &gType[FR_J32T],
                                &gType[FR_J32BP], &gType[FR_J32BT], &gType[FR_JSTAIR], &gType[FR_JSTAIR2],
                                &gType[FR_JDOOR], &gType[FR_Q26], &gType[FR_Q32], &gType[FR_Q32B],
                                &gType[FR_QSTAIR], &gType[FR_QSTAIR2], &gType[FR_QDOOR],
                                &gType[FR_BARS], &gType[FR_AC] };
static Group *PART_STRUCT[] = { &gType[FR_SLAB26], &gType[FR_SLAB26B], &gType[FR_SLAB26C], &gType[FR_SLAB26D],
                                &gType[FR_SLAB32], &gType[FR_SLAB32B], &gType[FR_SLAB32C], &gType[FR_SLAB32D],
                                &gType[FR_XWALL], &gType[FR_XWALLB], &gType[FR_XWALLC], &gType[FR_XWALLD],
                                &gType[FR_SPINE26], &gType[FR_SPINE32] };
static Group *PART_ROOF[]   = { &gType[FR_ROOF26], &gType[FR_ROOF26B], &gType[FR_ROOF26C], &gType[FR_ROOF26D],
                                &gType[FR_ROOF32], &gType[FR_ROOF32B], &gType[FR_ROOF32C], &gType[FR_ROOF32D],
                                &gType[FR_PAR26],
                                &gType[FR_PAR32], &gType[FR_PAREND], &gType[FR_VENT], &gType[FR_MAST] };
static Group *PART_BALC[]   = { &gType[FR_BALC], &gType[FR_BALCSHEET], &gType[FR_BALCGLZ], &gType[FR_BALCPANE] };
static Group *PART_ENTRY[]  = { &gType[FR_PDOOR], &gType[FR_JDOOR], &gType[FR_QDOOR], &gType[FR_CANOPY] };
static Group *PART_PLINTH[] = { &gPlinth };
static Group *PART_TREES[]  = { &gType[FR_BIRCH], &gType[FR_MAPLE] };
static Group *PART_CARS[]   = { &gType[FR_CARBODY], &gType[FR_CARTRIM] };
static Group *PART_YARD[]   = { &gType[FR_BENCH], &gType[FR_RUGFRAME], &gType[FR_LAMP], &gType[FR_BIN] };
static Group *PART_DUST[]   = { &gDust, &gFlash };
static Group *PART_GROUND[] = { &gGround };

static BoundingBox bShell, bGlaz, bStruct, bRoof, bBalc, bEntry, bPlinth, bGround, bTrees, bCars, bYard, bDust;

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

// Every opening must sit inside the panel it is cut into, clear of the border round it, and the openings of one panel must be listed left to right, which is what WallLayer's single sweep across a band assumes.
static void CheckOpenings(const char *name, float w, float h, const Rect *op, int nop)
{
    float worst = 1e9f;
    for (int i = 0; i < nop; i++) {
        worst = fminf(worst, op[i].x0 - (-w * 0.5f + JOINT_V));
        worst = fminf(worst, (w * 0.5f - JOINT_V) - op[i].x1);
        if (op[i].y0 > 1e-4f) worst = fminf(worst, op[i].y0 - JOINT_H);   // a doorway reaches the floor: there is no panel under it to leave a margin in
        worst = fminf(worst, (h - JOINT_H) - op[i].y1);
        if (i > 0 && op[i].x0 < op[i - 1].x1) {
            TraceLog(LOG_ERROR, "panelka: %s openings %d and %d are out of order or overlap", name, i - 1, i);
        }
    }
    if (nop > 0 && worst < 0.060f) {
        TraceLog(LOG_WARNING, "panelka: %s leaves only %.3f m of aggregate field beside an opening", name, worst);
    }
}

// The same walk, over the same table the builders read. The first build wrote every opening rectangle out a second time here, so this could only confirm that the two copies of the literals agreed; the panel it now tests is the panel that is actually built.
static void CheckPanels(void)
{
    for (int k = 0; k < PK_COUNT; k++) {
        Rect op[MAX_OPENINGS];
        int nop = PanelOpenings((PanelKind)k, op);
        CheckOpenings(PANEL_KIND_NAME[k], PANEL_W[k], STOREY, op, nop);

        // A break has to fall in solid panel, not through the middle of a window, or the two pieces meet with nothing between them and the panel is really three.
        const PanelTypes *pt = &PANEL_TYPE[k];
        if (pt->left == pt->whole) continue;
        float br = PANEL_BREAK[k];
        if (br <= -PANEL_W[k] * 0.5f + 0.100f || br >= PANEL_W[k] * 0.5f - 0.100f) {
            TraceLog(LOG_ERROR, "panelka: %s breaks at %.3f, which is off the panel", PANEL_KIND_NAME[k], br);
        }
        for (int i = 0; i < nop; i++) {
            if (br > op[i].x0 + 1e-4f && br < op[i].x1 - 1e-4f) {
                TraceLog(LOG_WARNING, "panelka: %s breaks at %.3f, inside its opening %d", PANEL_KIND_NAME[k], br, i);
            }
        }
    }
}

// A type that is placed but never built draws nothing at all, and there is no way to see the difference between that and a piece hidden behind another one. With sixty types and eleven of them differing from a neighbour only by which half of a panel they are, that is the failure this revision is most exposed to.
static void CheckTypesBuilt(void)
{
    int placed = 0, silent = 0, unused = 0;
    for (int t = 0; t < FR_COUNT; t++) {
        bool built = false;
        for (int m = 0; m < MAT_COUNT; m++) built = built || gType[t].has[m];
        if (gTypeCount[t] > 0) {
            placed++;
            if (!built) {
                TraceLog(LOG_ERROR, "panelka: type %s is placed %d times and has no mesh",
                         TYPE_NAME[t], gTypeCount[t]);
                silent++;
            }
        } else if (built) {
            TraceLog(LOG_WARNING, "panelka: type %s is built and never placed", TYPE_NAME[t]);
            unused++;
        }
    }
    TraceLog(LOG_INFO, "panelka: %d of %d types are placed, %d silent, %d built and unused",
             placed, FR_COUNT, silent, unused);
}

// The whole point of the flat is that no two of the sixty agree about everything, and every part of that is a hash: a seed that happened to collapse, or a table indexed past its end and clamped, would leave the facade uniform again without anything failing. So count what the hashes actually produced.
static void CheckFlatVariety(void)
{
    int pvc = 0, ac = 0, bal[3] = { 0, 0, 0 };
    int frames[COUNT_OF(TIMBER_PAINT)] = { 0 }, curtains[COUNT_OF(CURTAIN)] = { 0 };
    int n = 0;
    for (int s = 0; s < SECTIONS; s++) {
        for (int f = 0; f < FLOORS; f++) {
            for (int side = 0; side < 2; side++) {
                for (int h = 0; h < 2; h++) {
                    Flat fl = FlatAt(s, f, side, h);
                    n++;
                    if (fl.pvc) pvc++;
                    if (fl.ac) ac++;
                    bal[fl.balcony]++;
                    for (int k = 0; k < COUNT_OF(TIMBER_PAINT); k++) {
                        if (ColorToInt(TIMBER_PAINT[k]) == ColorToInt(fl.frame)) { frames[k]++; break; }
                    }
                    for (int k = 0; k < COUNT_OF(CURTAIN); k++) {
                        if (ColorToInt(CURTAIN[k]) == ColorToInt(fl.curtain)) { curtains[k]++; break; }
                    }
                }
            }
        }
    }
    int distinctCurtains = 0;
    for (int k = 0; k < COUNT_OF(CURTAIN); k++) if (curtains[k] > 0) distinctCurtains++;
    TraceLog(LOG_INFO, "panelka: %d flats, %d with plastic windows, %d with an air conditioner; balconies %d open, %d glazed in timber, %d in plastic; %d of %d curtain colours used",
             n, pvc, ac, bal[BAL_OPEN], bal[BAL_TIMBER], bal[BAL_PVC], distinctCurtains, COUNT_OF(CURTAIN));
    if (pvc == 0 || pvc == n || bal[BAL_OPEN] == n || distinctCurtains < 3) {
        TraceLog(LOG_ERROR, "panelka: the flat hashes have collapsed and the facade is uniform");
    }
}

// Nothing structural may reach the outside of the skin it stands behind.
//
// This is the check that the end cross wall needed and did not have. Standing 0.07 m proud of the gable, it hid every panel of both end walls behind a blank grey strip, through the first build and two review rounds; what a critique could say was that the gable read as under-divided, which sent two sessions looking at the gable's own decomposition, where the gable was not what was being drawn there at all. A defect that hides the thing you would inspect to find it does not get found by looking harder.
//
// Only the frame is asked. The parapet's capping oversails the facade on purpose, the balconies project a metre, the plinth stands 0.1 m out, and the canopy and the vent stacks are not behind anything.
static void CheckStructureInside(void)
{
    static const FragType FRAME[] = { FR_XWALL, FR_SPINE26, FR_SPINE32, FR_SLAB26, FR_SLAB32, FR_ROOF26, FR_ROOF32 };
    float outX = 0.0f, outZ = 0.0f;
    FragType worstX = FR_COUNT, worstZ = FR_COUNT;

    for (int i = 0; i < gFragCount; i++) {
        bool frame = false;
        for (int k = 0; k < COUNT_OF(FRAME); k++) frame = frame || (gFrag[i].type == FRAME[k]);
        if (!frame) continue;

        BoundingBox b = gType[gFrag[i].type].bounds;
        Matrix m = FragRest(&gFrag[i]);
        for (int c = 0; c < 8; c++) {
            Vector3 p = { (c & 1) ? b.max.x : b.min.x, (c & 2) ? b.max.y : b.min.y, (c & 4) ? b.max.z : b.min.z };
            p = Vector3Transform(p, m);
            if (fabsf(p.x) - BLOCK_LEN * 0.5f > outX) { outX = fabsf(p.x) - BLOCK_LEN * 0.5f; worstX = gFrag[i].type; }
            if (fabsf(p.z) - FACE_Z > outZ) { outZ = fabsf(p.z) - FACE_Z; worstZ = gFrag[i].type; }
        }
    }
    TraceLog(LOG_INFO, "panelka: the frame reaches %.3f m past the gable (%s) and %.3f m past the facade (%s)",
             outX, worstX < FR_COUNT ? TYPE_NAME[worstX] : "-",
             outZ, worstZ < FR_COUNT ? TYPE_NAME[worstZ] : "-");
    // KNIT is how far every piece is deliberately grown past its own edge, so anything at or under it is the knit and anything over it is a piece standing in front of the skin.
    if (outX > KNIT + 1e-4f || outZ > KNIT + 1e-4f) {
        TraceLog(LOG_WARNING, "panelka: a frame member stands outside the skin and will draw over it");
    }
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
        if (!IsBuilding(gFrag[i].type)) continue;
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

    // What the yard ends up under. A review round found a birch standing vertically through several metres of rubble, which is the kind of thing that is obvious in a render and invisible in a number until somebody prints the number.
    int buried = 0, standing = 0;
    float worstStanding = 0.0f;
    for (int i = 0; i < gFragCount; i++) {
        if (FR_CLASS[gFrag[i].type] != FC_YARD) continue;
        float pile = PileUnder(gFrag[i].pos.x, gFrag[i].pos.z, 1.0f, 1.0f);
        if (pile > 0.35f) buried++;
        else { standing++; worstStanding = fmaxf(worstStanding, pile); }
    }
    TraceLog(LOG_INFO, "panelka: %d things in the yard end up under the pile and go over, %d stay standing, the deepest of those in %.2f m of debris",
             buried, standing, worstStanding);
    if (worstStanding > 0.35f) {
        TraceLog(LOG_WARNING, "panelka: something in the yard is standing upright in %.2f m of rubble", worstStanding);
    }
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

// The one scene-wide thing this model sets that is normally the harness's business, and it is here because of a mismatch of scale that nothing else can fix.
//
// The harness lights with three point lights at (8,10,8), (-9,5,-7) and (0,-9,5). Those surround a 4.5 m vehicle. They sit *inside* a 58 m building, so every outward-facing surface on it points away from all three and lighting.fs falls back on its ambient term, which its line 75 divides by ten: against the harness's ambient of 0.39, a 118 texel comes out of the gamma correction at 0.16.
//
// An earlier revision put a sun in the shader's one free slot and called that fixed. It was not, and a full eight-frame turntable is what showed it: a single directional light lights one quarter of a compass, so the front facade and the +X gable came out well and the back facade and the -X gable rendered as black silhouettes. Three of eight frames of every turntable of this model were unusable, and every render anyone had looked at happened to be one of the lit five.
//
// Two directional lights is the fix, and the second one costs a slot the model does not have. It takes the third harness light instead, which is a 40/39/47 fill standing at (0,-9,5) -- below the ground plane, inside the footprint, and the least useful of the three at this scale by a wide margin. rlights hands out slots in order, so the sun is index 3 and that fill is index 2, and the uniforms are set by name because rlights offers no way to reach a light somebody else created. Both facts are asserted below rather than assumed.
//
// The ambient goes up as well. It is what every surface facing neither light gets, and the harness's value is calibrated for an object small enough to be surrounded.
static void SetLightUniform(Shader sh, int slot, const char *field, const void *v, int type)
{
    char name[64];
    snprintf(name, sizeof(name), "lights[%d].%s", slot, field);
    SetShaderValue(sh, GetShaderLocation(sh, name), v, type);
}

static void AddSun(void)
{
    Shader sh = HarnessLightingShader();
    if (sh.id == 0) return;

    Light sun = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ 46.0f, 58.0f, 38.0f }, Vector3Zero(),
                            (Color){ 176, 168, 148, 255 }, sh);
    if (!sun.enabled) {
        TraceLog(LOG_ERROR, "panelka: no free light slot, half of every turntable will render as a silhouette");
        return;
    }
    // rlights fills slots in order and the harness took the first three, so the sun must be index 3. If that ever stops being true the fill below would overwrite a light that is doing real work, which is worth an error rather than a surprise.
    if (sun.enabled && GetShaderLocation(sh, "lights[3].enabled") < 0) {
        TraceLog(LOG_ERROR, "panelka: the shader has no fourth light slot");
    }

    // The sky fill, from the opposite quarter and much weaker, so a surface facing away from the sun is shaded rather than unlit. Cool against the sun's warm, which is what an overcast north sky is.
    const int FILL = 2;
    int type = LIGHT_DIRECTIONAL, enabled = 1;
    float pos[3] = { -52.0f, 34.0f, -44.0f };
    float target[3] = { 0.0f, 0.0f, 0.0f };
    float col[4] = { 88.0f / 255.0f, 96.0f / 255.0f, 112.0f / 255.0f, 1.0f };
    SetLightUniform(sh, FILL, "enabled", &enabled, SHADER_UNIFORM_INT);
    SetLightUniform(sh, FILL, "type", &type, SHADER_UNIFORM_INT);
    SetLightUniform(sh, FILL, "position", pos, SHADER_UNIFORM_VEC3);
    SetLightUniform(sh, FILL, "target", target, SHADER_UNIFORM_VEC3);
    SetLightUniform(sh, FILL, "color", col, SHADER_UNIFORM_VEC4);

    // And what is left for a surface that faces neither: the underside of a balcony, the inside of a stairwell, a slab lying face down on the pile.
    float ambient[4] = { 0.62f, 0.63f, 0.70f, 1.0f };
    SetShaderValue(sh, GetShaderLocation(sh, "ambient"), ambient, SHADER_UNIFORM_VEC4);
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
    SortFragments();
    PlaceDust();
    gDust.instCount = gPuffCount;
    gFlash.instCount = gFlashCount;

    // How high the pile stands is the summed mesh volume of everything that falls, bulked, and
    // fitted to the pile's own shape. Nothing about it is chosen.
    float material = 0.0f;
    for (int i = 0; i < gFragCount; i++) {
        if (!IsBuilding(gFrag[i].type)) continue;
        float sc = gFrag[i].scale;
        material += gType[gFrag[i].type].volume * sc * sc * sc;
    }
    for (int i = 0; i < gFragCount; i++) PlanFragment(i);
    SolveLanding();
    TraceLog(LOG_INFO, "panelka: %.0f m3 of material bulks to %.0f; the packed pile crests at %.2f m against a %.2f m prediction",
             material, material * BULK_FACTOR, PileCrest(), PredictedCrest(material));

    // After the landings, not before: this is the one dust population that knows where anything went.
    AddImpactDust();
    gDust.instCount = gPuffCount;

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
    bDust = InstBounds(PART_DUST, COUNT_OF(PART_DUST), true);

    CheckBaysTile();
    CheckPanels();
    CheckTypesBuilt();
    CheckFlatVariety();
    CheckStructureInside();
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
        "A three-section five-storey 1-464 panel block -- a khrushchyovka -- with sixty households living in it, taken down by controlled explosion in its own courtyard, in one six-second cycle.\n"
        "\n"
        "One world unit is one metre. 58.20 m along the block, 11.82 m over the facades, 14.26 m to the top of the cornice.\n"
        "Origin sits on the ground at the centre of the footprint, +X along the block, +Z out through the main facade.\n"
        "\n"
        "Dimensions, and where they come from.\n"
        "The series specifies outer panels 2.6 and 3.2 m wide, floor slabs spanning 5.76 m, a 2.5 m clear ceiling, a 0.10 m slab and 0.21 to 0.35 m outer walls.\n"
        "references/panelka/ref_01.png is a plan of one section; scaled by that 5.76 m span it comes out at 66.9 px/m, and reading the section back off that scale gives seven bays of 2.69, 3.24, 2.64, 2.57, 2.54, 3.23 and 2.66 m over an 11.52 m depth between the outer wall centre-lines.\n"
        "Those are the specified 2.6 and 3.2 m to within 3.5 per cent, so the bays here are 2.6, 3.2, 2.6, 2.6, 2.6, 3.2, 2.6 = 19.40 m per section, the depth is 2 x 5.76 plus one 0.30 m wall, and the storey is 2.50 of clear ceiling plus a 0.10 structural slab plus 0.06 of screed = 2.66 m. The screed is the only invented number in that sum.\n"
        "\n"
        "The building is precast panels, not a shell with holes in it.\n"
        "There is one mesh per panel type and a list of transforms saying where the instances go, which is what makes the joint grid real rather than drawn on, and which is the decomposition the demolition will throw. 73 types and 3241 fragments.\n"
        "Openings are cut by decomposing a panel into horizontal bands at every opening edge and, within a band, into the x-segments the openings leave: any number of openings then needs no special case, and every reveal is the side of a closed box. A layer is given its absolute extent rather than an inset off the panel, which is what lets the same function cut a whole panel, the aggregate field set back inside it, and a fracture piece that ends at a break on one side and at the panel on the other.\n"
        "\n"
        "The panel face, which is the thing this revision is mostly about.\n"
        "Each panel is a cast core across its full width with an exposed-aggregate field standing 0.014 m proud of it, set back 0.045 m from its vertical edges and 0.072 m from its horizontal ones, so two neighbours leave 0.09 m of smooth render between them side by side and 0.145 m one above the other. Both are read off ref_05.jpg against a metric ruler drawn on the unscaled image at 117.7 px/m, the storey pitch recovered by autocorrelating a window-free column of its gable: 313 px for 2.66 m. The horizontal is genuinely the wider of the two, being the bed the floor slab bears in and packed with mortar where the vertical is a caulked butt.\n"
        "A revision of this file put both at 0.170, off a 45 px reading taken on a crop that had been upscaled two-fold and then divided by the unscaled image\'s px/m, and a review round caught it. The error is recorded rather than merely fixed, because the conclusion drawn from it was wrong twice over: the first build\'s border was about right, and the reason its facade rendered as a flat grey sheet was that both layers carried the same map, so there was nothing to see at any width. The fix that mattered was the material split, not the width.\n"
        "The two layers carry different maps, which is the other half of it. The core is smooth render, so the border and the reveal of every opening come out pale and untoothed; the field is washed exposed aggregate, dense pale pebbles of about a centimetre in a grey matrix, built as the larger of two high-frequency lattices because a single one spends half its area near its own mean and reads as a wash rather than as separate stones.\n"
        "Weathering is baked per vertex rather than into a map, because a map here is shared by every instance of a type and repeats every metre where a streak has to start under the one sill it runs from. Both shaders read the attribute -- lighting.fs computes tint = colDiffuse*fragColor -- so it is a straight multiplier, and darkening is all it is used for: a run under every sill, broken into separate runs by a noise field and squared away over 1.15 m; a wash along the foot of every panel where the run-off reaches the horizontal joint and stops; a lighter one along the head, which is what the panel above shed onto it. The aggregate layer is subdivided to 0.22 m cells for no other reason than to give that function vertices to write onto, and only the layer whose face the weather reaches is subdivided at all.\n"
        "\n"
        "Sixty flats, and what each of them chose.\n"
        "The unit of variety on a facade like this is the flat, not the panel and not the building, because everything that varies was decided by whoever lived in one: windows were replaced a flat at a time, balconies enclosed a flat at a time in whatever the resident could get, and every curtain in a flat came out of the same shop on the same afternoon. Scattered panel by panel the same amount of variation reads as noise. A 1-464 section has four flats to a landing, which here is the three bays either side of the stairwell on each facade, so 3 x 5 x 4 = 60, and every one of them owns one of the two 3.2 m balcony bays.\n"
        "Each is hashed from where it is: 25 of the 60 have replaced their windows with plastic and the rest still have painted timber in one of five colours; the pane behind the joinery carries what is behind the glass rather than the glass itself, in one of eight curtain and dark-room colours, which is the cheapest thing on this facade and the one that most makes it read as lived in; 26 balconies are open with a painted sheet front, 14 are glazed in timber and 20 in white plastic, over sheet fronts in six colours ref_05 and ref_07 between them carry on a single building; 10 flats have hung an air conditioner beside a window and rather under half of the ground-floor ones have barred theirs.\n"
        "The plastic window is two lights on an off-centre mullion. The original is heavier in section, splits down the middle, and carries a transom with a small hinged vent in one upper light, which is the detail that dates it. The stairwell is communal, so nobody has replaced its windows and it is the one part of the facade with no variation on it at all.\n"
        "Every one of those is a per-instance tint on a shared mesh, which is why the joinery, the pane, the balcony slab, its sheet front, its enclosure and the enclosure\'s glass are six fragment types rather than one: a tint reaches every material in the group it is applied to, so glass left in with a brown frame comes out brown.\n"
        "\n"
        "Composition.\n"
        "The stairwell is the middle 2.6 m bay of each section and touches the front facade, so its tall narrow lights are on the front and the entrance is on the back, which is the courtyard side. Those lights alternate storey by storey between a 0.45 and a 1.30 m sill: a landing sits half a flight above the floor its panel stands on, a one-storey panel cannot carry a light that straddles the slab, and the zigzag is how that offset reads from outside.\n"
        "Apartment windows are 1.30 m wide in a 2.6 m bay and 1.50 in a 3.2 m one, 1.45 m tall on a 0.85 m sill.\n"
        "The 3.2 m bays carry balconies on the four floors above the ground one, which carries a plain wide window instead. An earlier build ran them to the ground on the strength of ref_07; CheckTypesBuilt then reported the plain 3.2 m panel built and never placed anywhere in the block, and going back to the photographs, ref_03 and ref_05 both show the stacks starting one storey up over a ground floor of plain windows.\n"
        "Downpipes run at every section joint and gable corner on both facades, in one segment per storey so they break up with the building, and they are rusted through rather than painted, which is what ref_03 and ref_05 both show and the loudest colour on either facade. Each stands 0.125 m off the panel face on two visible brackets a storey apart: at 0.070 it was 12 mm clear of the wall and a review round read it as a strip painted on rather than a pipe bolted to it.\n"
        "The gable is four panels across its 11.82 m depth, not two: ref_05 reads four columns across its end wall at the same border pitch the facade carries, and two 5.91 m slabs gave the gable a grid four times coarser than the rest of the building. Two review rounds have now called that grid too weakly separated to read at whole-building distance, and the second was answered rather than acted on: the gable is built by the same function and the same constants as the facade, its border is the measured 0.09 m, and the tonal step across it is about +10 levels of 255 against the +5.5 measured on ref_05 itself, so it is already the more contrasty of the two. What differs is resolution. ref_05 resolves the joint at 117 px/m; this scene\'s 62 m orbit puts the building across about 19 px/m, where a 0.09 m band is under two pixels. Widening it to read at that distance is exactly the error the first round corrected, so it is left alone and the close gable render in renders/panelka/ is the evidence that the geometry is there.\n"        "It is blank, which ref_02, ref_03 and ref_07 show and ref_01 draws; ref_05 is the variant that does put two window columns in its gable, and ref_04 exists to show one painted instead.\n"
        "The roof is flat, with a 0.21 m upstand under a metal capping that oversails the facade by 0.085 -- a thin capped cornice rather than a parapet, which is what ref_03 and ref_05 show and which 0.32 m of exposed upstand read as a modern fascia band running the whole building. The series specifies the original directly, as a flat non-ventilated roof finished in rolled bitumen; the pitched metal roofs in ref_02 and ref_07 are the re-roofing thousands of these were given in the 1990s.\n"
        "\n"
        "Behind the skin the structure is real, because the demolition has to break it: floor slabs spanning bay by bay from the facade to the central spine, a spine wall segment per bay, and a cross wall on every bay boundary, which is the load-bearing direction in this series.\n"
        "Every slab and cross wall is cut into four cells by two straight lines that cross at its centre, each meeting the plate\'s edges off-square. That leaves two roughly triangular pieces and two roughly trapezoidal ones, no two the same shape and none of them rectangular, because concrete does not break at right angles and a plate cut into rectangles lands as a stack of tiles.\n"
        "The cells are defined by the cuts rather than by their own extents, so every interior edge is shared by the two cells either side of it and the two cuts meet at the centre by construction. They therefore tile the plate exactly and the intact building is unchanged: the cut lines are internal and invisible until the thing falls apart. Four rather than a jagged many, because a cell has to stay a hexahedron for the mesh builder, which caps it at four corners in plan. Earlier builds laid the same plates as two and then three rectangular strips, and a review round still read the pile as a heap of intact architectural cards.\n"
        "\n"
        "The yard.\n"
        "Sixteen trees, five cars, six benches, two rug-beating frames, two bins and four lamp posts, all of them instanced types like the building, because a tree the blast is going to lash and a car it is going to bury both have to be things the pose function can move.\n"
        "A tree crown is twenty-two lumpy ellipsoids on a golden-angle spiral over its own crown ellipsoid, each about a quarter of the crown radius, with the normal taken from the underlying ellipsoid rather than the lumpy surface: a true normal shades every lump as its own object and the crown breaks into a bag of boulders.\n"
        "The car is a VAZ-2101 from the specification and ref_08.jpg: 4.073 by 1.611 by 1.382, 2.424 wheelbase, 1.349 front track, 0.170 clearance, with the beltline read off the elevation at 0.83 and the bonnet crown standing above it rather than below, which is what gives the car its flat-topped nose. Its wheel arches are swept as vertical strips whose floor follows the arch circle, in the outer 0.22 m of each flank only, so the arch is a recess rather than a tunnel through the car.\n"
        "\n"
        "The demolition.\n"
        "A 1-464 has no frame: every cross wall carries load, which is why the series went up so fast and why it comes down the way it does. Charges are drilled into the cross walls of the bottom two storeys and fired in a sequence running the length of the block, so the building loses its footing progressively and folds instead of toppling; what is above the cut then descends almost vertically and pancakes, with the facade panels peeling off outwards because nothing holds them on once the cross walls behind them are gone.\n"
        "Which of those a fragment is comes off a table keyed by type rather than a range test on the enum. An earlier build asked whether the type was at or below the last panel to decide it was skin, which is correct exactly as long as nobody inserts a type, and this revision inserts thirty; a misfiled type there is silent, because the piece simply falls with the wrong timing among two thousand others.\n"
        "Every fragment does three things and no more.\n"
        "It rides down. The crush front leaves the cut when the charges under that part of the block fire and climbs at 11 m/s, and everything above it descends by 0.62 of the height that has gone underneath it, because that is what a crushed storey loses. Without it a fragment waits in place until the front arrives and the roof sits level over a stripped frame.\n"
        "It flies, ballistically, tumbling about its own centroid rather than its local origin, which would be a hinge no broken panel has. The drop it had already taken at its release time is exactly where the flight starts, so the two meet without a step.\n"
        "And it lands. There is no contact between fragments in flight: nothing here resolves a collision, because at this scale what reads is the timing and the peel rather than any individual impact.\n"
        "Four in five facade panels crack as they go, at the side of an opening, which is where the section is thinnest; the blank gable has no opening, so its break is simply off centre, because two congruent halves read as a cut rather than as a fracture. It is three in five rather than all of them because a demolition leaves some panels whole and a facade every one of which cracks identically reads as a pattern rather than as damage. A break is not an edge: where a slice ends inside the panel both layers run past it instead of setting back, so the two halves are indistinguishable from the whole panel until they separate.\n"
        "Glazing is the exception to the crush front: every pane in the building goes at the shock rather than when its own storey is reached, spinning five times faster than a panel -- but leaving at 1.1 to 2.8 m/s, not the 3.6 to 7.8 it once had. A balcony fails as its own storey is reached and 0.12 s after it, not 0.22 s before. The skin waits 0.42 s behind the structure at its own height, where a first attempt used 0.20.\n"
        "All three of those numbers moved for one reason. A review round read the first second of the sequence as the skin being explosively removed from a standing building rather than as a building losing its footing: 210 panes leaving at 8 m/s and a ring of balconies in the air while nothing above them had moved yet. The peel now has to come out of the tumble and the fall rather than out of a kick, which is what it comes out of in the films. Structural pieces also get their own release jitter of up to 0.19 s, because sharing a release time to the millisecond kept the roof legible as a complete deck through the middle of the sequence, sitting in its original plane over a facade that had already gone.\n"
        "\n"
        "Where a piece comes to rest is decided by what is already there. The pile is a heightfield, and the fragments are planned in the order they land: each is put on top of whatever is in the cells its own footprint covers, and then deposits its own material -- volume by the divergence theorem, bulked by 1.55 -- spread over that footprint. 1737 m3 becomes 2693 of rubble and the packed pile crests at 5.9 m, against 3.9 m for a smooth mound of the same material. The packed pile stands higher because it puts its material where the pieces actually land, which is mostly over the footprint, where the prediction spreads everything evenly over the footprint and the skirt together.\n"
        "Every cubic metre of that is a cubic metre the building was built from. It did not use to be: 800 lumpy ellipsoids used to be scattered through the collapsing volume as stand-in rubble, carrying 331 m3 of concrete that was in no panel and no slab, spawning from nothing in mid-air and thrown harder than anything else in the scene. They were written when nothing in the building could break, and they were kept -- and doubled -- after the panel fracture and the plate cells had made them unnecessary. The pile is a fifth shorter without them, which is the correct height rather than a loss.\n"
        "What a piece rests on is weighted towards the mean of the cells under it rather than their maximum. Taking the maximum is what a rigid plate really does, and it is what this did until the slabs and walls were broken into thirds; with three thousand fragments each perching on the last one\'s high corner the error compounded and the crest ran to 8.5 m on a 14.3 m building, where a real demolition pile is a quarter to a third of the building\'s height. The weight is the one number in the collapse fitted to an outcome rather than derived, and CheckCollapse prints the packed crest against the smooth-mound prediction on every build so that the fit stays visible.\n"
        "Both the landing point and the fragment\'s own extent depend on the flight time, and the flight time on both, so each is solved by four passes of the same substitution. The extent is taken at the orientation the piece actually lands in, from the support function of its transformed box, not from its upright height: a 2.66 m panel that comes down edge-on has a vertical extent of 0.15.\n"
        "\n"
        "Dust is four populations, because one averaging them reads as ground fog. 1557 puffs in all. The jets are what comes straight back out of the drill holes at the instant the charges fire -- small, fast, outward at 15 to 24 m/s, dead inside two seconds. The surge is air driven out sideways by the floors slamming down on it, which leaves the footprint low and rolls outwards along the ground. The column is what rises off the pile afterwards, slower and much taller.\n"
        "The fourth is new, and it is the only one that knows anything about this particular collapse. The other three are placed before anything is thrown, off the footprint and the firing sequence, so they bloom whether or not anything has landed, they bloom evenly along a block whose two ends land three tenths of a second apart, and they say nothing about the debris skirt, where the largest pieces end up and where a demolition throws its most conspicuous dust. This one is placed after the landings are solved, so a puff exists because a slab hit the pile there at that moment, scaled by how big the piece was and how hard it arrived. Anything the size of a slab raises dust every time and the smaller pieces take a hashed third, which is 887 landings. Leaving the big ones to a hash was the first attempt and it meant three quarters of the most conspicuous arrivals -- the plates that reach the debris skirt, which is where a demolition throws its most visible dust -- landed in silence. The column population was cut and delayed by 0.6 s in the same pass, because it was arriving in time to hang evenly over a block that was still coming down.\n"
        "A puff is thrown and then sheds its speed to the air as a first-order decay to a terminal displacement, which is why dust travels so much further than a thrown solid and then simply hangs.\n"
        "The soft edge is a second small shader, and both it and the technique are models/humvee.c\'s. A closed blob has a hard silhouette at any subdivision, so a puff\'s alpha is scaled by how squarely its surface faces the camera, which is zero exactly at its own silhouette; its normals are radial, of the sphere the lumps are pushed out from rather than of the lumpy surface, so that falloff runs monotonically to the edge instead of leaving a ring of zero alpha inside it. The detonation flashes run the same shader, additively.\n"
        "The charges are milliseconds of light, and --anim samples the cycle at N evenly spaced phases, so a truthful flash would be caught only by luck. It is held over 0.30 s instead -- a playback concession, not a claim about explosives -- which puts the window at 0.90 to 1.20 s and straddles t = 1.0, so every --frames that is a multiple of 6 lands a frame inside it.\n"
        "\n"
        "The yard is not destroyed but it is not untouched. An air blast leaves the building when the charges fire and travels at 105 m/s, so the lashing runs outwards across the scene rather than happening to everything at once; what it reaches gets one damped oscillation, scaled by distance. Trees lean 17 degrees at 0.62 Hz, cars rock on their springs at 1.65, and the benches and bins closest to the facade go over and stay over.\n"
        "And then the debris arrives. Nothing here resolves a collision and this does not add one: the pile is solved cell by cell before the first frame is drawn, so how deep the rubble ends up over any point of the yard is a number the pose function can look up. Anything standing in more than 0.35 m of it goes over, away from the building, and sinks into what buried it; ten of the forty things in the yard do. It exists because a review round found a birch standing vertically through several metres of rubble, with a lamp post and a car intact beside it, in every frame after the fourth. CheckCollapse now prints how many go under and how deep the deepest of those still standing is, which is 0.20 m. A tree can only hinge at its root here: skinning needs bone attributes the harness\'s shader does not declare, so a rigid trunk leaning is the honest limit rather than a choice, where a real tree bends most at the top.\n"
        "\n"
        "This is the one model here whose loop deliberately does not close. A demolition is a one-shot event, and --anim plays it once; rather than pretend otherwise, CheckLoopSeam measures the seam and reports it, and it is 12.9 m at the worst fragment. The last piece lands at 3.6 s of the 6 s cycle, thrown up to 7.5 m clear of the footprint.\n"
        "\n"
        "Thirteen build-time checks, of which three are about this revision and all three earned their place on the run that added them. CheckTypesBuilt reports any type that is placed and has no mesh -- which with sixty types, eleven of them differing from a neighbour only by which half of a panel they are, is the failure this decomposition is most exposed to -- and any type built and never placed, which is how the ground-floor balconies were caught. CheckFlatVariety counts what the flat hashes actually produced, because a seed that collapsed or a table indexed past its end would leave the facade uniform again without anything failing.\n"
        "CheckStructureInside measures how far any slab, cross wall or spine reaches past the skin it stands behind, and it is the check the block\'s ends needed and did not have. The cross wall closing each end was centred on the block\'s end plane, so it stood 0.07 m outside the gable and hid all twenty of its panels behind a blank grey strip banded once per storey by its own gap at each floor. That survived the first build and two review rounds, because what a critique could say was that the gable read as under-divided, which sent two sessions looking at the gable\'s own decomposition, where the gable was not what was being drawn there at all. A defect that hides the thing you would inspect to find it is not found by looking harder. The wall is now pulled inboard until its outer face is buried 10 mm inside the panel, derived from that panel rather than typed.\n"
        "\n"
        "--part frames the building\'s parts on their rest pose rather than on the swept cycle. Sweeping them gives the whole debris field, which frames every part identically and destroys exactly what --part exists for; only the debris and the dust, which have no rest pose worth framing, are swept.\n"
        "\n"
        "The plinth, the entrance steps and the ground exist once each and are placed groups rather than instanced ones. Each entrance is three risers and a shallow landing: the ground floor really is 0.75 m above grade, which is why there are steps at all, but what a khrushchyovka entrance does with the rest of that height is take it inside on the half flight up to the first landing. Five treads of 0.32 projected the flight 1.6 m into the courtyard and a review round read the entrances as later elevated annexes; ref_05\'s door stands within a step or two of grade under a thin shelf of a canopy, and ref_03 and ref_07 are the same.\n"
        "A balcony is four pieces: a concrete slab with the rusted steel that carries its front, a painted sheet standing on that slab, an enclosure of thin painted frames over six lights, and the glass in it. Both the sheet and the enclosure are set back 0.16 to 0.21 m from the slab\'s nose, so the concrete lip stays visible under and beside them at every floor. Flush with the nose, five storeys of enclosure merged into one tall box and two review rounds running read the enclosed balconies as coloured rectangles applied to the wall rather than as things standing on a cantilever.\n"
        "Lighting is normally the harness\'s business and here it cannot be. The harness lights with three point lights at (8,10,8), (-9,5,-7) and (0,-9,5), which surround a 4.5 m vehicle and sit inside a 58 m building, so every outward-facing surface on this one points away from all three and falls back on lighting.fs\'s ambient term, which its line 75 divides by ten: against the harness\'s 0.39, a 118 texel leaves the gamma correction at 0.16.\n"
        "An earlier revision put a sun in the shader\'s one free slot and treated that as the fix. It was not, and a full eight-frame turntable is what showed it: one directional light lights one quarter of a compass, so the front facade and one gable came out and the back facade and the other gable rendered as black silhouettes. Three frames in eight of every turntable of this model were unusable, and every render anyone had looked at happened to be one of the lit five.\n"
        "So there are two directional lights: a warm sun from (46,58,38) and a cool sky fill from (-52,34,-44) at about half its strength, with the ambient raised from 0.39 to 0.62 for the surfaces that face neither -- the underside of a balcony, the inside of a stairwell, a slab lying face down on the pile. The fill costs a slot the model does not have, so it takes the third harness light, a 40/39/47 fill standing below the ground plane inside the footprint and the least useful of the three at this scale by a wide margin. rlights hands out slots in order, so the sun is index 3 and that fill is index 2; both facts are asserted at build time rather than assumed, because rlights offers no way to reach a light somebody else created and the uniforms have to be set by name.\n"
        "Surfacing is twenty maps built in code from tiling value noise, projected planar in each mesh\'s own frame rather than the world\'s, because an instanced mesh has no world position; every instance would then carry an identical texture placement, so each also carries a hashed per-instance shade of +-8 to break that up.",
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
