#ifndef HARNESS_H
#define HARNESS_H

#include "raylib.h"

typedef struct Part {
    const char *name;
    void (*draw)(void);
    BoundingBox (*bounds)(void);
} Part;

typedef struct Scene {
    const char *name;
    const char *description;
    void (*init)(void);
    void (*draw)(void);
    void (*unload)(void);
    const Part *parts;
    int partCount;
    Vector3 target;
    float orbitRadius;
    float orbitHeight;
    Color background;
    bool hideGrid;
} Scene;

extern const Scene SCENE;

Shader HarnessLightingShader(void);
void HarnessApplyLighting(Model *model);

#endif
