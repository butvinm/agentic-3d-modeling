#ifndef HARNESS_H
#define HARNESS_H

#include "raylib.h"

typedef struct Scene {
    const char *name;
    void (*init)(void);
    void (*draw)(void);
    void (*unload)(void);
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
