RAYLIB_DIR := vendor/raylib
RAYLIB_SRC := $(RAYLIB_DIR)/src
RAYLIB_LIB := $(RAYLIB_SRC)/libraylib.a
SHADER_DIR := $(abspath $(RAYLIB_DIR)/examples/shaders/resources/shaders/glsl330)

MODEL_SRCS := $(wildcard models/*.c)
MODELS := $(patsubst models/%.c,build/%,$(MODEL_SRCS))

CFLAGS := -std=c11 -O2 -Wall -Wextra -I$(RAYLIB_SRC) -Isrc -I$(RAYLIB_DIR)/examples/shaders \
          -DRAYLIB_SHADER_DIR='"$(SHADER_DIR)"'
LDLIBS := -lm -lpthread -ldl -lrt -lX11

.PHONY: all raylib clean clean-raylib

all: $(MODELS)

build/%: models/%.c src/harness.c src/harness.h | $(RAYLIB_LIB)
	@mkdir -p build
	$(CC) $(CFLAGS) -o $@ $< src/harness.c $(RAYLIB_LIB) $(LDLIBS)

$(RAYLIB_LIB):
	$(MAKE) -C $(RAYLIB_SRC) PLATFORM=PLATFORM_DESKTOP RAYLIB_LIBTYPE=STATIC

raylib: $(RAYLIB_LIB)

clean:
	rm -rf build renders

clean-raylib:
	$(MAKE) -C $(RAYLIB_SRC) clean
