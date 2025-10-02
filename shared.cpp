#include "shared.h"
#include <stdint.h>

static __forceinline uint32_t aligned_diff(const uint8_t* sp1, int32_t spitch1,
                                           const uint8_t* sp2, int32_t spitch2,
                                           int32_t hblocks, int32_t incpitch,
                                           int32_t height) {
    uint32_t sad = 0;

    spitch2 += incpitch;
    incpitch += spitch1;

    int32_t counter = hblocks;
    do {
        // Process 32 bytes per block
        for (int32_t i = 0; i < 32; i++) {
            int32_t diff = (int32_t)sp1[i] - (int32_t)sp2[i];
            sad += (diff < 0) ? -diff : diff;
        }

        sp1 += 32;
        sp2 += 32;
        if (--counter > 0) {
            continue;
        }
        sp1 += incpitch;
        sp2 += spitch2;
        counter = hblocks;
    } while (--height > 0);

    return sad;
}

static __forceinline uint32_t unaligned_diff(const uint8_t* sp1,
                                             int32_t spitch1,
                                             const uint8_t* sp2,
                                             int32_t spitch2, int32_t hblocks,
                                             int32_t incpitch, int32_t height) {
    uint32_t sad = 0;

    spitch2 += incpitch;
    incpitch += spitch1;

    int32_t counter = hblocks;
    do {
        // Process 32 bytes per block
        for (int32_t i = 0; i < 32; i++) {
            int32_t diff = (int32_t)sp1[i] - (int32_t)sp2[i];
            sad += (diff < 0) ? -diff : diff;
        }

        sp1 += 32;
        sp2 += 32;
        if (--counter > 0) {
            continue;
        }
        sp1 += incpitch;
        sp2 += spitch2;
        counter = hblocks;
    } while (--height > 0);

    return sad;
}

uint32_t gdiff(const uint8_t* sp1, int32_t spitch1, const uint8_t* sp2,
               int32_t spitch2, int32_t hblocks, int32_t incpitch,
               int32_t height) {
    if ((((uintptr_t)sp1 & (16 - 1)) + ((uintptr_t)sp2 & (16 - 1))) == 0) {
        return aligned_diff(sp1, spitch1, sp2, spitch2, hblocks, incpitch,
                            height);
    } else {
        return unaligned_diff(sp1, spitch1, sp2, spitch2, hblocks, incpitch,
                              height);
    }
}

void copyChroma(VSFrameRef* dest, const VSFrameRef* source,
                const VSVideoInfo* vi, const VSAPI* vsapi) {
    int32_t destPitch = vsapi->getStride(dest, 1);
    int32_t srcPitch = vsapi->getStride(source, 1);
    vs_bitblt(vsapi->getWritePtr(dest, 1), destPitch,
              vsapi->getReadPtr(source, 1), srcPitch,
              vi->width >> vi->format->subSamplingW,
              vi->height >> vi->format->subSamplingH);
    vs_bitblt(vsapi->getWritePtr(dest, 2), destPitch,
              vsapi->getReadPtr(source, 2), srcPitch,
              vi->width >> vi->format->subSamplingW,
              vi->height >> vi->format->subSamplingH);
}

VS_EXTERNAL_API(void)
VapourSynthPluginInit(VSConfigPlugin configFunc,
                      VSRegisterFunction registerFunc, VSPlugin* plugin) {
    configFunc("com.fakeurl.removedirtvs", "rdvs",
               "RemoveDirt VapourSynth Port", VAPOURSYNTH_API_VERSION, 1,
               plugin);
    registerFunc("SCSelect",
                 "input:clip;"
                 "sceneBegin:clip;"
                 "sceneEnd:clip;"
                 "globalMotion:clip;"
                 "dfactor:float:opt;",
                 SCSelectCreate, 0, plugin);
    registerFunc("RestoreMotionBlocks",
                 "input:clip;"
                 "restore:clip;"
                 "neighbour:clip:opt;"
                 "neighbour2:clip:opt;"
                 "alternative:clip:opt;"
                 "gmthreshold:int:opt;"
                 "mthreshold:int:opt;"
                 "noise:int:opt;"
                 "noisy:int:opt;"
                 "dist:int:opt;"
                 "tolerance:int:opt;"
                 "dmode:int:opt;"
                 "pthreshold:int:opt;"
                 "cthreshold:int:opt;"
                 "grey:int:opt;",
                 RestoreMotionBlocksCreate, 0, plugin);
    registerFunc("DupBlocks",
                 "input:clip;"
                 "gmthreshold:int:opt;"
                 "mthreshold:int:opt;"
                 "noise:int:opt;"
                 "noisy:int:opt;"
                 "dist:int:opt;"
                 "tolerance:int:opt;"
                 "dmode:int:opt;"
                 "pthreshold:int:opt;"
                 "cthreshold:int:opt;"
                 "grey:int:opt;",
                 DupBlocksCreate, 0, plugin);
}
