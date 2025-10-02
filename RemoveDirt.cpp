// Avisynth filter for removing dirt from film clips
//
// By Rainer Wittmann <gorw@gmx.de>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// To get a copy of the GNU General Public License write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .

#include "VSHelper.h"
#include "shared.h"
#include <stdint.h>

#define MOTIONBLOCKWIDTH 8
#define MOTIONBLOCKHEIGHT 8

#define MOTION_FLAG 1
#define MOTION_FLAGN 2
#define MOTION_FLAGP 4
#define TO_CLEAN 8
#define BMARGIN 16
#define MOTION_FLAG1 (MOTION_FLAG | TO_CLEAN)
#define MOTION_FLAG2 (MOTION_FLAGN | TO_CLEAN)
#define MOTION_FLAG3 (MOTION_FLAGP | TO_CLEAN)
#define MOTION_FLAGS (MOTION_FLAG | MOTION_FLAGN | MOTION_FLAGP)

#define U_N 54u // green
#define V_N 34
#define U_M 90 // red
#define V_M 240
#define U_P 240 // blue
#define V_P 110
#define u_ncolor (U_N + (U_N << 8) + (U_N << 16) + (U_N << 24))
#define v_ncolor (V_N + (V_N << 8) + (V_N << 16) + (V_N << 24))
#define u_mcolor (U_M + (U_M << 8) + (U_M << 16) + (U_M << 24))
#define v_mcolor (V_M + (V_M << 8) + (V_M << 16) + (V_M << 24))
#define u_pcolor (U_P + (U_P << 8) + (U_P << 16) + (U_P << 24))
#define v_pcolor (V_P + (V_P << 8) + (V_P << 16) + (V_P << 24))

#define leftdp (-1)
#define rightdp 7
#define leftsp leftdp
#define rightsp rightdp
#define topdp (-dpitch)
#define topsp (-spitch)
#define rightbldp 8
#define rightblsp rightbldp
#define leftbldp (-8)
#define leftblsp leftbldp

#define Cleftdp (-1)
#define Crightdp 3
#define Cleftsp Cleftdp
#define Crightsp Crightdp
#define Crightbldp 4
#define Crightblsp Crightbldp
#define Ctopdp (-dpitchUV)
#define Ctopsp (-spitchUV)

uint32_t ALIGNED_ARRAY(blockcompare_result, 16)[4];

static __forceinline void SADcompareSSE2(const uint8_t* p1, const uint8_t* p2,
                                         int32_t pitch,
                                         const uint8_t* noiselevel) {
    (void)noiselevel;
    uint32_t sad0 = 0;
    uint32_t sad1 = 0;

    // Process 8x8 block, calculate SAD for two 8-byte halves
    for (int32_t row = 0; row < 8; row++) {
        // First 8 bytes (left half)
        for (int32_t col = 0; col < 8; col++) {
            int32_t diff =
                (int32_t)p1[row * pitch + col] - (int32_t)p2[row * pitch + col];
            sad0 += (diff < 0) ? -diff : diff;
        }
        // Second 8 bytes (right half)
        for (int32_t col = 8; col < 16; col++) {
            int32_t diff =
                (int32_t)p1[row * pitch + col] - (int32_t)p2[row * pitch + col];
            sad1 += (diff < 0) ? -diff : diff;
        }
    }

    blockcompare_result[0] = sad0;
    blockcompare_result[1] = 0;
    blockcompare_result[2] = sad1;
    blockcompare_result[3] = 0;
}

static __forceinline void NSADcompareSSE2(const uint8_t* p1, const uint8_t* p2,
                                          int32_t pitch,
                                          const uint8_t* noiselevel) {
    uint32_t sad0 = 0;
    uint32_t sad1 = 0;

    // Process 8x8 block with noise threshold
    for (int32_t row = 0; row < 8; row++) {
        for (int32_t col = 0; col < 16; col++) {
            uint8_t v1 = p1[row * pitch + col];
            uint8_t v2 = p2[row * pitch + col];
            uint8_t noise = noiselevel[col];

            // Calculate saturating subtraction
            uint8_t diff1 = (v1 > v2) ? (v1 - v2) : 0;
            uint8_t diff2 = (v2 > v1) ? (v2 - v1) : 0;

            // Apply noise threshold
            diff1 = (diff1 > noise) ? (diff1 - noise) : 0;
            diff2 = (diff2 > noise) ? (diff2 - noise) : 0;

            // Sum the differences
            uint32_t sum = diff1 + diff2;

            if (col < 8) {
                sad0 += sum;
            } else {
                sad1 += sum;
            }
        }
    }

    blockcompare_result[0] = sad0;
    blockcompare_result[1] = 0;
    blockcompare_result[2] = sad1;
    blockcompare_result[3] = 0;
}

uint8_t ALIGNED_ARRAY(excessaddSSE2, 16)[16] = {8, 8, 8, 8, 8, 8, 8, 8,
                                                8, 8, 8, 8, 8, 8, 8, 8};

static __forceinline void ExcessPixelsSSE2(const uint8_t* p1, const uint8_t* p2,
                                           int32_t pitch,
                                           const uint8_t* noiselevel) {
    uint32_t count0 = 0;
    uint32_t count1 = 0;

    // Process 8x8 block, count excess pixels
    for (int32_t row = 0; row < 8; row++) {
        for (int32_t col = 0; col < 16; col++) {
            uint8_t v1 = p1[row * pitch + col];
            uint8_t v2 = p2[row * pitch + col];
            uint8_t noise = noiselevel[col];

            // Calculate saturating subtraction
            uint8_t diff1 = (v1 > v2) ? (v1 - v2) : 0;
            uint8_t diff2 = (v2 > v1) ? (v2 - v1) : 0;

            // Apply noise threshold
            diff1 = (diff1 > noise) ? (diff1 - noise) : 0;
            diff2 = (diff2 > noise) ? (diff2 - noise) : 0;

            // Check if both are zero (equal after noise threshold)
            uint8_t match = (diff1 == 0 && diff2 == 0) ? 0xFF : 0;

            if (col < 8) {
                count0 += match;
            } else {
                count1 += match;
            }
        }
    }

    // Add offset and store results
    blockcompare_result[0] = count0 + (8 * 8);
    blockcompare_result[1] = 0;
    blockcompare_result[2] = count1 + (8 * 8);
    blockcompare_result[3] = 0;
}

static __forceinline uint32_t SADcompare(const uint8_t* p1, int32_t pitch1,
                                         const uint8_t* p2, int32_t pitch2,
                                         const uint8_t* noiselevel) {
    (void)noiselevel;
    uint32_t sad = 0;

    // Process 8x8 block (8 bytes wide, 8 rows)
    for (int32_t row = 0; row < 8; row++) {
        for (int32_t col = 0; col < 8; col++) {
            int32_t diff = (int32_t)p1[row * pitch1 + col] -
                           (int32_t)p2[row * pitch2 + col];
            sad += (diff < 0) ? -diff : diff;
        }
    }

    return sad;
}

static __forceinline uint32_t NSADcompare(const uint8_t* p1, int32_t pitch1,
                                          const uint8_t* p2, int32_t pitch2,
                                          const uint8_t* noiselevel) {
    uint32_t sad = 0;

    // Process 8x8 block with noise threshold
    for (int32_t row = 0; row < 8; row++) {
        for (int32_t col = 0; col < 8; col++) {
            uint8_t v1 = p1[row * pitch1 + col];
            uint8_t v2 = p2[row * pitch2 + col];
            uint8_t noise = noiselevel[col];

            // Calculate saturating subtraction
            uint8_t diff1 = (v1 > v2) ? (v1 - v2) : 0;
            uint8_t diff2 = (v2 > v1) ? (v2 - v1) : 0;

            // Apply noise threshold
            diff1 = (diff1 > noise) ? (diff1 - noise) : 0;
            diff2 = (diff2 > noise) ? (diff2 - noise) : 0;

            // Sum the differences
            sad += diff1 + diff2;
        }
    }

    return sad;
}

uint8_t ALIGNED_ARRAY(excessadd, 16)[16] = {4, 4, 4, 4, 4, 4, 4, 4,
                                            4, 4, 4, 4, 4, 4, 4, 4};

static __forceinline uint32_t ExcessPixels(const uint8_t* p1, int32_t pitch1,
                                           const uint8_t* p2, int32_t pitch2,
                                           const uint8_t* noiselevel) {
    uint32_t count = 0;

    // Process 8x8 block, count excess pixels
    for (int32_t row = 0; row < 8; row++) {
        for (int32_t col = 0; col < 8; col++) {
            uint8_t v1 = p1[row * pitch1 + col];
            uint8_t v2 = p2[row * pitch2 + col];
            uint8_t noise = noiselevel[col];

            // Calculate saturating subtraction
            uint8_t diff1 = (v1 > v2) ? (v1 - v2) : 0;
            uint8_t diff2 = (v2 > v1) ? (v2 - v1) : 0;

            // Apply noise threshold
            diff1 = (diff1 > noise) ? (diff1 - noise) : 0;
            diff2 = (diff2 > noise) ? (diff2 - noise) : 0;

            // Check if both are zero (equal after noise threshold)
            if (diff1 == 0 && diff2 == 0) {
                count += 255;
            }
        }
    }

    // Add offset (4 per pixel for 8 pixels)
    return count + (8 * 4);
}

static void processneighbours1(MotionDetectionDistData* mdd) {
    uint8_t* properties = mdd->md.blockproperties;
    int32_t j = mdd->md.vblocks;

    do {
        int32_t i = mdd->md.hblocks;

        do {
            if (properties[0] == MOTION_FLAG2) {
                ++mdd->distblocks;
            }
            ++properties;
        } while (--i);

        properties++;
    } while (--j);
}

static void processneighbours2(MotionDetectionDistData* mdd) {
    uint8_t* properties = mdd->md.blockproperties;
    int32_t j = mdd->md.vblocks;

    do {
        int32_t i = mdd->md.hblocks;

        do {
            if (properties[0] == MOTION_FLAG1) {
                --mdd->distblocks;
                properties[0] = 0;
            } else if (properties[0] == MOTION_FLAG2) {
                ++mdd->distblocks;
            }
            ++properties;
        } while (--i);

        properties++;
    } while (--j);
}

static void processneighbours3(MotionDetectionDistData* mdd) {
    uint8_t* properties = mdd->md.blockproperties;
    int32_t j = mdd->md.vblocks;

    do {
        int32_t i = mdd->md.hblocks;

        do {
            if (properties[0] != (MOTION_FLAG1 | MOTION_FLAG2)) {
                if (properties[0] == MOTION_FLAG1) {
                    --mdd->distblocks;
                }
                properties[0] = 0;
            }
            ++properties;
        } while (--i);

        properties++;
    } while (--j);
}

static void markneighbours(MotionDetectionDistData* mdd) {
    uint8_t* begin = mdd->md.blockproperties;
    uint8_t* end = begin;
    uint32_t* isum2 = mdd->isum;

    int32_t j = mdd->md.vblocks;
    do {
        uint32_t sum = 0;
        int32_t i = mdd->dist;
        do {
            sum += *end++;
        } while (--i);

        i = mdd->dist1;
        do {
            *isum2++ = (sum += *end++);
        } while (--i);

        i = mdd->hint32_terior;
        do {
            *isum2++ = (sum += *end++ - *begin++);
        } while (--i);

        i = mdd->dist;
        do {
            *isum2++ = (sum -= *begin++);
        } while (--i);

        begin += mdd->dist2;
        end++;
    } while (--j);

    uint32_t* isum1 = isum2 = mdd->isum;
    begin = mdd->md.blockproperties;
    j = mdd->md.hblocks;
    do {
        uint32_t sum = 0;
        int32_t i = mdd->dist;
        do {
            sum += *isum2;
            isum2 = (uint32_t*)((char*)isum2 + mdd->isumline);
        } while (--i);

        i = mdd->dist1;
        do {
            sum += *isum2;
            isum2 = (uint32_t*)((char*)isum2 + mdd->isumline);
            if (sum > mdd->tolerance) {
                *begin |= MOTION_FLAG2;
            }
            begin += mdd->md.nline;
        } while (--i);

        i = mdd->vint32_terior;
        do {
            sum += *isum2 - *isum1;
            isum2 = (uint32_t*)((char*)isum2 + mdd->isumline);
            isum1 = (uint32_t*)((char*)isum1 + mdd->isumline);
            if (sum > mdd->tolerance) {
                *begin |= MOTION_FLAG2;
            }
            begin += mdd->md.nline;
        } while (--i);

        i = mdd->dist;
        do {
            sum -= *isum1;
            isum1 = (uint32_t*)((char*)isum1 + mdd->isumline);
            if (sum > mdd->tolerance) {
                *begin |= MOTION_FLAG2;
            }
            begin += mdd->md.nline;
        } while (--i);

        begin += mdd->colinc;
        isum1 = (uint32_t*)((char*)isum1 + mdd->isuminc1);
        isum2 = (uint32_t*)((char*)isum2 + mdd->isuminc2);
    } while (--j);
}

static void markblocks1(MotionDetectionData* md, const uint8_t* p1,
                        int32_t pitch1, const uint8_t* p2, int32_t pitch2) {
    int32_t inc1 = MOTIONBLOCKHEIGHT * pitch1 - md->linewidth;
    int32_t inc2 = MOTIONBLOCKHEIGHT * pitch2 - md->linewidth;
    uint8_t* properties = md->blockproperties;

    int32_t j = md->vblocks;
    do {
        int32_t i = md->hblocks;

        do {
            properties[0] = 0;

            if (md->blockcompare(p1, pitch1, p2, pitch2, md->noiselevel) >=
                md->threshold) {
                properties[0] = MOTION_FLAG1;
                ++md->motionblocks;
            }

            p1 += MOTIONBLOCKWIDTH;
            p2 += MOTIONBLOCKWIDTH;
            ++properties;
        } while (--i);

        p1 += inc1;
        p2 += inc2;
        ++properties;
    } while (--j);
}

static void markblocks2(MotionDetectionData* md, const uint8_t* p1,
                        const uint8_t* p2, int32_t pitch) {
    int32_t inc = MOTIONBLOCKHEIGHT * pitch - md->linewidthSSE2;
    uint8_t* properties = md->blockproperties;

    int32_t j = md->vblocks;
    do {
        int32_t i = md->hblocksSSE2;

        do {
            md->blockcompareSSE2(p1, p2, pitch, md->noiselevel);
            properties[0] = properties[1] = 0;

            if (blockcompare_result[0] >= md->threshold) {
                properties[0] = MOTION_FLAG1;
                ++md->motionblocks;
            }

            if (blockcompare_result[2] >= md->threshold) {
                properties[1] = MOTION_FLAG1;
                ++md->motionblocks;
            }

            p1 += 2 * MOTIONBLOCKWIDTH;
            p2 += 2 * MOTIONBLOCKWIDTH;
            properties += 2;
        } while (--i);

        if (md->remainderSSE2) {
            properties[0] = 0;

            if (md->blockcompare(p1, pitch, p2, pitch, md->noiselevel) >=
                md->threshold) {
                properties[0] = MOTION_FLAG1;
                ++md->motionblocks;
            }
            ++properties;
        }

        p1 += inc;
        p2 += inc;
        ++properties;
    } while (--j);
}

static void markblocks(MotionDetectionDistData* mdd, const uint8_t* p1,
                       int32_t pitch1, const uint8_t* p2, int32_t pitch2) {
    mdd->md.motionblocks = 0;
    if (((pitch1 - pitch2) | (((uintptr_t)p1) & 15) | (((uintptr_t)p2) & 15)) ==
        0) {
        markblocks2(&mdd->md, p1, p2, pitch2);
    } else {
        markblocks1(&mdd->md, p1, pitch1, p2, pitch2);
    }
    mdd->distblocks = 0;
    if (mdd->dist) {
        markneighbours(mdd);
        (mdd->processneighbours)(mdd);
    }
}

static __forceinline int32_t vertical_diff(const uint8_t* p, int32_t pitch) {
    uint32_t sad = 0;

    // Process 4 bytes wide, 8 rows, comparing vertical neighbors
    // Compares pixels at even rows with pixels at odd rows
    for (int32_t i = 0; i < 4; i++) {
        for (int32_t row = 0; row < 8; row += 2) {
            uint8_t v_even = p[row * pitch + i];
            uint8_t v_odd = p[(row + 1) * pitch + i];
            int32_t diff = (int32_t)v_even - (int32_t)v_odd;
            sad += (diff < 0) ? -diff : diff;
        }
    }

    return sad;
}

static __forceinline int32_t horizontal_diff(const uint8_t* p, int32_t pitch) {
    uint32_t sad = 0;

    // Process 8 bytes, compare two rows
    for (int32_t col = 0; col < 8; col++) {
        int32_t diff = (int32_t)p[col] - (int32_t)p[pitch + col];
        sad += (diff < 0) ? -diff : diff;
    }

    return sad;
}

static void postprocessing_grey(PostProcessingData* pp, uint8_t* dp,
                                int32_t dpitch, const uint8_t* sp,
                                int32_t spitch) {
    int32_t bottomdp = 7 * dpitch;
    int32_t bottomsp = 7 * spitch;
    int32_t dinc = MOTIONBLOCKHEIGHT * dpitch - pp->mdd.md.linewidth;
    int32_t sinc = MOTIONBLOCKHEIGHT * spitch - pp->mdd.md.linewidth;

    pp->loops = pp->restored_blocks = 0;

    int32_t to_restore;

    do {
        uint8_t* dp2 = dp;
        const uint8_t* sp2 = sp;
        uint8_t* cl = pp->mdd.md.blockproperties;
        ++pp->loops;
        to_restore = 0;

        int32_t i = pp->mdd.md.vblocks;
        do {
            int32_t j = pp->mdd.md.hblocks;

            do {
                if ((cl[0] & TO_CLEAN) != 0) {
                    vs_bitblt(dp2, dpitch, sp2, spitch, 8, 8);
                    cl[0] &= ~TO_CLEAN;

                    if (cl[-1] == 0) {
                        if (vertical_diff(dp2 + leftdp, dpitch) >
                            vertical_diff(sp2 + leftsp, spitch) +
                                pp->pthreshold) {
                            ++to_restore;
                            cl[-1] = MOTION_FLAG3;
                        }
                    }

                    if (cl[1] == 0) {
                        if (vertical_diff(dp2 + rightdp, dpitch) >
                            vertical_diff(sp2 + rightsp, spitch) +
                                pp->pthreshold) {
                            ++to_restore;
                            cl[1] = MOTION_FLAG3;
                        }
                    }

                    if (cl[pp->mdd.md.pline] == 0) {
                        if (horizontal_diff(dp2 + topdp, dpitch) >
                            horizontal_diff(sp2 + topsp, spitch) +
                                pp->pthreshold) {
                            ++to_restore;
                            cl[pp->mdd.md.pline] = MOTION_FLAG3;
                        }
                    }

                    if (cl[pp->mdd.md.nline] == 0) {
                        if (horizontal_diff(dp2 + bottomdp, dpitch) >
                            horizontal_diff(sp2 + bottomsp, spitch) +
                                pp->pthreshold) {
                            ++to_restore;
                            cl[pp->mdd.md.nline] = MOTION_FLAG3;
                        }
                    }
                }

                ++cl;
                dp2 += rightbldp;
                sp2 += rightblsp;
            } while (--j);

            cl++;
            dp2 += dinc;
            sp2 += sinc;
        } while (--i);

        pp->restored_blocks += to_restore;
    } while (to_restore != 0);
}

static __forceinline int32_t
horizontal_diff_chroma(const uint8_t* u, const uint8_t* v, int32_t pitch) {
    uint32_t sad = 0;

    // Process U and V channels, 4 bytes each
    for (int32_t col = 0; col < 4; col++) {
        int32_t diff_u = (int32_t)u[col] - (int32_t)u[pitch + col];
        sad += (diff_u < 0) ? -diff_u : diff_u;

        int32_t diff_v = (int32_t)v[col] - (int32_t)v[pitch + col];
        sad += (diff_v < 0) ? -diff_v : diff_v;
    }

    return sad;
}

static __forceinline int32_t
vertical_diff_yv12_chroma(const uint8_t* u, const uint8_t* v, int32_t pitch,
                          const uint8_t* noiselevel) {
    (void)noiselevel;
    uint32_t sad = 0;

    // Process U channel: 4 bytes wide, 4 rows, comparing vertical neighbors
    for (int32_t i = 0; i < 4; i++) {
        for (int32_t row = 0; row < 4; row += 2) {
            uint8_t v_even = u[row * pitch + i];
            uint8_t v_odd = u[(row + 1) * pitch + i];
            int32_t diff = (int32_t)v_even - (int32_t)v_odd;
            sad += (diff < 0) ? -diff : diff;
        }
    }

    // Process V channel: 4 bytes wide, 4 rows, comparing vertical neighbors
    for (int32_t i = 0; i < 4; i++) {
        for (int32_t row = 0; row < 4; row += 2) {
            uint8_t v_even = v[row * pitch + i];
            uint8_t v_odd = v[(row + 1) * pitch + i];
            int32_t diff = (int32_t)v_even - (int32_t)v_odd;
            sad += (diff < 0) ? -diff : diff;
        }
    }

    return sad;
}

static __forceinline int32_t
vertical_diff_yuy2_chroma(const uint8_t* u, const uint8_t* v, int32_t pitch,
                          const uint8_t* noiselevel) {
    (void)noiselevel;
    uint32_t sad_u = 0;
    uint32_t sad_v = 0;

    // Process U channel: 4 bytes wide, 8 rows, comparing vertical neighbors
    for (int32_t i = 0; i < 4; i++) {
        for (int32_t row = 0; row < 8; row += 2) {
            uint8_t v_even = u[row * pitch + i];
            uint8_t v_odd = u[(row + 1) * pitch + i];
            int32_t diff = (int32_t)v_even - (int32_t)v_odd;
            sad_u += (diff < 0) ? -diff : diff;
        }
    }

    // Process V channel: 4 bytes wide, 8 rows, comparing vertical neighbors
    for (int32_t i = 0; i < 4; i++) {
        for (int32_t row = 0; row < 8; row += 2) {
            uint8_t v_even = v[row * pitch + i];
            uint8_t v_odd = v[(row + 1) * pitch + i];
            int32_t diff = (int32_t)v_even - (int32_t)v_odd;
            sad_v += (diff < 0) ? -diff : diff;
        }
    }

    // Average the two SAD values
    return (sad_u + sad_v) / 2;
}

static __forceinline void colorise(uint8_t* u, uint8_t* v, int32_t pitch,
                                   int32_t height, uint32_t ucolor,
                                   uint32_t vcolor) {
    int32_t i = height;

    do {
        *(uint32_t*)u = ucolor;
        *(uint32_t*)v = vcolor;
        u += pitch;
        v += pitch;
    } while (--i);
}

static void postprocessing(PostProcessingData* pp, uint8_t* dp, int32_t dpitch,
                           uint8_t* dpU, uint8_t* dpV, int32_t dpitchUV,
                           const uint8_t* sp, int32_t spitch,
                           const uint8_t* spU, const uint8_t* spV,
                           int32_t spitchUV, int32_t heightUV) {
    int32_t bottomdp = 7 * dpitch;
    int32_t bottomsp = 7 * spitch;
    int32_t Cbottomdp = pp->chromaheightm * dpitchUV;
    int32_t Cbottomsp = pp->chromaheightm * spitchUV;
    int32_t dinc = MOTIONBLOCKHEIGHT * dpitch - pp->mdd.md.linewidth;
    int32_t sinc = MOTIONBLOCKHEIGHT * spitch - pp->mdd.md.linewidth;
    int32_t dincUV = pp->chromaheight * dpitchUV - pp->linewidthUV;
    int32_t sincUV = pp->chromaheight * spitchUV - pp->linewidthUV;

    pp->loops = pp->restored_blocks = 0;

    int32_t to_restore;

    do {
        uint8_t* dp2 = dp;
        uint8_t* dpU2 = dpU;
        uint8_t* dpV2 = dpV;
        const uint8_t* sp2 = sp;
        const uint8_t* spU2 = spU;
        const uint8_t* spV2 = spV;
        uint8_t* cl = pp->mdd.md.blockproperties;
        ++pp->loops;
        to_restore = 0;

        int32_t i = pp->mdd.md.vblocks;

        do {
            int32_t j = pp->mdd.md.hblocks;

            do {
                if ((cl[0] & TO_CLEAN) != 0) {
                    vs_bitblt(dp2, dpitch, sp2, spitch, 8, 8);
                    vs_bitblt(dpU2, dpitchUV, spU2, spitchUV, 8, heightUV);
                    vs_bitblt(dpV2, dpitchUV, spV2, spitchUV, 8, heightUV);
                    cl[0] &= ~TO_CLEAN;

                    if (cl[-1] == 0) {
                        if ((vertical_diff(dp2 + leftdp, dpitch) >
                             vertical_diff(sp2 + leftsp, spitch) +
                                 pp->pthreshold) ||
                            (pp->vertical_diff_chroma(dpU2 + Cleftdp,
                                                      dpV2 + Cleftdp, dpitchUV,
                                                      pp->mdd.md.noiselevel) >
                             pp->vertical_diff_chroma(spU2 + Cleftsp,
                                                      spV2 + Cleftsp, spitchUV,
                                                      pp->mdd.md.noiselevel) +
                                 pp->cthreshold)) {
                            ++to_restore;
                            cl[-1] = MOTION_FLAG3;
                        }
                    }

                    if (cl[1] == 0) {
                        if ((vertical_diff(dp2 + rightdp, dpitch) >
                             vertical_diff(sp2 + rightsp, spitch) +
                                 pp->pthreshold) ||
                            (pp->vertical_diff_chroma(dpU2 + Crightdp,
                                                      dpV2 + Crightdp, dpitchUV,
                                                      pp->mdd.md.noiselevel) >
                             pp->vertical_diff_chroma(spU2 + Crightsp,
                                                      spV2 + Crightsp, spitchUV,
                                                      pp->mdd.md.noiselevel) +
                                 pp->cthreshold)) {
                            ++to_restore;
                            cl[1] = MOTION_FLAG3;
                        }
                    }

                    if (cl[pp->mdd.md.pline] == 0) {
                        if ((horizontal_diff(dp2 + topdp, dpitch) >
                             horizontal_diff(sp2 + topsp, spitch) +
                                 pp->pthreshold) ||
                            (horizontal_diff_chroma(dpU2 + Ctopdp,
                                                    dpV2 + Ctopdp, dpitchUV) >
                             horizontal_diff_chroma(spU2 + Ctopsp,
                                                    spV2 + Ctopsp, spitchUV) +
                                 pp->cthreshold)) {
                            ++to_restore;
                            cl[pp->mdd.md.pline] = MOTION_FLAG3;
                        }
                    }

                    if (cl[pp->mdd.md.nline] == 0) {
                        if ((horizontal_diff(dp2 + bottomdp, dpitch) >
                             horizontal_diff(sp2 + bottomsp, spitch) +
                                 pp->pthreshold) ||
                            (horizontal_diff_chroma(
                                 dpU2 + Cbottomdp, dpV2 + Cbottomdp, dpitchUV) >
                             horizontal_diff_chroma(
                                 spU2 + Cbottomsp, spV2 + Cbottomsp, spitchUV) +
                                 pp->cthreshold)) {
                            ++to_restore;
                            cl[pp->mdd.md.nline] = MOTION_FLAG3;
                        }
                    }
                }

                ++cl;
                dp2 += rightbldp;
                sp2 += rightblsp;
                dpU2 += Crightbldp;
                spU2 += Crightbldp;
                dpV2 += Crightbldp;
                spV2 += Crightbldp;
            } while (--j);

            cl++;
            dp2 += dinc;
            sp2 += sinc;
            dpU2 += dincUV;
            spU2 += sincUV;
            dpV2 += dincUV;
            spV2 += sincUV;
        } while (--i);

        pp->restored_blocks += to_restore;
    } while (to_restore != 0);
}

static void show_motion(PostProcessingData* pp, uint8_t* u, uint8_t* v,
                        int32_t pitchUV) {
    int32_t inc = pp->chromaheight * pitchUV - pp->linewidthUV;

    uint8_t* properties = pp->mdd.md.blockproperties;

    int32_t j = pp->mdd.md.vblocks;

    do {
        int32_t i = pp->mdd.md.hblocks;

        do {
            if (properties[0]) {
                uint32_t u_color = u_ncolor;
                uint32_t v_color = v_ncolor;
                if ((properties[0] & MOTION_FLAG) != 0) {
                    u_color = u_mcolor;
                    v_color = v_mcolor;
                }
                if ((properties[0] & MOTION_FLAGP) != 0) {
                    u_color = u_pcolor;
                    v_color = v_pcolor;
                }
                colorise(u, v, pitchUV, pp->chromaheight, u_color, v_color);
            }

            u += MOTIONBLOCKWIDTH / 2;
            v += MOTIONBLOCKWIDTH / 2;
            ++properties;
        } while (--i);

        u += inc;
        v += inc;
        ++properties;
    } while (--j);
}

static void FillMotionDetection(MotionDetectionData* md, const VSMap* in,
                                VSMap* out, const VSAPI* vsapi,
                                const VSVideoInfo* vi) {
    int32_t err;
    int32_t noise = (int32_t)vsapi->propGetInt(in, "noise", 0, &err);
    if (err) {
        noise = 0;
    }

    int32_t noisy = (int32_t)vsapi->propGetInt(in, "noisy", 0, &err);
    if (err) {
        noisy = -1;
    }

    int32_t width = vi->width;
    int32_t height = vi->height;

    md->hblocks = (md->linewidth = width) / MOTIONBLOCKWIDTH;
    md->vblocks = height / MOTIONBLOCKHEIGHT;

    md->linewidthSSE2 = md->linewidth;
    md->hblocksSSE2 = md->hblocks / 2;
    if ((md->remainderSSE2 = (md->hblocks & 1)) != 0) {
        md->linewidthSSE2 -= MOTIONBLOCKWIDTH;
    }
    if ((md->hblocksSSE2 == 0) || (md->vblocks == 0)) {
        vsapi->setError(out,
                        "RemoveDirt: width or height of the clip too small");
    }

    md->blockcompareSSE2 = SADcompareSSE2;
    md->blockcompare = SADcompare;

    if (noise > 0) {
        md->blockcompareSSE2 = NSADcompareSSE2;
        md->blockcompare = NSADcompare;
        memset(md->noiselevel, noise, 16);

        if (noisy >= 0) {
            md->blockcompareSSE2 = ExcessPixelsSSE2;
            md->blockcompare = ExcessPixels;
            md->threshold = noisy;
        }
    }
    int32_t size;
    md->blockproperties_addr =
        new uint8_t[size = (md->nline = md->hblocks + 1) * (md->vblocks + 2)];
    md->blockproperties = md->blockproperties_addr + md->nline;
    md->pline = -md->nline;
    memset(md->blockproperties_addr, BMARGIN, size);
}

static void FillMotionDetectionDist(MotionDetectionDistData* mdd,
                                    const VSMap* in, VSMap* out,
                                    const VSAPI* vsapi, const VSVideoInfo* vi) {
    FillMotionDetection(&mdd->md, in, out, vsapi, vi);

    int32_t err;
    uint32_t dmode = (int32_t)vsapi->propGetInt(in, "dmode", 0, &err);
    if (err) {
        dmode = 0;
    }

    uint32_t tolerance = (int32_t)vsapi->propGetInt(in, "", 0, &err);
    if (err) {
        tolerance = 12;
    }

    int32_t dist = (int32_t)vsapi->propGetInt(in, "dist", 0, &err);
    if (err) {
        dist = 1;
    }

    static void (*neighbourproc[3])(MotionDetectionDistData*) = {
        processneighbours1, processneighbours2, processneighbours3};

    mdd->blocks = mdd->md.hblocks * mdd->md.vblocks;
    mdd->isum = new uint32_t[mdd->blocks];

    if (dmode >= 3) {
        dmode = 0;
    }

    if (tolerance > 100) {
        tolerance = 100;
    } else if (tolerance == 0) {
        if (dmode == 2) {
            dist = 0;
        }
        dmode = 0;
    }

    mdd->processneighbours = neighbourproc[dmode];

    mdd->dist = dist;
    mdd->dist1 = mdd->dist + 1;
    mdd->dist2 = mdd->dist1 + 1;

    mdd->isumline = mdd->md.hblocks * sizeof(uint32_t);
    uint32_t d = mdd->dist1 + mdd->dist;
    tolerance = (d * d * tolerance * MOTION_FLAG1) / 100;
    mdd->hint32_terior = mdd->md.hblocks - d;
    mdd->vint32_terior = mdd->md.vblocks - d;
    mdd->colinc = 1 - (mdd->md.vblocks * mdd->md.nline);
    mdd->isuminc1 =
        (1 - (mdd->vint32_terior + dist) * mdd->md.hblocks) * sizeof(uint32_t);
    mdd->isuminc2 = (1 - mdd->md.vblocks * mdd->md.hblocks) * sizeof(uint32_t);
}

static void FillPostProcessing(PostProcessingData* pp, const VSMap* in,
                               VSMap* out, const VSAPI* vsapi,
                               const VSVideoInfo* vi) {
    int32_t err;
    pp->pthreshold = (int32_t)vsapi->propGetInt(in, "pthreshold", 0, &err);
    if (err) {
        pp->pthreshold = 10;
    }

    pp->cthreshold = (int32_t)vsapi->propGetInt(in, "cthreshold", 0, &err);
    if (err) {
        pp->cthreshold = 10;
    }

    FillMotionDetectionDist(&pp->mdd, in, out, vsapi, vi);

    pp->vertical_diff_chroma = vertical_diff_yv12_chroma;
    pp->linewidthUV = pp->mdd.md.linewidth / 2;
    pp->chromaheight = MOTIONBLOCKHEIGHT / 2;

    if (vi->format->id == pfYUV422P8) {
        pp->chromaheight *= 2;
        pp->vertical_diff_chroma = vertical_diff_yuy2_chroma;
    }
    pp->chromaheightm = pp->chromaheight - 1;
}

void FillRemoveDirt(RemoveDirtData* rd, const VSMap* in, VSMap* out,
                    const VSAPI* vsapi, const VSVideoInfo* vi) {
    int32_t err;
    rd->grey = vsapi->propGetInt(in, "grey", 0, &err) == 1;
    if (err) {
        rd->grey = false;
    }

    rd->show = vsapi->propGetInt(in, "show", 0, &err) == 1;
    if (err) {
        rd->show = false;
    }

    FillPostProcessing(&rd->pp, in, out, vsapi, vi);
}

int32_t RemoveDirtProcessFrame(RemoveDirtData* rd, VSFrameRef* dest,
                               const VSFrameRef* src,
                               const VSFrameRef* previous,
                               const VSFrameRef* next, const VSAPI* vsapi,
                               const VSVideoInfo* vi) {
    const uint8_t* nextY = vsapi->getReadPtr(next, 0);
    int32_t nextPitchY = vsapi->getStride(next, 0);
    markblocks(&rd->pp.mdd, vsapi->getReadPtr(previous, 0),
               vsapi->getStride(previous, 0), nextY, nextPitchY);

    uint8_t* destY = vsapi->getWritePtr(dest, 0);
    uint8_t* destU = vsapi->getWritePtr(dest, 1);
    uint8_t* destV = vsapi->getWritePtr(dest, 2);
    int32_t destPitchY = vsapi->getStride(dest, 0);
    int32_t destPitchUV = vsapi->getStride(dest, 1);
    const uint8_t* srcY = vsapi->getReadPtr(src, 0);
    const uint8_t* srcU = vsapi->getReadPtr(src, 1);
    const uint8_t* srcV = vsapi->getReadPtr(src, 2);
    int32_t srcPitchY = vsapi->getStride(src, 0);
    int32_t srcPitchUV = vsapi->getStride(src, 1);
    int32_t heightUV = vi->format->id == pfYUV422P8 ? 8 : 4;

    if (rd->grey) {
        postprocessing_grey(&rd->pp, destY, destPitchY, srcY, srcPitchY);
    } else {
        postprocessing(&rd->pp, destY, destPitchY, destU, destV, destPitchUV,
                       srcY, srcPitchY, srcU, srcV, srcPitchUV, heightUV);
    }

    if (rd->show) {
        show_motion(&rd->pp, destU, destV, destPitchUV);
    }

    return rd->pp.restored_blocks + rd->pp.mdd.distblocks +
           rd->pp.mdd.md.motionblocks;
}
