/*******************************************************************************
 * Size: 14 px
 * Bpp: 4
 * Opts: --bpp 4 --size 14 --font /home/george/SquareLine/assets/BebasNeue-Regular.ttf -o /home/george/SquareLine/assets/ui_font_bebas_14.c --format lvgl -r 0x20-0x7f --no-compress --no-prefilter
 ******************************************************************************/

#include "../ui.h"

#ifndef UI_FONT_BEBAS_14
#define UI_FONT_BEBAS_14 1
#endif

#if UI_FONT_BEBAS_14

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */

    /* U+0021 "!" */
    0x4f, 0x34, 0xf3, 0x4f, 0x34, 0xf3, 0x4f, 0x33,
    0xf2, 0x2f, 0x10, 0x40, 0x28, 0x14, 0xf3,

    /* U+0022 "\"" */
    0x6f, 0x5f, 0x14, 0xe3, 0xf0, 0x3c, 0x1e, 0x0,
    0x20, 0x20,

    /* U+0023 "#" */
    0x6, 0xc3, 0xf0, 0x8, 0xa5, 0xe0, 0x5f, 0xff,
    0xf7, 0xc, 0x89, 0xb0, 0xd, 0x5a, 0x90, 0xe,
    0x4b, 0x80, 0xaf, 0xff, 0xf1, 0x3f, 0x3e, 0x60,
    0x3f, 0xf, 0x30, 0x5e, 0x1f, 0x10,

    /* U+0024 "$" */
    0x0, 0x21, 0x0, 0x1, 0xe9, 0x0, 0x1e, 0xff,
    0xa0, 0x6f, 0x48, 0xf0, 0x5f, 0x41, 0x30, 0x1e,
    0xe2, 0x0, 0x2, 0xee, 0x30, 0x0, 0x1e, 0xd0,
    0x7e, 0x7, 0xf0, 0x5f, 0x8c, 0xf0, 0xb, 0xff,
    0x60, 0x0, 0xd7, 0x0,

    /* U+0025 "%" */
    0x2d, 0xd2, 0xa, 0x50, 0x78, 0x86, 0x1e, 0x0,
    0x87, 0x87, 0x69, 0x0, 0x87, 0x87, 0xc2, 0x0,
    0x78, 0x99, 0xc0, 0x0, 0x2d, 0xca, 0x7b, 0xe5,
    0x0, 0xe, 0x4c, 0x4b, 0x0, 0x5a, 0x3c, 0x3c,
    0x0, 0xb4, 0x3c, 0x3b, 0x1, 0xe0, 0xc, 0xd6,

    /* U+0026 "&" */
    0x9, 0xff, 0x90, 0x4f, 0x96, 0x30, 0x6f, 0x11,
    0x30, 0x4f, 0x67, 0xf2, 0xb, 0xff, 0xf9, 0x4f,
    0x89, 0xf4, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf2,
    0x6f, 0x6c, 0xf2, 0x1c, 0xea, 0xf3,

    /* U+0027 "'" */
    0x6f, 0x4, 0xe0, 0x3c, 0x0, 0x20,

    /* U+0028 "(" */
    0x0, 0x0, 0xa, 0xf6, 0x2f, 0x81, 0x4f, 0x40,
    0x4f, 0x40, 0x4f, 0x40, 0x4f, 0x40, 0x4f, 0x40,
    0x4f, 0x40, 0x4f, 0x40, 0x3f, 0x60, 0xd, 0xf6,
    0x0, 0x21,

    /* U+0029 ")" */
    0x0, 0x0, 0x8f, 0x80, 0x1a, 0xf0, 0x6, 0xf1,
    0x6, 0xf2, 0x6, 0xf2, 0x6, 0xf2, 0x6, 0xf2,
    0x6, 0xf2, 0x6, 0xf2, 0x9, 0xf1, 0x8f, 0xb0,
    0x12, 0x0,

    /* U+002A "*" */
    0x0, 0x98, 0x0, 0x54, 0x76, 0x54, 0x6b, 0xee,
    0xb5, 0x3, 0xcd, 0x20, 0xd, 0x46, 0xc0, 0x0,
    0x0, 0x0,

    /* U+002B "+" */
    0x0, 0xb5, 0x0, 0x0, 0xc5, 0x0, 0x9f, 0xff,
    0xf3, 0x12, 0xc6, 0x20, 0x0, 0xc5, 0x0,

    /* U+002C "," */
    0x38, 0x6, 0xf0, 0x1b, 0x2, 0x30,

    /* U+002D "-" */
    0x12, 0x20, 0xaf, 0xf7, 0x34, 0x42,

    /* U+002E "." */
    0x38, 0x6, 0xf1,

    /* U+002F "/" */
    0x0, 0x3, 0xf2, 0x0, 0x9, 0xb0, 0x0, 0xf,
    0x50, 0x0, 0x5e, 0x0, 0x0, 0xc9, 0x0, 0x2,
    0xf3, 0x0, 0x8, 0xc0, 0x0, 0xe, 0x60, 0x0,
    0x4f, 0x10, 0x0, 0xaa, 0x0, 0x0,

    /* U+0030 "0" */
    0xa, 0xfe, 0x50, 0x5f, 0x9c, 0xf0, 0x8f, 0x6,
    0xf1, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f,
    0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf1,
    0x5f, 0x9c, 0xf0, 0xa, 0xfe, 0x50,

    /* U+0031 "1" */
    0x3, 0xf4, 0xdf, 0xf4, 0x15, 0xf4, 0x4, 0xf4,
    0x4, 0xf4, 0x4, 0xf4, 0x4, 0xf4, 0x4, 0xf4,
    0x4, 0xf4, 0x4, 0xf4,

    /* U+0032 "2" */
    0x9, 0xfe, 0x50, 0x4f, 0x9c, 0xf0, 0x7f, 0x6,
    0xf1, 0x37, 0x9, 0xf0, 0x0, 0x2f, 0x80, 0x1,
    0xdc, 0x0, 0xb, 0xe1, 0x0, 0x3f, 0x50, 0x0,
    0x6f, 0x76, 0x60, 0x7f, 0xff, 0xf0,

    /* U+0033 "3" */
    0xa, 0xfd, 0x40, 0x5f, 0x8d, 0xd0, 0x6c, 0x8,
    0xf0, 0x0, 0x2b, 0xd0, 0x0, 0xff, 0x40, 0x0,
    0x5d, 0xd0, 0x24, 0x8, 0xf0, 0x8e, 0x7, 0xf0,
    0x5f, 0x8d, 0xe0, 0xa, 0xfd, 0x40,

    /* U+0034 "4" */
    0x0, 0x3f, 0xb0, 0x0, 0x9f, 0xb0, 0x0, 0xff,
    0xb0, 0x6, 0xed, 0xb0, 0xd, 0x8d, 0xb0, 0x3f,
    0x2d, 0xb0, 0xac, 0x1d, 0xb0, 0xcf, 0xff, 0xf6,
    0x34, 0x4e, 0xc2, 0x0, 0xd, 0xb0,

    /* U+0035 "5" */
    0x3f, 0xff, 0xe0, 0x4f, 0x86, 0x50, 0x5f, 0x10,
    0x0, 0x5f, 0xcf, 0x90, 0x6f, 0x7c, 0xf0, 0x25,
    0x6, 0xf1, 0x24, 0x6, 0xf2, 0x7f, 0x7, 0xf1,
    0x4f, 0x9d, 0xe0, 0x9, 0xfe, 0x40,

    /* U+0036 "6" */
    0x9, 0xfe, 0x50, 0x5f, 0x9b, 0xe0, 0x7f, 0x13,
    0xc1, 0x8f, 0x0, 0x0, 0x8f, 0xbf, 0xb0, 0x8f,
    0x8b, 0xf1, 0x8f, 0x5, 0xf2, 0x7f, 0x5, 0xf2,
    0x5f, 0x9c, 0xf0, 0x9, 0xfe, 0x60,

    /* U+0037 "7" */
    0x8f, 0xff, 0xf1, 0x36, 0x6b, 0xf0, 0x0, 0xb,
    0xc0, 0x0, 0xf, 0x70, 0x0, 0x4f, 0x30, 0x0,
    0x9e, 0x0, 0x0, 0xea, 0x0, 0x2, 0xf6, 0x0,
    0x7, 0xf1, 0x0, 0xb, 0xd0, 0x0,

    /* U+0038 "8" */
    0xa, 0xfe, 0x50, 0x6f, 0x9c, 0xf0, 0x8f, 0x6,
    0xf2, 0x6f, 0x4a, 0xf0, 0xd, 0xff, 0x70, 0x6f,
    0x6b, 0xf0, 0x9f, 0x5, 0xf2, 0x9f, 0x5, 0xf2,
    0x6f, 0x8c, 0xf0, 0xa, 0xfe, 0x50,

    /* U+0039 "9" */
    0xa, 0xfd, 0x40, 0x6f, 0x8d, 0xe0, 0x9f, 0x7,
    0xf1, 0x9f, 0x7, 0xf1, 0x8f, 0x4a, 0xf1, 0x4f,
    0xfd, 0xf1, 0x2, 0x37, 0xf1, 0x5a, 0x7, 0xf1,
    0x5f, 0x8d, 0xe0, 0xa, 0xfd, 0x40,

    /* U+003A ":" */
    0x2, 0x6, 0xf1, 0x25, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x38, 0x6, 0xf1,

    /* U+003B ";" */
    0x2, 0x6, 0xf1, 0x25, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x38, 0x6, 0xf0, 0x2b, 0x3, 0x20,

    /* U+003C "<" */
    0x0, 0x0, 0x10, 0x4, 0xbc, 0x3d, 0xc6, 0x4,
    0xfa, 0x30, 0x0, 0x7d, 0xb0, 0x0, 0x3,

    /* U+003D "=" */
    0x7f, 0xff, 0xf1, 0x12, 0x22, 0x20, 0x7f, 0xff,
    0xf1, 0x12, 0x22, 0x20,

    /* U+003E ">" */
    0x10, 0x0, 0x0, 0x3e, 0x82, 0x0, 0x2, 0x8e,
    0xb0, 0x0, 0x6d, 0xd0, 0x2e, 0xb4, 0x0, 0x12,
    0x0, 0x0,

    /* U+003F "?" */
    0x1c, 0xfc, 0x29, 0xe8, 0xfa, 0xbb, 0xc, 0xc5,
    0x50, 0xe9, 0x0, 0x7f, 0x20, 0x1f, 0x70, 0x4,
    0xf1, 0x0, 0x26, 0x0, 0x3, 0x80, 0x0, 0x7f,
    0x0,

    /* U+0040 "@" */
    0x0, 0x19, 0xdf, 0xe9, 0x10, 0x2, 0xeb, 0x53,
    0x5d, 0xb0, 0xb, 0xb0, 0x0, 0x12, 0xf2, 0x3f,
    0x27, 0xfb, 0xf3, 0xd5, 0x7d, 0xe, 0xa9, 0xf1,
    0xc5, 0x9b, 0x1f, 0x45, 0xf0, 0xe3, 0xab, 0x2f,
    0x59, 0xe5, 0xe0, 0x8d, 0xe, 0xfc, 0xff, 0x40,
    0x4f, 0x31, 0x10, 0x31, 0x0, 0xc, 0xe4, 0x11,
    0x49, 0x0, 0x1, 0xaf, 0xff, 0xe6, 0x0, 0x0,
    0x0, 0x21, 0x0, 0x0,

    /* U+0041 "A" */
    0x5, 0xfe, 0x0, 0x7, 0xff, 0x10, 0xa, 0xcf,
    0x30, 0xc, 0x8f, 0x60, 0xf, 0x6e, 0x90, 0x1f,
    0x4c, 0xb0, 0x4f, 0x2a, 0xe0, 0x6f, 0xff, 0xf0,
    0x9d, 0x47, 0xf3, 0xca, 0x2, 0xf5,

    /* U+0042 "B" */
    0x6f, 0xfe, 0x70, 0x6f, 0x7c, 0xf1, 0x6f, 0x15,
    0xf2, 0x6f, 0x39, 0xf0, 0x6f, 0xff, 0x80, 0x6f,
    0x6a, 0xf2, 0x6f, 0x13, 0xf4, 0x6f, 0x13, 0xf5,
    0x6f, 0x7a, 0xf2, 0x6f, 0xfe, 0x70,

    /* U+0043 "C" */
    0xa, 0xfd, 0x40, 0x5f, 0x9d, 0xd0, 0x8f, 0x7,
    0xf0, 0x8f, 0x2, 0x60, 0x8f, 0x0, 0x0, 0x8f,
    0x0, 0x0, 0x8f, 0x5, 0xc0, 0x8f, 0x7, 0xf0,
    0x5f, 0x9d, 0xd0, 0xa, 0xfd, 0x40,

    /* U+0044 "D" */
    0x6f, 0xfe, 0x60, 0x6f, 0x7b, 0xf1, 0x6f, 0x15,
    0xf3, 0x6f, 0x15, 0xf3, 0x6f, 0x15, 0xf3, 0x6f,
    0x15, 0xf3, 0x6f, 0x15, 0xf3, 0x6f, 0x15, 0xf3,
    0x6f, 0x7b, 0xf1, 0x6f, 0xfe, 0x60,

    /* U+0045 "E" */
    0x6f, 0xff, 0xc6, 0xf7, 0x65, 0x6f, 0x10, 0x6,
    0xf3, 0x10, 0x6f, 0xff, 0x36, 0xf5, 0x41, 0x6f,
    0x10, 0x6, 0xf1, 0x0, 0x6f, 0x76, 0x56, 0xff,
    0xfc,

    /* U+0046 "F" */
    0x6f, 0xff, 0xa6, 0xf7, 0x64, 0x6f, 0x10, 0x6,
    0xf3, 0x10, 0x6f, 0xff, 0x16, 0xf5, 0x40, 0x6f,
    0x10, 0x6, 0xf1, 0x0, 0x6f, 0x10, 0x6, 0xf1,
    0x0,

    /* U+0047 "G" */
    0xa, 0xfe, 0x40, 0x5f, 0x9c, 0xe0, 0x8f, 0x6,
    0xf0, 0x8f, 0x1, 0x10, 0x8f, 0x2f, 0xf0, 0x8f,
    0x9, 0xf0, 0x8f, 0x6, 0xf0, 0x8f, 0x6, 0xf0,
    0x5f, 0x9c, 0xd0, 0xa, 0xfd, 0x30,

    /* U+0048 "H" */
    0x6f, 0x13, 0xf5, 0x6f, 0x13, 0xf5, 0x6f, 0x13,
    0xf5, 0x6f, 0x35, 0xf5, 0x6f, 0xff, 0xf5, 0x6f,
    0x57, 0xf5, 0x6f, 0x13, 0xf5, 0x6f, 0x13, 0xf5,
    0x6f, 0x13, 0xf5, 0x6f, 0x13, 0xf5,

    /* U+0049 "I" */
    0x6f, 0x16, 0xf1, 0x6f, 0x16, 0xf1, 0x6f, 0x16,
    0xf1, 0x6f, 0x16, 0xf1, 0x6f, 0x16, 0xf1,

    /* U+004A "J" */
    0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2,
    0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2,
    0x5c, 0xf0, 0xce, 0x60,

    /* U+004B "K" */
    0x6f, 0x12, 0xf5, 0x6f, 0x19, 0xd0, 0x6f, 0x3f,
    0x50, 0x6f, 0xae, 0x0, 0x6f, 0xff, 0x0, 0x6f,
    0xef, 0x50, 0x6f, 0x4f, 0x90, 0x6f, 0x1b, 0xe0,
    0x6f, 0x16, 0xf3, 0x6f, 0x11, 0xf7,

    /* U+004C "L" */
    0x6f, 0x10, 0x6, 0xf1, 0x0, 0x6f, 0x10, 0x6,
    0xf1, 0x0, 0x6f, 0x10, 0x6, 0xf1, 0x0, 0x6f,
    0x10, 0x6, 0xf1, 0x0, 0x6f, 0x76, 0x46, 0xff,
    0xfa,

    /* U+004D "M" */
    0x6f, 0xd0, 0x4f, 0xf6, 0xff, 0x7, 0xff, 0x6f,
    0xf1, 0x9f, 0xf6, 0xee, 0x3b, 0xcf, 0x6e, 0xb6,
    0xda, 0xf6, 0xe9, 0x8f, 0x8f, 0x6e, 0x7c, 0xd8,
    0xf6, 0xe4, 0xfb, 0x8f, 0x6e, 0x2f, 0x98, 0xf6,
    0xe0, 0xf6, 0x8f,

    /* U+004E "N" */
    0x6f, 0x90, 0xf6, 0x6f, 0xe0, 0xf6, 0x6f, 0xf2,
    0xf6, 0x6f, 0xd6, 0xf6, 0x6f, 0x9a, 0xf6, 0x6f,
    0x5e, 0xf6, 0x6f, 0x1f, 0xf6, 0x6f, 0xc, 0xf6,
    0x6f, 0x8, 0xf6, 0x6f, 0x4, 0xf6,

    /* U+004F "O" */
    0xa, 0xfe, 0x50, 0x5f, 0x9c, 0xf0, 0x8f, 0x6,
    0xf1, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f,
    0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf1,
    0x5f, 0x9c, 0xf0, 0xa, 0xfe, 0x50,

    /* U+0050 "P" */
    0x6f, 0xfe, 0x50, 0x6f, 0x7c, 0xe0, 0x6f, 0x16,
    0xf1, 0x6f, 0x16, 0xf2, 0x6f, 0x3a, 0xf0, 0x6f,
    0xff, 0x90, 0x6f, 0x52, 0x0, 0x6f, 0x10, 0x0,
    0x6f, 0x10, 0x0, 0x6f, 0x10, 0x0,

    /* U+0051 "Q" */
    0xa, 0xfe, 0x50, 0x5f, 0x9c, 0xf0, 0x8f, 0x6,
    0xf2, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f,
    0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf1,
    0x5f, 0x9c, 0xe0, 0xa, 0xfd, 0xf8, 0x0, 0x0,
    0x22,

    /* U+0052 "R" */
    0x6f, 0xfe, 0x70, 0x6f, 0x7c, 0xf0, 0x6f, 0x16,
    0xf2, 0x6f, 0x3a, 0xf0, 0x6f, 0xff, 0x70, 0x6f,
    0x6c, 0xf0, 0x6f, 0x16, 0xf2, 0x6f, 0x16, 0xf2,
    0x6f, 0x16, 0xf2, 0x6f, 0x15, 0xf3,

    /* U+0053 "S" */
    0x1b, 0xfd, 0x27, 0xf8, 0xeb, 0xae, 0x8, 0xb7,
    0xf5, 0x0, 0xc, 0xf4, 0x0, 0x1c, 0xf3, 0x0,
    0x1e, 0xba, 0xc0, 0xad, 0x8f, 0x8e, 0xb1, 0xbf,
    0xc2,

    /* U+0054 "T" */
    0xdf, 0xff, 0xe5, 0x8f, 0x95, 0x3, 0xf5, 0x0,
    0x3f, 0x50, 0x3, 0xf5, 0x0, 0x3f, 0x50, 0x3,
    0xf5, 0x0, 0x3f, 0x50, 0x3, 0xf5, 0x0, 0x3f,
    0x50,

    /* U+0055 "U" */
    0x7f, 0x16, 0xf1, 0x7f, 0x16, 0xf1, 0x7f, 0x16,
    0xf1, 0x7f, 0x16, 0xf1, 0x7f, 0x16, 0xf1, 0x7f,
    0x16, 0xf1, 0x7f, 0x16, 0xf1, 0x7f, 0x16, 0xf1,
    0x5f, 0x9c, 0xe0, 0x9, 0xfe, 0x40,

    /* U+0056 "V" */
    0xcc, 0x4, 0xf1, 0x9e, 0x6, 0xf0, 0x7f, 0x8,
    0xd0, 0x5f, 0x2b, 0xa0, 0x2f, 0x4d, 0x80, 0xf,
    0x7f, 0x50, 0xd, 0xaf, 0x30, 0xb, 0xef, 0x10,
    0x9, 0xfe, 0x0, 0x6, 0xfc, 0x0,

    /* U+0057 "W" */
    0xcb, 0xe, 0xd0, 0xc8, 0xad, 0xf, 0xf0, 0xd7,
    0x8e, 0x1f, 0xf0, 0xf5, 0x7f, 0x3f, 0xf3, 0xf3,
    0x5f, 0x6d, 0xe6, 0xf2, 0x4f, 0x9b, 0xc9, 0xf0,
    0x2f, 0xc9, 0xbc, 0xf0, 0xf, 0xe8, 0x9e, 0xd0,
    0xf, 0xf6, 0x7f, 0xb0, 0xd, 0xf4, 0x6f, 0xa0,

    /* U+0058 "X" */
    0x9f, 0x2, 0xf4, 0x3f, 0x58, 0xe0, 0xe, 0xad,
    0x90, 0x8, 0xff, 0x30, 0x3, 0xfe, 0x0, 0x4,
    0xfe, 0x0, 0xa, 0xef, 0x40, 0xf, 0x7e, 0xa0,
    0x5f, 0x19, 0xf0, 0xac, 0x4, 0xf5,

    /* U+0059 "Y" */
    0xbe, 0x4, 0xf3, 0x6f, 0x38, 0xe0, 0x1f, 0x7d,
    0x90, 0xb, 0xdf, 0x40, 0x6, 0xfe, 0x0, 0x1,
    0xf9, 0x0, 0x0, 0xf8, 0x0, 0x0, 0xf8, 0x0,
    0x0, 0xf8, 0x0, 0x0, 0xf8, 0x0,

    /* U+005A "Z" */
    0x9f, 0xff, 0xc3, 0x66, 0xfb, 0x0, 0x4f, 0x50,
    0xb, 0xe0, 0x1, 0xf8, 0x0, 0x7f, 0x20, 0xe,
    0xc0, 0x4, 0xf5, 0x0, 0xaf, 0x66, 0x5b, 0xff,
    0xfc,

    /* U+005B "[" */
    0x0, 0x0, 0x2f, 0xf6, 0x2f, 0x71, 0x2f, 0x50,
    0x2f, 0x50, 0x2f, 0x50, 0x2f, 0x50, 0x2f, 0x50,
    0x2f, 0x50, 0x2f, 0x50, 0x2f, 0x60, 0x2f, 0xf6,
    0x3, 0x31,

    /* U+005C "\\" */
    0xaa, 0x0, 0x0, 0x4f, 0x10, 0x0, 0xe, 0x60,
    0x0, 0x8, 0xc0, 0x0, 0x2, 0xf2, 0x0, 0x0,
    0xc9, 0x0, 0x0, 0x5e, 0x0, 0x0, 0xf, 0x50,
    0x0, 0x9, 0xb0, 0x0, 0x3, 0xf2,

    /* U+005D "]" */
    0x0, 0x0, 0x8f, 0xf0, 0x19, 0xf0, 0x8, 0xf0,
    0x8, 0xf0, 0x8, 0xf0, 0x8, 0xf0, 0x8, 0xf0,
    0x8, 0xf0, 0x8, 0xf0, 0x8, 0xf0, 0x8f, 0xf0,
    0x13, 0x30,

    /* U+005E "^" */
    0x0, 0xe8, 0x0, 0x7, 0xdf, 0x10, 0xe, 0x4b,
    0x80, 0x7c, 0x3, 0xf1,

    /* U+005F "_" */
    0xff, 0xff, 0x32, 0x22, 0x20,

    /* U+0060 "`" */
    0x9, 0x50, 0x4, 0xe1,

    /* U+0061 "a" */
    0x5, 0xfe, 0x0, 0x7, 0xff, 0x10, 0xa, 0xcf,
    0x30, 0xc, 0x8f, 0x60, 0xf, 0x6e, 0x90, 0x1f,
    0x4c, 0xb0, 0x4f, 0x2a, 0xe0, 0x6f, 0xff, 0xf0,
    0x9d, 0x47, 0xf3, 0xca, 0x2, 0xf5,

    /* U+0062 "b" */
    0x6f, 0xfe, 0x70, 0x6f, 0x7c, 0xf1, 0x6f, 0x15,
    0xf2, 0x6f, 0x39, 0xf0, 0x6f, 0xff, 0x80, 0x6f,
    0x6a, 0xf2, 0x6f, 0x13, 0xf4, 0x6f, 0x13, 0xf5,
    0x6f, 0x7a, 0xf2, 0x6f, 0xfe, 0x70,

    /* U+0063 "c" */
    0xa, 0xfd, 0x40, 0x5f, 0x9d, 0xd0, 0x8f, 0x7,
    0xf0, 0x8f, 0x2, 0x60, 0x8f, 0x0, 0x0, 0x8f,
    0x0, 0x0, 0x8f, 0x5, 0xc0, 0x8f, 0x7, 0xf0,
    0x5f, 0x9d, 0xd0, 0xa, 0xfd, 0x40,

    /* U+0064 "d" */
    0x6f, 0xfe, 0x60, 0x6f, 0x7b, 0xf1, 0x6f, 0x15,
    0xf3, 0x6f, 0x15, 0xf3, 0x6f, 0x15, 0xf3, 0x6f,
    0x15, 0xf3, 0x6f, 0x15, 0xf3, 0x6f, 0x15, 0xf3,
    0x6f, 0x7b, 0xf1, 0x6f, 0xfe, 0x60,

    /* U+0065 "e" */
    0x6f, 0xff, 0xc6, 0xf7, 0x65, 0x6f, 0x10, 0x6,
    0xf3, 0x10, 0x6f, 0xff, 0x36, 0xf5, 0x41, 0x6f,
    0x10, 0x6, 0xf1, 0x0, 0x6f, 0x76, 0x56, 0xff,
    0xfc,

    /* U+0066 "f" */
    0x6f, 0xff, 0xa6, 0xf7, 0x64, 0x6f, 0x10, 0x6,
    0xf3, 0x10, 0x6f, 0xff, 0x16, 0xf5, 0x40, 0x6f,
    0x10, 0x6, 0xf1, 0x0, 0x6f, 0x10, 0x6, 0xf1,
    0x0,

    /* U+0067 "g" */
    0xa, 0xfe, 0x40, 0x5f, 0x9c, 0xe0, 0x8f, 0x6,
    0xf0, 0x8f, 0x1, 0x10, 0x8f, 0x2f, 0xf0, 0x8f,
    0x9, 0xf0, 0x8f, 0x6, 0xf0, 0x8f, 0x6, 0xf0,
    0x5f, 0x9c, 0xd0, 0xa, 0xfd, 0x30,

    /* U+0068 "h" */
    0x6f, 0x13, 0xf5, 0x6f, 0x13, 0xf5, 0x6f, 0x13,
    0xf5, 0x6f, 0x35, 0xf5, 0x6f, 0xff, 0xf5, 0x6f,
    0x57, 0xf5, 0x6f, 0x13, 0xf5, 0x6f, 0x13, 0xf5,
    0x6f, 0x13, 0xf5, 0x6f, 0x13, 0xf5,

    /* U+0069 "i" */
    0x6f, 0x16, 0xf1, 0x6f, 0x16, 0xf1, 0x6f, 0x16,
    0xf1, 0x6f, 0x16, 0xf1, 0x6f, 0x16, 0xf1,

    /* U+006A "j" */
    0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2,
    0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2, 0x5, 0xf2,
    0x5c, 0xf0, 0xce, 0x60,

    /* U+006B "k" */
    0x6f, 0x12, 0xf5, 0x6f, 0x19, 0xd0, 0x6f, 0x3f,
    0x50, 0x6f, 0xae, 0x0, 0x6f, 0xff, 0x0, 0x6f,
    0xef, 0x50, 0x6f, 0x4f, 0x90, 0x6f, 0x1b, 0xe0,
    0x6f, 0x16, 0xf3, 0x6f, 0x11, 0xf7,

    /* U+006C "l" */
    0x6f, 0x10, 0x6, 0xf1, 0x0, 0x6f, 0x10, 0x6,
    0xf1, 0x0, 0x6f, 0x10, 0x6, 0xf1, 0x0, 0x6f,
    0x10, 0x6, 0xf1, 0x0, 0x6f, 0x76, 0x46, 0xff,
    0xfa,

    /* U+006D "m" */
    0x6f, 0xd0, 0x4f, 0xf6, 0xff, 0x7, 0xff, 0x6f,
    0xf1, 0x9f, 0xf6, 0xee, 0x3b, 0xcf, 0x6e, 0xb6,
    0xda, 0xf6, 0xe9, 0x8f, 0x8f, 0x6e, 0x7c, 0xd8,
    0xf6, 0xe4, 0xfb, 0x8f, 0x6e, 0x2f, 0x98, 0xf6,
    0xe0, 0xf6, 0x8f,

    /* U+006E "n" */
    0x6f, 0x90, 0xf6, 0x6f, 0xe0, 0xf6, 0x6f, 0xf2,
    0xf6, 0x6f, 0xd6, 0xf6, 0x6f, 0x9a, 0xf6, 0x6f,
    0x5e, 0xf6, 0x6f, 0x1f, 0xf6, 0x6f, 0xc, 0xf6,
    0x6f, 0x8, 0xf6, 0x6f, 0x4, 0xf6,

    /* U+006F "o" */
    0xa, 0xfe, 0x50, 0x5f, 0x9c, 0xf0, 0x8f, 0x6,
    0xf1, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f,
    0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf1,
    0x5f, 0x9c, 0xf0, 0xa, 0xfe, 0x50,

    /* U+0070 "p" */
    0x6f, 0xfe, 0x50, 0x6f, 0x7c, 0xe0, 0x6f, 0x16,
    0xf1, 0x6f, 0x16, 0xf2, 0x6f, 0x3a, 0xf0, 0x6f,
    0xff, 0x90, 0x6f, 0x52, 0x0, 0x6f, 0x10, 0x0,
    0x6f, 0x10, 0x0, 0x6f, 0x10, 0x0,

    /* U+0071 "q" */
    0xa, 0xfe, 0x50, 0x5f, 0x9c, 0xf0, 0x8f, 0x6,
    0xf2, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f,
    0x6, 0xf2, 0x8f, 0x6, 0xf2, 0x8f, 0x6, 0xf1,
    0x5f, 0x9c, 0xe0, 0xa, 0xfd, 0xf8, 0x0, 0x0,
    0x22,

    /* U+0072 "r" */
    0x6f, 0xfe, 0x70, 0x6f, 0x7c, 0xf0, 0x6f, 0x16,
    0xf2, 0x6f, 0x3a, 0xf0, 0x6f, 0xff, 0x70, 0x6f,
    0x6c, 0xf0, 0x6f, 0x16, 0xf2, 0x6f, 0x16, 0xf2,
    0x6f, 0x16, 0xf2, 0x6f, 0x15, 0xf3,

    /* U+0073 "s" */
    0x1b, 0xfd, 0x27, 0xf8, 0xeb, 0xae, 0x8, 0xb7,
    0xf5, 0x0, 0xc, 0xf4, 0x0, 0x1c, 0xf3, 0x0,
    0x1e, 0xba, 0xc0, 0xad, 0x8f, 0x8e, 0xb1, 0xbf,
    0xc2,

    /* U+0074 "t" */
    0xdf, 0xff, 0xe5, 0x8f, 0x95, 0x3, 0xf5, 0x0,
    0x3f, 0x50, 0x3, 0xf5, 0x0, 0x3f, 0x50, 0x3,
    0xf5, 0x0, 0x3f, 0x50, 0x3, 0xf5, 0x0, 0x3f,
    0x50,

    /* U+0075 "u" */
    0x7f, 0x16, 0xf1, 0x7f, 0x16, 0xf1, 0x7f, 0x16,
    0xf1, 0x7f, 0x16, 0xf1, 0x7f, 0x16, 0xf1, 0x7f,
    0x16, 0xf1, 0x7f, 0x16, 0xf1, 0x7f, 0x16, 0xf1,
    0x5f, 0x9c, 0xe0, 0x9, 0xfe, 0x40,

    /* U+0076 "v" */
    0xcc, 0x4, 0xf1, 0x9e, 0x6, 0xf0, 0x7f, 0x8,
    0xd0, 0x5f, 0x2b, 0xa0, 0x2f, 0x4d, 0x80, 0xf,
    0x7f, 0x50, 0xd, 0xaf, 0x30, 0xb, 0xef, 0x10,
    0x9, 0xfe, 0x0, 0x6, 0xfc, 0x0,

    /* U+0077 "w" */
    0xcb, 0xe, 0xd0, 0xc8, 0xad, 0xf, 0xf0, 0xd7,
    0x8e, 0x1f, 0xf0, 0xf5, 0x7f, 0x3f, 0xf3, 0xf3,
    0x5f, 0x6d, 0xe6, 0xf2, 0x4f, 0x9b, 0xc9, 0xf0,
    0x2f, 0xc9, 0xbc, 0xf0, 0xf, 0xe8, 0x9e, 0xd0,
    0xf, 0xf6, 0x7f, 0xb0, 0xd, 0xf4, 0x6f, 0xa0,

    /* U+0078 "x" */
    0x9f, 0x2, 0xf4, 0x3f, 0x58, 0xe0, 0xe, 0xad,
    0x90, 0x8, 0xff, 0x30, 0x3, 0xfe, 0x0, 0x4,
    0xfe, 0x0, 0xa, 0xef, 0x40, 0xf, 0x7e, 0xa0,
    0x5f, 0x19, 0xf0, 0xac, 0x4, 0xf5,

    /* U+0079 "y" */
    0xbe, 0x4, 0xf3, 0x6f, 0x38, 0xe0, 0x1f, 0x7d,
    0x90, 0xb, 0xdf, 0x40, 0x6, 0xfe, 0x0, 0x1,
    0xf9, 0x0, 0x0, 0xf8, 0x0, 0x0, 0xf8, 0x0,
    0x0, 0xf8, 0x0, 0x0, 0xf8, 0x0,

    /* U+007A "z" */
    0x9f, 0xff, 0xc3, 0x66, 0xfb, 0x0, 0x4f, 0x50,
    0xb, 0xe0, 0x1, 0xf8, 0x0, 0x7f, 0x20, 0xe,
    0xc0, 0x4, 0xf5, 0x0, 0xaf, 0x66, 0x5b, 0xff,
    0xfc,

    /* U+007B "{" */
    0x0, 0x0, 0x6, 0xf8, 0xc, 0xc2, 0xd, 0x80,
    0xd, 0x80, 0xe, 0x70, 0xbe, 0x10, 0x3e, 0x70,
    0xd, 0x80, 0xd, 0x80, 0xd, 0xb0, 0x9, 0xf8,
    0x0, 0x21,

    /* U+007C "|" */
    0x1, 0x2, 0xf2, 0x2f, 0x22, 0xf2, 0x2f, 0x22,
    0xf2, 0x2f, 0x22, 0xf2, 0x2f, 0x22, 0xf2, 0x2f,
    0x22, 0xf2, 0x2f, 0x22, 0xf2, 0x2f, 0x20,

    /* U+007D "}" */
    0x0, 0x0, 0xaf, 0x40, 0x2d, 0xa0, 0xb, 0xb0,
    0xb, 0xb0, 0xa, 0xc0, 0x3, 0xf9, 0x9, 0xd2,
    0xb, 0xb0, 0xb, 0xb0, 0x1d, 0xa0, 0xaf, 0x60,
    0x22, 0x0,

    /* U+007E "~" */
    0x0, 0x0, 0x0, 0x2d, 0xb2, 0xa3, 0x97, 0x8f,
    0xd0, 0x0, 0x1, 0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 36, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 47, .box_w = 3, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 15, .adv_w = 75, .box_w = 5, .box_h = 4, .ofs_x = 0, .ofs_y = 6},
    {.bitmap_index = 25, .adv_w = 92, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 55, .adv_w = 90, .box_w = 6, .box_h = 12, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 91, .adv_w = 132, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 131, .adv_w = 93, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 161, .adv_w = 42, .box_w = 3, .box_h = 4, .ofs_x = 0, .ofs_y = 6},
    {.bitmap_index = 167, .adv_w = 62, .box_w = 4, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 193, .adv_w = 62, .box_w = 4, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 219, .adv_w = 95, .box_w = 6, .box_h = 6, .ofs_x = 0, .ofs_y = 4},
    {.bitmap_index = 237, .adv_w = 90, .box_w = 6, .box_h = 5, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 252, .adv_w = 42, .box_w = 3, .box_h = 4, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 258, .adv_w = 60, .box_w = 4, .box_h = 3, .ofs_x = 0, .ofs_y = 4},
    {.bitmap_index = 264, .adv_w = 42, .box_w = 3, .box_h = 2, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 267, .adv_w = 87, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 297, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 327, .adv_w = 90, .box_w = 4, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 347, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 377, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 407, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 437, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 467, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 497, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 527, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 557, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 587, .adv_w = 42, .box_w = 3, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 599, .adv_w = 42, .box_w = 3, .box_h = 10, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 614, .adv_w = 90, .box_w = 5, .box_h = 6, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 629, .adv_w = 90, .box_w = 6, .box_h = 4, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 641, .adv_w = 90, .box_w = 6, .box_h = 6, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 659, .adv_w = 81, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 684, .adv_w = 156, .box_w = 10, .box_h = 12, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 744, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 774, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 804, .adv_w = 86, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 834, .adv_w = 91, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 864, .adv_w = 81, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 889, .adv_w = 77, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 914, .adv_w = 88, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 944, .adv_w = 94, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 974, .adv_w = 43, .box_w = 3, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 989, .adv_w = 59, .box_w = 4, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1009, .adv_w = 93, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1039, .adv_w = 77, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1064, .adv_w = 121, .box_w = 7, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1099, .adv_w = 96, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1129, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1159, .adv_w = 86, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1189, .adv_w = 90, .box_w = 6, .box_h = 11, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 1222, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1252, .adv_w = 83, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1277, .adv_w = 82, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1302, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1332, .adv_w = 86, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1362, .adv_w = 125, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1402, .adv_w = 91, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1432, .adv_w = 88, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1462, .adv_w = 81, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1487, .adv_w = 62, .box_w = 4, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 1513, .adv_w = 87, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1543, .adv_w = 62, .box_w = 4, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 1569, .adv_w = 90, .box_w = 6, .box_h = 4, .ofs_x = 0, .ofs_y = 6},
    {.bitmap_index = 1581, .adv_w = 67, .box_w = 5, .box_h = 2, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1586, .adv_w = 112, .box_w = 4, .box_h = 2, .ofs_x = 1, .ofs_y = 11},
    {.bitmap_index = 1590, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1620, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1650, .adv_w = 86, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1680, .adv_w = 91, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1710, .adv_w = 81, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1735, .adv_w = 77, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1760, .adv_w = 88, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1790, .adv_w = 94, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1820, .adv_w = 43, .box_w = 3, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1835, .adv_w = 59, .box_w = 4, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1855, .adv_w = 93, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1885, .adv_w = 77, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1910, .adv_w = 121, .box_w = 7, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1945, .adv_w = 96, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1975, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2005, .adv_w = 86, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2035, .adv_w = 90, .box_w = 6, .box_h = 11, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 2068, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2098, .adv_w = 83, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2123, .adv_w = 82, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2148, .adv_w = 90, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2178, .adv_w = 86, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2208, .adv_w = 125, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2248, .adv_w = 91, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2278, .adv_w = 88, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2308, .adv_w = 81, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2333, .adv_w = 62, .box_w = 4, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 2359, .adv_w = 112, .box_w = 3, .box_h = 15, .ofs_x = 2, .ofs_y = -3},
    {.bitmap_index = 2382, .adv_w = 62, .box_w = 4, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 2408, .adv_w = 90, .box_w = 6, .box_h = 4, .ofs_x = 0, .ofs_y = 3}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Map glyph_ids to kern left classes*/
static const uint8_t kern_left_class_mapping[] =
{
    0, 0, 0, 0, 0, 0, 0, 1,
    0, 2, 0, 0, 0, 0, 3, 0,
    4, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 5, 6, 7, 8, 9, 10, 11,
    12, 0, 0, 13, 14, 15, 0, 0,
    9, 16, 9, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 2, 0, 0, 0,
    0, 0, 6, 7, 8, 9, 10, 11,
    12, 0, 0, 0, 14, 15, 0, 0,
    9, 16, 9, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 2, 0, 0, 0
};

/*Map glyph_ids to kern right classes*/
static const uint8_t kern_right_class_mapping[] =
{
    0, 0, 0, 1, 0, 0, 0, 2,
    1, 0, 3, 4, 0, 5, 6, 5,
    7, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 8, 8, 0, 0, 0,
    9, 10, 11, 0, 12, 0, 0, 0,
    12, 0, 0, 13, 0, 0, 0, 0,
    12, 0, 12, 0, 14, 15, 16, 17,
    18, 19, 20, 21, 0, 0, 3, 0,
    0, 0, 11, 0, 12, 0, 0, 0,
    12, 0, 0, 0, 0, 0, 0, 0,
    12, 0, 12, 0, 14, 15, 16, 17,
    18, 19, 20, 21, 0, 0, 3, 0
};

/*Kern values between classes*/
static const int8_t kern_class_values[] =
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -2, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 2,
    0, 2, 0, 3, 0, 2, 2, 1,
    2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, 0, -14, 0,
    -16, 0, -4, -2, -8, -11, -3, 0,
    0, 0, 0, 0, 0, -31, 0, 0,
    0, -6, 0, -10, 0, 2, 0, 2,
    2, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -3, 0,
    -2, 0, 0, 0, 0, 0, -4, -3,
    0, -12, 0, 2, -10, 2, -4, 2,
    2, -7, -1, 1, 0, 0, -1, -10,
    0, -6, -3, 0, -10, 1, -1, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0,
    -2, -4, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, -2, -2, 0,
    -1, 0, 0, 0, -1, 0, 0, 0,
    0, 0, 0, 0, -1, 0, -1, 0,
    0, 0, -4, -4, 0, 0, 0, 2,
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 3, 2, -16, -1,
    -7, 0, 1, -1, -7, 0, -11, 0,
    2, 0, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -3, -3, 0, 0, 0, 0, 0,
    0, 0, -2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 1, -4, 1, -8, 0,
    1, -2, -2, 0, -4, 0, -2, 0,
    0, 0, 0, 0, 0, 0, -16, 2,
    3, -22, 2, -16, 2, 2, -10, 0,
    1, -1, 2, 0, -17, 0, -10, -4,
    0, -20, 0, 0, 0, 0, 2, -21,
    0, -15, 0, 2, 0, -8, 0, -13,
    1, 0, 0, 0, 0, -3, -1, 0,
    -1, 0, 0, 0, 1, 0, 2, 1,
    0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, -4, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, -1,
    -1, 0, 2, 0, 3, 2, -16, -16,
    -10, -8, 1, -6, -10, -1, -11, 0,
    2, 0, 1, 1, 0, 1, 0, 0,
    0, 0, 0, 0, 0, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, -1, 2, 0,
    -12, -4, -6, -3, 0, -2, -6, 0,
    -11, 0, 1, 0, 1, 1, 0, 0,
    0, 2, 0, 2, 0, -6, -2, 0,
    -1, 0, -1, -3, 0, -5, 0, 1,
    0, 1, 1, 0, 0, 0, 0, -1,
    1, -4, 1, -8, 2, 1, -2, -2,
    0, -4, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 2, -4, 2, -2, -21,
    -11, -12, -9, -2, -9, -10, -4, -13,
    -4, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, -3, 0, 2,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1
};


/*Collect the kern class' data in one place*/
static const lv_font_fmt_txt_kern_classes_t kern_classes =
{
    .class_pair_values   = kern_class_values,
    .left_class_mapping  = kern_left_class_mapping,
    .right_class_mapping = kern_right_class_mapping,
    .left_class_cnt      = 25,
    .right_class_cnt     = 21,
};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = &kern_classes,
    .kern_scale = 16,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 1,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t ui_font_bebas_14 = {
#else
lv_font_t ui_font_bebas_14 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 16,          /*The maximum line height required by the font*/
    .base_line = 3,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if UI_FONT_BEBAS_14*/
