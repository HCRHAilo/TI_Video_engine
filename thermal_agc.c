/*
 * thermal_agc.c  -  ATTO640 raw frame -> 2 display images (before / after AGC)
 *
 * Input : one raw USB frame, exactly 614400 bytes
 *         (640 x 480 pixels, 2 bytes each, big-endian, value = word >> 2)
 * Output: before.pgm  - simple full-range scaling 
 *         after.pgm   - percentile AGC (1% .. 99%)
 *
 * Build : gcc -O2 -Wall -o thermal_agc thermal_agc.c
 * Run   : ./thermal_agc frame.raw
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define W     640
#define H     480
#define NPIX  (W * H)              /* 307200 pixels          */
#define RAWSZ (NPIX * 2)           /* 614400 bytes per frame */

static uint8_t  raw[RAWSZ];        /* file bytes       */
static uint16_t img[NPIX];         /* decoded 14-bit values     */
static uint8_t  out8[NPIX];        /* screen 8-bit values        */
static uint32_t hist[16384];       /* 14-bit value ki count     */

/* 8-bit grey image in  PGM file  save  */
static void save_pgm(const char *name, const uint8_t *pix)
{
    FILE *f = fopen(name, "wb");
    if (!f) { perror(name); return; }
    fprintf(f, "P5\n%d %d\n255\n", W, H);
    fwrite(pix, 1, NPIX, f);
    fclose(f);
    printf("  saved %s\n", name);
}

/* lo..hi range  0..255  stretch  */
static void stretch(int lo, int hi)
{
    if (hi <= lo) hi = lo + 1;
    for (int i = 0; i < NPIX; i++) {
        int v = img[i];
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        out8[i] = (uint8_t)((v - lo) * 255 / (hi - lo));
    }
}

/* histogram  'pct' percent pixels value */
static int percentile(double pct)
{
    uint32_t target = (uint32_t)(NPIX * pct / 100.0);
    uint32_t acc = 0;
    for (int v = 0; v < 16384; v++) {
        acc += hist[v];
        if (acc > target) return v;
    }
    return 16383;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s frame.raw\n", argv[0]);
        return 1;
    }

    /* ---- STEP 1: raw file ---- */
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    size_t n = fread(raw, 1, RAWSZ, f);
    fclose(f);
    if (n != RAWSZ) {
        printf("ERROR: %zu bytes chadivanu, %d kavali. Idi raw frame kaadu.\n", n, RAWSZ);
        return 1;
    }

    /* ---- STEP 2: decode (big-endian, >>2) ---- */
    long sum = 0;
    int mn = 16383, mx = 0;
    for (int i = 0; i < NPIX; i++) {
        uint16_t word = (uint16_t)((raw[2*i] << 8) | raw[2*i + 1]);
        img[i] = word >> 2;
        sum += img[i];
        if (img[i] < mn) mn = img[i];
        if (img[i] > mx) mx = img[i];
        hist[img[i]]++;
    }
    printf("STEP 2 decode: min=%d  max=%d  mean=%ld DL\n", mn, mx, sum / NPIX);

    /* ---- STEP 3: BEFORE - full 14-bit range  0..255  ---- */
    printf("STEP 3 before: range 0..16383 -> 0..255\n");
    stretch(0, 16383);
    save_pgm("before.pgm", out8);

    /* ---- STEP 4: AFTER - percentile AGC ---- */
    int lo = percentile(1.0);
    int hi = percentile(99.0);
    printf("STEP 4 after : 1%% point=%d DL, 99%% point=%d DL  (range %d DL)\n",
           lo, hi, hi - lo);
    stretch(lo, hi);
    save_pgm("after.pgm", out8);

    return 0;
}
