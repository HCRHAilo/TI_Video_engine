/*
 * thermal_live.c - ATTO640 live viewer
 *
 * Pipeline (per frame):
 *   V4L2 capture -> 14-bit decode -> 1-point NUC (offset) ->
 *   bad pixel replacement -> AGC (14-bit -> 8-bit) -> SDL2 window
 *
 * Usage:
 *   ./thermal_live                    (default /dev/video0)
 *   ./thermal_live -d /dev/video1
 *   ./thermal_live -f obs.bin         (replay a saved usb_observe file, loops)
 *
 * Keys (in the window or in the terminal):
 *   n  = take NUC (put the lens cap on / show a uniform object FIRST)
 *   r  = reset NUC (off)
 *   s  = snapshot: snap_XX.raw (raw 14-bit frame) + snap_XX.pgm (display image)
 *   q  = quit (Esc or closing the window also quits)
 *
 * Setup : sudo apt install libsdl2-dev
 * Build : gcc -O2 -Wall -o thermal_live thermal_live.c -lm $(sdl2-config --cflags --libs)
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <SDL.h>

#define W       640
#define H       480
#define NPIX    (W * H)
#define FRAMESZ (NPIX * 2)      /* 2 bytes per pixel on USB */
#define NAVG    64              /* frames averaged for NUC (noise / sqrt(64) = /8) */
#define NBUF    4               /* V4L2 capture buffers */

/* Bad pixel thresholds (applied to the 64-frame NUC average) */
#define DEAD_DL        1000     /* below this: dead pixel */
#define HOT_DL         15000    /* above this: hot / saturated pixel */
#define OUTLIER_SIGMA  6.0      /* farther than this many std from the mean: outlier */

/* AGC settings */
#define AGC_LOW_PCT    1        /* ignore the coldest 1% of pixels */
#define AGC_HIGH_PCT   99       /* ignore the hottest 1% of pixels */
#define AGC_MIN_RANGE  16       /* minimum DL span, avoids blowing up noise on flat scenes */
#define AGC_SMOOTH     0.1f     /* per-frame smoothing of the range, avoids flicker */

/* ======================= input: V4L2 device or raw file ======================= */

static int   vfd = -1;
static FILE *rawf = NULL;
static struct { void *p; size_t len; } buf[NBUF];
static struct v4l2_buffer cur;
static uint8_t filebuf[FRAMESZ];

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do r = ioctl(fd, req, arg); while (r == -1 && errno == EINTR);
    return r;
}

static int v4l2_open(const char *dev)
{
    vfd = open(dev, O_RDWR);
    if (vfd < 0) { perror(dev); return -1; }

    /* The USB3-A3 labels the stream "YUYV", but each 16-bit word carries
     * one 14-bit raw pixel (USB3-A3 user guide, section 9.1). */
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    fmt.fmt.pix.width = W;
    fmt.fmt.pix.height = H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(vfd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); return -1; }
    if (fmt.fmt.pix.width != W || fmt.fmt.pix.height != H) {
        fprintf(stderr, "format is %ux%u, expected %dx%d\n",
                fmt.fmt.pix.width, fmt.fmt.pix.height, W, H);
        return -1;
    }

    struct v4l2_requestbuffers req = { .count = NBUF,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    if (xioctl(vfd, VIDIOC_REQBUFS, &req) < 0) { perror("REQBUFS"); return -1; }

    for (unsigned i = 0; i < req.count && i < NBUF; i++) {
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                 .memory = V4L2_MEMORY_MMAP, .index = i };
        if (xioctl(vfd, VIDIOC_QUERYBUF, &b) < 0) { perror("QUERYBUF"); return -1; }
        buf[i].len = b.length;
        buf[i].p = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, vfd, b.m.offset);
        if (buf[i].p == MAP_FAILED) { perror("mmap"); return -1; }
        if (xioctl(vfd, VIDIOC_QBUF, &b) < 0) { perror("QBUF"); return -1; }
    }
    enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(vfd, VIDIOC_STREAMON, &t) < 0) { perror("STREAMON"); return -1; }
    return 0;
}

/* Return one raw frame (614400 bytes), or NULL on error. */
static const uint8_t *get_frame(void)
{
    if (rawf) {
        if (fread(filebuf, 1, FRAMESZ, rawf) != FRAMESZ) {      /* EOF: loop the file */
            rewind(rawf);
            if (fread(filebuf, 1, FRAMESZ, rawf) != FRAMESZ) return NULL;
        }
        usleep(16000);                                          /* ~60 fps */
        return filebuf;
    }
    struct pollfd p = { .fd = vfd, .events = POLLIN };
    if (poll(&p, 1, 2000) <= 0) { fprintf(stderr, "frame timeout\n"); return NULL; }
    memset(&cur, 0, sizeof cur);
    cur.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cur.memory = V4L2_MEMORY_MMAP;
    if (xioctl(vfd, VIDIOC_DQBUF, &cur) < 0) { perror("DQBUF"); return NULL; }
    if (cur.bytesused != FRAMESZ) {                             /* incomplete frame: skip it */
        xioctl(vfd, VIDIOC_QBUF, &cur);
        return get_frame();
    }
    return buf[cur.index].p;
}

/* Give the buffer back to the driver as early as possible. A slow consumer
 * makes the USB3-A3 overflow and reset the stream (user guide, chapter 14). */
static void put_frame(void)
{
    if (vfd >= 0) xioctl(vfd, VIDIOC_QBUF, &cur);
}

/* ============================== processing state ============================== */

static uint16_t img[NPIX];          /* decoded 14-bit pixels (DL) */
static float    offset[NPIX];       /* NUC offset map: per-pixel fixed error */
static uint8_t  bad[NPIX];          /* 1 = bad pixel */
static int32_t  corr[NPIX];         /* corrected image */
static uint8_t  out8[NPIX];         /* 8-bit display image */
static double   acc[NPIX];          /* NUC accumulator (sum of NAVG frames) */
static int nuc_on = 0;              /* 1 = offset correction active */
static int nuc_count = -1;          /* >= 0 while collecting NUC frames */
static int nbad = 0;
static float agc_lo = -1, agc_hi = -1;

/* USB word = [14-bit pixel][2 padding bits], big-endian. */
static void decode(const uint8_t *raw)
{
    for (int i = 0; i < NPIX; i++)
        img[i] = (uint16_t)(((raw[2*i] << 8) | raw[2*i + 1]) >> 2);
}

/*
 * 1-point NUC. Called once per frame while collecting.
 * With a uniform scene (lens cap), every pixel sees the same temperature, so
 * any difference between pixels is the sensor's own fixed pattern.
 *   avg[i]    = mean of NAVG frames          (random noise averages out)
 *   offset[i] = avg[i] - mean(avg)           (fixed error of pixel i)
 * Bad pixels are detected from the same average.
 */
static void nuc_collect(void)
{
    for (int i = 0; i < NPIX; i++) acc[i] += img[i];
    if (++nuc_count < NAVG) return;

    /* mean and spread of the averaged frame */
    double m = 0, m2 = 0;
    for (int i = 0; i < NPIX; i++) { double a = acc[i] / NAVG; m += a; m2 += a * a; }
    m /= NPIX;
    double sd = sqrt(m2 / NPIX - m * m);

    /* bad pixel detection: dead, hot, or far from the rest */
    nbad = 0;
    double gm = 0; long ng = 0;
    for (int i = 0; i < NPIX; i++) {
        double a = acc[i] / NAVG;
        bad[i] = (a < DEAD_DL || a > HOT_DL || fabs(a - m) > OUTLIER_SIGMA * sd);
        if (bad[i]) nbad++; else { gm += a; ng++; }
    }
    gm /= ng;                                       /* mean of good pixels only */

    for (int i = 0; i < NPIX; i++)
        offset[i] = bad[i] ? 0 : (float)(acc[i] / NAVG - gm);

    nuc_on = 1;
    nuc_count = -1;
    agc_lo = agc_hi = -1;                           /* restart AGC on the new image */
    fprintf(stderr, "\n[NUC done] mean %.0f DL, FPN std %.1f DL, bad pixels %d\n"
                    "           remove the lens cap now\n", gm, sd, nbad);
}

static int cmp_i32(const void *a, const void *b)
{
    int32_t x = *(const int32_t *)a, y = *(const int32_t *)b;
    return (x > y) - (x < y);
}

/* Bad pixel replacement: median of the good pixels among the 8 neighbours.
 * Median (not mean) keeps edges sharp when a bad pixel sits on an edge. */
static void fix_bad(void)
{
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int i = y * W + x;
            if (!bad[i]) continue;
            int32_t v[8]; int n = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    if (!dx && !dy) continue;
                    int xx = x + dx, yy = y + dy;
                    if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
                    int j = yy * W + xx;
                    if (!bad[j]) v[n++] = corr[j];
                }
            if (n) { qsort(v, n, sizeof v[0], cmp_i32); corr[i] = v[n / 2]; }
        }
}

/* corrected = raw - offset, then fix bad pixels (only when NUC is on) */
static void correct(void)
{
    if (nuc_on) {
        for (int i = 0; i < NPIX; i++) corr[i] = (int32_t)lrintf(img[i] - offset[i]);
        fix_bad();
    } else {
        for (int i = 0; i < NPIX; i++) corr[i] = img[i];
    }
}

/*
 * Linear percentile AGC.
 * Take the 1% and 99% points of the histogram as black and white, so a few
 * very hot/cold pixels do not compress the rest of the image. The range is
 * smoothed over frames so the brightness does not flicker.
 */
static void agc(void)
{
    static uint32_t hist[16384];
    memset(hist, 0, sizeof hist);
    for (int i = 0; i < NPIX; i++) {
        int v = corr[i]; if (v < 0) v = 0; if (v > 16383) v = 16383;
        hist[v]++;
    }
    uint32_t c = 0;
    uint32_t tlo = (uint32_t)((uint64_t)NPIX * AGC_LOW_PCT / 100);
    uint32_t thi = (uint32_t)((uint64_t)NPIX * AGC_HIGH_PCT / 100);
    int lo = 0, hi = 16383, v;
    for (v = 0; v < 16384; v++) { c += hist[v]; if (c > tlo) { lo = v; break; } }
    for (; v < 16384; v++)      { c += hist[v]; if (c > thi) { hi = v; break; } }
    if (hi - lo < AGC_MIN_RANGE) hi = lo + AGC_MIN_RANGE;

    if (agc_lo < 0) { agc_lo = lo; agc_hi = hi; }
    agc_lo += AGC_SMOOTH * (lo - agc_lo);
    agc_hi += AGC_SMOOTH * (hi - agc_hi);

    float scale = 255.0f / (agc_hi - agc_lo);
    for (int i = 0; i < NPIX; i++) {
        float f = (corr[i] - agc_lo) * scale;
        out8[i] = f < 0 ? 0 : f > 255 ? 255 : (uint8_t)f;
    }
}

/* Focus score: mean absolute gradient over the central 320x240 region.
 * Higher = sharper. Only meaningful with NUC on (stripes inflate it). */
static double focus_score(void)
{
    double s = 0; long n = 0;
    for (int y = H / 4; y < 3 * H / 4; y++)
        for (int x = W / 4; x < 3 * W / 4; x++) {
            int i = y * W + x;
            s += abs(corr[i] - corr[i + 1]) + abs(corr[i] - corr[i + W]);
            n++;
        }
    return s / n;
}

/* ========================= keyboard (terminal stdin) ========================= */

static struct termios tio_old;
static int tio_saved = 0;
static volatile sig_atomic_t stop = 0;

static void on_sig(int s) { (void)s; stop = 1; }

/* Non-canonical, no-echo stdin so single key presses are read immediately. */
static void kb_init(void)
{
    if (!isatty(0)) return;
    tcgetattr(0, &tio_old); tio_saved = 1;
    struct termios t = tio_old;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &t);
}

static int kb_get(void)
{
    struct pollfd p = { .fd = 0, .events = POLLIN };
    char ch;
    if (poll(&p, 1, 0) > 0 && read(0, &ch, 1) == 1) return ch;
    return -1;
}

static void snapshot(const uint8_t *raw)
{
    static int n = 0;
    char name[64];
    snprintf(name, sizeof name, "snap_%02d.raw", n);
    FILE *f = fopen(name, "wb");
    if (f) { fwrite(raw, 1, FRAMESZ, f); fclose(f); }
    snprintf(name, sizeof name, "snap_%02d.pgm", n);
    f = fopen(name, "wb");
    if (f) { fprintf(f, "P5\n%d %d\n255\n", W, H); fwrite(out8, 1, NPIX, f); fclose(f); }
    fprintf(stderr, "\n[saved snap_%02d.raw / .pgm]\n", n++);
}

/* ==================================== main ==================================== */

int main(int argc, char **argv)
{
    const char *dev = "/dev/video0", *file = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-d") && i + 1 < argc) dev = argv[++i];
        else if (!strcmp(argv[i], "-f") && i + 1 < argc) file = argv[++i];
        else { fprintf(stderr, "usage: %s [-d /dev/videoN | -f raw_file]\n", argv[0]); return 1; }
    }
    if (file) { rawf = fopen(file, "rb"); if (!rawf) { perror(file); return 1; } }
    else if (v4l2_open(dev) < 0) return 1;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    kb_init();

    if (SDL_Init(SDL_INIT_VIDEO) != 0) { fprintf(stderr, "SDL: %s\n", SDL_GetError()); return 1; }
    SDL_Window *win = SDL_CreateWindow("ATTO640 thermal_live", SDL_WINDOWPOS_CENTERED,
                        SDL_WINDOWPOS_CENTERED, W, H, SDL_WINDOW_RESIZABLE);
    SDL_Renderer *ren = win ? SDL_CreateRenderer(win, -1, 0) : NULL;
    SDL_Texture *tex = ren ? SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                        SDL_TEXTUREACCESS_STREAMING, W, H) : NULL;
    if (!tex) { fprintf(stderr, "SDL window: %s\n", SDL_GetError()); return 1; }
    SDL_RenderSetLogicalSize(ren, W, H);           /* keep aspect ratio when resized */
    static uint32_t argb[NPIX];
    static uint8_t rawcopy[FRAMESZ];

    fprintf(stderr, "keys (window or terminal): n=NUC (lens cap first)  r=NUC off  "
                    "s=snapshot  q/Esc=quit\n");

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    long frames = 0;

    while (!stop) {
        const uint8_t *raw = get_frame();
        if (!raw) break;
        decode(raw);

        /* --- keys --- */
        int want_snap = 0;
        int k = kb_get();
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) stop = 1;
            else if (ev.type == SDL_KEYDOWN) {
                SDL_Keycode kc = ev.key.keysym.sym;
                if (kc == SDLK_n || kc == SDLK_r || kc == SDLK_s || kc == SDLK_q) k = (int)kc;
                if (kc == SDLK_ESCAPE) k = 'q';
            }
        }
        if (k == 'q') {
            stop = 1;
        } else if (k == 'n') {
            memset(acc, 0, sizeof acc);
            nuc_count = 0;
            fprintf(stderr, "\n[NUC] collecting %d frames... keep the lens cap on\n", NAVG);
        } else if (k == 'r') {
            nuc_on = 0; nbad = 0;
            memset(bad, 0, sizeof bad);
            agc_lo = agc_hi = -1;
            fprintf(stderr, "\n[NUC off]\n");
        } else if (k == 's') {
            memcpy(rawcopy, raw, FRAMESZ);
            want_snap = 1;
        }
        put_frame();                                   /* return the buffer before processing */

        /* --- processing --- */
        if (nuc_count >= 0) nuc_collect();
        correct();
        agc();
        if (want_snap) snapshot(rawcopy);

        /* --- display: 8-bit grey -> ARGB texture --- */
        for (int i = 0; i < NPIX; i++) argb[i] = 0xFF000000u | (out8[i] * 0x010101u);
        SDL_UpdateTexture(tex, NULL, argb, W * 4);
        SDL_RenderClear(ren);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);

        /* --- status line once per 60 frames --- */
        if (++frames % 60 == 0) {
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            double dt = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
            double sum = 0;
            for (int i = 0; i < NPIX; i++) sum += img[i];
            fprintf(stderr, "\rfps %5.1f | raw mean %6.0f | AGC %5.0f..%5.0f | focus %6.2f | NUC %s bad %d   ",
                    60 / dt, sum / NPIX, agc_lo, agc_hi, focus_score(),
                    nuc_count >= 0 ? "collecting" : nuc_on ? "ON " : "off", nbad);
            t0 = t1;
        }
    }

    if (vfd >= 0) {
        enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(vfd, VIDIOC_STREAMOFF, &t);
    }
    if (tio_saved) tcsetattr(0, TCSANOW, &tio_old);
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    fprintf(stderr, "\nbye\n");
    return 0;
}
