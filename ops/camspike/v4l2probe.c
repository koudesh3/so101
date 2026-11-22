// v4l2probe: capture from a UVC camera, report delivered fps, dropped frames
// (driver sequence gaps + timestamp gaps), per-minute drop counts, and
// capture->dequeue latency (p50/p99/max).
// usage: v4l2probe /dev/videoN WIDTH HEIGHT FPS SECONDS [MJPG|YUYV]
// build: gcc -O2 -o v4l2probe v4l2probe.c -lm
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define NBUF 4
#define WARMUP_S 2.0
static int xioctl(int fd, unsigned long r, void *a) { int e; do e = ioctl(fd, r, a); while (e == -1 && errno == EINTR); return e; }
static double now_mono(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static int cmpd(const void *a, const void *b) { double x = *(double *)a, y = *(double *)b; return (x > y) - (x < y); }
static void die(const char *m) { perror(m); exit(1); }

int main(int ac, char **av) {
  if (ac < 6) { fprintf(stderr, "usage: %s dev W H FPS SECS [MJPG|YUYV]\n", av[0]); return 2; }
  int W = atoi(av[2]), H = atoi(av[3]), FPS = atoi(av[4]); double secs = atof(av[5]);
  const char *fcc = ac > 6 ? av[6] : "MJPG";
  int fd = open(av[1], O_RDWR); if (fd < 0) die("open");

  struct v4l2_format f = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
  f.fmt.pix.width = W; f.fmt.pix.height = H; f.fmt.pix.field = V4L2_FIELD_ANY;
  f.fmt.pix.pixelformat = v4l2_fourcc(fcc[0], fcc[1], fcc[2], fcc[3]);
  if (xioctl(fd, VIDIOC_S_FMT, &f)) die("S_FMT");
  struct v4l2_streamparm p = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
  p.parm.capture.timeperframe.numerator = 1; p.parm.capture.timeperframe.denominator = FPS;
  if (xioctl(fd, VIDIOC_S_PARM, &p)) die("S_PARM");
  double ival = (double)p.parm.capture.timeperframe.numerator / p.parm.capture.timeperframe.denominator;
  char got[5] = {0}; memcpy(got, &f.fmt.pix.pixelformat, 4);
  printf("negotiated %s %ux%u @ %.3f fps, sizeimage %u\n", got, f.fmt.pix.width, f.fmt.pix.height, 1 / ival, f.fmt.pix.sizeimage);

  struct v4l2_requestbuffers rb = {.count = NBUF, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
  if (xioctl(fd, VIDIOC_REQBUFS, &rb)) die("REQBUFS");
  for (unsigned i = 0; i < rb.count; i++) {
    struct v4l2_buffer b = {.type = rb.type, .memory = rb.memory, .index = i};
    if (xioctl(fd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF");
    if (mmap(NULL, b.length, PROT_READ, MAP_SHARED, fd, b.m.offset) == MAP_FAILED) die("mmap");
    if (xioctl(fd, VIDIOC_QBUF, &b)) die("QBUF");
  }
  enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_STREAMON, &t)) die("STREAMON");

  size_t cap = (size_t)(secs * 120) + 1000; double *lat = malloc(cap * sizeof *lat);
  long n = 0, seqmiss = 0, tsgaps = 0, errs = 0, permin[1000] = {0}; double t0 = 0, tl = 0, maxgap = 0, bytes = 0;
  unsigned seq0 = 0, seql = 0; int started = 0; unsigned tsflags = 0;
  double start = now_mono();
  for (;;) {
    struct v4l2_buffer b = {.type = t, .memory = V4L2_MEMORY_MMAP};
    if (xioctl(fd, VIDIOC_DQBUF, &b)) die("DQBUF");
    double deq = now_mono(), ts = b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6;
    if (deq - start >= WARMUP_S) {
      if (!started) { started = 1; t0 = tl = ts; seq0 = seql = b.sequence; tsflags = b.flags; }
      else {
        long ds = (long)b.sequence - (long)seql - 1; if (ds > 0) { seqmiss += ds; permin[(int)((ts - t0) / 60)] += ds; }
        double g = ts - tl; if (g > maxgap) maxgap = g;
        if (g > 1.5 * ival) { long m = lround(g / ival) - 1; tsgaps += m; printf("  gap %.0fms at t=%.1fs (seq %u->%u)\n", g * 1000, ts - t0, seql, b.sequence); }
        tl = ts; seql = b.sequence;
      }
      if (b.flags & V4L2_BUF_FLAG_ERROR) errs++;
      if (n < (long)cap) lat[n] = deq - ts;
      n++; bytes += b.bytesused;
      if (deq - start >= WARMUP_S + secs) { xioctl(fd, VIDIOC_QBUF, &b); break; }
    }
    if (xioctl(fd, VIDIOC_QBUF, &b)) die("QBUF");
  }
  xioctl(fd, VIDIOC_STREAMOFF, &t);
  long m = n < (long)cap ? n : (long)cap; qsort(lat, m, sizeof *lat, cmpd);
  const char *src = (tsflags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) == V4L2_BUF_FLAG_TSTAMP_SRC_SOE ? "start-of-exposure/first packet" : "end-of-frame";
  printf("RESULT %s %ux%u req %d fps | %.1fs: %ld frames, %.2f fps delivered | dropped: %ld by driver sequence, %ld by timestamp gaps, %ld error-flagged | maxgap %.0fms | avg frame %.0f KB\n",
         got, f.fmt.pix.width, f.fmt.pix.height, FPS, tl - t0, n, (n - 1) / (tl - t0), seqmiss, tsgaps, errs, maxgap * 1000, bytes / n / 1024);
  printf("LATENCY buffer timestamp (%s) -> app dequeue, ms: p50 %.1f  p99 %.1f  max %.1f\n", src, lat[m / 2] * 1000, lat[m * 99 / 100] * 1000, lat[m - 1] * 1000);
  printf("DROPS_PER_MINUTE"); for (int i = 0; i <= (int)((tl - t0) / 60); i++) printf(" %ld", permin[i]); printf("\n");
  return 0;
}
