// g2g_cap: stream MJPEG from one V4L2 camera; for each frame write a record to stdout:
//   double buf_mono, double deq_mono, double deq_real, uint32 seq, uint32 len, then len bytes of JPEG.
// Runs until SIGTERM/SIGINT or stdout closes.
// usage: g2g_cap /dev/videoN W H FPS
// build: gcc -O2 -o g2g_cap g2g_cap.c
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stop;
static void on_sig(int s) { (void)s; stop = 1; }
static int xioctl(int fd, unsigned long r, void *a) { int e; do e = ioctl(fd, r, a); while (e == -1 && errno == EINTR && !stop); return e; }
static double clk(clockid_t c) { struct timespec t; clock_gettime(c, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static void die(const char *m) { perror(m); exit(1); }
static int wr(const void *p, size_t n) { const char *c = p; while (n) { ssize_t k = write(1, c, n); if (k <= 0) return -1; c += k; n -= k; } return 0; }

int main(int ac, char **av) {
  if (ac < 5) { fprintf(stderr, "usage: %s dev W H FPS\n", av[0]); return 2; }
  signal(SIGTERM, on_sig); signal(SIGINT, on_sig); signal(SIGPIPE, on_sig);
  int fd = open(av[1], O_RDWR); if (fd < 0) die("open");
  struct v4l2_format f = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
  f.fmt.pix.width = atoi(av[2]); f.fmt.pix.height = atoi(av[3]);
  f.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; f.fmt.pix.field = V4L2_FIELD_ANY;
  if (xioctl(fd, VIDIOC_S_FMT, &f)) die("S_FMT");
  if (f.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) { fprintf(stderr, "MJPEG not accepted\n"); return 1; }
  struct v4l2_streamparm p = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
  p.parm.capture.timeperframe.numerator = 1; p.parm.capture.timeperframe.denominator = atoi(av[4]);
  if (xioctl(fd, VIDIOC_S_PARM, &p)) die("S_PARM");
  fprintf(stderr, "%s: MJPG %ux%u @ %u/%u s\n", av[1], f.fmt.pix.width, f.fmt.pix.height,
          p.parm.capture.timeperframe.numerator, p.parm.capture.timeperframe.denominator);

  struct v4l2_requestbuffers rb = {.count = 4, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
  if (xioctl(fd, VIDIOC_REQBUFS, &rb)) die("REQBUFS");
  void *mem[8];
  for (unsigned i = 0; i < rb.count; i++) {
    struct v4l2_buffer b = {.type = rb.type, .memory = rb.memory, .index = i};
    if (xioctl(fd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF");
    mem[i] = mmap(NULL, b.length, PROT_READ, MAP_SHARED, fd, b.m.offset);
    if (mem[i] == MAP_FAILED) die("mmap");
    if (xioctl(fd, VIDIOC_QBUF, &b)) die("QBUF");
  }
  enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_STREAMON, &t)) die("STREAMON");
  while (!stop) {
    struct v4l2_buffer b = {.type = t, .memory = V4L2_MEMORY_MMAP};
    if (xioctl(fd, VIDIOC_DQBUF, &b)) { if (stop) break; die("DQBUF"); }
    double deq_mono = clk(CLOCK_MONOTONIC), deq_real = clk(CLOCK_REALTIME);
    double buf_mono = b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6;
    uint32_t hdr2[2] = {b.sequence, b.bytesused};
    double hdr[3] = {buf_mono, deq_mono, deq_real};
    if (wr(hdr, sizeof hdr) || wr(hdr2, sizeof hdr2) || wr(mem[b.index], b.bytesused)) break;
    if (xioctl(fd, VIDIOC_QBUF, &b)) die("QBUF");
  }
  xioctl(fd, VIDIOC_STREAMOFF, &t);
  return 0;
}
