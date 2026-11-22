// Probe the UVC AE-priority control (Camera Terminal sel 3) that the descriptor does not advertise.
// Detaches uvcvideo from interface 0, sends standard UVC requests, always reattaches.
// usage: sudo aeprio get        -> GET_INFO/CUR/MIN/MAX/DEF for AE mode (sel 2, control) and AE priority (sel 3)
//        sudo aeprio set 0|1    -> SET_CUR AE priority (0 = constant frame rate, 1 = may lower frame rate), then read back
#include <errno.h>
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
static int fd;
static int ctrl(int reqtype, int req, int sel, unsigned char *b, int len) {
  struct usbdevfs_ctrltransfer c = {.bRequestType = reqtype, .bRequest = req, .wValue = sel << 8,
                                    .wIndex = (1 << 8) | 0 /* unit 1 (camera terminal), interface 0 */, .wLength = len, .timeout = 1000, .data = b};
  return ioctl(fd, USBDEVFS_CONTROL, &c);
}
static void dump(const char *name, int sel) {
  const int reqs[] = {0x86, 0x81, 0x82, 0x83, 0x87}; const char *n[] = {"INFO", "CUR", "MIN", "MAX", "DEF"};
  printf("%-12s", name);
  for (int k = 0; k < 5; k++) { unsigned char b[1] = {0}; int r = ctrl(0xA1, reqs[k], sel, b, 1);
    if (r < 0) printf(" %s=ERR(%s)", n[k], strerror(errno)); else printf(" %s=0x%02x", n[k], b[0]); }
  printf("\n");
}
int main(int ac, char **av) {
  // find the icSpring camera's usbfs node
  char path[64] = {0};
  FILE *p = popen("for d in /sys/bus/usb/devices/*; do [ \"$(cat $d/idVendor 2>/dev/null)\" = 2993 ] && [ \"$(cat $d/idProduct)\" = 0858 ] && printf /dev/bus/usb/%03d/%03d $(cat $d/busnum) $(cat $d/devnum); done", "r");
  if (!p || !fgets(path, sizeof path, p) || !path[0]) { fprintf(stderr, "camera 2993:0858 not found\n"); return 1; } pclose(p);
  printf("device %s\n", path);
  fd = open(path, O_RDWR); if (fd < 0) { perror("open (need sudo)"); return 1; }
  struct usbdevfs_disconnect_claim dc = {.interface = 0, .flags = USBDEVFS_DISCONNECT_CLAIM_IF_DRIVER, .driver = "uvcvideo"};
  if (ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &dc)) { perror("detach uvcvideo"); return 1; }
  dump("AE mode", 2);  // advertised control, sanity check
  dump("AE priority", 3);  // NOT advertised
  if (ac > 2 && !strcmp(av[1], "set")) {
    unsigned char v = atoi(av[2]);
    int r = ctrl(0x21, 0x01, 3, &v, 1);
    printf("SET_CUR AE priority=%d -> %s\n", v, r < 0 ? strerror(errno) : "accepted");
    dump("AE priority", 3);
  }
  unsigned int ifn = 0; ioctl(fd, USBDEVFS_RELEASEINTERFACE, &ifn);
  struct usbdevfs_ioctl cmd = {.ifno = 0, .ioctl_code = USBDEVFS_CONNECT, .data = NULL};
  if (ioctl(fd, USBDEVFS_IOCTL, &cmd)) perror("reattach uvcvideo"); else printf("uvcvideo reattached\n");
  return 0;
}
