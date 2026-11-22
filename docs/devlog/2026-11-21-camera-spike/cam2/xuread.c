// Read-only enumeration of a UVC extension unit via UVCIOC_CTRL_QUERY (GET_* only, never SET_CUR).
// usage: xuread /dev/videoN UNIT
#include <fcntl.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
static int q(int fd,int u,int s,int qq,int sz,unsigned char*b){struct uvc_xu_control_query x={.unit=u,.selector=s,.query=qq,.size=sz,.data=b};return ioctl(fd,UVCIOC_CTRL_QUERY,&x);}
static void hex(const char*n,unsigned char*b,int l){printf(" %s=",n);for(int i=0;i<l&&i<64;i++)printf("%02x",b[i]);if(l>64)printf("...");}
int main(int ac,char**av){int fd=open(av[1],O_RDWR);int u=atoi(av[2]);
 for(int s=1;s<=24;s++){unsigned char b[1024]={0};unsigned short len=0;
  if(q(fd,u,s,UVC_GET_LEN,2,(unsigned char*)&len)){printf("sel %2d: GET_LEN failed\n",s);continue;}
  unsigned char info=0;q(fd,u,s,UVC_GET_INFO,1,&info);
  printf("sel %2d: len %3u info 0x%02x (%s%s%s)",s,len,info,info&1?"get ":"",info&2?"set ":"",info&8?"autoupdate":"");
  if(len&&len<=1024&&(info&1)){const int qs[]={UVC_GET_CUR,UVC_GET_MIN,UVC_GET_MAX,UVC_GET_DEF};const char*ns[]={"cur","min","max","def"};
   for(int k=0;k<4;k++){memset(b,0,sizeof b);if(!q(fd,u,s,qs[k],len,b))hex(ns[k],b,len);}}
  printf("\n");}
 return 0;}
