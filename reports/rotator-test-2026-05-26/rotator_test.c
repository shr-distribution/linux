/*
 * Comprehensive qcom-rotator V4L2 m2m test harness.
 * Exercises a single transform and prints machine-readable results so the
 * host verifier can compare device output against a CPU reference.
 *
 * usage: rotator_test <in> <out> <fmt> <sw> <sh> <rot> <hflip> <vflip>
 *                     [cx cy cw ch] [colorspace]
 *   fmt: nv12 nv21 nv16 nv61 rgb565 argb32 xrgb32 nv12mt
 *   rot: 0 90 180 270 ; hflip/vflip: 0/1
 *   optional crop rect cx cy cw ch (source ROI)
 *   optional colorspace: integer V4L2_COLORSPACE_* to set on OUTPUT
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <errno.h>
#include <linux/videodev2.h>
#define die(m) do{perror(m);exit(1);}while(0)

static unsigned pf(const char*s){
    if(!strcmp(s,"nv12"))return V4L2_PIX_FMT_NV12;
    if(!strcmp(s,"nv21"))return V4L2_PIX_FMT_NV21;
    if(!strcmp(s,"nv16"))return V4L2_PIX_FMT_NV16;
    if(!strcmp(s,"nv61"))return V4L2_PIX_FMT_NV61;
    if(!strcmp(s,"rgb565"))return V4L2_PIX_FMT_RGB565;
    if(!strcmp(s,"argb32"))return V4L2_PIX_FMT_ARGB32;
    if(!strcmp(s,"xrgb32"))return V4L2_PIX_FMT_XRGB32;
    if(!strcmp(s,"nv12mt"))return V4L2_PIX_FMT_NV12MT;
    fprintf(stderr,"bad fmt %s\n",s);exit(1);
}

int main(int argc,char**argv){
    if(argc<9){fprintf(stderr,"usage: %s in out fmt sw sh rot hflip vflip [cx cy cw ch] [cs]\n",argv[0]);return 1;}
    const char*in=argv[1], *out=argv[2];
    unsigned fmt=pf(argv[3]);
    int sw=atoi(argv[4]), sh=atoi(argv[5]);
    int rot=atoi(argv[6]), hf=atoi(argv[7]), vf=atoi(argv[8]);
    int have_crop=0, cx=0,cy=0,cw=0,ch=0, cs=-1;
    if(argc>=13){have_crop=1;cx=atoi(argv[9]);cy=atoi(argv[10]);cw=atoi(argv[11]);ch=atoi(argv[12]);
        if(argc>=14)cs=atoi(argv[13]);}
    else if(argc>=10)cs=atoi(argv[9]);
    const char*dev="/dev/video10";

    int fd=open(dev,O_RDWR); if(fd<0)die("open");

    /* OUTPUT (source) format */
    struct v4l2_format f={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT};
    f.fmt.pix.width=sw; f.fmt.pix.height=sh; f.fmt.pix.pixelformat=fmt;
    if(cs>=0) f.fmt.pix.colorspace=cs;
    if(ioctl(fd,VIDIOC_S_FMT,&f))die("S_FMT OUT");
    unsigned osz=f.fmt.pix.sizeimage;
    printf("SRC w=%u h=%u bpl=%u sz=%u cs=%u\n",
           f.fmt.pix.width,f.fmt.pix.height,f.fmt.pix.bytesperline,osz,f.fmt.pix.colorspace);

    /* rotation / flip controls (set before crop so dst swap is known) */
    struct v4l2_control c;
    c.id=V4L2_CID_ROTATE;c.value=rot; if(ioctl(fd,VIDIOC_S_CTRL,&c))perror("ROTATE");
    c.id=V4L2_CID_HFLIP;c.value=hf; if(ioctl(fd,VIDIOC_S_CTRL,&c))perror("HFLIP");
    c.id=V4L2_CID_VFLIP;c.value=vf; if(ioctl(fd,VIDIOC_S_CTRL,&c))perror("VFLIP");

    /* optional source crop */
    if(have_crop){
        struct v4l2_selection sel={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .target=V4L2_SEL_TGT_CROP};
        sel.r.left=cx;sel.r.top=cy;sel.r.width=cw;sel.r.height=ch;
        if(ioctl(fd,VIDIOC_S_SELECTION,&sel))perror("S_SELECTION");
        printf("CROP set=%d,%d %ux%u -> got=%d,%d %ux%u\n",cx,cy,cw,ch,
               sel.r.left,sel.r.top,sel.r.width,sel.r.height);
    }

    /* CAPTURE format is forced by the driver; read it back */
    struct v4l2_format fc={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE};
    if(ioctl(fd,VIDIOC_G_FMT,&fc))die("G_FMT CAP");
    unsigned csz=fc.fmt.pix.sizeimage;
    char fourcc[5]={0}; memcpy(fourcc,&fc.fmt.pix.pixelformat,4);
    printf("CAP w=%u h=%u bpl=%u sz=%u cs=%u fourcc=%s\n",
           fc.fmt.pix.width,fc.fmt.pix.height,fc.fmt.pix.bytesperline,csz,
           fc.fmt.pix.colorspace,fourcc);

    struct v4l2_requestbuffers ro={.count=1,.type=V4L2_BUF_TYPE_VIDEO_OUTPUT,.memory=V4L2_MEMORY_MMAP};
    if(ioctl(fd,VIDIOC_REQBUFS,&ro))die("REQBUFS OUT");
    struct v4l2_requestbuffers rc={.count=1,.type=V4L2_BUF_TYPE_VIDEO_CAPTURE,.memory=V4L2_MEMORY_MMAP};
    if(ioctl(fd,VIDIOC_REQBUFS,&rc))die("REQBUFS CAP");

    struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT,.memory=V4L2_MEMORY_MMAP,.index=0};
    ioctl(fd,VIDIOC_QUERYBUF,&b); void*om=mmap(0,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
    struct v4l2_buffer bc={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE,.memory=V4L2_MEMORY_MMAP,.index=0};
    ioctl(fd,VIDIOC_QUERYBUF,&bc); void*cm=mmap(0,bc.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,bc.m.offset);

    memset(cm,0xCD,csz);    /* poison so we can tell what the HW actually wrote */
    int ifd=open(in,O_RDONLY); if(ifd<0)die("open in");
    unsigned rd=read(ifd,om,osz); close(ifd);
    printf("READ src=%u\n",rd);

    struct v4l2_buffer q={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT,.memory=V4L2_MEMORY_MMAP,.index=0,.bytesused=osz};
    if(ioctl(fd,VIDIOC_QBUF,&q))die("QBUF OUT");
    struct v4l2_buffer qc={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE,.memory=V4L2_MEMORY_MMAP,.index=0};
    if(ioctl(fd,VIDIOC_QBUF,&qc))die("QBUF CAP");
    int t=V4L2_BUF_TYPE_VIDEO_OUTPUT; if(ioctl(fd,VIDIOC_STREAMON,&t))die("STREAMON OUT");
    t=V4L2_BUF_TYPE_VIDEO_CAPTURE; if(ioctl(fd,VIDIOC_STREAMON,&t))die("STREAMON CAP");

    struct pollfd pfd={.fd=fd,.events=POLLIN};
    int pr=poll(&pfd,1,2000);
    if(pr<=0){printf("RESULT status=TIMEOUT\n");return 2;}
    struct v4l2_buffer d={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE,.memory=V4L2_MEMORY_MMAP};
    if(ioctl(fd,VIDIOC_DQBUF,&d))die("DQBUF CAP");
    if(d.flags & V4L2_BUF_FLAG_ERROR){printf("RESULT status=BUFERROR bytesused=%u\n",d.bytesused);return 3;}

    int of=open(out,O_WRONLY|O_CREAT|O_TRUNC,0644); write(of,cm,csz); close(of);
    printf("RESULT status=OK bytesused=%u wrote=%u\n",d.bytesused,csz);
    return 0;
}
