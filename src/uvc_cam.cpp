#include <cstring>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <uvc_cam/uvc_cam.h>

using std::string;

namespace uvc_cam {

Cam::Cam(const char *_device, mode_t _mode, int _width, int _height, int _fps)
    : mode(_mode), device(_device), width(_width), height(_height), fps(_fps),
      motion_threshold_luminance(100), motion_threshold_count(-1), rgb_frame(nullptr) 
{
    printf("Opening %s\n", _device);
    if ((fd = open(_device, O_RDWR)) == -1)
        throw std::runtime_error("Couldn't open " + device);

    memset(&fmt, 0, sizeof(v4l2_format));
    memset(&cap, 0, sizeof(v4l2_capability));
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
        throw std::runtime_error("Couldn't query " + device);

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        throw std::runtime_error(device + " does not support capture");
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
        throw std::runtime_error(device + " does not support streaming");

    // Enumerate formats
    v4l2_fmtdesc f;
    memset(&f, 0, sizeof(f));
    f.index = 0;
    f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret;
    while ((ret = ioctl(fd, VIDIOC_ENUM_FMT, &f)) == 0)
    {
        printf("pixfmt %d = '%4s' desc = '%s'\n", f.index++, (char *)&f.pixelformat, f.description);
        // Enumerate frame sizes
        v4l2_frmsizeenum fsize;
        fsize.index = 0;
        fsize.pixel_format = f.pixelformat;
        while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0)
        {
            fsize.index++;
            if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
            {
                printf("  discrete: %ux%u:   ", fsize.discrete.width, fsize.discrete.height);
                // Enumerate frame rates
                v4l2_frmivalenum fival;
                fival.index = 0;
                fival.pixel_format = f.pixelformat;
                fival.width = fsize.discrete.width;
                fival.height = fsize.discrete.height;
                while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0)
                {
                    fival.index++;
                    if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
                    {
                        printf("%u/%u ", fival.discrete.numerator, fival.discrete.denominator);
                    }
                    else
                        printf("I only handle discrete frame intervals...\n");
                }
                printf("\n");
            }
            else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
            {
                printf("  continuous: %ux%u to %ux%u\n", fsize.stepwise.min_width, fsize.stepwise.min_height, fsize.stepwise.max_width, fsize.stepwise.max_height);
            }
            else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
            {
                printf("  stepwise: %ux%u to %ux%u step %ux%u\n",
                        fsize.stepwise.min_width, fsize.stepwise.min_height,
                        fsize.stepwise.max_width, fsize.stepwise.max_height,
                        fsize.stepwise.step_width, fsize.stepwise.step_height);
            }
            else
            {
                printf("  fsize.type not supported: %d\n", fsize.type);
            }
        }
    }
    if (errno != EINVAL)
        throw std::runtime_error("Error enumerating frame formats");

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = (mode == MODE_RGB || mode == MODE_YUYV) ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if ((ret = ioctl(fd, VIDIOC_S_FMT, &fmt)) < 0)
        throw std::runtime_error("Couldn't set format");

    if (fmt.fmt.pix.width != width || fmt.fmt.pix.height != height)
        throw std::runtime_error("Pixel format unavailable");

    v4l2_streamparm streamparm;
    memset(&streamparm, 0, sizeof(streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = fps;

    ret = ioctl(fd, VIDIOC_S_PARM, &streamparm);
    if (ret < 0)
    {
        if (errno == ENOTTY)
        {
            printf("VIDIOC_S_PARM not supported on this v4l2 device, framerate not set\n");
        }
        else
        {
            throw std::runtime_error("Unable to set framerate");
        }
    }

    // Buffer allocation
    memset(&rb, 0, sizeof(rb));
    rb.count = NUM_BUFFER;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
        throw std::runtime_error("Unable to allocate buffers");

    if (NUM_BUFFER != rb.count)
        printf("Asked for %d buffers, got %d\n", NUM_BUFFER, rb.count);

    for (unsigned i = 0; i < NUM_BUFFER; i++)
    {
        memset(&buf, 0, sizeof(buf));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
            throw std::runtime_error("Unable to query buffer");
        if (buf.length <= 0)
            throw std::runtime_error("Buffer length is bogus");

        mem[i] = mmap(0, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (mem[i] == MAP_FAILED)
            throw std::runtime_error("Couldn't map buffer");
    }

    buf_length = buf.length;

    for (unsigned i = 0; i < NUM_BUFFER; i++)
    {
        memset(&buf, 0, sizeof(buf));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
            throw std::runtime_error("Unable to queue buffer");
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
        throw std::runtime_error("Unable to start capture");

    rgb_frame = new unsigned char[width * height * 3];
    last_yuv_frame = new unsigned char[width * height * 2];
}

Cam::~Cam() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    for (unsigned i = 0; i < NUM_BUFFER; i++)
    {
        munmap(mem[i], buf_length);
    }

    if (fd >= 0)
        close(fd);

    delete[] rgb_frame;
    delete[] last_yuv_frame;
}

inline unsigned char sat(float f)
{
    return (unsigned char)( f >= 255 ? 255 : (f < 0 ? 0 : f));
}

int Cam::grab(unsigned char **frame, uint32_t &bytes_used) {
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
    {
        if (errno == EAGAIN)
            return -EAGAIN; // Try again later
        throw std::runtime_error("Couldn't dequeue buffer");
    }

    if (buf.index >= NUM_BUFFER)
        throw std::runtime_error("Buffer index out of bounds");

    bytes_used = buf.bytesused;
    unsigned char *yuyv = static_cast<unsigned char*>(mem[buf.index]);

    // YUYV to RGB conversion
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x += 2)
        {
            int y1 = yuyv[(y * width + x) * 2];
            int u = yuyv[(y * width + x) * 2 + 1] - 128;
            int y2 = yuyv[(y * width + x + 1) * 2];
            int v = yuyv[(y * width + x + 1) * 2 + 1] - 128;

            rgb_frame[(y * width + x) * 3] = sat(y1 + 1.402 * v);
            rgb_frame[(y * width + x) * 3 + 1] = sat(y1 - 0.344136 * u - 0.714136 * v);
            rgb_frame[(y * width + x) * 3 + 2] = sat(y1 + 1.772 * u);

            rgb_frame[(y * width + x + 1) * 3] = sat(y2 + 1.402 * v);
            rgb_frame[(y * width + x + 1) * 3 + 1] = sat(y2 - 0.344136 * u - 0.714136 * v);
            rgb_frame[(y * width + x + 1) * 3 + 2] = sat(y2 + 1.772 * u);
        }
    }

    release(buf.index);
    *frame = rgb_frame;

    return buf.index;
}

void Cam::release(unsigned buf_idx) {
    memset(&buf, 0, sizeof(buf));
    buf.index = buf_idx;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
    {
        printf("Failed to requeue buffer\n");
    }
}

bool Cam::v4l2_query(int id, const std::string& name)
{
  if (fd < 0) {
    printf("Capture file not open: Can't %s\n", name.c_str());
    return false;
  }

  struct v4l2_queryctrl queryctrl;
  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = id;
  if (v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
    if (errno == EINVAL) {
      //error(FLF("Setting %s is not supported\n"), name.c_str());
    } else {
    //   ROS_WARN("Failed query %s", name.c_str());
    }
    return false;
  }

  return true;
}

bool Cam::set_v4l2_control(int id, int value, const std::string& name)
{
  if (fd < 0) {
    printf("Capture file not open: Can't %s\n", name.c_str());
    return false;
  }

  if (!v4l2_query(id, name)) {
      printf("Setting %s is not supported\n", name.c_str());
      return false;
  }

  struct v4l2_control control;
  memset(&control, 0, sizeof(control));
  control.id = id;
  control.value = value;
  if (v4l2_ioctl(fd, VIDIOC_S_CTRL, &control) < 0) {
    // ROS_WARN("Failed to change %s to %d", name.c_str(), control.value);
    return false;
  }

  return true;
}

void Cam::set_control(uint32_t id, int val) {
    v4l2_control control;
    memset(&control, 0, sizeof(control));
    control.id = id;
    control.value = val;

    if (ioctl(fd, VIDIOC_S_CTRL, &control) < 0)
        printf("Failed to set control %u to %d\n", id, val);
}

void Cam::set_motion_thresholds(int lum, int count) {
    motion_threshold_luminance = lum;
    motion_threshold_count = count;
}

} // namespace uvc_cam
