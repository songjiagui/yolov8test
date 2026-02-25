// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <iostream>
#include <sys/stat.h>
#include <csignal>
#include <string>
#include <sstream>
#include <fstream> // 添加 fstream 头文件

#include "yolov8.h"
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
// ===== GPIO函数声明 =====
void gpio_init();
void gpio_high();
void gpio_low();



// 全局变量
cv::VideoWriter *g_writer = nullptr;  // 保存视频的 VideoWriter 指针
bool end = false;  // 控制程序结束的标志
// ===== 串口 =====
#define UART_DEV "/dev/ttyS1"
int uart_fd=-1;

void uart_init()
{
    uart_fd = open(UART_DEV, O_RDWR | O_NOCTTY);
    if (uart_fd < 0)
    {
        perror("open uart fail");
        return;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);

    cfmakeraw(&options);

    // 波特率
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // 8N1
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    tcsetattr(uart_fd, TCSANOW, &options);

    tcflush(uart_fd, TCIOFLUSH);

    fcntl(uart_fd, F_SETFL, 0);

    printf("UART init OK\n");
}


void uart_send_detect()
{
    if (uart_fd < 0) return;

    uint8_t cmd[] = {0x01,0x05,0x00,0x00,0xFF,0x00,0x8C,0x3A};
    write(uart_fd, cmd, sizeof(cmd));
    tcdrain(uart_fd);

    printf("RS485发送: 开继电器\n");
}

void uart_send_nodetect()
{
    if (uart_fd < 0) return;

    uint8_t cmd[] = {0x01,0x05,0x00,0x00,0x00,0x00,0xCD,0xCA};
    write(uart_fd, cmd, sizeof(cmd));
    tcdrain(uart_fd);

    printf("RS485发送: 关继电器\n");
}




    // ===== GPIO =====
    #define GPIO_NUM 104
    int gpio_fd = -1;

    void gpio_init()
    {
        system("echo 104 > /sys/class/gpio/export 2>/dev/null");
        system("echo out > /sys/class/gpio/gpio104/direction");

        gpio_fd = open("/sys/class/gpio/gpio104/value", O_WRONLY);

        if(gpio_fd < 0)
            printf("GPIO open fail\n");
        else
            printf("GPIO open OK\n");
    }

    void gpio_high()
    {
        if (gpio_fd >= 0)
        {
            lseek(gpio_fd,0,SEEK_SET);
            write(gpio_fd,"1",1);
        }
    }   

    void gpio_low()
    {
        if (gpio_fd >= 0)
        {
            lseek(gpio_fd,0,SEEK_SET);
            write(gpio_fd,"0",1);
        }
    }

// ===== 保存图片 =====
void save_result_image(cv::Mat frame, object_detect_result_list *res)
{
    for(int i=0;i<res->count;i++)
    {
        rectangle(frame,
        cv::Point(res->results[i].box.left,res->results[i].box.top),
        cv::Point(res->results[i].box.right,res->results[i].box.bottom),
        cv::Scalar(0,255,0),2);
    }

    // ===== 获取当前时间 =====
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    // ===== 图片显示时间 =====
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", t);

    putText(frame,time_str,cv::Point(30,50),
    cv::FONT_HERSHEY_SIMPLEX,1,
    cv::Scalar(0,0,255),2);

    // ===== 文件名时间（注意不能有冒号）=====
    char filename[128];
    char file_time[64];

    strftime(file_time,sizeof(file_time),"%Y-%m-%d_%H-%M-%S",t);

    sprintf(filename,"runs/%s.jpg",file_time);

    imwrite(filename,frame);

    printf("保存图片: %s\n", filename);
}



// 信号处理函数
void signal_handler(int signal)
{
    printf("Signal %d received. Terminating program...\n", signal);
    end = true;  // 设置结束标志
    if (g_writer != nullptr)
    {
        g_writer->release();  // 释放 VideoWriter
        delete g_writer;
        g_writer = nullptr;
    }
    exit(signal);  // 退出程序
}

/*-------------------------------------------
                  Functions
-------------------------------------------*/

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

// 生成唯一的输出文件名
std::string generate_unique_filename(const std::string& base_name, const std::string& output_dir)
{
    std::string filename = base_name + ".mp4";
    int counter = 1;
    while (std::ifstream((output_dir + "/" + filename).c_str()))
    {
        filename = base_name + "_" + std::to_string(counter) + ".mp4";
        counter++;
    }
    return output_dir + "/" + filename;
}

// 创建输出目录（如果不存在）
void create_directories(const std::string& output_dir)
{
    if (mkdir(output_dir.c_str(), 0777) == -1)
    {
        if (errno != EEXIST)
        {
            printf("Failed to create directory: %s\n", output_dir.c_str());
        }
    }
}

/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    bool detected_last_frame = false;

    // 注册信号处理函数
    signal(SIGINT, signal_handler);

    uart_init();
    gpio_init();
    gpio_low();


    if (argc < 3)
    {
        printf("%s <model path> <camera device id/video path/rtsp url> [nosave]\n", argv[0]);
        printf("Usage: %s  yolov8s.rknn  0\n", argv[0]);
        printf("Usage: %s  yolov8s.rknn /path/xxxx.mp4 nosave\n", argv[0]);
        printf("Usage: %s  yolov8s.rknn rtsp://example.com/stream\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_name = argv[2];
    bool save_result = false;

    for (int i = 3; i < argc; ++i)
    {
        if (std::string(argv[i]) == "nosave")
        {
            save_result = false;
        }
    }

    int ret;
    cv::Mat image, frame;
    struct timeval start_time, stop_time;
    rknn_app_context_t rknn_app_ctx;
    image_buffer_t src_image;
    object_detect_result_list od_results;

    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    memset(&src_image, 0, sizeof(image_buffer_t));

    cv::VideoCapture cap;
    if (strlen(device_name) == 1 && isdigit(device_name[0])) {
        // 打开摄像头
        int camera_id = atoi(argv[2]);
        cap.open(camera_id);
        if (!cap.isOpened()) {
            printf("Error: Could not open camera.\n");
            return -1;
        }
    } else {
        // 打开视频文件或 RTSP 流
        cap.open(argv[2]);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        
        // 丢弃堆积帧，只取最新帧（降低RTSP延迟）
        for (int i = 0; i < 5; ++i) {
            if (!cap.grab()) {
                printf("cap grab frame fail!\n");
                end = true;
            break;
            }
        }
        if (end) break;

        if (!cap.retrieve(frame)) {
            printf("cap retrieve frame fail!\n");
            break;
        }
    }

    std::string output_dir = "runs";
    create_directories(output_dir);

    if (save_result) {
        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        double fps = cap.get(cv::CAP_PROP_FPS);
        cv::Size frame_size(static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
                            static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
        std::string output_file = generate_unique_filename("result", output_dir);
        g_writer = new cv::VideoWriter(output_file, fourcc, fps, frame_size);
        if (!g_writer->isOpened()) {
            printf("Error: Could not open output video file.\n");
            return -1;
        }
        printf("Saving results to: %s\n", output_file.c_str());
    }

    // 初始化
    init_post_process();
    ret = init_yolov8_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov8_seg_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

    // 推理，画框，显示
    while (!end) {
        gettimeofday(&start_time, NULL);

        if (!cap.read(frame)) {  
            printf("cap read frame fail!\n");
            break;  
        }  

        cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
        src_image.width  = image.cols;
        src_image.height = image.rows;
        src_image.format = IMAGE_FORMAT_RGB888;
        src_image.virt_addr = (unsigned char*)image.data;

        ret = inference_yolov8_model(&rknn_app_ctx, &src_image, &od_results);
        if (ret != 0)
        {
            printf("init_yolov8_seg_model fail! ret=%d\n", ret);
            goto out;
        }

        bool detected_now = (od_results.count > 0);

// ===== GPIO持续电平 =====
if(detected_now)
    gpio_high();
else
    gpio_low();

// ===== RS485只发一次 =====
if(detected_now && !detected_last_frame)
{
    printf("检测到 -> 发送开启\n");
    uart_send_detect();
    save_result_image(frame,&od_results);
}
else if(!detected_now && detected_last_frame)
{
    printf("目标消失 -> 发送关闭\n");
    uart_send_nodetect();
}

detected_last_frame = detected_now;


        // draw boxes
        char text[256];
        for (int i = 0; i < od_results.count; i++)
        {
            object_detect_result *det_result = &(od_results.results[i]);
            //printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
            //    det_result->box.left, det_result->box.top,
            //    det_result->box.right, det_result->box.bottom,
            //    det_result->prop);
            int x1 = det_result->box.left;
            int y1 = det_result->box.top;
            int x2 = det_result->box.right;
            int y2 = det_result->box.bottom;

            rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0, 255), 2);
            sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
            putText(frame, text, cv::Point(x1, y1 - 6), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0,0,255), 1, cv::LINE_AA);
        }

        // 计算FPS
        gettimeofday(&stop_time, NULL);
        float t = (__get_us(stop_time) - __get_us(start_time))/1000;
        static int log_cnt = 0;
        if ((log_cnt++ % 30) == 0) {
            printf("Infer time(ms): %.3f ms\n", t);
        }
        putText(frame, cv::format("FPS: %.2f", 1.0 / (t / 1000)), cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255, 0, 0), 2, 8);
        cv::imshow("YOLOv8 C++ Demo", frame);

        //禁止保存视频
        /* if (save_result) {
            g_writer->write(frame);
        } */

        char c = cv::waitKey(1);
        if (c == 27) { // ESC
            break;
        }
    }

out:
    deinit_post_process();

    ret = release_yolov8_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolov8_seg_model fail! ret=%d\n", ret);
    }

    if (save_result) {
        g_writer->release();
        delete g_writer;
        g_writer = nullptr;
    }
    cap.release();
    cv::destroyAllWindows();

    return 0;
}    
