#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>

#include "struct_header.h"
#include "zmq.h"

int main() {
  frame_test frame;
  frame_test* buffer = new frame_test[10];
  std::cout << "size0:  "<<sizeof(*buffer) << std::endl;

  printf("Hello sub!\n");

  void* context = zmq_ctx_new();  // 创建上下文
  assert(context != NULL);

  void* subscriber = zmq_socket(context, ZMQ_SUB);  // 创建发布者
  assert(subscriber != NULL);

  const std::string address = "tcp://127.0.0.1:8888";  // 接收地址

  int ret = zmq_connect(subscriber, address.c_str());
  assert(ret == 0);

  // 设置订阅该端口上的所有信息，没有这一句，则收不到消息，可以使用过滤器过滤接收
  ret = zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
  assert(ret == 0);

  while (1) {
    printf("enter while to sub ctx\n");
    std::cout << "size:  "<<sizeof(*buffer) << std::endl;
    // note: 需要指定发送的数据长度 sizeof(buffer[0]) * 10
    ret = zmq_recv(subscriber, buffer, sizeof(buffer[0])*10, 0);
    if (ret > 0) {
      for (int i = 0; i < 5; i++) {
        printf("id:  %d\n", buffer[i].frame_id);
        printf("status:  %d\n", buffer[i].status);
        printf("radius:  %f\n", buffer[i].radius);
        printf("area:  %f\n", buffer[i].area);
        printf("\n");
        printf("\n");

      }
    }
  }

  delete[] buffer;
  buffer=NULL;

  zmq_close(subscriber);
  zmq_ctx_destroy(context);

  return 0;
}
