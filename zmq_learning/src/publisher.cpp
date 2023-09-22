#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>

#include "struct_header.h"
#include "zmq.h"

int main() {
  void* context = zmq_ctx_new();  // 创建上下文
  assert(context != NULL);

  void* publisher = zmq_socket(context, ZMQ_PUB);  // 创建发布者
  assert(publisher != NULL);

  const std::string address = "tcp://127.0.0.1:8888";  // 发布地址

  int ret = zmq_bind(publisher, address.c_str());  // 发布者与端口地址绑定

  assert(ret == 0);

  frame_test frame;

  frame_test* buffer = new frame_test[10];
  for (int i = 0; i < 10; i++) {
    frame.frame_id = i;
    frame.status = 1;
    frame.radius = 10 * i;
    frame.area = frame.radius * frame.radius * 3.14159;
    buffer[i] = frame;
  }
  for (int i = 0; i < 10; i++) {
    std::cout << "size_i:  " << sizeof(buffer[i]) << "    "
              << buffer[i].frame_id << std::endl;
  }

  std::cout << "size:  " << sizeof(*buffer) << std::endl;  // 16

  int i = 0;

  while (1) {
    std::cout << "publishing....   " << i++ << std::endl;
    // 返回值为发送的数据字节数，使用send将buffer中的数据发送出去，需要指定发送的数据长度
    // note: 需要指定发送的数据长度 sizeof(buffer[0]) * 10
    ret = zmq_send(publisher, buffer, sizeof(buffer[0]) * 10, 0);
    sleep(1);
  }
  delete[] buffer;
  buffer = NULL;

  zmq_close(publisher);      // 关闭发布者
  zmq_ctx_destroy(context);  // 销毁上下文

  return 0;
}