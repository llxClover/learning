/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/07/29/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     proto_test.cpp
 * @brief          :     1. 向 Student 消息类对象进行赋值，并且进行序列化操作
 *                       2. 再从序列化结果进行反序列化操作，解析需要的字段信息
 * ===================================================================*/

#include "print_hello_world.h"
#include "student.pb.h"

#include <iostream>
#include <string>

int main(int argc, char *argv[]) {

  PrintHelloWorld print_hello_word;

  protobuf_learning::Student student;

  // 1. 对于简单的非嵌套，用 set_ 赋值
  student.set_id(202022030042);
  student.set_name("lilinxiang");
  // *student.mutable_name()="lilinxiang";
  student.set_email("lilinxiang@tsari.tsinghua.edu.cn");

  // 2. 自己定义的复杂嵌套消息，用 mutable_ 赋值
  // 3. 带有repeated字段的消息，通过  add_  依次赋值
  // student_1
  protobuf_learning::Student::PhoneNumber *phone_num_1 = student.add_phone();
  phone_num_1->set_number("0564-4762652");
  phone_num_1->set_type(protobuf_learning::Student::HOME);

  // student_2
  protobuf_learning::Student::PhoneNumber *phone_num_2 = student.add_phone();
  // mutable_ 内部new操作，所以是指针
  *phone_num_2->mutable_number() = "18532003967";
  phone_num_2->set_type(protobuf_learning::Student::MOBILE);

  // 序列化到 string
  std::string serialized_str;
  student.SerializeToString(&serialized_str);

  std::cout << "serialized results:    " << serialized_str << std::endl;
  std::cout << "debug string:          " << student.DebugString() << std::endl;

  // 反序列化 得到消息对象
  protobuf_learning::Student deserialized_student;
  if (!deserialized_student.ParseFromString(serialized_str)) {
    std::cerr << "Failed to parse student." << std::endl;
    return -1;
  }

  std::cout << "deserizalized result:    " << deserialized_student.DebugString()
            << std::endl;
  std::cout << "student ID:  " << deserialized_student.id() << "    "
            << "Name:   " << deserialized_student.name() << std::endl;

  // optional 字段，有个has_()方法用于检查是否赋值
  if (deserialized_student.has_email()) {
    std::cout << "Email:    " << deserialized_student.email() << std::endl;
  }

  for (int i = 0; i < deserialized_student.phone_size(); i++) {
    const protobuf_learning::Student::PhoneNumber phone_number =
        deserialized_student.phone(i);

    switch (phone_number.type()) {
    case protobuf_learning::Student::MOBILE:
      std::cout << "Mobile number:   ";
      break;
    case protobuf_learning::Student::HOME:
      std::cout << "Home number:   ";
      break;
    }
    std::cout << phone_number.number() << std::endl;
  }

  // 关闭protobuf库
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}

// g++ 编译指令
// llx@llx-pc:~/learning/protobuf_learning$ g++ -I
// /home/llx/learning/protobuf_learning
// src/proto_test.cpp proto/student.pb.cc
// -o proto_test.out `pkg-config --libs protobuf`

// 运行指令：
// llx@llx-pc:~/learning/protobuf_learning$ ./proto_test.out

// 输出：
/*
llx@llx-pc:~/learning/protobuf_learning$ ./proto_test.out
serialized results:   ڕ���
lilinxiang� lilinxiang@tsari.tsinghua.edu.cn"

0564-4762652"

18532003967
debug string:          id: 202022030042
name: "lilinxiang"
email: "lilinxiang@tsari.tsinghua.edu.cn"
phone {
  number: "0564-4762652"
  type: HOME
}
phone {
  number: "18532003967"
  type: MOBILE
}

deserizalized result:    id: 202022030042
name: "lilinxiang"
email: "lilinxiang@tsari.tsinghua.edu.cn"
phone {
  number: "0564-4762652"
  type: HOME
}
phone {
  number: "18532003967"
  type: MOBILE
}

student ID:  202022030042    Name:   lilinxiang
Email:    lilinxiang@tsari.tsinghua.edu.cn
Home number:   0564-4762652
Mobile number:   18532003967
*/