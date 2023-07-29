/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/07/29/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     print_hello_world.h
 * @brief          :     print "hello world"
 * ===================================================================*/
#ifndef CMAKE_LEARNING_PRINT_HELLO_WORLD_PRINT_HELLO_WORLD_H_
#define CMAKE_LEARNING_PRINT_HELLO_WORLD_PRINT_HELLO_WORLD_H_

#include <iostream>

class PrintHelloWorld {
public:
  PrintHelloWorld();
  ~PrintHelloWorld();
};

PrintHelloWorld::PrintHelloWorld() {
  std::cout << "hello world !" << std::endl;
}

PrintHelloWorld::~PrintHelloWorld() {}

#endif // CMAKE_LEARNING_PRINT_HELLO_WORLD_PRINT_HELLO_WORLD_H_