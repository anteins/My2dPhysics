#cmake最低版本需求，不加入此行会受到警告信息
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#项目名称
PROJECT(main)

#指定头文件目录,PROJECT_SOURCE_DIR为工程的根目录  
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PROJECT_SOURCE_DIR}/libs)

ADD_SUBDIRECTORY( src )

#把当前目录(.)下所有源代码文件和头文件加入变量SRC_LIST
AUX_SOURCE_DIRECTORY(./src SRC_LIST)

#生成应用程序 main (在windows下会自动生成main.exe)
ADD_EXECUTABLE(main ${SRC_LIST})
