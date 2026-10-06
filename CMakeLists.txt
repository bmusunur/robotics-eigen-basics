cmake_minimum_required(VERSION 3.16)
project(eigen_basics CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

foreach(lesson 01_vectors_matrices 02_multiply_inverse 03_rotations 04_transforms 05_mini_project_fk 06_fk_project)
  add_executable(${lesson} ${lesson}.cpp)
  target_link_libraries(${lesson} Eigen3::Eigen)
endforeach()
