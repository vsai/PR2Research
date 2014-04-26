# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "yoloswag: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iyoloswag:/home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(yoloswag_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(yoloswag
  "/home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yoloswag
)

### Generating Services
_generate_srv_cpp(yoloswag
  "/home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/srv/RecordAudio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yoloswag
)

### Generating Module File
_generate_module_cpp(yoloswag
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yoloswag
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(yoloswag_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(yoloswag_generate_messages yoloswag_generate_messages_cpp)

# target for backward compatibility
add_custom_target(yoloswag_gencpp)
add_dependencies(yoloswag_gencpp yoloswag_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yoloswag_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(yoloswag
  "/home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yoloswag
)

### Generating Services
_generate_srv_lisp(yoloswag
  "/home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/srv/RecordAudio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yoloswag
)

### Generating Module File
_generate_module_lisp(yoloswag
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yoloswag
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(yoloswag_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(yoloswag_generate_messages yoloswag_generate_messages_lisp)

# target for backward compatibility
add_custom_target(yoloswag_genlisp)
add_dependencies(yoloswag_genlisp yoloswag_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yoloswag_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(yoloswag
  "/home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yoloswag
)

### Generating Services
_generate_srv_py(yoloswag
  "/home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/srv/RecordAudio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yoloswag
)

### Generating Module File
_generate_module_py(yoloswag
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yoloswag
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(yoloswag_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(yoloswag_generate_messages yoloswag_generate_messages_py)

# target for backward compatibility
add_custom_target(yoloswag_genpy)
add_dependencies(yoloswag_genpy yoloswag_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yoloswag_generate_messages_py)


debug_message(2 "yoloswag: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yoloswag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yoloswag
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(yoloswag_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yoloswag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yoloswag
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(yoloswag_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yoloswag)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yoloswag\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yoloswag
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(yoloswag_generate_messages_py std_msgs_generate_messages_py)
