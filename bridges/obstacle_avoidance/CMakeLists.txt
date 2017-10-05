set (TGT_OBSTACLE_AVOIDANCE           "${BRIDGE_TARGET_PREFIX}_${HUMOTO_BRIDGE}")
set (OBSTACLE_AVOIDANCE_INTERFACE_DIR "${CMAKE_CURRENT_LIST_DIR}")

humoto_copy_file_if_exists( TGT_humoto_copy_headers                     "obstacle_avoidance.h"
                            "${OBSTACLE_AVOIDANCE_INTERFACE_DIR}/obstacle_avoidance.h"  "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(  TGT_humoto_copy_headers                     "obstacle_avoidance_dir"
                            "${OBSTACLE_AVOIDANCE_INTERFACE_DIR}/obstacles"    "${HUMOTO_INCLUDE_DIR}/obstacles")

set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES "" PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS "" PARENT_SCOPE)
