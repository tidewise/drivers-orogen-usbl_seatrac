# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(usbl_seatracTaskLib)
ADD_LIBRARY(${USBL_SEATRAC_TASKLIB_NAME} SHARED
    ${USBL_SEATRAC_TASKLIB_SOURCES})
add_dependencies(${USBL_SEATRAC_TASKLIB_NAME}
    regen-typekit)


TARGET_LINK_LIBRARIES(${USBL_SEATRAC_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${USBL_SEATRAC_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${USBL_SEATRAC_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${USBL_SEATRAC_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${USBL_SEATRAC_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${USBL_SEATRAC_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${USBL_SEATRAC_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${USBL_SEATRAC_TASKLIB_HEADERS}
    DESTINATION include/orocos/usbl_seatrac)

