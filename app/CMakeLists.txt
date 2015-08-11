#*********************************************************************
#**                                                                 **
#** File   : apps/CMakeLists.txt                                    **
#** Authors: Qiang Li                                        **
#**                                                                 **
#**                                                                 **
#** GNU LESSER GENERAL PUBLIC LICENSE                               **
#** This file may be used under the terms of the GNU Lesser General **
#** Public License version 3.0 as published by the                  **
#**                                                                 **
#** Free Software Foundation and appearing in the file LICENSE.LGPL **
#** included in the packaging of this file.  Please review the      **
#** following information to ensure the license requirements will   **
#** be met: http://www.gnu.org/licenses/lgpl-3.0.txt                **
#**                                                                 **
#*********************************************************************

# set all apps
SET(APPS
         ApproachTest
         TipComTest
         TipServo
#        CQTest
#        KukaCartImpTest
#        RigTFEst
#        CBFTest
#        ExploringTest
#        LeftCurFolForce
#        SelfTouchTest
#        FMTestNoContact
#        LeftKukaTest
#        StiffTest
#        CompositionCtrlTest
#        FollowMotionTest
#        NewComTest
#        test_smoothing_filter
#        ContactTest
#        ICLCartImpTest
#        RightKukaTest
#        TwistFollowTest
#        NewappTest
        RemoteGUITest
        RemoteGuiControl
#        anotherapp
   )

# add headers
include_directories("${PROJECT_SOURCE_DIR}/src/ControllerModule")
include_directories("${PROJECT_SOURCE_DIR}/src/TaskModule")
include_directories("${PROJECT_SOURCE_DIR}/src/ComModule")
include_directories("${PROJECT_SOURCE_DIR}/src/UtilModule")
include_directories("${PROJECT_SOURCE_DIR}/src/RobotModule")
include_directories("${PROJECT_SOURCE_DIR}/src/LearningModule")
include_directories("${PROJECT_SOURCE_DIR}/src/TacModule")

## add icl headers
#include_directories(${ICL_INCLUDE_DIRS})

## add icl defines
#add_definitions(${ICL_DEFINITIONS})

# create executables
foreach(APP ${APPS})
  message(STATUS "-- Adding executable: ${APP}")

  add_executable("${PROJECT_NAME}-${APP}"
    "${PROJECT_SOURCE_DIR}/app/${APP}.cpp"
  )

  target_link_libraries("${PROJECT_NAME}-${APP}"
    ${CORE_LIBS}
    ControllerModule
    ComModule
    TaskModule
    RobotModule
    UtilModule
    LearningModule
    TacModule
  )

# set app version
#SET_TARGET_PROPERTIES("${PROJECT_NAME}-${APP}"
#                      PROPERTIES VERSION "${PROJECT_VERSION_MAJOR}"
#                      SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
#                      )

# install app
install(TARGETS "${PROJECT_NAME}-${APP}"
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE
       )
       
endforeach(APP)

