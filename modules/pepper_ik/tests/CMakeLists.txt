if (HUMOTO_BUILD_TESTS)
    # Test model with floating base
    humoto_define_test(${HUMOTO_MODULE} "000" "qpOASES;config_yaml;rbdl")

    # Test model with planar root joint
    humoto_define_test(${HUMOTO_MODULE} "001" "config_yaml;rbdl")

    # Test IK + MPC
    humoto_define_test(${HUMOTO_MODULE} "002" "rbdl;config_yaml;qpOASES")

    # Feasibility test
    humoto_define_test(${HUMOTO_MODULE} "003" "rbdl;config_yaml;qpOASES")
    
    # Test MPC + IK head control using cartesion angular velocity command
    humoto_define_test(${HUMOTO_MODULE} "004" "rbdl;config_yaml;qpOASES")
endif()

if (HUMOTO_BUILD_REGRESSION_TESTS)
    # Crosscheck RBDL, rmt, and RBDL interface functions
    #   computation of the CoM position
    #   CoM Jacobians
    humoto_define_regression_test(${HUMOTO_MODULE} "000" "rbdl;config_yaml")

    # Test IK
    humoto_define_regression_test(${HUMOTO_MODULE} "001" "rbdl;config_yaml")
endif()
