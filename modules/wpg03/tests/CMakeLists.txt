if (HUMOTO_BUILD_TESTS)
    humoto_define_test(${HUMOTO_MODULE} "000" "qpOASES")
endif()

if (HUMOTO_BUILD_REGRESSION_TESTS)
    humoto_define_regression_test(${HUMOTO_MODULE} "000" "qpOASES;config_yaml")
endif()
